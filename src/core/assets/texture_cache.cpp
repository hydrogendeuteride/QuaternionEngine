#include "texture_cache.h"

#include <core/context.h>
#include <core/device/resource.h>
#include <core/descriptor/descriptors.h>
#include <core/config.h>
#include <algorithm>
#include "stb_image.h"
#include <ktx.h>
#include <ktxvulkan.h>
#include <algorithm>
#include "core/device/device.h"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <cmath>

void TextureCache::init(EngineContext *ctx)
{
    _context = ctx;
    _running = true;
    unsigned int threads = std::max(1u, std::min(4u, std::thread::hardware_concurrency()));
    _decodeThreads.reserve(threads);
    for (unsigned int i = 0; i < threads; ++i)
    {
        _decodeThreads.emplace_back([this]() { worker_loop(); });
    }
}

void TextureCache::cleanup()
{
    // Stop worker thread first
    if (_running.exchange(false))
    {
        {
            std::lock_guard<std::mutex> lk(_qMutex);
        }
        _qCV.notify_all();
        for (auto &t : _decodeThreads) if (t.joinable()) t.join();
        _decodeThreads.clear();
    }
    // Clear any pending decode/upload work (freeing decode heap pointers)
    {
        std::lock_guard<std::mutex> lk(_qMutex);
        _queue.clear();
    }
    {
        std::lock_guard<std::mutex> lk(_readyMutex);
        for (auto &r : _ready)
        {
            if (r.heap)
            {
                stbi_image_free(r.heap);
                r.heap = nullptr;
            }
        }
        _ready.clear();
    }
    if (!_context || !_context->getResources()) return;
    auto *rm = _context->getResources();
    for (TextureHandle h = 0; h < _entries.size(); ++h)
    {
        auto &e = _entries[h];
        if (e.state == EntryState::Resident && e.image.image)
        {
            fmt::println("[TextureCache] cleanup destroy handle={} path='{}' bytes={}",
                         h,
                         e.path.empty() ? "<bytes>" : e.path,
                         e.sizeBytes);
            rm->destroy_image(e.image);
            e.image = {};
        }
        e.state = EntryState::Evicted;
    }
    _residentBytes = 0;
    _lookup.clear();
    _setToHandles.clear();
}

TextureCache::TextureHandle TextureCache::request(const TextureKey &key, VkSampler sampler)
{
    // Ensure we have a valid, stable hash for deduplication.
    TextureKey normKey = key;
    if (normKey.hash == 0)
    {
        if (normKey.kind == TextureKey::SourceKind::FilePath)
        {
            std::string id = std::string("PATH:") + normKey.path + (normKey.srgb ? "#sRGB" : "#UNORM");
            normKey.hash = texcache::fnv1a64(id);
        }
        else if (!normKey.bytes.empty())
        {
            uint64_t h = texcache::fnv1a64(normKey.bytes.data(), normKey.bytes.size());
            normKey.hash = h ^ (normKey.srgb ? 0x9E3779B97F4A7C15ull : 0ull);
        }
    }

    auto it = _lookup.find(normKey.hash);
    if (it != _lookup.end())
    {
        TextureHandle h = it->second;
        if (h < _entries.size())
        {
            Entry &e = _entries[h];
            // Keep most recent sampler for future patches if provided
            if (sampler != VK_NULL_HANDLE)
            {
                e.sampler = sampler;
            }
            // Allow re-supplying CPU source bytes for Bytes-backed textures after an unload.
            if (normKey.kind == TextureKey::SourceKind::Bytes && !normKey.bytes.empty() &&
                e.key.kind == TextureKey::SourceKind::Bytes && e.bytes.empty() && e.state != EntryState::Resident)
            {
                e.bytes = normKey.bytes;
                _cpuSourceBytes += e.bytes.size();
            }
        }
        return h;
    }

    TextureHandle h = static_cast<TextureHandle>(_entries.size());
    _lookup.emplace(normKey.hash, h);

    Entry e{};
    e.key = normKey;
    // Keep only metadata in the key to avoid duplicating potentially large payloads.
    e.key.path.clear();
    e.key.bytes.clear();
    e.key.bytes.shrink_to_fit();
    e.sampler = sampler;
    e.state = EntryState::Unloaded;
    if (normKey.kind == TextureKey::SourceKind::FilePath)
    {
        e.path = normKey.path;
    }
    else
    {
        e.bytes = normKey.bytes;
        _cpuSourceBytes += e.bytes.size();
    }
    fmt::println("[TextureCache] request handle={} kind={} path='{}' srgb={} mipmapped={} hash=0x{:016x}",
                 h,
                 (normKey.kind == TextureKey::SourceKind::FilePath ? "FilePath" : "Bytes"),
                 normKey.kind == TextureKey::SourceKind::FilePath ? normKey.path : "<bytes>",
                 normKey.srgb,
                 normKey.mipmapped,
                 normKey.hash);
    _entries.push_back(std::move(e));
    return h;
}

void TextureCache::watchBinding(TextureHandle handle, VkDescriptorSet set, uint32_t binding,
                                VkSampler sampler, VkImageView fallbackView)
{
    if (handle == InvalidHandle) return;
    if (handle >= _entries.size()) return;
    Entry &e = _entries[handle];
    // Track patch
    Patch p{};
    p.set = set;
    p.binding = binding;
    p.sampler = sampler ? sampler : e.sampler;
    p.fallbackView = fallbackView;
    e.patches.push_back(p);

    // Back-reference for fast per-set markUsed
    _setToHandles[set].push_back(handle);

    // If the texture is already resident, immediately patch the new descriptor
    // so re-spawned models using cached textures get the correct bindings.
    if (e.state == EntryState::Resident && e.image.imageView != VK_NULL_HANDLE && set != VK_NULL_HANDLE)
    {
        if (!_context || !_context->getDevice()) return;
        DescriptorWriter writer;
        writer.write_image(static_cast<int>(binding), e.image.imageView,
                           p.sampler ? p.sampler : e.sampler,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(_context->getDevice()->device(), set);
    }
}

void TextureCache::unwatchSet(VkDescriptorSet set)
{
    if (set == VK_NULL_HANDLE) return;
    auto it = _setToHandles.find(set);
    if (it == _setToHandles.end()) return;

    const auto &handles = it->second;
    for (TextureHandle h : handles)
    {
        if (h >= _entries.size()) continue;
        auto &patches = _entries[h].patches;
        patches.erase(std::remove_if(patches.begin(), patches.end(),
                                     [&](const Patch &p){ return p.set == set; }),
                      patches.end());
    }
    _setToHandles.erase(it);
}

void TextureCache::markUsed(TextureHandle handle, uint32_t frameIndex)
{
    if (handle == InvalidHandle) return;
    if (handle >= _entries.size()) return;
    _entries[handle].lastUsedFrame = frameIndex;
}

void TextureCache::markSetUsed(VkDescriptorSet set, uint32_t frameIndex)
{
    auto it = _setToHandles.find(set);
    if (it == _setToHandles.end()) return;
    for (TextureHandle h : it->second)
    {
        if (h < _entries.size())
        {
            _entries[h].lastUsedFrame = frameIndex;
        }
    }
}

void TextureCache::pin(TextureHandle handle)
{
    if (handle == InvalidHandle) return;
    if (handle >= _entries.size()) return;
    _entries[handle].pinned = true;
}

void TextureCache::unpin(TextureHandle handle)
{
    if (handle == InvalidHandle) return;
    if (handle >= _entries.size()) return;
    _entries[handle].pinned = false;
}

bool TextureCache::is_pinned(TextureHandle handle) const
{
    if (handle == InvalidHandle) return false;
    if (handle >= _entries.size()) return false;
    return _entries[handle].pinned;
}

static inline size_t bytes_per_texel(VkFormat fmt)
{
    switch (fmt)
    {
        case VK_FORMAT_R8_UNORM:
        case VK_FORMAT_R8_SRGB:
            return 1;
        case VK_FORMAT_R8G8_UNORM:
        case VK_FORMAT_R8G8_SRGB:
            return 2;
        case VK_FORMAT_R8G8B8A8_UNORM:
        case VK_FORMAT_R8G8B8A8_SRGB:
        case VK_FORMAT_B8G8R8A8_UNORM:
        case VK_FORMAT_B8G8R8A8_SRGB:
            return 4;
        default:
            return 4;
    }
}

static inline VkFormat to_srgb_variant(VkFormat fmt)
{
    switch (fmt)
    {
        case VK_FORMAT_BC1_RGB_UNORM_BLOCK:   return VK_FORMAT_BC1_RGB_SRGB_BLOCK;
        case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:  return VK_FORMAT_BC1_RGBA_SRGB_BLOCK;
        case VK_FORMAT_BC2_UNORM_BLOCK:       return VK_FORMAT_BC2_SRGB_BLOCK;
        case VK_FORMAT_BC3_UNORM_BLOCK:       return VK_FORMAT_BC3_SRGB_BLOCK;
        case VK_FORMAT_BC7_UNORM_BLOCK:       return VK_FORMAT_BC7_SRGB_BLOCK;
        case VK_FORMAT_R8G8B8A8_UNORM:        return VK_FORMAT_R8G8B8A8_SRGB;
        case VK_FORMAT_B8G8R8A8_UNORM:        return VK_FORMAT_B8G8R8A8_SRGB;
        case VK_FORMAT_R8_UNORM:              return VK_FORMAT_R8_SRGB;
        case VK_FORMAT_R8G8_UNORM:            return VK_FORMAT_R8G8_SRGB;
        default: return fmt;
    }
}

static inline VkFormat to_unorm_variant(VkFormat fmt)
{
    switch (fmt)
    {
        case VK_FORMAT_BC1_RGB_SRGB_BLOCK:   return VK_FORMAT_BC1_RGB_UNORM_BLOCK;
        case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:  return VK_FORMAT_BC1_RGBA_UNORM_BLOCK;
        case VK_FORMAT_BC2_SRGB_BLOCK:       return VK_FORMAT_BC2_UNORM_BLOCK;
        case VK_FORMAT_BC3_SRGB_BLOCK:       return VK_FORMAT_BC3_UNORM_BLOCK;
        case VK_FORMAT_BC7_SRGB_BLOCK:       return VK_FORMAT_BC7_UNORM_BLOCK;
        case VK_FORMAT_R8G8B8A8_SRGB:        return VK_FORMAT_R8G8B8A8_UNORM;
        case VK_FORMAT_B8G8R8A8_SRGB:        return VK_FORMAT_B8G8R8A8_UNORM;
        case VK_FORMAT_R8_SRGB:              return VK_FORMAT_R8_UNORM;
        case VK_FORMAT_R8G8_SRGB:            return VK_FORMAT_R8G8_UNORM;
        default: return fmt;
    }
}

static inline float mip_factor_for_levels(uint32_t levels)
{
    if (levels <= 1) return 1.0f;
    // Sum of geometric series for area across mips (base * (1 + 1/4 + ...))
    // factor = (1 - 4^{-L}) / (1 - 1/4) = 4/3 * (1 - 4^{-L})
    float L = static_cast<float>(levels);
    return 1.3333333f * (1.0f - std::pow(0.25f, L));
}

static inline VkFormat choose_format(TextureCache::TextureKey::ChannelsHint hint, bool srgb)
{
    using CH = TextureCache::TextureKey::ChannelsHint;
    switch (hint)
    {
        case CH::R:  return srgb ? VK_FORMAT_R8_SRGB     : VK_FORMAT_R8_UNORM;
        case CH::RG: return srgb ? VK_FORMAT_R8G8_SRGB   : VK_FORMAT_R8G8_UNORM;
        case CH::RGBA:
        case CH::Auto:
        default:     return srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
    }
}

// Nearest-neighbor downscale-by-2 in-place helper (returns newly allocated buffer)
static std::vector<uint8_t> downscale_half(const unsigned char* src, int w, int h, int comps)
{
    int nw = std::max(1, w / 2);
    int nh = std::max(1, h / 2);
    std::vector<uint8_t> out(static_cast<size_t>(nw) * nh * comps);
    for (int y = 0; y < nh; ++y)
    {
        for (int x = 0; x < nw; ++x)
        {
            int sx = std::min(w - 1, x * 2);
            int sy = std::min(h - 1, y * 2);
            const unsigned char* sp = src + (static_cast<size_t>(sy) * w + sx) * comps;
            unsigned char* dp = out.data() + (static_cast<size_t>(y) * nw + x) * comps;
            std::memcpy(dp, sp, comps);
        }
    }
    return out;
}

void TextureCache::start_load(Entry &e, ResourceManager &rm)
{
    // Legacy synchronous path retained for completeness but not used by pumpLoads now.
    enqueue_decode(e);
}

void TextureCache::patch_ready_entry(const Entry &e)
{
    if (!_context || !_context->getDevice()) return;
    if (e.state != EntryState::Resident) return;

    DescriptorWriter writer;
    for (const Patch &p : e.patches)
    {
        if (p.set == VK_NULL_HANDLE) continue;
        writer.clear();
        writer.write_image(static_cast<int>(p.binding), e.image.imageView,
                           p.sampler ? p.sampler : e.sampler,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(_context->getDevice()->device(), p.set);
    }
}

void TextureCache::patch_to_fallback(const Entry &e)
{
    if (!_context || !_context->getDevice()) return;
    DescriptorWriter writer;
    for (const Patch &p : e.patches)
    {
        if (p.set == VK_NULL_HANDLE || p.fallbackView == VK_NULL_HANDLE) continue;
        writer.clear();
        writer.write_image(static_cast<int>(p.binding), p.fallbackView,
                           p.sampler ? p.sampler : e.sampler,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(_context->getDevice()->device(), p.set);
    }
}

void TextureCache::pumpLoads(ResourceManager &rm, FrameResources &)
{
    // Simple throttle to avoid massive spikes.
    int started = 0;
    const uint32_t now = _context ? _context->frameIndex : 0u;
    // First, drain decoded results with a byte budget.
    size_t admitted = drain_ready_uploads(rm, _maxBytesPerPump);

    // If we exhausted the budget, avoid scheduling more decodes this frame.
    bool budgetRemaining = (admitted < _maxBytesPerPump);

    for (auto &e : _entries)
    {
        // Allow both Unloaded and Evicted entries to start work if seen again.
        if (e.state == EntryState::Unloaded || e.state == EntryState::Evicted)
        {
            // Visibility-driven residency: only start uploads for textures
            // that were marked used recently (current or previous frame).
            // This avoids uploading assets that are not visible.
            bool recentlyUsed = true;
            if (_context)
            {
                // Schedule when first seen (previous frame) or if seen again.
                recentlyUsed = (now == 0u) || (now - e.lastUsedFrame <= 1u);
            }
            // Gate reload attempts to avoid rapid oscillation right after eviction.
            bool cooldownPassed = (now >= e.nextAttemptFrame);
            if (recentlyUsed && cooldownPassed && budgetRemaining)
            {
                enqueue_decode(e);
                if (++started >= _maxLoadsPerPump) break;
            }
        }
    }

    // Drain any remaining decoded results if we still have headroom.
    if (budgetRemaining)
    {
        drain_ready_uploads(rm, _maxBytesPerPump - admitted);
    }

    // Optionally trim retained compressed sources to CPU budget.
    evictCpuToBudget();
}

void TextureCache::evictToBudget(size_t budgetBytes)
{
    if (_residentBytes <= budgetBytes) return;

    // Gather candidates
    std::vector<std::pair<TextureHandle, uint32_t>> order;
    order.reserve(_entries.size());
    for (TextureHandle h = 0; h < _entries.size(); ++h)
    {
        const auto &e = _entries[h];
        if (e.state == EntryState::Resident && !e.pinned)
        {
            order.emplace_back(h, e.lastUsedFrame);
        }
    }
    std::sort(order.begin(), order.end(), [](auto &a, auto &b) { return a.second < b.second; });

    const uint32_t now = _context ? _context->frameIndex : 0u;
    for (auto &pair : order)
    {
        if (_residentBytes <= budgetBytes) break;
        TextureHandle h = pair.first;
        Entry &e = _entries[h];
        if (e.state != EntryState::Resident) continue;
        // Never evict pinned textures
        if (e.pinned) continue;
        // Prefer not to evict textures used this frame unless strictly necessary.
        if (e.lastUsedFrame == now) continue;

        // Rewrite watchers back to fallback before destroying
        patch_to_fallback(e);

        fmt::println("[TextureCache] evictToBudget destroy handle={} path='{}' bytes={} residentBytesBefore={}",
                     h,
                     e.path.empty() ? "<bytes>" : e.path,
                     e.sizeBytes,
                     _residentBytes);
        _context->getResources()->destroy_image(e.image);
        e.image = {};
        e.state = EntryState::Evicted;
        e.lastEvictedFrame = now;
        e.nextAttemptFrame = std::max(e.nextAttemptFrame, now + _reloadCooldownFrames);
        if (_residentBytes >= e.sizeBytes) _residentBytes -= e.sizeBytes; else _residentBytes = 0;
    }
}

bool TextureCache::unload(TextureHandle handle, bool drop_source_bytes)
{
    if (handle == InvalidHandle) return false;
    if (handle >= _entries.size()) return false;

    Entry &e = _entries[handle];
    const uint32_t now = _context ? _context->frameIndex : 0u;

    // Invalidate any in-flight decode results for this entry.
    e.generation++;

    // Drop queued decode requests for this handle.
    {
        std::lock_guard<std::mutex> lk(_qMutex);
        _queue.erase(std::remove_if(_queue.begin(), _queue.end(),
                                    [&](const DecodeRequest &rq) { return rq.handle == handle; }),
                     _queue.end());
    }

    // Drop already-decoded results waiting for upload for this handle.
    {
        std::lock_guard<std::mutex> lk(_readyMutex);
        for (auto it = _ready.begin(); it != _ready.end();)
        {
            if (it->handle == handle)
            {
                if (it->heap)
                {
                    stbi_image_free(it->heap);
                    it->heap = nullptr;
                }
                it = _ready.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    // If resident, patch watchers back to fallback and destroy the image.
    if (e.state == EntryState::Resident && e.image.image != VK_NULL_HANDLE)
    {
        patch_to_fallback(e);

        if (_context && _context->getResources())
        {
            fmt::println("[TextureCache] unload destroy handle={} path='{}' bytes={} residentBytesBefore={}",
                         handle,
                         e.path.empty() ? "<bytes>" : e.path,
                         e.sizeBytes,
                         _residentBytes);
            _context->getResources()->destroy_image(e.image);
        }

        e.image = {};
        if (_residentBytes >= e.sizeBytes) _residentBytes -= e.sizeBytes; else _residentBytes = 0;
    }

    e.state = EntryState::Evicted;
    e.lastEvictedFrame = now;
    e.nextAttemptFrame = std::max(e.nextAttemptFrame, now + _reloadCooldownFrames);

    if (drop_source_bytes)
    {
        this->drop_source_bytes(e);
    }

    return true;
}

void TextureCache::enqueue_decode(Entry &e)
{
    if (e.state != EntryState::Unloaded && e.state != EntryState::Evicted) return;
    e.state = EntryState::Loading;
    DecodeRequest rq{};
    rq.handle = static_cast<TextureHandle>(&e - _entries.data());
    rq.generation = e.generation;
    rq.key = e.key;
    if (e.key.kind == TextureKey::SourceKind::FilePath) rq.path = e.path; else rq.bytes = e.bytes;
    {
        std::lock_guard<std::mutex> lk(_qMutex);
        _queue.push_back(std::move(rq));
    }
    _qCV.notify_one();
}

void TextureCache::worker_loop()
{
    while (_running)
    {
        DecodeRequest rq{};
        {
            std::unique_lock<std::mutex> lk(_qMutex);
            _qCV.wait(lk, [this]{ return !_running || !_queue.empty(); });
            if (!_running) break;
            rq = std::move(_queue.front());
            _queue.pop_front();
        }

        DecodedResult out{};
        out.handle = rq.handle;
        out.generation = rq.generation;
        out.mipmapped = rq.key.mipmapped;
        out.srgb = rq.key.srgb;
        out.channels = rq.key.channels;
        out.mipClampLevels = rq.key.mipClampLevels;

        // 1) Prefer KTX2 when source is a file path and a .ktx2 version exists
        bool attemptedKTX2 = false;
        if (rq.key.kind == TextureKey::SourceKind::FilePath)
        {
            std::filesystem::path p = rq.path;
            std::filesystem::path ktxPath;
            if (p.extension() == ".ktx2")
            {
                ktxPath = p;
            }
            else
            {
                ktxPath = p;
                ktxPath.replace_extension(".ktx2");
            }
            std::error_code ec;
            bool hasKTX2 = (!ktxPath.empty() && std::filesystem::exists(ktxPath, ec) && !ec);
            if (hasKTX2)
            {
                attemptedKTX2 = true;
                ktxTexture2* ktex = nullptr;
                ktxResult kres = ktxTexture2_CreateFromNamedFile(ktxPath.string().c_str(), KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &ktex);
                if (kres != KTX_SUCCESS || !ktex)
                {
                    fmt::println("[TextureCache] libktx open failed for '{}': {}", ktxPath.string(), ktxErrorString(kres));
                }
                else
                {
                    if (ktxTexture2_NeedsTranscoding(ktex))
                    {
                        ktx_transcode_fmt_e target = (rq.key.channels == TextureKey::ChannelsHint::RG) ? KTX_TTF_BC5_RG : KTX_TTF_BC7_RGBA;
                        kres = ktxTexture2_TranscodeBasis(ktex, target, 0);
                        if (kres != KTX_SUCCESS)
                        {
                            fmt::println("[TextureCache] libktx transcode failed for '{}': {}", ktxPath.string(), ktxErrorString(kres));
                            ktxTexture_Destroy(ktxTexture(ktex));
                            ktex = nullptr;
                        }
                    }
                    if (ktex)
                    {
                        VkFormat vkfmt = static_cast<VkFormat>(ktex->vkFormat);
                        uint32_t mipLevels = ktex->numLevels;
                        uint32_t baseW = ktex->baseWidth;
                        uint32_t baseH = ktex->baseHeight;
                        ktx_size_t totalSize = ktxTexture_GetDataSize(ktxTexture(ktex));
                        const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(ktxTexture_GetData(ktxTexture(ktex)));

                        switch (vkfmt)
                        {
                            case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
                            case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
                            case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
                            case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
                            case VK_FORMAT_BC2_UNORM_BLOCK:
                            case VK_FORMAT_BC2_SRGB_BLOCK:
                            case VK_FORMAT_BC3_UNORM_BLOCK:
                            case VK_FORMAT_BC3_SRGB_BLOCK:
                            case VK_FORMAT_BC4_UNORM_BLOCK:
                            case VK_FORMAT_BC4_SNORM_BLOCK:
                            case VK_FORMAT_BC5_UNORM_BLOCK:
                            case VK_FORMAT_BC5_SNORM_BLOCK:
                            case VK_FORMAT_BC6H_UFLOAT_BLOCK:
                            case VK_FORMAT_BC6H_SFLOAT_BLOCK:
                            case VK_FORMAT_BC7_UNORM_BLOCK:
                            case VK_FORMAT_BC7_SRGB_BLOCK:
                                break;
                            default:
                                fmt::println("[TextureCache] libktx returned non-BC format {} — skipping KTX2", string_VkFormat(vkfmt));
                                ktxTexture_Destroy(ktxTexture(ktex));
                                ktex = nullptr;
                                break;
                        }

                        if (ktex)
                        {
                            out.isKTX2 = true;
                            out.ktxFormat = vkfmt;
                            out.ktxMipLevels = mipLevels;
                            out.ktx.bytes.assign(dataPtr, dataPtr + totalSize);
                            out.ktx.levels.clear();
                            out.ktx.levels.reserve(mipLevels);
                            for (uint32_t mip = 0; mip < mipLevels; ++mip)
                            {
                                ktx_size_t off = 0, len = 0;
                                ktxTexture_GetImageOffset(ktxTexture(ktex), mip, 0, 0, &off);
                                len = ktxTexture_GetImageSize(ktxTexture(ktex), mip);
                                uint32_t w = std::max(1u, baseW >> mip);
                                uint32_t h = std::max(1u, baseH >> mip);
                                out.ktx.levels.push_back({ static_cast<uint64_t>(off), static_cast<uint64_t>(len), w, h });
                            }
                            out.width = static_cast<int>(baseW);
                            out.height = static_cast<int>(baseH);
                            ktxTexture_Destroy(ktxTexture(ktex));
                        }
                    }
                }
            }
            else if (p.extension() == ".ktx2")
            {
                fmt::println("[TextureCache] Requested .ktx2 '{}' but file not found (ec={})", p.string(), ec.value());
            }
        }

        // 2) Raster fallback via stb_image if not KTX2 or unsupported
        if (!out.isKTX2)
        {
            int w = 0, h = 0, comp = 0;
            unsigned char *data = nullptr;
            if (rq.key.kind == TextureKey::SourceKind::FilePath)
            {
                data = stbi_load(rq.path.c_str(), &w, &h, &comp, 4);
            }
            else if (!rq.bytes.empty())
            {
                data = stbi_load_from_memory(rq.bytes.data(), static_cast<int>(rq.bytes.size()), &w, &h, &comp, 4);
            }

            out.width = w;
            out.height = h;
            if (data && w > 0 && h > 0)
            {
                // Progressive downscale if requested
                if (_maxUploadDimension > 0 && (w > static_cast<int>(_maxUploadDimension) || h > static_cast<int>(_maxUploadDimension)))
                {
                    std::vector<uint8_t> scaled;
                    scaled.assign(data, data + static_cast<size_t>(w) * h * 4);
                    int cw = w, ch = h;
                    while (cw > static_cast<int>(_maxUploadDimension) || ch > static_cast<int>(_maxUploadDimension))
                    {
                        auto tmp = downscale_half(scaled.data(), cw, ch, 4);
                        scaled.swap(tmp);
                        cw = std::max(1, cw / 2);
                        ch = std::max(1, ch / 2);
                    }
                    stbi_image_free(data);
                    out.rgba = std::move(scaled);
                    out.width = cw;
                    out.height = ch;
                }
                else
                {
                    out.heap = data;
                    out.heapBytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 4u;
                }
            }
            else if (data)
            {
                stbi_image_free(data);
            }
        }

        {
            std::lock_guard<std::mutex> lk(_readyMutex);
            _ready.push_back(std::move(out));
        }
    }
}

size_t TextureCache::drain_ready_uploads(ResourceManager &rm, size_t budgetBytes)
{
    std::deque<DecodedResult> local;
    {
        std::lock_guard<std::mutex> lk(_readyMutex);
        if (_ready.empty()) return 0;
        local.swap(_ready);
    }

    size_t admitted = 0;
    for (auto &res : local)
    {
        const uint32_t now = _context ? _context->frameIndex : 0u;
        if (res.handle == InvalidHandle || res.handle >= _entries.size())
        {
            if (res.heap) { stbi_image_free(res.heap); res.heap = nullptr; }
            continue;
        }

        Entry &e = _entries[res.handle];

        // Drop stale results from cancelled/unloaded requests.
        if (res.generation != e.generation || e.state != EntryState::Loading)
        {
            if (res.heap) { stbi_image_free(res.heap); res.heap = nullptr; }
            continue;
        }

        if (!res.isKTX2 && ((res.heap == nullptr && res.rgba.empty()) || res.width <= 0 || res.height <= 0))
        {
            if (res.heap) { stbi_image_free(res.heap); res.heap = nullptr; }
            e.state = EntryState::Evicted; // failed decode; keep fallback
            e.lastEvictedFrame = now;
            e.nextAttemptFrame = std::max(e.nextAttemptFrame, now + _reloadCooldownFrames);
            continue;
        }

        VkExtent3D extent{static_cast<uint32_t>(std::max(0, res.width)), static_cast<uint32_t>(std::max(0, res.height)), 1u};
        TextureKey::ChannelsHint hint = (e.key.channels == TextureKey::ChannelsHint::Auto)
                                            ? TextureKey::ChannelsHint::Auto
                                            : e.key.channels;

        size_t expectedBytes = 0;
        VkFormat fmt = VK_FORMAT_UNDEFINED;
        uint32_t desiredLevels = 1;
        if (res.isKTX2)
        {
            fmt = res.ktxFormat;
            // Nudge format to sRGB/UNORM variant based on request to avoid gamma mistakes
            VkFormat reqFmt = e.key.srgb ? to_srgb_variant(fmt) : to_unorm_variant(fmt);
            if (reqFmt != fmt)
            {
                fmt = reqFmt;
                fmt::println("[TextureCache] Overriding KTX2 format to {} based on request (original {})",
                              string_VkFormat(fmt), string_VkFormat(res.ktxFormat));
            }
            desiredLevels = res.ktxMipLevels;
            for (const auto &lv : res.ktx.levels) expectedBytes += static_cast<size_t>(lv.length);
        }
        else
        {
            fmt = choose_format(hint, res.srgb);
            if (res.mipmapped)
            {
                if (res.mipClampLevels > 0) desiredLevels = res.mipClampLevels;
                else desiredLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(extent.width, extent.height)))) + 1u;
            }
            const float mipFactor = res.mipmapped ? mip_factor_for_levels(desiredLevels) : 1.0f;
            expectedBytes = static_cast<size_t>(extent.width) * extent.height * bytes_per_texel(fmt) * mipFactor;
        }

        // Byte budget for this pump (frame)
        if (admitted + expectedBytes > budgetBytes)
        {
            // push back to be retried next frame/pump
            std::lock_guard<std::mutex> lk(_readyMutex);
            _ready.push_front(std::move(res));
            continue;
        }

        if (_gpuBudgetBytes != std::numeric_limits<size_t>::max())
        {
            if (_residentBytes + expectedBytes > _gpuBudgetBytes)
            {
                size_t need = (_residentBytes + expectedBytes) - _gpuBudgetBytes;
                (void)try_make_space(need, now);
            }
            if (_residentBytes + expectedBytes > _gpuBudgetBytes)
            {
                // Not enough space even after eviction → back off; free decode heap
                if (res.heap) { stbi_image_free(res.heap); res.heap = nullptr; }
                e.state = EntryState::Evicted;
                e.lastEvictedFrame = now;
                e.nextAttemptFrame = std::max(e.nextAttemptFrame, now + _reloadCooldownFrames);
                continue;
            }
        }

        if (res.isKTX2)
        {
            // Basic format support check: ensure the GPU can sample this format
            bool supported = true;
            if (_context && _context->getDevice())
            {
                VkFormatProperties props{};
                vkGetPhysicalDeviceFormatProperties(_context->getDevice()->physicalDevice(), fmt, &props);
                supported = (props.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT) != 0;
            }

            if (!supported)
            {
                VkFormatProperties props{};
                if (_context && _context->getDevice())
                {
                    vkGetPhysicalDeviceFormatProperties(_context->getDevice()->physicalDevice(), fmt, &props);
                }
                fmt::println("[TextureCache] Compressed format unsupported: format={} (optimalFeatures=0x{:08x}) — fallback raster for {}",
                              string_VkFormat(fmt), props.optimalTilingFeatures, e.path);
                // Fall back to raster path: requeue by synthesizing a non-KTX result
                // Attempt synchronous fallback decode from file if available.
                int fw = 0, fh = 0, comp = 0;
                unsigned char *fdata = nullptr;
                if (e.key.kind == TextureKey::SourceKind::FilePath)
                {
                    fdata = stbi_load(e.path.c_str(), &fw, &fh, &comp, 4);
                }
                if (!fdata)
                {
                    e.state = EntryState::Evicted;
                    continue;
                }
                VkExtent3D fext{ (uint32_t)fw, (uint32_t)fh, 1 };
                VkFormat ffmt = choose_format(hint, res.srgb);
                uint32_t mips = (res.mipmapped) ? static_cast<uint32_t>(std::floor(std::log2(std::max(fext.width, fext.height)))) + 1u : 1u;
                e.image = rm.create_image(fdata, fext, ffmt, VK_IMAGE_USAGE_SAMPLED_BIT, res.mipmapped, mips);
                stbi_image_free(fdata);
                e.sizeBytes = static_cast<size_t>(fext.width) * fext.height * bytes_per_texel(ffmt) * (res.mipmapped ? mip_factor_for_levels(mips) : 1.0f);
            }
            else
            {
                // Prepare level table for ResourceManager
                std::vector<ResourceManager::MipLevelCopy> levels;
                levels.reserve(res.ktx.levels.size());
                for (const auto &lv : res.ktx.levels)
                {
                    levels.push_back(ResourceManager::MipLevelCopy{ lv.offset, lv.length, lv.width, lv.height });
                }
                fmt::println("[TextureCache] upload KTX2 handle={} fmt={} levels={} size={}x{} srgb={} path='{}'",
                             res.handle,
                             string_VkFormat(fmt),
                             res.ktxMipLevels,
                             extent.width,
                             extent.height,
                             res.srgb,
                             e.path);
                e.image = rm.create_image_compressed(res.ktx.bytes.data(), res.ktx.bytes.size(), fmt, levels);
                e.sizeBytes = expectedBytes;
            }
        }
        else
        {
            // Optionally repack channels to R or RG to save memory
            std::vector<uint8_t> packed;
            const void *src = nullptr;
            if (hint == TextureKey::ChannelsHint::R)
            {
                packed.resize(static_cast<size_t>(extent.width) * extent.height);
                const uint8_t* in = res.heap ? res.heap : res.rgba.data();
                for (size_t i = 0, px = static_cast<size_t>(extent.width) * extent.height; i < px; ++i)
                {
                    packed[i] = in[i * 4 + 0];
                }
                src = packed.data();
            }
            else if (hint == TextureKey::ChannelsHint::RG)
            {
                packed.resize(static_cast<size_t>(extent.width) * extent.height * 2);
                const uint8_t* in = res.heap ? res.heap : res.rgba.data();
                for (size_t i = 0, px = static_cast<size_t>(extent.width) * extent.height; i < px; ++i)
                {
                    packed[i * 2 + 0] = in[i * 4 + 0];
                    packed[i * 2 + 1] = in[i * 4 + 1];
                }
                src = packed.data();
            }
            else
            {
                src = res.heap ? static_cast<const void *>(res.heap)
                               : static_cast<const void *>(res.rgba.data());
            }

            uint32_t mipOverride = (res.mipmapped ? desiredLevels : 1);
            fmt::println("[TextureCache] upload raster handle={} fmt={} levels={} size={}x{} srgb={} path='{}'",
                         res.handle,
                         string_VkFormat(fmt),
                         mipOverride,
                         extent.width,
                         extent.height,
                         res.srgb,
                         e.path);
            e.image = rm.create_image(src, extent, fmt, VK_IMAGE_USAGE_SAMPLED_BIT, res.mipmapped, mipOverride);
            e.sizeBytes = expectedBytes;
        }

        if (vmaDebugEnabled())
        {
            std::string name = e.key.kind == TextureKey::SourceKind::FilePath ? e.path : std::string("tex.bytes");
            vmaSetAllocationName(_context->getDevice()->allocator(), e.image.allocation, name.c_str());
        }

        _residentBytes += e.sizeBytes;
        e.state = EntryState::Resident;
        e.nextAttemptFrame = 0; // clear backoff after success

        // Drop source bytes if policy says so (only for Bytes-backed keys).
        if (!_keepSourceBytes && e.key.kind == TextureKey::SourceKind::Bytes)
        {
            drop_source_bytes(e);
        }

        // Free temporary decode heap if present
        if (res.heap)
        {
            stbi_image_free(res.heap);
        }

        // Patch descriptors now; data becomes valid before sampling due to RG upload pass
        patch_ready_entry(e);
        admitted += expectedBytes;
    }
    return admitted;
}

void TextureCache::drop_source_bytes(Entry &e)
{
    if (e.bytes.empty()) return;
    if (e.key.kind != TextureKey::SourceKind::Bytes) return;
    if (_cpuSourceBytes >= e.bytes.size()) _cpuSourceBytes -= e.bytes.size();
    e.bytes.clear();
    e.bytes.shrink_to_fit();
    e.path.clear();
}

void TextureCache::evictCpuToBudget()
{
    if (_cpuSourceBytes <= _cpuSourceBudget) return;
    // Collect candidates: Resident entries with retained bytes
    std::vector<TextureHandle> cands;
    cands.reserve(_entries.size());
    for (TextureHandle h = 0; h < _entries.size(); ++h)
    {
        const Entry &e = _entries[h];
        if (e.state == EntryState::Resident && !e.bytes.empty() && e.key.kind == TextureKey::SourceKind::Bytes)
        {
            cands.push_back(h);
        }
    }
    // LRU-ish: sort by lastUsed ascending
    std::sort(cands.begin(), cands.end(), [&](TextureHandle a, TextureHandle b){
        return _entries[a].lastUsedFrame < _entries[b].lastUsedFrame;
    });
    for (TextureHandle h : cands)
    {
        if (_cpuSourceBytes <= _cpuSourceBudget) break;
        drop_source_bytes(_entries[h]);
    }
}

bool TextureCache::try_make_space(size_t bytesNeeded, uint32_t now)
{
    if (bytesNeeded == 0) return true;
    if (_residentBytes == 0) return false;

    // Collect candidates that were not used this frame, oldest first
    std::vector<std::pair<TextureHandle, uint32_t>> order;
    order.reserve(_entries.size());
    for (TextureHandle h = 0; h < _entries.size(); ++h)
    {
        const auto &e = _entries[h];
        if (e.state == EntryState::Resident && e.lastUsedFrame != now && !e.pinned)
        {
            order.emplace_back(h, e.lastUsedFrame);
        }
    }
    std::sort(order.begin(), order.end(), [](auto &a, auto &b) { return a.second < b.second; });

    size_t freed = 0;
    for (auto &pair : order)
    {
        if (freed >= bytesNeeded) break;
        TextureHandle h = pair.first;
        Entry &e = _entries[h];
        if (e.state != EntryState::Resident) continue;
        // Never evict pinned textures
        if (e.pinned) continue;

        patch_to_fallback(e);
        fmt::println("[TextureCache] try_make_space destroy handle={} path='{}' bytes={} residentBytesBefore={}",
                     h,
                     e.path.empty() ? "<bytes>" : e.path,
                     e.sizeBytes,
                     _residentBytes);
        _context->getResources()->destroy_image(e.image);
        e.image = {};
        e.state = EntryState::Evicted;
        e.lastEvictedFrame = now;
        e.nextAttemptFrame = std::max(e.nextAttemptFrame, now + _reloadCooldownFrames);
        if (_residentBytes >= e.sizeBytes) _residentBytes -= e.sizeBytes; else _residentBytes = 0;
        freed += e.sizeBytes;
    }
    return freed >= bytesNeeded;
}

void TextureCache::debug_snapshot(std::vector<DebugRow> &outRows, DebugStats &outStats) const
{
    outRows.clear();
    outStats = DebugStats{};
    outStats.residentBytes = _residentBytes;

    auto stateToByteable = [&](const Entry &e) -> bool { return e.state == EntryState::Resident; };

    for (const auto &e : _entries)
    {
        switch (e.state)
        {
            case EntryState::Resident: outStats.countResident++; break;
            case EntryState::Evicted:  outStats.countEvicted++;  break;
            case EntryState::Unloaded: outStats.countUnloaded++; break;
            case EntryState::Loading: /* ignore */ break;
        }

        DebugRow row{};
        if (e.key.kind == TextureKey::SourceKind::FilePath)
        {
            row.name = e.path.empty() ? std::string("<path>") : e.path;
        }
        else
        {
            row.name = std::string("<bytes> (") + std::to_string(e.bytes.size()) + ")";
        }
        if (e.state == EntryState::Resident && e.image.image)
        {
            row.name += std::string(" [") + string_VkFormat(e.image.imageFormat) + "]";
        }
        row.bytes = e.sizeBytes;
        row.lastUsed = e.lastUsedFrame;
        row.state = static_cast<uint8_t>(e.state);
        outRows.push_back(std::move(row));
    }
    std::sort(outRows.begin(), outRows.end(), [](const DebugRow &a, const DebugRow &b) {
        return a.bytes > b.bytes;
    });
}

TextureCache::EntryState TextureCache::state(TextureHandle handle) const
{
    if (handle == InvalidHandle) return EntryState::Unloaded;
    if (handle >= _entries.size()) return EntryState::Unloaded;
    return _entries[handle].state;
}

VkImageView TextureCache::image_view(TextureHandle handle) const
{
    if (handle == InvalidHandle) return VK_NULL_HANDLE;
    if (handle >= _entries.size()) return VK_NULL_HANDLE;

    const Entry &e = _entries[handle];
    if (e.state != EntryState::Resident) return VK_NULL_HANDLE;
    return e.image.imageView;
}
