#include "texture_cache.h"

#include <core/engine_context.h>
#include <core/vk_resource.h>
#include <core/vk_descriptors.h>
#include <core/config.h>
#include <algorithm>
#include "stb_image.h"
#include <algorithm>
#include "vk_device.h"

void TextureCache::init(EngineContext *ctx)
{
    _context = ctx;
}

void TextureCache::cleanup()
{
    if (!_context || !_context->getResources()) return;
    auto *rm = _context->getResources();
    for (auto &e : _entries)
    {
        if (e.state == EntryState::Resident && e.image.image)
        {
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
    auto it = _lookup.find(key.hash);
    if (it != _lookup.end())
    {
        TextureHandle h = it->second;
        // Keep most recent sampler for future patches if provided
        if (h < _entries.size() && sampler != VK_NULL_HANDLE)
        {
            _entries[h].sampler = sampler;
        }
        return h;
    }

    TextureHandle h = static_cast<TextureHandle>(_entries.size());
    _lookup.emplace(key.hash, h);

    Entry e{};
    e.key = key;
    e.sampler = sampler;
    e.state = EntryState::Unloaded;
    if (key.kind == TextureKey::SourceKind::FilePath)
    {
        e.path = key.path;
    }
    else
    {
        e.bytes = key.bytes;
    }
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

static inline size_t estimate_rgba8_bytes(uint32_t w, uint32_t h)
{
    return static_cast<size_t>(w) * static_cast<size_t>(h) * 4u;
}

void TextureCache::start_load(Entry &e, ResourceManager &rm)
{
    if (e.state == EntryState::Resident || e.state == EntryState::Loading) return;

    int width = 0, height = 0, comp = 0;
    unsigned char *data = nullptr;

    if (e.key.kind == TextureKey::SourceKind::FilePath)
    {
        data = stbi_load(e.path.c_str(), &width, &height, &comp, 4);
    }
    else
    {
        if (!e.bytes.empty())
        {
            data = stbi_load_from_memory(e.bytes.data(), static_cast<int>(e.bytes.size()), &width, &height, &comp, 4);
        }
    }

    if (!data || width <= 0 || height <= 0)
    {
        // Failed decode; keep fallbacks bound. Mark as evicted/unloaded.
        if (data) stbi_image_free(data);
        e.state = EntryState::Evicted;
        return;
    }

    VkExtent3D extent{static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1u};
    VkFormat fmt = e.key.srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;

    // Queue upload via ResourceManager (deferred pass if enabled)
    e.image = rm.create_image(static_cast<void *>(data), extent, fmt, VK_IMAGE_USAGE_SAMPLED_BIT, e.key.mipmapped);

    // Name VMA allocation for diagnostics
    if (vmaDebugEnabled())
    {
        std::string name = e.key.kind == TextureKey::SourceKind::FilePath ? e.path : std::string("tex.bytes");
        vmaSetAllocationName(_context->getDevice()->allocator(), e.image.allocation, name.c_str());
    }

    const float mipFactor = e.key.mipmapped ? 1.3333333f : 1.0f; // approx sum of 1/4^i
    e.sizeBytes = static_cast<size_t>(estimate_rgba8_bytes(extent.width, extent.height) * mipFactor);
    _residentBytes += e.sizeBytes;
    e.state = EntryState::Resident;

    stbi_image_free(data);

    // Patch all watched descriptors to the new image
    patch_ready_entry(e);
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
    const int kMaxLoadsPerPump = 4;
    int started = 0;
    for (auto &e : _entries)
    {
        if (e.state == EntryState::Unloaded)
        {
            start_load(e, rm);
            if (++started >= kMaxLoadsPerPump) break;
        }
    }
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
        if (e.state == EntryState::Resident)
        {
            order.emplace_back(h, e.lastUsedFrame);
        }
    }
    std::sort(order.begin(), order.end(), [](auto &a, auto &b) { return a.second < b.second; });

    for (auto &pair : order)
    {
        if (_residentBytes <= budgetBytes) break;
        TextureHandle h = pair.first;
        Entry &e = _entries[h];
        if (e.state != EntryState::Resident) continue;

        // Rewrite watchers back to fallback before destroying
        patch_to_fallback(e);

        _context->getResources()->destroy_image(e.image);
        e.image = {};
        e.state = EntryState::Evicted;
        if (_residentBytes >= e.sizeBytes) _residentBytes -= e.sizeBytes; else _residentBytes = 0;
    }
}
