#pragma once

#include <core/types.h>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <limits>

class EngineContext;
class ResourceManager;
struct FrameResources;

// Lightweight texture streaming cache.
// - Requests are deduplicated by a hashable TextureKey.
// - Loads happen via ResourceManager (deferred uploads supported).
// - Descriptors registered via watchBinding() are patched in-place
//   when the image becomes Resident, leveraging UPDATE_AFTER_BIND.
// - evictToBudget() rewrites watchers to provided fallbacks.
class TextureCache
{
public:
    struct TextureKey
    {
        enum class SourceKind : uint8_t { FilePath, Bytes };
        enum class ChannelsHint : uint8_t { Auto, R, RG, RGBA };
        SourceKind kind{SourceKind::FilePath};
        std::string path;                 // used when kind==FilePath
        std::vector<uint8_t> bytes;       // used when kind==Bytes
        bool srgb{false};                 // desired sampling format
        bool mipmapped{true};             // generate full mip chain
        ChannelsHint channels{ChannelsHint::Auto}; // prefer narrower formats when possible
        uint32_t mipClampLevels{0};       // 0 = full chain, otherwise limit to N mips
        uint64_t hash{0};                 // stable dedup key
    };

    using TextureHandle = uint32_t;
    static constexpr TextureHandle InvalidHandle = 0xFFFFFFFFu;

    enum class EntryState : uint8_t { Unloaded = 0, Loading = 1, Resident = 2, Evicted = 3 };

    void init(EngineContext *ctx);
    void cleanup();

    // Deduplicated request; returns a stable handle.
    TextureHandle request(const TextureKey &key, VkSampler sampler);

    // Register a descriptor binding to patch when the texture is ready.
    void watchBinding(TextureHandle handle, VkDescriptorSet set, uint32_t binding,
                      VkSampler sampler, VkImageView fallbackView);

    // Remove all watches for a descriptor set (call before destroying the
    // pool that owns the set). Prevents attempts to patch dead sets.
    void unwatchSet(VkDescriptorSet set);

    // Mark a texture as used this frame (for LRU).
    void markUsed(TextureHandle handle, uint32_t frameIndex);
    // Convenience: mark all handles watched by a descriptor set.
    void markSetUsed(VkDescriptorSet set, uint32_t frameIndex);

    // Pin a texture to prevent eviction (useful for UI elements, critical assets).
    // Pinned textures are never evicted by LRU or budget constraints.
    void pin(TextureHandle handle);
    // Unpin a texture, allowing it to be evicted normally.
    void unpin(TextureHandle handle);
    // Check if a texture is currently pinned.
    bool is_pinned(TextureHandle handle) const;

    // Schedule pending loads and patch descriptors for newly created images.
    void pumpLoads(ResourceManager &rm, FrameResources &frame);

    // Evict least-recently-used entries to fit within a budget in bytes.
    void evictToBudget(size_t budgetBytes);

    // Manually unload a texture. This immediately frees GPU memory (if resident),
    // patches watched descriptor bindings back to their fallbacks, and cancels any
    // in-flight decode/upload work for the handle. The handle remains valid and
    // can be re-requested/reloaded later.
    bool unload(TextureHandle handle, bool drop_source_bytes = true);

    // Debug snapshot for UI
    struct DebugRow
    {
        std::string name;
        size_t bytes{0};
        uint32_t lastUsed{0};
        uint8_t state{0}; // cast of EntryState
    };
    struct DebugStats
    {
        size_t residentBytes{0};
        size_t countResident{0};
        size_t countEvicted{0};
        size_t countUnloaded{0};
    };
    void debug_snapshot(std::vector<DebugRow>& outRows, DebugStats& outStats) const;
    // Read-only per-handle state query (main-thread only).
    EntryState state(TextureHandle handle) const;
    // Returns the default image view for a Resident texture, otherwise VK_NULL_HANDLE.
    VkImageView image_view(TextureHandle handle) const;
    size_t resident_bytes() const { return _residentBytes; }
    // CPU-side source bytes currently retained (compressed image payloads kept
    // for potential re-decode). Only applies to entries created with Bytes keys.
    size_t cpu_source_bytes() const { return _cpuSourceBytes; }

    // Runtime controls
    void set_max_loads_per_pump(int n) { _maxLoadsPerPump = (n > 0) ? n : 1; }
    int  max_loads_per_pump() const { return _maxLoadsPerPump; }
    // Limit total bytes admitted for uploads per pump (frame).
    void set_max_bytes_per_pump(size_t bytes) { _maxBytesPerPump = bytes; }
    size_t max_bytes_per_pump() const { return _maxBytesPerPump; }
    // Clamp decoded image dimensions before upload (progressive resolution).
    // 0 disables clamping. When >0, images larger than this dimension on any axis
    // are downscaled by powers of 2 on the decode thread until within limit.
    void set_max_upload_dimension(uint32_t dim) { _maxUploadDimension = dim; }
    uint32_t max_upload_dimension() const { return _maxUploadDimension; }

    // If false (default), compressed source bytes are dropped once an image is
    // uploaded to the GPU and descriptors patched. Set true to retain sources
    // for potential re-decode after eviction.
    void set_keep_source_bytes(bool keep) { _keepSourceBytes = keep; }
    bool keep_source_bytes() const { return _keepSourceBytes; }

    // Set a soft CPU budget (in bytes) for retained compressed sources. After
    // each upload drain, the cache will try to free source bytes for Resident
    // entries until under budget.
    void set_cpu_source_budget(size_t bytes) { _cpuSourceBudget = bytes; }
    size_t cpu_source_budget() const { return _cpuSourceBudget; }

    // Optional GPU residency budget, used to avoid immediate thrashing when
    // accepting new uploads. The engine should refresh this each frame.
    void set_gpu_budget_bytes(size_t bytes) { _gpuBudgetBytes = bytes; }
    size_t gpu_budget_bytes() const { return _gpuBudgetBytes; }

private:
    struct Patch
    {
        VkDescriptorSet set{VK_NULL_HANDLE};
        uint32_t binding{0};
        VkSampler sampler{VK_NULL_HANDLE};
        VkImageView fallbackView{VK_NULL_HANDLE};
    };

    struct Entry
    {
        TextureKey key{};
        VkSampler sampler{VK_NULL_HANDLE};
        EntryState state{EntryState::Unloaded};
        uint32_t generation{1};    // bumps to invalidate in-flight decode results
        bool pinned{false};        // if true, never evict (for UI, critical assets)
        AllocatedImage image{};     // valid when Resident
        size_t sizeBytes{0};        // approximate VRAM cost
        uint32_t lastUsedFrame{0};
        uint32_t lastEvictedFrame{0};
        uint32_t nextAttemptFrame{0}; // gate reload attempts to reduce churn
        std::vector<Patch> patches; // descriptor patches to rewrite

        // Source payload for deferred load
        std::string path;                // for FilePath
        std::vector<uint8_t> bytes;      // for Bytes
    };

    EngineContext *_context{nullptr};
    std::vector<Entry> _entries;
    std::unordered_map<uint64_t, TextureHandle> _lookup; // key.hash -> handle
    std::unordered_map<VkDescriptorSet, std::vector<TextureHandle>> _setToHandles;
    size_t _residentBytes{0};
    size_t _cpuSourceBytes{0};

    // Controls
    int _maxLoadsPerPump{4};
    bool _keepSourceBytes{false};
    size_t _cpuSourceBudget{64ull * 1024ull * 1024ull}; // 64 MiB default
    size_t _gpuBudgetBytes{std::numeric_limits<size_t>::max()}; // unlimited unless set
    uint32_t _reloadCooldownFrames{2};
    size_t _maxBytesPerPump{128ull * 1024ull * 1024ull}; // 128 MiB/frame upload budget
    uint32_t _maxUploadDimension{4096};                   // progressive downscale cap

    void start_load(Entry &e, ResourceManager &rm);
    void patch_ready_entry(const Entry &e);
    void patch_to_fallback(const Entry &e);

    // --- Async decode backend ---
    struct DecodeRequest
    {
        TextureHandle handle{InvalidHandle};
        uint32_t generation{0};
        TextureKey key{};
        std::string path;
        std::vector<uint8_t> bytes;
    };
    struct DecodedResult
    {
        TextureHandle handle{InvalidHandle};
        uint32_t generation{0};
        int width{0};
        int height{0};
        // Prefer heap pointer from stb to avoid an extra memcpy into a vector.
        // If 'heap' is non-null, it must be freed with stbi_image_free() after
        // the upload has copied the data. 'rgba' remains as a fallback path.
        unsigned char *heap{nullptr};
        size_t heapBytes{0};
        std::vector<uint8_t> rgba;
        bool mipmapped{true};
        bool srgb{false};
        TextureKey::ChannelsHint channels{TextureKey::ChannelsHint::Auto};
        uint32_t mipClampLevels{0};

        // Compressed path (KTX2 pre-transcoded BCn). When true, 'rgba/heap'
        // are ignored and the fields below describe the payload.
        bool isKTX2{false};
        VkFormat ktxFormat{VK_FORMAT_UNDEFINED};
        uint32_t ktxMipLevels{0};
        struct KTXPack {
            struct L { uint64_t offset{0}, length{0}; uint32_t width{0}, height{0}; };
            std::vector<uint8_t> bytes;     // full file content
            std::vector<L> levels;           // per-mip region description
        } ktx;
    };

    void worker_loop();
    void enqueue_decode(Entry &e);
    // Returns total resident bytes admitted this pump (after GPU budget gate).
    size_t drain_ready_uploads(ResourceManager &rm, size_t budgetBytes);
    void drop_source_bytes(Entry &e);
    void evictCpuToBudget();

    // Try to free at least 'bytesNeeded' by evicting least-recently-used
    // Resident entries that were not used in the current frame. Returns true
    // if enough space was reclaimed. Does not evict textures used this frame.
    bool try_make_space(size_t bytesNeeded, uint32_t now);

    std::vector<std::thread> _decodeThreads;
    std::mutex _qMutex;
    std::condition_variable _qCV;
    std::deque<DecodeRequest> _queue;
    std::mutex _readyMutex;
    std::deque<DecodedResult> _ready;
    std::atomic<bool> _running{false};
};

// Helpers to build/digest keys
namespace texcache
{
    // 64-bit FNV-1a
    inline uint64_t fnv1a64(std::string_view s)
    {
        const uint64_t FNV_OFFSET = 1469598103934665603ull;
        const uint64_t FNV_PRIME  = 1099511628211ull;
        uint64_t h = FNV_OFFSET;
        for (unsigned char c : s) { h ^= c; h *= FNV_PRIME; }
        return h;
    }
    inline uint64_t fnv1a64(const uint8_t *data, size_t n)
    {
        const uint64_t FNV_OFFSET = 1469598103934665603ull;
        const uint64_t FNV_PRIME  = 1099511628211ull;
        uint64_t h = FNV_OFFSET;
        for (size_t i = 0; i < n; ++i) { h ^= data[i]; h *= FNV_PRIME; }
        return h;
    }
}
