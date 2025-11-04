#pragma once

#include <core/vk_types.h>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

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
        SourceKind kind{SourceKind::FilePath};
        std::string path;                 // used when kind==FilePath
        std::vector<uint8_t> bytes;       // used when kind==Bytes
        bool srgb{false};                 // desired sampling format
        bool mipmapped{true};             // generate full mip chain
        uint64_t hash{0};                 // stable dedup key
    };

    using TextureHandle = uint32_t;
    static constexpr TextureHandle InvalidHandle = 0xFFFFFFFFu;

    void init(EngineContext *ctx);
    void cleanup();

    // Deduplicated request; returns a stable handle.
    TextureHandle request(const TextureKey &key, VkSampler sampler);

    // Register a descriptor binding to patch when the texture is ready.
    void watchBinding(TextureHandle handle, VkDescriptorSet set, uint32_t binding,
                      VkSampler sampler, VkImageView fallbackView);

    // Mark a texture as used this frame (for LRU).
    void markUsed(TextureHandle handle, uint32_t frameIndex);
    // Convenience: mark all handles watched by a descriptor set.
    void markSetUsed(VkDescriptorSet set, uint32_t frameIndex);

    // Schedule pending loads and patch descriptors for newly created images.
    void pumpLoads(ResourceManager &rm, FrameResources &frame);

    // Evict least-recently-used entries to fit within a budget in bytes.
    void evictToBudget(size_t budgetBytes);

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
    size_t resident_bytes() const { return _residentBytes; }

private:
    struct Patch
    {
        VkDescriptorSet set{VK_NULL_HANDLE};
        uint32_t binding{0};
        VkSampler sampler{VK_NULL_HANDLE};
        VkImageView fallbackView{VK_NULL_HANDLE};
    };

    enum class EntryState : uint8_t { Unloaded, Loading, Resident, Evicted };

    struct Entry
    {
        TextureKey key{};
        VkSampler sampler{VK_NULL_HANDLE};
        EntryState state{EntryState::Unloaded};
        AllocatedImage image{};     // valid when Resident
        size_t sizeBytes{0};        // approximate VRAM cost
        uint32_t lastUsedFrame{0};
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

    void start_load(Entry &e, ResourceManager &rm);
    void patch_ready_entry(const Entry &e);
    void patch_to_fallback(const Entry &e);

    // --- Async decode backend ---
    struct DecodeRequest
    {
        TextureHandle handle{InvalidHandle};
        TextureKey key{};
        std::string path;
        std::vector<uint8_t> bytes;
    };
    struct DecodedResult
    {
        TextureHandle handle{InvalidHandle};
        int width{0};
        int height{0};
        std::vector<uint8_t> rgba;
        bool mipmapped{true};
        bool srgb{false};
    };

    void worker_loop();
    void enqueue_decode(Entry &e);
    void drain_ready_uploads(ResourceManager &rm);

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
