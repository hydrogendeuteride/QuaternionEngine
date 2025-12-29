#pragma once
#include <core/types.h>
#include <functional>
#include <vector>
#include <mutex>

class DeviceManager;
class RenderGraph;
struct FrameResources;

// VMA-backed allocator + upload helper.
// Creates buffers/images, offers an immediate-submit path, and supports
// deferring uploads into a single Render Graph transfer pass per frame.
class ResourceManager
{
public:
    struct MipLevelCopy
    {
        uint64_t offset{0};
        uint64_t length{0};
        uint32_t width{0};
        uint32_t height{0};
    };
    struct BufferCopyRegion
    {
        VkBuffer destination = VK_NULL_HANDLE;
        VkDeviceSize dstOffset = 0;
        VkDeviceSize size = 0;
        VkDeviceSize stagingOffset = 0;
    };

    struct PendingBufferUpload
    {
        AllocatedBuffer staging;
        std::vector<BufferCopyRegion> copies;
    };

    struct PendingImageUpload
    {
        AllocatedBuffer staging;
        VkImage image = VK_NULL_HANDLE;
        VkExtent3D extent{0, 0, 0};
        VkFormat format = VK_FORMAT_UNDEFINED;
        VkImageLayout initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VkImageLayout finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        bool generateMips = false;
        uint32_t mipLevels = 1;
        // For multi-region (per-mip) uploads
        std::vector<VkBufferImageCopy> copies;
    };

    void init(DeviceManager *deviceManager);

    void cleanup();

    AllocatedBuffer create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage) const;

    void destroy_buffer(const AllocatedBuffer &buffer) const;

    AllocatedImage create_image(VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                bool mipmapped = false) const;
    // Variant with explicit mip level count (>=1). If 0, computes full chain when mipmapped.
    AllocatedImage create_image(VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                bool mipmapped, uint32_t mipLevelsOverride) const;

    AllocatedImage create_image(const void *data, VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                bool mipmapped = false);
    // Variant with explicit mip level count used for generate_mipmaps.
    AllocatedImage create_image(const void *data, VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                bool mipmapped, uint32_t mipLevelsOverride);

    // Create an image from a compressed payload (e.g., KTX2 pre-transcoded BCn).
    // 'bytes' backs a single staging buffer; 'levels' provides per-mip copy regions.
    // No GPU mip generation is performed; the number of mips equals levels.size().
    AllocatedImage create_image_compressed(const void* bytes, size_t size,
                                           VkFormat fmt,
                                           std::span<const MipLevelCopy> levels,
                                           VkImageUsageFlags usage = VK_IMAGE_USAGE_SAMPLED_BIT);

    // Create a layered image (2D array or cubemap) from a compressed payload.
    // - 'bytes' is the full KTX2 data payload staged into one buffer
    // - 'regions' lists VkBufferImageCopy entries (one per mip × layer)
    // - 'mipLevels' and 'layerCount' define the image subresource counts
    // - for cubemaps, pass flags |= VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT and layerCount=6
    AllocatedImage create_image_compressed_layers(const void* bytes, size_t size,
                                                  VkFormat fmt,
                                                  uint32_t mipLevels,
                                                  uint32_t layerCount,
                                                  std::span<const VkBufferImageCopy> regions,
                                                  VkImageUsageFlags usage = VK_IMAGE_USAGE_SAMPLED_BIT,
                                                  VkImageCreateFlags flags = 0);

    void destroy_image(const AllocatedImage &img) const;

    GPUMeshBuffers uploadMesh(std::span<uint32_t> indices, std::span<Vertex> vertices);

    // Upload raw bytes into a GPU buffer. The destination buffer is created with the provided 'usage'
    // flags plus VK_BUFFER_USAGE_TRANSFER_DST_BIT. Staging is handled internally and freed via the
    // per-frame deletion queue when deferred uploads are enabled.
    AllocatedBuffer upload_buffer(const void *data, size_t size, VkBufferUsageFlags usage,
                                  VmaMemoryUsage memoryUsage = VMA_MEMORY_USAGE_GPU_ONLY);

    void immediate_submit(std::function<void(VkCommandBuffer)> &&function) const;

    bool has_pending_uploads() const;
    void clear_pending_uploads();
    void process_queued_uploads_immediate();

    void register_upload_pass(RenderGraph &graph, FrameResources &frame);

    void set_deferred_uploads(bool enabled) { _deferUploads = enabled; }
    bool deferred_uploads() const { return _deferUploads; }

private:
    DeviceManager *_deviceManager = nullptr;

    // immediate submit structures
    VkFence _immFence = nullptr;
    VkCommandBuffer _immCommandBuffer = nullptr;
    VkCommandPool _immCommandPool = nullptr;

    std::vector<PendingBufferUpload> _pendingBufferUploads;
    std::vector<PendingImageUpload> _pendingImageUploads;
    bool _deferUploads = false;

    DeletionQueue _deletionQueue;

    mutable std::mutex _pendingMutex;
};
