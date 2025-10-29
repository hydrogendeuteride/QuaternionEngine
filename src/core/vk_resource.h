#pragma once
#include <core/vk_types.h>
#include <functional>
#include <vector>

class DeviceManager;
class RenderGraph;
struct FrameResources;

// VMA-backed allocator + upload helper.
// Creates buffers/images, offers an immediate-submit path, and supports
// deferring uploads into a single Render Graph transfer pass per frame.
class ResourceManager
{
public:
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
    };

    void init(DeviceManager *deviceManager);

    void cleanup();

    AllocatedBuffer create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage) const;

    void destroy_buffer(const AllocatedBuffer &buffer) const;

    AllocatedImage create_image(VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                bool mipmapped = false) const;

    AllocatedImage create_image(const void *data, VkExtent3D size, VkFormat format, VkImageUsageFlags usage,
                                bool mipmapped = false);

    void destroy_image(const AllocatedImage &img) const;

    GPUMeshBuffers uploadMesh(std::span<uint32_t> indices, std::span<Vertex> vertices);

    void immediate_submit(std::function<void(VkCommandBuffer)> &&function) const;

    bool has_pending_uploads() const;
    const std::vector<PendingBufferUpload> &pending_buffer_uploads() const { return _pendingBufferUploads; }
    const std::vector<PendingImageUpload> &pending_image_uploads() const { return _pendingImageUploads; }
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
};
