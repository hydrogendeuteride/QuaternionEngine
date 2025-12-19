#pragma once

#include <core/types.h>
#include <render/graph/types.h>
#include <string>
#include <vector>
#include <unordered_map>

class EngineContext;

struct RGImageRecord
{
    std::string name;
    bool imported = true;

    // Unified view for either imported or transient
    VkImage image = VK_NULL_HANDLE;
    VkImageView imageView = VK_NULL_HANDLE;
    VkFormat format = VK_FORMAT_UNDEFINED;
    VkExtent2D extent{0, 0};
    VkImageLayout initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	VkPipelineStageFlags2 initialStage = VK_PIPELINE_STAGE_2_NONE;
	VkAccessFlags2 initialAccess = 0;
    VkImageUsageFlags creationUsage = 0; // if transient; 0 for imported

    // If transient, keep allocation owner for cleanup
    AllocatedImage allocation{};

    // Lifetime indices within the compiled pass list (for aliasing/debug)
    int firstUse = -1;
    int lastUse = -1;
};

struct RGBufferRecord
{
    std::string name;
    bool imported = true;

	VkBuffer buffer = VK_NULL_HANDLE;
	VkDeviceSize size = 0;
	VkBufferUsageFlags usage = 0;
	VkPipelineStageFlags2 initialStage = VK_PIPELINE_STAGE_2_NONE;
    VkAccessFlags2 initialAccess = 0;

    AllocatedBuffer allocation{};

    // Lifetime indices (for aliasing/debug)
    int firstUse = -1;
    int lastUse = -1;
};

class RGResourceRegistry
{
public:
    void init(EngineContext* ctx) { _ctx = ctx; }

    void reset();

    RGImageHandle add_imported(const RGImportedImageDesc& d);
    RGImageHandle add_transient(const RGImageDesc& d);

    RGBufferHandle add_imported(const RGImportedBufferDesc& d);
    RGBufferHandle add_transient(const RGBufferDesc& d);

    // Lookup existing handles by raw Vulkan objects (deduplicates imports)
    RGBufferHandle find_buffer(VkBuffer buffer) const;
    RGImageHandle find_image(VkImage image) const;

    const RGImageRecord* get_image(RGImageHandle h) const;
    RGImageRecord* get_image(RGImageHandle h);

	const RGBufferRecord* get_buffer(RGBufferHandle h) const;
	RGBufferRecord* get_buffer(RGBufferHandle h);

	size_t image_count() const { return _images.size(); }
	size_t buffer_count() const { return _buffers.size(); }

	VkImageLayout initial_layout(RGImageHandle h) const;
	VkFormat image_format(RGImageHandle h) const;
	VkPipelineStageFlags2 initial_stage(RGImageHandle h) const;
	VkAccessFlags2 initial_access(RGImageHandle h) const;

	VkPipelineStageFlags2 initial_stage(RGBufferHandle h) const;
	VkAccessFlags2 initial_access(RGBufferHandle h) const;

private:
    EngineContext* _ctx = nullptr;
    std::vector<RGImageRecord> _images;
    std::vector<RGBufferRecord> _buffers;

    // Reverse lookup to avoid duplicate imports of the same VkBuffer/VkImage
    std::unordered_map<VkImage, uint32_t> _imageLookup;
    std::unordered_map<VkBuffer, uint32_t> _bufferLookup;
};
