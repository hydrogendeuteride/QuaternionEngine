#include "resources.h"
#include <core/context.h>
#include <core/device/resource.h>
#include <vk_mem_alloc.h>
#include <core/config.h>

#include "core/frame/resources.h"
#include "core/device/device.h"

void RGResourceRegistry::reset()
{
    _images.clear();
    _buffers.clear();
    _imageLookup.clear();
    _bufferLookup.clear();
}

RGImageHandle RGResourceRegistry::add_imported(const RGImportedImageDesc& d)
{
    // Deduplicate by VkImage
    auto it = _imageLookup.find(d.image);
    if (it != _imageLookup.end())
    {
        auto& rec = _images[it->second];
        rec.name = d.name;
        rec.image = d.image;
        rec.imageView = d.imageView;
        rec.format = d.format;
        rec.extent = d.extent;
        rec.initialLayout = d.currentLayout;
		// Keep the earliest known stage/access if set; otherwise record provided
		if (rec.initialStage == VK_PIPELINE_STAGE_2_NONE) rec.initialStage = d.currentStage;
		if (rec.initialAccess == 0) rec.initialAccess = d.currentAccess;
        return RGImageHandle{it->second};
    }

    RGImageRecord rec{};
    rec.name = d.name;
    rec.imported = true;
    rec.image = d.image;
    rec.imageView = d.imageView;
    rec.format = d.format;
    rec.extent = d.extent;
    rec.initialLayout = d.currentLayout;
	rec.initialStage = d.currentStage;
	rec.initialAccess = d.currentAccess;
    _images.push_back(rec);
    uint32_t id = static_cast<uint32_t>(_images.size() - 1);
    if (d.image != VK_NULL_HANDLE) _imageLookup[d.image] = id;
    return RGImageHandle{ id };
}

RGImageHandle RGResourceRegistry::add_transient(const RGImageDesc& d)
{
    RGImageRecord rec{};
    rec.name = d.name;
    rec.imported = false;
    rec.format = d.format;
    rec.extent = d.extent;
    rec.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	rec.initialStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
	rec.initialAccess = 0;
    rec.creationUsage = d.usage;

	VkExtent3D size{ d.extent.width, d.extent.height, 1 };
    rec.allocation = _ctx->getResources()->create_image(size, d.format, d.usage);
    // Name the allocation for diagnostics (optional)
    if (vmaDebugEnabled() && _ctx && _ctx->getDevice())
    {
        std::string nm = std::string("rg.image:") + d.name;
        vmaSetAllocationName(_ctx->getDevice()->allocator(), rec.allocation.allocation, nm.c_str());
    }
	rec.image = rec.allocation.image;
	rec.imageView = rec.allocation.imageView;

	// Cleanup at end of frame
	if (_ctx && _ctx->currentFrame)
	{
		auto img = rec.allocation;
		_ctx->currentFrame->_deletionQueue.push_function([ctx=_ctx, img]() {
			ctx->getResources()->destroy_image(img);
		});
	}

	_images.push_back(rec);
	return RGImageHandle{ static_cast<uint32_t>(_images.size() - 1) };
}

RGBufferHandle RGResourceRegistry::add_imported(const RGImportedBufferDesc& d)
{
    // Deduplicate by VkBuffer
    auto it = _bufferLookup.find(d.buffer);
    if (it != _bufferLookup.end())
    {
        auto& rec = _buffers[it->second];
        rec.name = d.name;
        rec.buffer = d.buffer;
        rec.size = d.size;
        // Keep the earliest known stage/access if set; otherwise record provided
        if (rec.initialStage == VK_PIPELINE_STAGE_2_NONE) rec.initialStage = d.currentStage;
        if (rec.initialAccess == 0) rec.initialAccess = d.currentAccess;
        return RGBufferHandle{it->second};
    }

    RGBufferRecord rec{};
    rec.name = d.name;
    rec.imported = true;
    rec.buffer = d.buffer;
    rec.size = d.size;
    rec.initialStage = d.currentStage;
    rec.initialAccess = d.currentAccess;
    _buffers.push_back(rec);
    uint32_t id = static_cast<uint32_t>(_buffers.size() - 1);
    if (d.buffer != VK_NULL_HANDLE) _bufferLookup[d.buffer] = id;
    return RGBufferHandle{ id };
}

RGBufferHandle RGResourceRegistry::add_transient(const RGBufferDesc& d)
{
	RGBufferRecord rec{};
	rec.name = d.name;
	rec.imported = false;
	rec.size = d.size;
	rec.usage = d.usage;
	rec.initialStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
	rec.initialAccess = 0;

	rec.allocation = _ctx->getResources()->create_buffer(d.size, d.usage, d.memoryUsage);
	rec.buffer = rec.allocation.buffer;

	if (_ctx && _ctx->currentFrame)
	{
		auto buf = rec.allocation;
		_ctx->currentFrame->_deletionQueue.push_function([ctx=_ctx, buf]() {
			ctx->getResources()->destroy_buffer(buf);
		});
	}

    _buffers.push_back(rec);
    uint32_t id = static_cast<uint32_t>(_buffers.size() - 1);
    if (rec.buffer != VK_NULL_HANDLE) _bufferLookup[rec.buffer] = id;
    return RGBufferHandle{ id };
}

const RGImageRecord* RGResourceRegistry::get_image(RGImageHandle h) const
{
    if (!h.valid() || h.id >= _images.size()) return nullptr;
    return &_images[h.id];
}

RGImageRecord* RGResourceRegistry::get_image(RGImageHandle h)
{
	if (!h.valid() || h.id >= _images.size()) return nullptr;
	return &_images[h.id];
}

const RGBufferRecord* RGResourceRegistry::get_buffer(RGBufferHandle h) const
{
	if (!h.valid() || h.id >= _buffers.size()) return nullptr;
	return &_buffers[h.id];
}

RGBufferRecord* RGResourceRegistry::get_buffer(RGBufferHandle h)
{
	if (!h.valid() || h.id >= _buffers.size()) return nullptr;
	return &_buffers[h.id];
}

VkImageLayout RGResourceRegistry::initial_layout(RGImageHandle h) const
{
	const RGImageRecord* rec = get_image(h);
	return rec ? rec->initialLayout : VK_IMAGE_LAYOUT_UNDEFINED;
}

VkFormat RGResourceRegistry::image_format(RGImageHandle h) const
{
    const RGImageRecord* rec = get_image(h);
    return rec ? rec->format : VK_FORMAT_UNDEFINED;
}

VkPipelineStageFlags2 RGResourceRegistry::initial_stage(RGImageHandle h) const
{
	const RGImageRecord* rec = get_image(h);
	return rec ? rec->initialStage : VK_PIPELINE_STAGE_2_NONE;
}

VkAccessFlags2 RGResourceRegistry::initial_access(RGImageHandle h) const
{
	const RGImageRecord* rec = get_image(h);
	return rec ? rec->initialAccess : VkAccessFlags2{0};
}

VkPipelineStageFlags2 RGResourceRegistry::initial_stage(RGBufferHandle h) const
{
	const RGBufferRecord* rec = get_buffer(h);
	return rec ? rec->initialStage : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
}

VkAccessFlags2 RGResourceRegistry::initial_access(RGBufferHandle h) const
{
    const RGBufferRecord* rec = get_buffer(h);
    return rec ? rec->initialAccess : VkAccessFlags2{0};
}

RGBufferHandle RGResourceRegistry::find_buffer(VkBuffer buffer) const
{
    auto it = _bufferLookup.find(buffer);
    if (it == _bufferLookup.end()) return RGBufferHandle{};
    return RGBufferHandle{it->second};
}

RGImageHandle RGResourceRegistry::find_image(VkImage image) const
{
    auto it = _imageLookup.find(image);
    if (it == _imageLookup.end()) return RGImageHandle{};
    return RGImageHandle{it->second};
}
