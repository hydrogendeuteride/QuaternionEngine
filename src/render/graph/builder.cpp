#include "builder.h"
#include "resources.h"

// ---- RGPassResources ----
VkImage RGPassResources::image(RGImageHandle h) const
{
	const RGImageRecord *rec = _registry ? _registry->get_image(h) : nullptr;
	return rec ? rec->image : VK_NULL_HANDLE;
}

VkImageView RGPassResources::image_view(RGImageHandle h) const
{
	const RGImageRecord *rec = _registry ? _registry->get_image(h) : nullptr;
	return rec ? rec->imageView : VK_NULL_HANDLE;
}

VkBuffer RGPassResources::buffer(RGBufferHandle h) const
{
	const RGBufferRecord *rec = _registry ? _registry->get_buffer(h) : nullptr;
	return rec ? rec->buffer : VK_NULL_HANDLE;
}

// ---- RGPassBuilder ----
void RGPassBuilder::read(RGImageHandle h, RGImageUsage usage)
{
	_imageReads.push_back({h, usage});
}

void RGPassBuilder::write(RGImageHandle h, RGImageUsage usage)
{
	_imageWrites.push_back({h, usage});
}

void RGPassBuilder::read_buffer(RGBufferHandle h, RGBufferUsage usage)
{
	_bufferReads.push_back({h, usage});
}

void RGPassBuilder::write_buffer(RGBufferHandle h, RGBufferUsage usage)
{
    _bufferWrites.push_back({h, usage});
}

void RGPassBuilder::read_buffer(VkBuffer buffer, RGBufferUsage usage, VkDeviceSize size, const char* name)
{
    if (!_registry || buffer == VK_NULL_HANDLE) return;
    // Dedup/import
    RGBufferHandle h = _registry->find_buffer(buffer);
    if (!h.valid())
    {
        RGImportedBufferDesc d{};
        d.name = name ? name : "external.buffer";
        d.buffer = buffer;
        d.size = size;
        d.currentStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
        d.currentAccess = 0;
        h = _registry->add_imported(d);
    }
    read_buffer(h, usage);
}

void RGPassBuilder::write_buffer(VkBuffer buffer, RGBufferUsage usage, VkDeviceSize size, const char* name)
{
    if (!_registry || buffer == VK_NULL_HANDLE) return;
    RGBufferHandle h = _registry->find_buffer(buffer);
    if (!h.valid())
    {
        RGImportedBufferDesc d{};
        d.name = name ? name : "external.buffer";
        d.buffer = buffer;
        d.size = size;
        d.currentStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
        d.currentAccess = 0;
        h = _registry->add_imported(d);
    }
    write_buffer(h, usage);
}

void RGPassBuilder::write_color(RGImageHandle h, bool clearOnLoad, VkClearValue clear)
{
    RGAttachmentInfo a{};
    a.image = h;
    a.clearOnLoad = clearOnLoad;
	a.clear = clear;
	a.store = true;
	_colors.push_back(a);
	write(h, RGImageUsage::ColorAttachment);
}

void RGPassBuilder::write_depth(RGImageHandle h, bool clearOnLoad, VkClearValue clear)
{
	if (_depthRef == nullptr) _depthRef = &_depthTemp;
	_depthRef->image = h;
	_depthRef->clearOnLoad = clearOnLoad;
	_depthRef->clear = clear;
	_depthRef->store = true;
	write(h, RGImageUsage::DepthAttachment);
}
