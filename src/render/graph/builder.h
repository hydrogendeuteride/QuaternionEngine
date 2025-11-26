#pragma once

#include <render/graph/types.h>
#include <vector>

class RGResourceRegistry;
class EngineContext;

struct RGPassImageAccess
{
	RGImageHandle image;
	RGImageUsage usage;
};

struct RGPassBufferAccess
{
	RGBufferHandle buffer;
	RGBufferUsage usage;
};

// Read-only interface for pass record callbacks to fetch resolved resources
class RGPassResources
{
public:
	RGPassResources(const RGResourceRegistry *registry) : _registry(registry)
	{
	}

	VkImage image(RGImageHandle h) const;

	VkImageView image_view(RGImageHandle h) const;

	VkBuffer buffer(RGBufferHandle h) const;

private:
	const RGResourceRegistry *_registry;
};

// Builder used inside add_*_pass setup lambda to declare reads/writes/attachments
class RGPassBuilder
{
public:
    RGPassBuilder(RGResourceRegistry *registry,
                  std::vector<RGPassImageAccess> &reads,
                  std::vector<RGPassImageAccess> &writes,
                  std::vector<RGPassBufferAccess> &bufferReads,
                  std::vector<RGPassBufferAccess> &bufferWrites,
                  std::vector<RGAttachmentInfo> &colorAttachments,
                  RGAttachmentInfo *&depthAttachmentRef)
        : _registry(registry)
          , _imageReads(reads)
          , _imageWrites(writes)
          , _bufferReads(bufferReads)
          , _bufferWrites(bufferWrites)
          , _colors(colorAttachments)
          , _depthRef(depthAttachmentRef)
    {
    }

	// Declare that the pass will sample/read an image
	void read(RGImageHandle h, RGImageUsage usage);

	// Declare that the pass will write to an image
	void write(RGImageHandle h, RGImageUsage usage);

    // Declare buffer accesses
    void read_buffer(RGBufferHandle h, RGBufferUsage usage);

    void write_buffer(RGBufferHandle h, RGBufferUsage usage);

    // Convenience: declare access to external VkBuffer. Will import/dedup and
    // register the access for this pass.
    void read_buffer(VkBuffer buffer, RGBufferUsage usage, VkDeviceSize size = 0, const char* name = nullptr);
    void write_buffer(VkBuffer buffer, RGBufferUsage usage, VkDeviceSize size = 0, const char* name = nullptr);

	// Graphics attachments
	void write_color(RGImageHandle h, bool clearOnLoad = false, VkClearValue clear = {});

	void write_depth(RGImageHandle h, bool clearOnLoad = false, VkClearValue clear = {});

private:
    RGResourceRegistry *_registry;
    std::vector<RGPassImageAccess> &_imageReads;
    std::vector<RGPassImageAccess> &_imageWrites;
    std::vector<RGPassBufferAccess> &_bufferReads;
    std::vector<RGPassBufferAccess> &_bufferWrites;
    std::vector<RGAttachmentInfo> &_colors;
    RGAttachmentInfo *&_depthRef;
    RGAttachmentInfo _depthTemp{}; // temporary storage used during build
};
