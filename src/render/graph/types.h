#pragma once

#include <core/types.h>
#include <string>
#include <vector>

// Lightweight, initial Render Graph types. These will expand as we migrate passes.

enum class RGPassType
{
	Graphics,
	Compute,
	Transfer
};

enum class RGImageUsage
{
	// Read usages
	SampledFragment,
	SampledCompute,
	TransferSrc,

	// Write usages
	ColorAttachment,
	DepthAttachment,
	ComputeWrite,
	TransferDst,

	// Terminal
	Present
};

enum class RGBufferUsage
{
	TransferSrc,
	TransferDst,
	VertexRead,
	IndexRead,
	UniformRead,
	StorageRead,
	StorageReadWrite,
	IndirectArgs
};

struct RGImageHandle
{
	uint32_t id = 0xFFFFFFFFu;
	bool valid() const { return id != 0xFFFFFFFFu; }
	explicit operator bool() const { return valid(); }
};

struct RGBufferHandle
{
	uint32_t id = 0xFFFFFFFFu;
	bool valid() const { return id != 0xFFFFFFFFu; }
	explicit operator bool() const { return valid(); }
};

struct RGImportedImageDesc
{
	std::string name;
	VkImage image = VK_NULL_HANDLE;
	VkImageView imageView = VK_NULL_HANDLE;
	VkFormat format = VK_FORMAT_UNDEFINED;
	VkExtent2D extent{0, 0};
	VkImageLayout currentLayout = VK_IMAGE_LAYOUT_UNDEFINED; // layout at graph begin
	// Optional: last known access state at graph begin. If left as NONE/0 and
	// currentLayout is not UNDEFINED, the graph conservatively assumes an
	// unknown prior write (ALL_COMMANDS + MEMORY_READ|WRITE) for the first barrier.
	VkPipelineStageFlags2 currentStage = VK_PIPELINE_STAGE_2_NONE;
	VkAccessFlags2 currentAccess = 0;
};

struct RGImportedBufferDesc
{
	std::string name;
	VkBuffer buffer = VK_NULL_HANDLE;
	VkDeviceSize size = 0;
	VkPipelineStageFlags2 currentStage = VK_PIPELINE_STAGE_2_NONE;
	VkAccessFlags2 currentAccess = 0;
};

struct RGImageDesc
{
	std::string name;
	VkFormat format = VK_FORMAT_UNDEFINED;
	VkExtent2D extent{0, 0};
	VkImageUsageFlags usage = 0; // creation usage mask; graph sets layouts per-pass
};

struct RGBufferDesc
{
	std::string name;
	VkDeviceSize size = 0;
	VkBufferUsageFlags usage = 0;
	VmaMemoryUsage memoryUsage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
};

// Simple attachment info for dynamic rendering; expanded later for load/store.
struct RGAttachmentInfo
{
	RGImageHandle image;
	VkClearValue clear{};              // default 0
	bool clearOnLoad = false;          // if true, use clear; else load
	bool store = true;                 // store results
};
