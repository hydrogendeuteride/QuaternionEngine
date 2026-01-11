#include "graph.h"
#include <core/context.h>
#include <core/device/images.h>
#include <core/util/initializers.h>
#include <core/pipeline/manager.h>
#include <core/descriptor/descriptors.h>
#include <core/descriptor/manager.h>
#include <core/frame/resources.h>
#include <core/pipeline/sampler.h>

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <cstdio>
#include <glm/glm.hpp>

#include <core/device/swapchain.h>
#include <core/util/initializers.h>
#include <core/util/debug.h>
#include <fmt/core.h>

#include "core/device/device.h"
#include <chrono>

#include "assets/manager.h"

void RenderGraph::init(EngineContext *ctx)
{
	_context = ctx;
	_resources.init(ctx);
}

void RenderGraph::clear()
{
	_passes.clear();
	_resources.reset();
}

void RenderGraph::shutdown()
{
	// If a timestamp pool exists, ensure the GPU is not using it and destroy it.
	if (_timestampPool != VK_NULL_HANDLE && _context && _context->getDevice())
	{
		// Be conservative here: make sure the graphics queue is idle before destroying.
		vkQueueWaitIdle(_context->getDevice()->graphicsQueue());
		vkDestroyQueryPool(_context->getDevice()->device(), _timestampPool, nullptr);
		_timestampPool = VK_NULL_HANDLE;
	}
}

RGImageHandle RenderGraph::import_image(const RGImportedImageDesc &desc)
{
	return _resources.add_imported(desc);
}

RGBufferHandle RenderGraph::import_buffer(const RGImportedBufferDesc &desc)
{
	return _resources.add_imported(desc);
}

RGImageHandle RenderGraph::create_image(const RGImageDesc &desc)
{
	return _resources.add_transient(desc);
}

RGImageHandle RenderGraph::create_depth_image(const char *name, VkExtent2D extent, VkFormat format)
{
	RGImageDesc d{};
	d.name = name ? name : "depth.transient";
	d.format = format;
	d.extent = extent;
	d.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	return create_image(d);
}

RGBufferHandle RenderGraph::create_buffer(const RGBufferDesc &desc)
{
	return _resources.add_transient(desc);
}

// Render Graph: builds a per-frame DAG from declared image/buffer accesses,
// inserts precise barriers and layouts, and records passes using dynamic rendering.
//
// Key steps:
//  - add_pass(): store declarations and callbacks (build to declare, record to issue commands)
//  - compile():  topologically sort by read/write hazards and generate Vk*Barrier2 sequences
//  - execute():  emit pre-pass barriers, begin dynamic rendering if attachments exist, invoke record()
//
// See docs/RenderGraph.md for API overview and pass patterns.
void RenderGraph::add_pass(const char *name, RGPassType type, BuildCallback build, RecordCallback record)
{
	Pass p{};
	p.name = name;
	p.type = type;
	p.record = std::move(record);

	// Build declarations via builder
	RGAttachmentInfo *depthRef = nullptr;
	RGPassBuilder builder(&_resources,
	                      p.imageReads,
	                      p.imageWrites,
	                      p.bufferReads,
	                      p.bufferWrites,
	                      p.colorAttachments,
	                      depthRef);
	if (build) build(builder, _context);
	if (depthRef)
	{
		p.hasDepth = true;
		p.depthAttachment = *depthRef; // copy declared depth attachment
	}

	_passes.push_back(std::move(p));
}

void RenderGraph::add_pass(const char *name, RGPassType type, RecordCallback record)
{
	// No declarations
	add_pass(name, type, nullptr, std::move(record));
}

bool RenderGraph::compile()
{
	if (!_context) return false;

	// --- Build dependency graph (topological sort) from declared reads/writes ---
	const int n = static_cast<int>(_passes.size());
	if (n <= 1)
	{
		// trivial order; still compute barriers below
	}
	else
	{
		std::vector<std::unordered_set<int> > adjSet(n);
		std::vector<int> indeg(n, 0);

		auto add_edge = [&](int u, int v) {
			if (u == v) return;
			if (u < 0 || v < 0 || u >= n || v >= n) return;
			if (adjSet[u].insert(v).second) indeg[v]++;
		};

		std::unordered_map<uint32_t, int> lastWriterImage;
		std::unordered_map<uint32_t, std::vector<int> > lastReadersImage;
		std::unordered_map<uint32_t, int> lastWriterBuffer;
		std::unordered_map<uint32_t, std::vector<int> > lastReadersBuffer;

		for (int i = 0; i < n; ++i)
		{
			const auto &p = _passes[i];
			if (!p.enabled) continue;

			// Image reads
			for (const auto &r: p.imageReads)
			{
				if (!r.image.valid()) continue;
				auto it = lastWriterImage.find(r.image.id);
				if (it != lastWriterImage.end()) add_edge(it->second, i);
				lastReadersImage[r.image.id].push_back(i);
			}

			// Image writes
			for (const auto &w: p.imageWrites)
			{
				if (!w.image.valid()) continue;
				auto itW = lastWriterImage.find(w.image.id);
				if (itW != lastWriterImage.end()) add_edge(itW->second, i); // WAW
				auto itR = lastReadersImage.find(w.image.id);
				if (itR != lastReadersImage.end())
				{
					for (int rIdx: itR->second) add_edge(rIdx, i); // WAR
					itR->second.clear();
				}
				lastWriterImage[w.image.id] = i;
			}

			// Buffer reads
			for (const auto &r: p.bufferReads)
			{
				if (!r.buffer.valid()) continue;
				auto it = lastWriterBuffer.find(r.buffer.id);
				if (it != lastWriterBuffer.end()) add_edge(it->second, i);
				lastReadersBuffer[r.buffer.id].push_back(i);
			}

			// Buffer writes
			for (const auto &w: p.bufferWrites)
			{
				if (!w.buffer.valid()) continue;
				auto itW = lastWriterBuffer.find(w.buffer.id);
				if (itW != lastWriterBuffer.end()) add_edge(itW->second, i); // WAW
				auto itR = lastReadersBuffer.find(w.buffer.id);
				if (itR != lastReadersBuffer.end())
				{
					for (int rIdx: itR->second) add_edge(rIdx, i); // WAR
					itR->second.clear();
				}
				lastWriterBuffer[w.buffer.id] = i;
			}
		}

		// Kahn's algorithm
		std::queue<int> q;
		for (int i = 0; i < n; ++i) if (indeg[i] == 0) q.push(i);
		std::vector<int> order;
		order.reserve(n);
		while (!q.empty())
		{
			int u = q.front();
			q.pop();
			order.push_back(u);
			for (int v: adjSet[u])
			{
				if (--indeg[v] == 0) q.push(v);
			}
		}

		if (static_cast<int>(order.size()) == n)
		{
			// Reorder passes by topological order
			std::vector<Pass> sorted;
			sorted.reserve(n);
			for (int idx: order) sorted.push_back(std::move(_passes[idx]));
			_passes = std::move(sorted);
		}
		else
		{
			// Cycle detected; keep insertion order but still compute barriers
		}
	}

	struct ImageState
	{
		VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
		// Accumulate read stages/accesses since last barrier or write.
		VkPipelineStageFlags2 readStage = VK_PIPELINE_STAGE_2_NONE;
		VkAccessFlags2 readAccess = 0;
		// Track last write since last barrier.
		VkPipelineStageFlags2 writeStage = VK_PIPELINE_STAGE_2_NONE;
		VkAccessFlags2 writeAccess = 0;
	};

	struct BufferState
	{
		VkPipelineStageFlags2 readStage = VK_PIPELINE_STAGE_2_NONE;
		VkAccessFlags2 readAccess = 0;
		VkPipelineStageFlags2 writeStage = VK_PIPELINE_STAGE_2_NONE;
		VkAccessFlags2 writeAccess = 0;
	};

	auto access_has_write = [](VkAccessFlags2 access) -> bool {
		constexpr VkAccessFlags2 WRITE_MASK =
				VK_ACCESS_2_TRANSFER_WRITE_BIT |
				VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT |
				VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT |
				VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
				VK_ACCESS_2_HOST_WRITE_BIT |
				VK_ACCESS_2_MEMORY_WRITE_BIT;
		return (access & WRITE_MASK) != 0;
	};

	auto is_depth_format = [](VkFormat format) {
		switch (format)
		{
			case VK_FORMAT_D16_UNORM:
			case VK_FORMAT_D16_UNORM_S8_UINT:
			case VK_FORMAT_D24_UNORM_S8_UINT:
			case VK_FORMAT_D32_SFLOAT:
			case VK_FORMAT_D32_SFLOAT_S8_UINT:
				return true;
			default:
				return false;
		}
	};

	auto usage_requires_flag = [](RGImageUsage usage) -> VkImageUsageFlags {
		switch (usage)
		{
			case RGImageUsage::SampledFragment:
			case RGImageUsage::SampledCompute:
				return VK_IMAGE_USAGE_SAMPLED_BIT;
			case RGImageUsage::TransferSrc:
				return VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
			case RGImageUsage::TransferDst:
				return VK_IMAGE_USAGE_TRANSFER_DST_BIT;
			case RGImageUsage::ColorAttachment:
				return VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
			case RGImageUsage::DepthAttachment:
				return VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
			case RGImageUsage::ComputeWrite:
				return VK_IMAGE_USAGE_STORAGE_BIT;
			case RGImageUsage::Present:
				return 0; // swapchain image
			default:
				return 0;
		}
	};

	struct ImageUsageInfo
	{
		VkPipelineStageFlags2 stage;
		VkAccessFlags2 access;
		VkImageLayout layout;
	};

	struct BufferUsageInfo
	{
		VkPipelineStageFlags2 stage;
		VkAccessFlags2 access;
	};

	auto usage_info_image = [](RGImageUsage usage) {
		ImageUsageInfo info{};
		switch (usage)
		{
			case RGImageUsage::SampledFragment:
				info.stage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
				info.access = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
				info.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
				break;
			case RGImageUsage::SampledCompute:
				info.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
				info.access = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
				info.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
				break;
			case RGImageUsage::TransferSrc:
				info.stage = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
				info.access = VK_ACCESS_2_TRANSFER_READ_BIT;
				info.layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
				break;
			case RGImageUsage::TransferDst:
				info.stage = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
				info.access = VK_ACCESS_2_TRANSFER_WRITE_BIT;
				info.layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
				break;
			case RGImageUsage::ColorAttachment:
				info.stage = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
				info.access = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT;
				info.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
				break;
			case RGImageUsage::DepthAttachment:
				info.stage = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
				info.access = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
				              VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
				info.layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
				break;
			case RGImageUsage::ComputeWrite:
				info.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
				info.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
				info.layout = VK_IMAGE_LAYOUT_GENERAL;
				break;
			case RGImageUsage::Present:
				info.stage = VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT;
				info.access = VK_ACCESS_2_MEMORY_READ_BIT;
				info.layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
				break;
			default:
				info.stage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
				info.access = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
				info.layout = VK_IMAGE_LAYOUT_GENERAL;
				break;
		}
		return info;
	};

	auto usage_info_buffer = [](RGBufferUsage usage) {
		BufferUsageInfo info{};
		switch (usage)
		{
			case RGBufferUsage::TransferSrc:
				info.stage = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
				info.access = VK_ACCESS_2_TRANSFER_READ_BIT;
				break;
			case RGBufferUsage::TransferDst:
				info.stage = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
				info.access = VK_ACCESS_2_TRANSFER_WRITE_BIT;
				break;
			case RGBufferUsage::VertexRead:
				info.stage = VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT;
				info.access = VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT;
				break;
			case RGBufferUsage::IndexRead:
				info.stage = VK_PIPELINE_STAGE_2_INDEX_INPUT_BIT;
				info.access = VK_ACCESS_2_INDEX_READ_BIT;
				break;
			case RGBufferUsage::UniformRead:
				info.stage = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
				info.access = VK_ACCESS_2_UNIFORM_READ_BIT;
				break;
			case RGBufferUsage::StorageRead:
				// Storage buffers can be read from compute and any graphics stage (including vertex).
				info.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;
				info.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
				break;
			case RGBufferUsage::StorageReadWrite:
				// Storage buffers can be read/write from compute and read in graphics stages.
				info.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;
				info.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
				break;
			case RGBufferUsage::IndirectArgs:
				info.stage = VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT;
				info.access = VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT;
				break;
			default:
				info.stage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
				info.access = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
				break;
		}
		return info;
	};

	auto buffer_usage_requires_flag = [](RGBufferUsage usage) -> VkBufferUsageFlags {
		switch (usage)
		{
			case RGBufferUsage::TransferSrc: return VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
			case RGBufferUsage::TransferDst: return VK_BUFFER_USAGE_TRANSFER_DST_BIT;
			case RGBufferUsage::VertexRead: return VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
			case RGBufferUsage::IndexRead: return VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
			case RGBufferUsage::UniformRead: return VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
			case RGBufferUsage::StorageRead:
			case RGBufferUsage::StorageReadWrite: return VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
			case RGBufferUsage::IndirectArgs: return VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
			default: return 0;
		}
	};

	const size_t imageCount = _resources.image_count();
	const size_t bufferCount = _resources.buffer_count();
	std::vector<ImageState> imageStates(imageCount);
	std::vector<BufferState> bufferStates(bufferCount);

	// Seed initial states from imported/transient records. If an imported image has a known
	// starting layout but no stage/access, be conservative and assume an unknown prior write.
	for (size_t i = 0; i < imageCount; ++i)
	{
		const RGImageRecord *rec = _resources.get_image(RGImageHandle{static_cast<uint32_t>(i)});
		if (!rec) continue;
		imageStates[i].layout = rec->initialLayout;
		if (rec->initialLayout == VK_IMAGE_LAYOUT_UNDEFINED) continue;

		VkPipelineStageFlags2 st = rec->initialStage;
		VkAccessFlags2 ac = rec->initialAccess;
		if (st == VK_PIPELINE_STAGE_2_NONE && ac == 0)
		{
			st = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
			ac = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
		}
		if (access_has_write(ac))
		{
			imageStates[i].writeStage = st;
			imageStates[i].writeAccess = ac;
		}
		else if (ac != 0)
		{
			imageStates[i].readStage = st;
			imageStates[i].readAccess = ac;
		}
	}

	for (size_t i = 0; i < bufferCount; ++i)
	{
		const RGBufferRecord *rec = _resources.get_buffer(RGBufferHandle{static_cast<uint32_t>(i)});
		if (!rec) continue;
		VkPipelineStageFlags2 st = rec->initialStage;
		VkAccessFlags2 ac = rec->initialAccess;
		if (st == VK_PIPELINE_STAGE_2_NONE) st = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
		if (access_has_write(ac))
		{
			bufferStates[i].writeStage = st;
			bufferStates[i].writeAccess = ac;
		}
		else if (ac != 0)
		{
			bufferStates[i].readStage = st;
			bufferStates[i].readAccess = ac;
		}
	}

	// Track first/last use for lifetime diagnostics and future aliasing
	std::vector<int> imageFirst(imageCount, -1), imageLast(imageCount, -1);
	std::vector<int> bufferFirst(bufferCount, -1), bufferLast(bufferCount, -1);

	for (auto &pass: _passes)
	{
		pass.preImageBarriers.clear();
		pass.preBufferBarriers.clear();
		if (!pass.enabled) { continue; }

		struct DesiredImageAccess
		{
			ImageUsageInfo info{};
			RGImageUsage canonical = RGImageUsage::SampledFragment;
			bool hasAny = false;
			bool hasDepthUsage = false;
			bool warnedLayoutMismatch = false;
		};

		auto image_usage_priority = [](RGImageUsage usage) -> int {
			switch (usage)
			{
				case RGImageUsage::DepthAttachment: return 30;
				case RGImageUsage::ColorAttachment: return 25;
				case RGImageUsage::ComputeWrite: return 20;
				case RGImageUsage::TransferDst: return 15;
				case RGImageUsage::TransferSrc: return 10;
				case RGImageUsage::Present: return 5;
				case RGImageUsage::SampledCompute: return 1;
				case RGImageUsage::SampledFragment: return 1;
				default: return 0;
			}
		};

		std::unordered_map<uint32_t, DesiredImageAccess> desiredImages;
		desiredImages.reserve(pass.imageReads.size() + pass.imageWrites.size());

		auto merge_desired_image = [&](uint32_t id, RGImageUsage usage) {
			ImageUsageInfo u = usage_info_image(usage);
			DesiredImageAccess &d = desiredImages[id];
			if (!d.hasAny)
			{
				d.info = u;
				d.canonical = usage;
				d.hasAny = true;
				d.hasDepthUsage = (usage == RGImageUsage::DepthAttachment);
				return;
			}

			d.info.stage |= u.stage;
			d.info.access |= u.access;
			d.hasDepthUsage = d.hasDepthUsage || (usage == RGImageUsage::DepthAttachment);

			if (d.info.layout != u.layout)
			{
				// Conflicting usages/layouts for the same image within one pass is almost
				// always a bug in the pass declarations (the graph cannot insert mid-pass barriers).
				if (!d.warnedLayoutMismatch)
				{
					fmt::println("[RG][Warn] Pass '{}' declares multiple layouts for image id {} ({} vs {}).",
					             pass.name, id, (int) d.info.layout, (int) u.layout);
					d.warnedLayoutMismatch = true;
				}
			}

			if (image_usage_priority(usage) >= image_usage_priority(d.canonical))
			{
				d.canonical = usage;
				// Layout is derived from the canonical (highest priority) usage; stages/access are unioned.
				d.info.layout = u.layout;
			}
		};

		for (const auto &access: pass.imageReads)
		{
			if (!access.image.valid()) continue;
			merge_desired_image(access.image.id, access.usage);
			if (access.image.id < imageCount)
			{
				if (imageFirst[access.image.id] == -1) imageFirst[access.image.id] = (int) (&pass - _passes.data());
				imageLast[access.image.id] = (int) (&pass - _passes.data());
			}
		}
		for (const auto &access: pass.imageWrites)
		{
			if (!access.image.valid()) continue;
			merge_desired_image(access.image.id, access.usage);
			if (access.image.id < imageCount)
			{
				if (imageFirst[access.image.id] == -1) imageFirst[access.image.id] = (int) (&pass - _passes.data());
				imageLast[access.image.id] = (int) (&pass - _passes.data());
			}
		}

		// Validation: basic layout/format/usage checks for images used by this pass
		// Also build barriers
		for (const auto &[id, d]: desiredImages)
		{
			if (id >= imageCount) continue;

			const RGImageUsage usage = d.canonical;
			const ImageUsageInfo desired = d.info;

			ImageState &state = imageStates[id];
			const VkImageLayout prevLayout = state.layout;
			const bool layoutChange = prevLayout != desired.layout;
			const bool desiredWrite = access_has_write(desired.access);
			const bool prevHasWrite = state.writeAccess != 0;
			const bool prevHasReads = state.readAccess != 0;

			// Hazards requiring a barrier:
			//  - Any layout change
			//  - Any prior write before a new read or write (RAW/WAW)
			//  - Prior reads before a new write (WAR)
			bool needBarrier = layoutChange || prevHasWrite || (prevHasReads && desiredWrite);

			if (needBarrier)
			{
				VkPipelineStageFlags2 srcStage = VK_PIPELINE_STAGE_2_NONE;
				VkAccessFlags2 srcAccess = 0;
				if (prevHasWrite)
				{
					srcStage = state.writeStage;
					srcAccess = state.writeAccess;
				}
				else if (prevHasReads)
				{
					srcStage = state.readStage;
					srcAccess = state.readAccess;
				}
				else if (prevLayout == VK_IMAGE_LAYOUT_UNDEFINED)
				{
					srcStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
					srcAccess = 0;
				}
				else
				{
					// Known layout but unknown access; be conservative.
					srcStage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
					srcAccess = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
				}
				if (srcStage == VK_PIPELINE_STAGE_2_NONE) srcStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;

				VkImageMemoryBarrier2 barrier{.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
				barrier.srcStageMask = srcStage;
				barrier.srcAccessMask = srcAccess;
				barrier.dstStageMask = desired.stage;
				barrier.dstAccessMask = desired.access;
				barrier.oldLayout = prevLayout;
				barrier.newLayout = desired.layout;
				barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
				barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

				const RGImageRecord *rec = _resources.get_image(RGImageHandle{id});
				barrier.image = rec ? rec->image : VK_NULL_HANDLE;

				VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT;
				if (d.hasDepthUsage || (rec && is_depth_format(rec->format)))
				{
					aspect = VK_IMAGE_ASPECT_DEPTH_BIT;
				}
				barrier.subresourceRange = vkinit::image_subresource_range(aspect);
				pass.preImageBarriers.push_back(barrier);

				// Validation messages (debug-only style):
				if (rec)
				{
					// Color attachments should not be depth formats and vice versa
					if (usage == RGImageUsage::ColorAttachment && is_depth_format(rec->format))
					{
						fmt::println("[RG][Warn] Pass '{}' binds depth-format image '{}' as color attachment.",
						             pass.name, rec->name);
					}
					if (usage == RGImageUsage::DepthAttachment && !is_depth_format(rec->format))
					{
						fmt::println("[RG][Warn] Pass '{}' binds non-depth image '{}' as depth attachment.",
						             pass.name, rec->name);
					}
					// Usage flag sanity for transients we created
					if (!rec->imported)
					{
						VkImageUsageFlags need = usage_requires_flag(usage);
						if ((need & rec->creationUsage) != need)
						{
							fmt::println(
								"[RG][Warn] Image '{}' used as '{}' but created without needed usage flags (0x{:x}).",
								rec->name, (int) usage, (unsigned) need);
						}
					}
				}
			}

			if (needBarrier)
			{
				state.readStage = VK_PIPELINE_STAGE_2_NONE;
				state.readAccess = 0;
				state.writeStage = VK_PIPELINE_STAGE_2_NONE;
				state.writeAccess = 0;
			}
			state.layout = desired.layout;
			if (desiredWrite)
			{
				state.readStage = VK_PIPELINE_STAGE_2_NONE;
				state.readAccess = 0;
				state.writeStage = desired.stage;
				state.writeAccess = desired.access;
			}
			else
			{
				state.writeStage = VK_PIPELINE_STAGE_2_NONE;
				state.writeAccess = 0;
				state.readStage |= desired.stage;
				state.readAccess |= desired.access;
			}
		}

		if (bufferCount == 0) continue;

		struct DesiredBufferAccess
		{
			BufferUsageInfo info{};
			RGBufferUsage canonical = RGBufferUsage::UniformRead;
			bool hasAny = false;
		};

		auto buffer_usage_priority = [](RGBufferUsage usage) -> int {
			switch (usage)
			{
				case RGBufferUsage::TransferDst: return 30;
				case RGBufferUsage::TransferSrc: return 25;
				case RGBufferUsage::StorageReadWrite: return 20;
				case RGBufferUsage::StorageRead: return 15;
				case RGBufferUsage::IndirectArgs: return 10;
				case RGBufferUsage::VertexRead: return 5;
				case RGBufferUsage::IndexRead: return 5;
				case RGBufferUsage::UniformRead: return 1;
				default: return 0;
			}
		};

		std::unordered_map<uint32_t, DesiredBufferAccess> desiredBuffers;
		desiredBuffers.reserve(pass.bufferReads.size() + pass.bufferWrites.size());

		auto merge_desired_buffer = [&](uint32_t id, RGBufferUsage usage) {
			BufferUsageInfo u = usage_info_buffer(usage);
			DesiredBufferAccess &d = desiredBuffers[id];
			if (!d.hasAny)
			{
				d.info = u;
				d.canonical = usage;
				d.hasAny = true;
				return;
			}
			d.info.stage |= u.stage;
			d.info.access |= u.access;
			if (buffer_usage_priority(usage) >= buffer_usage_priority(d.canonical))
			{
				d.canonical = usage;
			}
		};

		for (const auto &access: pass.bufferReads)
		{
			if (!access.buffer.valid()) continue;
			merge_desired_buffer(access.buffer.id, access.usage);
			if (access.buffer.id < bufferCount)
			{
				if (bufferFirst[access.buffer.id] == -1) bufferFirst[access.buffer.id] = (int) (&pass - _passes.data());
				bufferLast[access.buffer.id] = (int) (&pass - _passes.data());
			}
		}
		for (const auto &access: pass.bufferWrites)
		{
			if (!access.buffer.valid()) continue;
			merge_desired_buffer(access.buffer.id, access.usage);
			if (access.buffer.id < bufferCount)
			{
				if (bufferFirst[access.buffer.id] == -1) bufferFirst[access.buffer.id] = (int) (&pass - _passes.data());
				bufferLast[access.buffer.id] = (int) (&pass - _passes.data());
			}
		}

		for (const auto &[id, d]: desiredBuffers)
		{
			if (id >= bufferCount) continue;

			const RGBufferUsage usage = d.canonical;
			const BufferUsageInfo desired = d.info;

			BufferState &state = bufferStates[id];
			const bool desiredWrite = access_has_write(desired.access);
			const bool prevHasWrite = state.writeAccess != 0;
			const bool prevHasReads = state.readAccess != 0;

			// Hazards requiring a barrier:
			//  - Any prior write before a new read or write (RAW/WAW)
			//  - Prior reads before a new write (WAR)
			bool needBarrier = prevHasWrite || (prevHasReads && desiredWrite);

			if (needBarrier)
			{
				VkPipelineStageFlags2 srcStage = VK_PIPELINE_STAGE_2_NONE;
				VkAccessFlags2 srcAccess = 0;
				if (prevHasWrite)
				{
					srcStage = state.writeStage;
					srcAccess = state.writeAccess;
				}
				else if (prevHasReads)
				{
					srcStage = state.readStage;
					srcAccess = state.readAccess;
				}
				if (srcStage == VK_PIPELINE_STAGE_2_NONE) srcStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;

				VkBufferMemoryBarrier2 barrier{.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2};
				barrier.srcStageMask = srcStage;
				barrier.srcAccessMask = srcAccess;
				barrier.dstStageMask = desired.stage;
				barrier.dstAccessMask = desired.access;
				barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
				barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

				const RGBufferRecord *rec = _resources.get_buffer(RGBufferHandle{id});
				barrier.buffer = rec ? rec->buffer : VK_NULL_HANDLE;
				barrier.offset = 0;
				// For imported buffers we don't always know the exact VkBuffer size, so use WHOLE_SIZE
				// to avoid violating VUID-VkBufferMemoryBarrier2-size-01189. For transient buffers
				// created by the graph, we track the exact size.
				if (rec && !rec->imported && rec->size > 0)
				{
					barrier.size = rec->size;
				}
				else
				{
					barrier.size = VK_WHOLE_SIZE;
				}
				pass.preBufferBarriers.push_back(barrier);

				if (rec && !rec->imported)
				{
					VkBufferUsageFlags need = buffer_usage_requires_flag(usage);
					if ((need & rec->usage) != need)
					{
						fmt::println(
							"[RG][Warn] Buffer '{}' used as '{}' but created without needed usage flags (0x{:x}).",
							rec->name, (int) usage, (unsigned) need);
					}
				}
			}

			if (needBarrier)
			{
				state.readStage = VK_PIPELINE_STAGE_2_NONE;
				state.readAccess = 0;
				state.writeStage = VK_PIPELINE_STAGE_2_NONE;
				state.writeAccess = 0;
			}
			if (desiredWrite)
			{
				state.readStage = VK_PIPELINE_STAGE_2_NONE;
				state.readAccess = 0;
				state.writeStage = desired.stage;
				state.writeAccess = desired.access;
			}
			else
			{
				state.writeStage = VK_PIPELINE_STAGE_2_NONE;
				state.writeAccess = 0;
				state.readStage |= desired.stage;
				state.readAccess |= desired.access;
			}
		}
	}

	// Store lifetimes into records for diagnostics/aliasing
	for (size_t i = 0; i < imageCount; ++i)
	{
		if (auto *rec = _resources.get_image(RGImageHandle{static_cast<uint32_t>(i)}))
		{
			rec->firstUse = imageFirst[i];
			rec->lastUse = imageLast[i];
		}
	}
	for (size_t i = 0; i < bufferCount; ++i)
	{
		if (auto *rec = _resources.get_buffer(RGBufferHandle{static_cast<uint32_t>(i)}))
		{
			rec->firstUse = bufferFirst[i];
			rec->lastUse = bufferLast[i];
		}
	}

	return true;
}

void RenderGraph::execute(VkCommandBuffer cmd)
{
	// Create/reset timestamp query pool for this execution (2 queries per pass)
	if (_timestampPool != VK_NULL_HANDLE)
	{
		vkDestroyQueryPool(_context->getDevice()->device(), _timestampPool, nullptr);
		_timestampPool = VK_NULL_HANDLE;
	}
	const uint32_t queryCount = static_cast<uint32_t>(_passes.size() * 2);
	if (queryCount > 0)
	{
		VkQueryPoolCreateInfo qpci{.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
		qpci.queryType = VK_QUERY_TYPE_TIMESTAMP;
		qpci.queryCount = queryCount;
		VK_CHECK(vkCreateQueryPool(_context->getDevice()->device(), &qpci, nullptr, &_timestampPool));
		vkCmdResetQueryPool(cmd, _timestampPool, 0, queryCount);
	}

	_lastCpuMillis.assign(_passes.size(), -1.0f);
	_wroteTimestamps.assign(_passes.size(), false);

	for (size_t passIndex = 0; passIndex < _passes.size(); ++passIndex)
	{
		auto &p = _passes[passIndex];
		if (!p.enabled) continue;

		// Debug label per pass
		if (_context && _context->getDevice())
		{
			char labelName[128];
			std::snprintf(labelName, sizeof(labelName), "RG: %s", p.name.c_str());
			vkdebug::cmd_begin_label(_context->getDevice()->device(), cmd, labelName);
		}

		if (!p.preImageBarriers.empty() || !p.preBufferBarriers.empty())
		{
			VkDependencyInfo dep{.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
			dep.imageMemoryBarrierCount = static_cast<uint32_t>(p.preImageBarriers.size());
			dep.pImageMemoryBarriers = p.preImageBarriers.empty() ? nullptr : p.preImageBarriers.data();
			dep.bufferMemoryBarrierCount = static_cast<uint32_t>(p.preBufferBarriers.size());
			dep.pBufferMemoryBarriers = p.preBufferBarriers.empty() ? nullptr : p.preBufferBarriers.data();
			vkCmdPipelineBarrier2(cmd, &dep);
		}

		// Timestamp begin and CPU start after barriers
		if (_timestampPool != VK_NULL_HANDLE)
		{
			const uint32_t qidx = static_cast<uint32_t>(passIndex * 2 + 0);
			vkCmdWriteTimestamp2(cmd, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, _timestampPool, qidx);
		}
		auto cpuStart = std::chrono::high_resolution_clock::now();

		// Begin dynamic rendering if the pass declared attachments
		bool doRendering = (!p.colorAttachments.empty() || p.hasDepth);
		if (doRendering)
		{
			std::vector<VkRenderingAttachmentInfo> colorInfos;
			colorInfos.reserve(p.colorAttachments.size());
			VkRenderingAttachmentInfo depthInfo{};
			bool hasDepth = false;

			// Choose renderArea as the min of all attachment extents.
			// Do not pre-clamp to drawExtent here: swapchain passes (ImGui, present)
			// should be able to use the full window extent.
			VkExtent2D chosenExtent{0, 0};
			auto clamp_min = [](VkExtent2D a, VkExtent2D b) {
				return VkExtent2D{std::min(a.width, b.width), std::min(a.height, b.height)};
			};
			auto set_or_clamp = [&](VkExtent2D e) {
				if (chosenExtent.width == 0 || chosenExtent.height == 0) chosenExtent = e;
				else chosenExtent = clamp_min(chosenExtent, e);
			};

			// Resolve color attachments
			VkExtent2D firstColorExtent{0, 0};
			bool warnedExtentMismatch = false;
			for (const auto &a: p.colorAttachments)
			{
				const RGImageRecord *rec = _resources.get_image(a.image);
				if (!rec || rec->imageView == VK_NULL_HANDLE) continue;
				VkClearValue *pClear = nullptr;
				VkClearValue clear = a.clear;
				if (a.clearOnLoad) pClear = &clear;
				VkRenderingAttachmentInfo info = vkinit::attachment_info(rec->imageView, pClear,
				                                                         VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
				if (!a.store) info.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
				colorInfos.push_back(info);
				if (rec->extent.width && rec->extent.height) set_or_clamp(rec->extent);
				if (firstColorExtent.width == 0 && firstColorExtent.height == 0)
				{
					firstColorExtent = rec->extent;
				}
				else if (!warnedExtentMismatch && (
					         rec->extent.width != firstColorExtent.width || rec->extent.height != firstColorExtent.
					         height))
				{
					fmt::println(
						"[RG][Warn] Pass '{}' has color attachments with mismatched extents ({}x{} vs {}x{}). Using min().",
						p.name,
						firstColorExtent.width, firstColorExtent.height,
						rec->extent.width, rec->extent.height);
					warnedExtentMismatch = true;
				}
			}

			if (p.hasDepth)
			{
				const RGImageRecord *rec = _resources.get_image(p.depthAttachment.image);
				if (rec && rec->imageView != VK_NULL_HANDLE)
				{
					depthInfo = vkinit::depth_attachment_info(rec->imageView, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
					if (p.depthAttachment.clearOnLoad)
					{
						depthInfo.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
						depthInfo.clearValue = p.depthAttachment.clear;
					}
					else
					{
						depthInfo.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
					}
					if (!p.depthAttachment.store) depthInfo.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
					hasDepth = true;
					if (rec->extent.width && rec->extent.height) set_or_clamp(rec->extent);
				}
			}

			if (chosenExtent.width == 0 || chosenExtent.height == 0)
			{
				chosenExtent = _context->getDrawExtent();
			}

			VkRenderingInfo ri{};
			ri.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
			ri.renderArea = VkRect2D{VkOffset2D{0, 0}, chosenExtent};
			ri.layerCount = 1;
			ri.colorAttachmentCount = static_cast<uint32_t>(colorInfos.size());
			ri.pColorAttachments = colorInfos.empty() ? nullptr : colorInfos.data();
			ri.pDepthAttachment = hasDepth ? &depthInfo : nullptr;
			ri.pStencilAttachment = nullptr;

			vkCmdBeginRendering(cmd, &ri);
		}

		if (p.record)
		{
			RGPassResources res(&_resources);
			p.record(cmd, res, _context);
		}

		if (doRendering)
		{
			vkCmdEndRendering(cmd);
		}

		// CPU end and timestamp end
		auto cpuEnd = std::chrono::high_resolution_clock::now();
		_lastCpuMillis[passIndex] = std::chrono::duration<float, std::milli>(cpuEnd - cpuStart).count();
		if (_timestampPool != VK_NULL_HANDLE)
		{
			const uint32_t qidx = static_cast<uint32_t>(passIndex * 2 + 1);
			vkCmdWriteTimestamp2(cmd, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, _timestampPool, qidx);
			_wroteTimestamps[passIndex] = true;
		}

		if (_context && _context->getDevice())
		{
			vkdebug::cmd_end_label(_context->getDevice()->device(), cmd);
		}
	}
}

// --- Import helpers ---
void RenderGraph::add_present_chain(RGImageHandle sourceDraw,
                                    RGImageHandle targetSwapchain,
                                    std::function<void(RenderGraph &)> appendExtra)
{
	if (!sourceDraw.valid() || !targetSwapchain.valid()) return;

	add_pass(
		"PresentLetterbox",
		RGPassType::Graphics,
		[sourceDraw, targetSwapchain](RGPassBuilder &builder, EngineContext *) {
			builder.read(sourceDraw, RGImageUsage::SampledFragment);
			VkClearValue clear{};
			clear.color = {{0.f, 0.f, 0.f, 1.f}};
			builder.write_color(targetSwapchain, true, clear);
		},
		[sourceDraw, targetSwapchain](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
			if (!ctx || !ctx->currentFrame || !ctx->pipelines) return;

			VkImageView srcView = res.image_view(sourceDraw);
			VkImageView dstView = res.image_view(targetSwapchain);
			if (srcView == VK_NULL_HANDLE || dstView == VK_NULL_HANDLE) return;

			VkPipeline pipeline = VK_NULL_HANDLE;
			VkPipelineLayout layout = VK_NULL_HANDLE;
			if (!ctx->pipelines->getGraphics("present_letterbox", pipeline, layout))
			{
				GraphicsPipelineCreateInfo info{};
				info.vertexShaderPath = ctx->getAssets()->shaderPath("fullscreen.vert.spv");
				info.fragmentShaderPath = ctx->getAssets()->shaderPath("present_letterbox.frag.spv");
				info.setLayouts = {ctx->getDescriptorLayouts()->singleImageLayout()};

				VkPushConstantRange pcr{};
				pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
				pcr.offset = 0;
				pcr.size = sizeof(glm::vec4);
				info.pushConstants = {pcr};

				VkFormat swapFmt = ctx->getSwapchain()->swapchainImageFormat();
				info.configure = [swapFmt](PipelineBuilder &b) {
					b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
					b.set_polygon_mode(VK_POLYGON_MODE_FILL);
					b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
					b.set_multisampling_none();
					b.disable_depthtest();
					b.disable_blending();
					b.set_color_attachment_format(swapFmt);
				};

				if (!ctx->pipelines->createGraphicsPipeline("present_letterbox", info))
				{
					return;
				}
				if (!ctx->pipelines->getGraphics("present_letterbox", pipeline, layout))
				{
					return;
				}
			}

			VkDevice device = ctx->getDevice()->device();
			VkDescriptorSetLayout setLayout = ctx->getDescriptorLayouts()->singleImageLayout();
			VkDescriptorSet set = ctx->currentFrame->_frameDescriptors.allocate(device, setLayout);
			DescriptorWriter writer;
			writer.write_image(0, srcView, ctx->getSamplers()->defaultLinear(),
			                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
			                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
			writer.update_set(device, set);

			vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
			vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &set, 0, nullptr);

			VkExtent2D srcSize = ctx->getDrawExtent();
			VkExtent2D dstSize = ctx->getSwapchain()->swapchainExtent();
			VkRect2D dstRect = vkutil::compute_letterbox_rect(srcSize, dstSize);

			float minX = dstSize.width > 0 ? float(dstRect.offset.x) / float(dstSize.width) : 0.f;
			float minY = dstSize.height > 0 ? float(dstRect.offset.y) / float(dstSize.height) : 0.f;
			float sizeX = dstSize.width > 0 ? float(dstRect.extent.width) / float(dstSize.width) : 1.f;
			float sizeY = dstSize.height > 0 ? float(dstRect.extent.height) / float(dstSize.height) : 1.f;

			glm::vec4 pc{minX, minY, sizeX, sizeY};
			vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(glm::vec4), &pc);

			VkViewport vp{0.f, 0.f, float(dstSize.width), float(dstSize.height), 0.f, 1.f};
			VkRect2D sc{{0, 0}, dstSize};
			vkCmdSetViewport(cmd, 0, 1, &vp);
			vkCmdSetScissor(cmd, 0, 1, &sc);
			vkCmdDraw(cmd, 3, 1, 0, 0);
		});

	if (appendExtra)
	{
		appendExtra(*this);
	}

	add_pass(
		"PreparePresent",
		RGPassType::Transfer,
		[targetSwapchain](RGPassBuilder &builder, EngineContext *) {
			builder.write(targetSwapchain, RGImageUsage::Present);
		},
		[](VkCommandBuffer, const RGPassResources &, EngineContext *) {
		});
}

RGImageHandle RenderGraph::import_draw_image()
{
	RGImportedImageDesc d{};
	d.name = "drawImage";
	d.image = _context->getSwapchain()->drawImage().image;
	d.imageView = _context->getSwapchain()->drawImage().imageView;
	d.format = _context->getSwapchain()->drawImage().imageFormat;
	d.extent = _context->getDrawExtent();
	// Treat layout as unknown at frame start to force an explicit barrier
	// into the first declared usage (compute write / color attach). This
	// avoids mismatches when the previous frame ended in a different layout.
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

// --- Debug helpers ---
void RenderGraph::debug_get_passes(std::vector<RGDebugPassInfo> &out) const
{
	out.clear();
	out.reserve(_passes.size());
	for (const auto &p: _passes)
	{
		RGDebugPassInfo info{};
		info.name = p.name;
		info.type = p.type;
		info.enabled = p.enabled;
		info.imageReads = static_cast<uint32_t>(p.imageReads.size());
		info.imageWrites = static_cast<uint32_t>(p.imageWrites.size());
		info.bufferReads = static_cast<uint32_t>(p.bufferReads.size());
		info.bufferWrites = static_cast<uint32_t>(p.bufferWrites.size());
		info.colorAttachmentCount = static_cast<uint32_t>(p.colorAttachments.size());
		info.hasDepth = p.hasDepth;
		size_t idx = &p - _passes.data();
		if (idx < _lastGpuMillis.size()) info.gpuMillis = _lastGpuMillis[idx];
		if (idx < _lastCpuMillis.size()) info.cpuMillis = _lastCpuMillis[idx];
		out.push_back(std::move(info));
	}
}

void RenderGraph::debug_get_images(std::vector<RGDebugImageInfo> &out) const
{
	out.clear();
	out.reserve(_resources.image_count());
	for (uint32_t i = 0; i < _resources.image_count(); ++i)
	{
		const RGImageRecord *rec = _resources.get_image(RGImageHandle{i});
		if (!rec) continue;
		RGDebugImageInfo info{};
		info.id = i;
		info.name = rec->name;
		info.imported = rec->imported;
		info.format = rec->format;
		info.extent = rec->extent;
		info.creationUsage = rec->creationUsage;
		info.firstUse = rec->firstUse;
		info.lastUse = rec->lastUse;
		out.push_back(std::move(info));
	}
}

void RenderGraph::debug_get_buffers(std::vector<RGDebugBufferInfo> &out) const
{
	out.clear();
	out.reserve(_resources.buffer_count());
	for (uint32_t i = 0; i < _resources.buffer_count(); ++i)
	{
		const RGBufferRecord *rec = _resources.get_buffer(RGBufferHandle{i});
		if (!rec) continue;
		RGDebugBufferInfo info{};
		info.id = i;
		info.name = rec->name;
		info.imported = rec->imported;
		info.size = rec->size;
		info.usage = rec->usage;
		info.firstUse = rec->firstUse;
		info.lastUse = rec->lastUse;
		out.push_back(std::move(info));
	}
}

RGImageHandle RenderGraph::import_depth_image()
{
	RGImportedImageDesc d{};
	d.name = "depthImage";
	d.image = _context->getSwapchain()->depthImage().image;
	d.imageView = _context->getSwapchain()->depthImage().imageView;
	d.format = _context->getSwapchain()->depthImage().imageFormat;
	d.extent = _context->getDrawExtent();
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

RGImageHandle RenderGraph::import_gbuffer_position()
{
	RGImportedImageDesc d{};
	d.name = "gBuffer.position";
	d.image = _context->getSwapchain()->gBufferPosition().image;
	d.imageView = _context->getSwapchain()->gBufferPosition().imageView;
	d.format = _context->getSwapchain()->gBufferPosition().imageFormat;
	d.extent = _context->getDrawExtent();
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

RGImageHandle RenderGraph::import_gbuffer_normal()
{
	RGImportedImageDesc d{};
	d.name = "gBuffer.normal";
	d.image = _context->getSwapchain()->gBufferNormal().image;
	d.imageView = _context->getSwapchain()->gBufferNormal().imageView;
	d.format = _context->getSwapchain()->gBufferNormal().imageFormat;
	d.extent = _context->getDrawExtent();
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

RGImageHandle RenderGraph::import_gbuffer_albedo()
{
	RGImportedImageDesc d{};
	d.name = "gBuffer.albedo";
	d.image = _context->getSwapchain()->gBufferAlbedo().image;
	d.imageView = _context->getSwapchain()->gBufferAlbedo().imageView;
	d.format = _context->getSwapchain()->gBufferAlbedo().imageFormat;
	d.extent = _context->getDrawExtent();
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

RGImageHandle RenderGraph::import_gbuffer_extra()
{
	RGImportedImageDesc d{};
	d.name = "gBuffer.extra";
	d.image = _context->getSwapchain()->gBufferExtra().image;
	d.imageView = _context->getSwapchain()->gBufferExtra().imageView;
	d.format = _context->getSwapchain()->gBufferExtra().imageFormat;
	d.extent = _context->getDrawExtent();
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

RGImageHandle RenderGraph::import_id_buffer()
{
	RGImportedImageDesc d{};
	d.name = "idBuffer.objectID";
	d.image = _context->getSwapchain()->idBuffer().image;
	d.imageView = _context->getSwapchain()->idBuffer().imageView;
	d.format = _context->getSwapchain()->idBuffer().imageFormat;
	d.extent = _context->getDrawExtent();
	d.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	return import_image(d);
}

RGImageHandle RenderGraph::import_swapchain_image(uint32_t index)
{
	RGImportedImageDesc d{};
	d.name = "swapchain.image";
	const auto &views = _context->getSwapchain()->swapchainImageViews();
	const auto &imgs = _context->getSwapchain()->swapchainImages();
	d.image = imgs[index];
	d.imageView = views[index];
	d.format = _context->getSwapchain()->swapchainImageFormat();
	d.extent = _context->getSwapchain()->swapchainExtent();
	// Track actual layout across frames. After present, images are in PRESENT_SRC_KHR.
	d.currentLayout = _context->getSwapchain()->swapchain_image_layout(index);
	return import_image(d);
}

void RenderGraph::resolve_timings()
{
	if (_timestampPool == VK_NULL_HANDLE || _passes.empty())
	{
		_lastGpuMillis.assign(_passes.size(), -1.0f);
		return;
	}

	const uint32_t queryCount = static_cast<uint32_t>(_passes.size() * 2);
	std::vector<uint64_t> results(queryCount, 0);
	VkResult r = vkGetQueryPoolResults(
		_context->getDevice()->device(), _timestampPool,
		0, queryCount,
		sizeof(uint64_t) * results.size(), results.data(), sizeof(uint64_t),
		VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
	VkPhysicalDeviceProperties props{};
	vkGetPhysicalDeviceProperties(_context->getDevice()->physicalDevice(), &props);
	const double tickNs = props.limits.timestampPeriod;

	_lastGpuMillis.assign(_passes.size(), -1.0f);
	for (size_t i = 0; i < _passes.size(); ++i)
	{
		if (!_wroteTimestamps.empty() && !_wroteTimestamps[i])
		{
			_lastGpuMillis[i] = -1.0f;
			continue;
		}
		const uint64_t t0 = results[i * 2 + 0];
		const uint64_t t1 = results[i * 2 + 1];
		if (t1 > t0)
		{
			double ns = double(t1 - t0) * tickNs;
			_lastGpuMillis[i] = static_cast<float>(ns / 1.0e6);
		}
		else
		{
			_lastGpuMillis[i] = -1.0f;
		}
	}

	// Ensure any pending work that might still reference the pool is complete
	vkQueueWaitIdle(_context->getDevice()->graphicsQueue());
	vkDestroyQueryPool(_context->getDevice()->device(), _timestampPool, nullptr);
	_timestampPool = VK_NULL_HANDLE;
}
