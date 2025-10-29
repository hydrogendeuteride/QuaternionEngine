#include <render/rg_graph.h>
#include <core/engine_context.h>
#include <core/vk_images.h>
#include <core/vk_initializers.h>

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <cstdio>

#include <core/vk_swapchain.h>
#include <core/vk_initializers.h>
#include <core/vk_debug.h>
#include <fmt/core.h>

#include "vk_device.h"
#include <chrono>

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

RGImageHandle RenderGraph::create_depth_image(const char* name, VkExtent2D extent, VkFormat format)
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
        bool initialized = false;
        VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
        VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
        VkAccessFlags2 access = 0;
    };

	struct BufferState
	{
		bool initialized = false;
		VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
		VkAccessFlags2 access = 0;
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
				info.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
				info.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
				break;
			case RGBufferUsage::StorageReadWrite:
				info.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
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
            case RGBufferUsage::VertexRead:  return VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
            case RGBufferUsage::IndexRead:   return VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
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

    // Track first/last use for lifetime diagnostics and future aliasing
    std::vector<int> imageFirst(imageCount, -1), imageLast(imageCount, -1);
    std::vector<int> bufferFirst(bufferCount, -1), bufferLast(bufferCount, -1);

    for (auto &pass: _passes)
    {
        pass.preImageBarriers.clear();
        pass.preBufferBarriers.clear();
        if (!pass.enabled) { continue; }

		std::unordered_map<uint32_t, RGImageUsage> desiredImageUsages;
		desiredImageUsages.reserve(pass.imageReads.size() + pass.imageWrites.size());

        for (const auto &access: pass.imageReads)
        {
            if (!access.image.valid()) continue;
            desiredImageUsages.emplace(access.image.id, access.usage);
            if (access.image.id < imageCount)
            {
                if (imageFirst[access.image.id] == -1) imageFirst[access.image.id] = (int)(&pass - _passes.data());
                imageLast[access.image.id] = (int)(&pass - _passes.data());
            }
        }
        for (const auto &access: pass.imageWrites)
        {
            if (!access.image.valid()) continue;
            desiredImageUsages[access.image.id] = access.usage;
            if (access.image.id < imageCount)
            {
                if (imageFirst[access.image.id] == -1) imageFirst[access.image.id] = (int)(&pass - _passes.data());
                imageLast[access.image.id] = (int)(&pass - _passes.data());
            }
        }

        // Validation: basic layout/format/usage checks for images used by this pass
        // Also build barriers
        for (const auto &[id, usage]: desiredImageUsages)
        {
            if (id >= imageCount) continue;

            ImageUsageInfo desired = usage_info_image(usage);

            ImageState prev = imageStates[id];
            VkImageLayout prevLayout = prev.initialized ? prev.layout : _resources.initial_layout(RGImageHandle{id});
			VkPipelineStageFlags2 srcStage = prev.initialized
				                                 ? prev.stage
				                                 : (prevLayout == VK_IMAGE_LAYOUT_UNDEFINED
					                                    ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
					                                    : VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT);
			VkAccessFlags2 srcAccess = prev.initialized
				                           ? prev.access
				                           : (prevLayout == VK_IMAGE_LAYOUT_UNDEFINED
					                              ? VkAccessFlags2{0}
					                              : (VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT));

			bool needBarrier = !prev.initialized
			                   || prevLayout != desired.layout
			                   || prev.stage != desired.stage
			                   || prev.access != desired.access;

            if (needBarrier)
            {
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
                if (usage == RGImageUsage::DepthAttachment || (rec && is_depth_format(rec->format)))
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
                            fmt::println("[RG][Warn] Image '{}' used as '{}' but created without needed usage flags (0x{:x}).",
                                         rec->name, (int)usage, (unsigned)need);
                        }
                    }
                }
            }

            imageStates[id].initialized = true;
            imageStates[id].layout = desired.layout;
            imageStates[id].stage = desired.stage;
            imageStates[id].access = desired.access;
        }

        if (bufferCount == 0) continue;

		std::unordered_map<uint32_t, RGBufferUsage> desiredBufferUsages;
		desiredBufferUsages.reserve(pass.bufferReads.size() + pass.bufferWrites.size());

        for (const auto &access: pass.bufferReads)
        {
            if (!access.buffer.valid()) continue;
            desiredBufferUsages.emplace(access.buffer.id, access.usage);
            if (access.buffer.id < bufferCount)
            {
                if (bufferFirst[access.buffer.id] == -1) bufferFirst[access.buffer.id] = (int)(&pass - _passes.data());
                bufferLast[access.buffer.id] = (int)(&pass - _passes.data());
            }
        }
        for (const auto &access: pass.bufferWrites)
        {
            if (!access.buffer.valid()) continue;
            desiredBufferUsages[access.buffer.id] = access.usage;
            if (access.buffer.id < bufferCount)
            {
                if (bufferFirst[access.buffer.id] == -1) bufferFirst[access.buffer.id] = (int)(&pass - _passes.data());
                bufferLast[access.buffer.id] = (int)(&pass - _passes.data());
            }
        }

        for (const auto &[id, usage]: desiredBufferUsages)
        {
            if (id >= bufferCount) continue;

            BufferUsageInfo desired = usage_info_buffer(usage);

            BufferState prev = bufferStates[id];
			VkPipelineStageFlags2 srcStage = prev.initialized
				                                 ? prev.stage
				                                 : _resources.initial_stage(RGBufferHandle{id});
			if (srcStage == VK_PIPELINE_STAGE_2_NONE)
			{
				srcStage = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
			}
			VkAccessFlags2 srcAccess = prev.initialized
				                           ? prev.access
				                           : _resources.initial_access(RGBufferHandle{id});

			bool needBarrier = !prev.initialized
			                   || prev.stage != desired.stage
			                   || prev.access != desired.access;

            if (needBarrier)
            {
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
                // If size is unknown or 0 for imported buffers, use WHOLE_SIZE to satisfy VUID 01188
                barrier.size = (rec && rec->size > 0) ? rec->size : VK_WHOLE_SIZE;
                pass.preBufferBarriers.push_back(barrier);

                if (rec && !rec->imported)
                {
                    VkBufferUsageFlags need = buffer_usage_requires_flag(usage);
                    if ((need & rec->usage) != need)
                    {
                        fmt::println("[RG][Warn] Buffer '{}' used as '{}' but created without needed usage flags (0x{:x}).",
                                     rec->name, (int)usage, (unsigned)need);
                    }
                }
            }

			bufferStates[id].initialized = true;
			bufferStates[id].stage = desired.stage;
			bufferStates[id].access = desired.access;
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
        VkQueryPoolCreateInfo qpci{ .sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
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

            // Choose renderArea as the min of all attachment extents and the desired draw extent
            VkExtent2D chosenExtent{_context->getDrawExtent()};
            auto clamp_min = [](VkExtent2D a, VkExtent2D b) {
                return VkExtent2D{std::min(a.width, b.width), std::min(a.height, b.height)};
            };

            // Resolve color attachments
            VkExtent2D firstColorExtent{0,0};
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
                if (rec->extent.width && rec->extent.height) chosenExtent = clamp_min(chosenExtent, rec->extent);
                if (firstColorExtent.width == 0 && firstColorExtent.height == 0)
                {
                    firstColorExtent = rec->extent;
                }
                else if (!warnedExtentMismatch && (rec->extent.width != firstColorExtent.width || rec->extent.height != firstColorExtent.height))
                {
                    fmt::println("[RG][Warn] Pass '{}' has color attachments with mismatched extents ({}x{} vs {}x{}). Using min().",
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
                	if (p.depthAttachment.clearOnLoad) depthInfo.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                    else depthInfo.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
                    if (!p.depthAttachment.store) depthInfo.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                    hasDepth = true;
                    if (rec->extent.width && rec->extent.height) chosenExtent = clamp_min(chosenExtent, rec->extent);
                }
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
		"CopyToSwapchain",
		RGPassType::Transfer,
		[sourceDraw, targetSwapchain](RGPassBuilder &builder, EngineContext *) {
			builder.read(sourceDraw, RGImageUsage::TransferSrc);
			builder.write(targetSwapchain, RGImageUsage::TransferDst);
		},
		[sourceDraw, targetSwapchain](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
			VkImage src = res.image(sourceDraw);
			VkImage dst = res.image(targetSwapchain);
			if (src == VK_NULL_HANDLE || dst == VK_NULL_HANDLE) return;
			vkutil::copy_image_to_image(cmd, src, dst, ctx->getDrawExtent(), ctx->getSwapchain()->swapchainExtent());
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
	d.currentLayout = VK_IMAGE_LAYOUT_GENERAL;
    return import_image(d);
}

// --- Debug helpers ---
void RenderGraph::debug_get_passes(std::vector<RGDebugPassInfo> &out) const
{
    out.clear();
    out.reserve(_passes.size());
    for (const auto &p : _passes)
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
	d.currentLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
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
        VK_QUERY_RESULT_64_BIT);
    // Convert ticks to ms
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(_context->getDevice()->physicalDevice(), &props);
    const double tickNs = props.limits.timestampPeriod;

    _lastGpuMillis.assign(_passes.size(), -1.0f);
    for (size_t i = 0; i < _passes.size(); ++i)
    {
        if (!_wroteTimestamps.empty() && !_wroteTimestamps[i]) { _lastGpuMillis[i] = -1.0f; continue; }
        const uint64_t t0 = results[i*2 + 0];
        const uint64_t t1 = results[i*2 + 1];
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

    vkDestroyQueryPool(_context->getDevice()->device(), _timestampPool, nullptr);
    _timestampPool = VK_NULL_HANDLE;
}
