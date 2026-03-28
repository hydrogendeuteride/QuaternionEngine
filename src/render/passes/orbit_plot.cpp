#include "orbit_plot.h"

#include <core/assets/manager.h>
#include <core/context.h>
#include <core/device/device.h>
#include <core/device/resource.h>
#include <core/device/swapchain.h>
#include <core/frame/resources.h>
#include <core/orbit_plot/orbit_plot.h>
#include <core/pipeline/manager.h>
#include <render/graph/graph.h>
#include <scene/vk_scene.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace
{
    struct OrbitPlotPushConstants
    {
        glm::mat4 viewproj;
        VkDeviceAddress vertex_buffer;
        glm::vec2 inv_viewport_size_ndc;
        float half_line_width_px;
        float aa_px;
    };
    static_assert(offsetof(OrbitPlotPushConstants, vertex_buffer) == 64);
    static_assert(offsetof(OrbitPlotPushConstants, inv_viewport_size_ndc) == 72);
    static_assert(offsetof(OrbitPlotPushConstants, half_line_width_px) == 80);
    static_assert(offsetof(OrbitPlotPushConstants, aa_px) == 84);
    static_assert(sizeof(OrbitPlotPushConstants) >= 88);
    static_assert((sizeof(OrbitPlotPushConstants) % 4) == 0);

    constexpr const char *k_orbit_vert = "debug_lines.vert.spv";
    constexpr const char *k_orbit_frag = "debug_lines.frag.spv";

    constexpr const char *k_ldr_depth = "orbit_lines.ldr.depth";
    constexpr const char *k_ldr_overlay = "orbit_lines.ldr.overlay";
    constexpr const char *k_hdr_depth = "orbit_lines.hdr.depth";
    constexpr const char *k_hdr_overlay = "orbit_lines.hdr.overlay";

    constexpr std::size_t k_min_upload_capacity_bytes = 4ull * 1024ull;

    std::size_t round_up_upload_capacity(const std::size_t required_bytes,
                                         const std::size_t hard_cap_bytes,
                                         const std::size_t current_capacity_bytes)
    {
        std::size_t capacity = std::max(current_capacity_bytes, k_min_upload_capacity_bytes);
        while (capacity < required_bytes && capacity < hard_cap_bytes)
        {
            const std::size_t next = capacity * 2ull;
            if (next <= capacity)
            {
                break;
            }
            capacity = std::min(next, hard_cap_bytes);
        }
        capacity = std::max(capacity, required_bytes);
        return std::min(capacity, hard_cap_bytes);
    }
} // namespace

void OrbitPlotPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getAssets() || !_context->pipelines || !_context->getSwapchain())
    {
        return;
    }

    const VkFormat ldr_format = _context->getSwapchain()->swapchainImageFormat();
    const VkFormat hdr_format = _context->getSwapchain()->drawImage().imageFormat;
    const VkFormat depth_format = _context->getSwapchain()->depthImage().imageFormat;

    GraphicsPipelineCreateInfo base{};
    base.vertexShaderPath = _context->getAssets()->shaderPath(k_orbit_vert);
    base.fragmentShaderPath = _context->getAssets()->shaderPath(k_orbit_frag);
    base.setLayouts = {};

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(OrbitPlotPushConstants);
    base.pushConstants = {pcr};

    auto make_cfg = [depth_format](VkFormat color_format, bool depth_test) {
        return [color_format, depth_format, depth_test](PipelineBuilder &b) {
            b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
            b.set_polygon_mode(VK_POLYGON_MODE_FILL);
            b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
            b.set_multisampling_none();
            b.enable_blending_alphablend();
            if (depth_test)
            {
                b.enable_depthtest(false, VK_COMPARE_OP_GREATER_OR_EQUAL);
            }
            else
            {
                b.disable_depthtest();
            }
            b.set_color_attachment_format(color_format);
            b.set_depth_format(depth_format);
        };
    };

    GraphicsPipelineCreateInfo ldr_depth = base;
    ldr_depth.configure = make_cfg(ldr_format, true);
    _context->pipelines->createGraphicsPipeline(k_ldr_depth, ldr_depth);

    GraphicsPipelineCreateInfo ldr_overlay = base;
    ldr_overlay.configure = make_cfg(ldr_format, false);
    _context->pipelines->createGraphicsPipeline(k_ldr_overlay, ldr_overlay);

    GraphicsPipelineCreateInfo hdr_depth = base;
    hdr_depth.configure = make_cfg(hdr_format, true);
    _context->pipelines->createGraphicsPipeline(k_hdr_depth, hdr_depth);

    GraphicsPipelineCreateInfo hdr_overlay = base;
    hdr_overlay.configure = make_cfg(hdr_format, false);
    _context->pipelines->createGraphicsPipeline(k_hdr_overlay, hdr_overlay);

}

void OrbitPlotPass::cleanup()
{
    if (_context && _context->getResources())
    {
        ResourceManager *rm = _context->getResources();
        for (UploadSlot &slot : _upload_ring)
        {
            if (slot.buffer.buffer != VK_NULL_HANDLE)
            {
                rm->destroy_buffer(slot.buffer);
            }
            slot.buffer = {};
            slot.capacity_bytes = 0;
        }
    }
    _deletionQueue.flush();
}

void OrbitPlotPass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void OrbitPlotPass::register_graph(RenderGraph *graph,
                                   RGImageHandle target_color,
                                   RGImageHandle depth,
                                   bool is_ldr_target)
{
    if (!graph || !_context || !target_color.valid())
    {
        return;
    }

    OrbitPlotSystem *plot = _context->orbit_plot;
    if (!plot || !plot->settings().enabled)
    {
        return;
    }

    if (!plot->has_active_lines())
    {
        return;
    }

    graph->add_pass(
            "OrbitPlot",
            RGPassType::Graphics,
            [target_color, depth](RGPassBuilder &builder, EngineContext *) {
                builder.write_color(target_color);
                if (depth.valid())
                {
                    builder.write_depth(depth, false /*load existing depth*/);
                }
            },
            [this, is_ldr_target](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx) {
                draw_orbit_plot(cmd, ctx, is_ldr_target);
            });
}

void OrbitPlotPass::draw_orbit_plot(VkCommandBuffer cmd,
                                    EngineContext *ctx,
                                    bool is_ldr_target)
{
    EngineContext *ctx_local = ctx ? ctx : _context;
    if (!ctx_local || !ctx_local->currentFrame || !ctx_local->getDevice() || !ctx_local->getResources() || !ctx_local->pipelines)
    {
        return;
    }

    OrbitPlotSystem *plot = ctx_local->orbit_plot;
    if (!plot || !plot->settings().enabled)
    {
        return;
    }

    WorldVec3 origin_world{0.0, 0.0, 0.0};
    if (ctx_local->scene)
    {
        origin_world = ctx_local->scene->get_world_origin();
    }

    OrbitPlotSystem::LineVertexLists lists = plot->build_line_vertices(origin_world);
    if (lists.vertices.empty())
    {
        return;
    }

    ResourceManager *rm = ctx_local->getResources();
    DeviceManager *dm = ctx_local->getDevice();
    const auto upload_start_tp = std::chrono::steady_clock::now();

    std::size_t upload_budget_bytes = plot->settings().upload_budget_bytes;
    const std::size_t min_upload_budget_bytes = sizeof(OrbitPlotVertex) * 2ull;
    if (upload_budget_bytes < min_upload_budget_bytes)
    {
        upload_budget_bytes = min_upload_budget_bytes;
    }

    std::size_t max_upload_vertices = upload_budget_bytes / sizeof(OrbitPlotVertex);
    max_upload_vertices &= ~std::size_t(1);
    max_upload_vertices = std::max<std::size_t>(2ull, max_upload_vertices);

    uint32_t depth_vertex_count = lists.depth_vertex_count & ~1u;
    uint32_t overlay_vertex_count = lists.overlay_vertex_count & ~1u;
    bool upload_cap_hit = false;

    if (lists.vertices.size() > max_upload_vertices)
    {
        upload_cap_hit = true;
        depth_vertex_count = static_cast<uint32_t>(std::min<std::size_t>(max_upload_vertices, depth_vertex_count));
        depth_vertex_count &= ~1u;

        if (depth_vertex_count >= lists.depth_vertex_count)
        {
            const std::size_t remaining_vertices = max_upload_vertices - static_cast<std::size_t>(depth_vertex_count);
            overlay_vertex_count = static_cast<uint32_t>(std::min<std::size_t>(remaining_vertices, overlay_vertex_count));
            overlay_vertex_count &= ~1u;
        }
        else
        {
            overlay_vertex_count = 0;
        }
    }

    const std::size_t upload_vertex_count =
            static_cast<std::size_t>(depth_vertex_count) + static_cast<std::size_t>(overlay_vertex_count);
    const std::size_t upload_bytes = upload_vertex_count * sizeof(OrbitPlotVertex);
    if (upload_vertex_count == 0 || upload_bytes == 0)
    {
        plot->record_upload_stats(0, upload_budget_bytes, 0.0, true, 0, 0);
        return;
    }

    const std::size_t ring_slot = static_cast<std::size_t>(ctx_local->frameIndex) % k_upload_ring_slots;
    UploadSlot &slot = _upload_ring[ring_slot];

    if (slot.buffer.buffer == VK_NULL_HANDLE || slot.buffer.allocation == VK_NULL_HANDLE ||
        slot.buffer.info.pMappedData == nullptr || slot.capacity_bytes < upload_bytes)
    {
        const std::size_t previous_capacity = slot.capacity_bytes;
        if (slot.buffer.buffer != VK_NULL_HANDLE)
        {
            rm->destroy_buffer(slot.buffer);
            slot.buffer = {};
            slot.capacity_bytes = 0;
        }

        const std::size_t hard_cap_bytes = std::max(upload_budget_bytes, upload_bytes);
        const std::size_t new_capacity_bytes =
                round_up_upload_capacity(upload_bytes, hard_cap_bytes, previous_capacity);

        slot.buffer = rm->create_buffer(
                new_capacity_bytes,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                VMA_MEMORY_USAGE_CPU_TO_GPU);
        slot.capacity_bytes = new_capacity_bytes;
    }

    if (slot.buffer.buffer == VK_NULL_HANDLE || slot.buffer.allocation == VK_NULL_HANDLE || slot.buffer.info.pMappedData == nullptr)
    {
        slot.buffer = {};
        slot.capacity_bytes = 0;
        plot->record_upload_stats(0, upload_budget_bytes, 0.0, true, 0, 0);
        return;
    }

    std::memcpy(slot.buffer.info.pMappedData, lists.vertices.data(), upload_bytes);
    vmaFlushAllocation(dm->allocator(), slot.buffer.allocation, 0, upload_bytes);

    VkBufferDeviceAddressInfo addr_info{};
    addr_info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    addr_info.buffer = slot.buffer.buffer;
    const VkDeviceAddress addr = vkGetBufferDeviceAddress(dm->device(), &addr_info);

    OrbitPlotPushConstants pc{};
    pc.viewproj = ctx_local->getSceneData().viewproj;
    pc.vertex_buffer = addr;
    const VkExtent2D extent = ctx_local->getDrawExtent();
    const float width_px = std::clamp(plot->settings().line_width_px, 1.0f, 8.0f);
    const float aa_px = std::clamp(plot->settings().line_aa_px, 0.0f, 4.0f);
    const float inv_w = 2.0f / static_cast<float>(std::max(1u, extent.width));
    const float inv_h = 2.0f / static_cast<float>(std::max(1u, extent.height));
    pc.inv_viewport_size_ndc = glm::vec2(inv_w, inv_h);
    pc.half_line_width_px = 0.5f * width_px;
    pc.aa_px = aa_px;

    const uint32_t depth_segment_count = depth_vertex_count / 2u;
    const uint32_t overlay_segment_count = overlay_vertex_count / 2u;
    const double upload_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - upload_start_tp)
                                     .count();
    plot->record_upload_stats(upload_bytes,
                              upload_budget_bytes,
                              upload_ms,
                              upload_cap_hit,
                              depth_segment_count,
                              overlay_segment_count);

    VkViewport vp{0.f, 0.f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.f, 1.f};
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);

    const char *depth_pipe_name = is_ldr_target ? k_ldr_depth : k_hdr_depth;
    const char *overlay_pipe_name = is_ldr_target ? k_ldr_overlay : k_hdr_overlay;

    if (depth_segment_count > 0)
    {
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;
        if (ctx_local->pipelines->getGraphics(depth_pipe_name, pipeline, layout))
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
            vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
            vkCmdDraw(cmd, 6, depth_segment_count, 0, 0);
        }
    }

    if (overlay_segment_count > 0)
    {
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;
        if (ctx_local->pipelines->getGraphics(overlay_pipe_name, pipeline, layout))
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
            vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
            vkCmdDraw(cmd, 6, overlay_segment_count, 0, depth_segment_count);
        }
    }
}
