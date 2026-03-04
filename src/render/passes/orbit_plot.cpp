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
    if (!plot || !plot->settings().enabled || !plot->has_active_lines())
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

    const VkDeviceSize bytes = static_cast<VkDeviceSize>(lists.vertices.size() * sizeof(OrbitPlotVertex));
    if (bytes == 0)
    {
        return;
    }

    ResourceManager *rm = ctx_local->getResources();
    DeviceManager *dm = ctx_local->getDevice();

    AllocatedBuffer vb = rm->create_buffer(
            static_cast<size_t>(bytes),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
            VMA_MEMORY_USAGE_CPU_TO_GPU);
    if (vb.buffer == VK_NULL_HANDLE || vb.allocation == VK_NULL_HANDLE || vb.info.pMappedData == nullptr)
    {
        return;
    }

    std::memcpy(vb.info.pMappedData, lists.vertices.data(), static_cast<size_t>(bytes));
    vmaFlushAllocation(dm->allocator(), vb.allocation, 0, bytes);

    ctx_local->currentFrame->_deletionQueue.push_function([rm, vb]() {
        rm->destroy_buffer(vb);
    });

    VkBufferDeviceAddressInfo addr_info{};
    addr_info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    addr_info.buffer = vb.buffer;
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

    const uint32_t depth_segment_count = lists.depth_vertex_count / 2u;
    const uint32_t overlay_segment_count = lists.overlay_vertex_count / 2u;

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
