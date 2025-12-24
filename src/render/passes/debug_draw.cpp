#include "debug_draw.h"

#include <core/assets/manager.h>
#include <core/context.h>
#include <core/debug_draw/debug_draw.h>
#include <core/device/device.h>
#include <core/device/resource.h>
#include <core/device/swapchain.h>
#include <core/frame/resources.h>
#include <core/pipeline/manager.h>
#include <render/graph/graph.h>
#include <render/graph/resources.h>
#include <scene/vk_scene.h>

#include <cstring>

namespace
{
    struct DebugDrawPushConstants
    {
        glm::mat4 viewproj;
        VkDeviceAddress vertex_buffer;
    };
    static_assert(offsetof(DebugDrawPushConstants, vertex_buffer) == 64);
    static_assert(sizeof(DebugDrawPushConstants) >= 72);
    static_assert((sizeof(DebugDrawPushConstants) % 4) == 0);

    constexpr const char *k_debug_vert = "debug_lines.vert.spv";
    constexpr const char *k_debug_frag = "debug_lines.frag.spv";

    constexpr const char *k_ldr_depth = "debug_lines.ldr.depth";
    constexpr const char *k_ldr_overlay = "debug_lines.ldr.overlay";
    constexpr const char *k_hdr_depth = "debug_lines.hdr.depth";
    constexpr const char *k_hdr_overlay = "debug_lines.hdr.overlay";
}

void DebugDrawPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getAssets() || !_context->pipelines || !_context->getSwapchain())
    {
        return;
    }

    const VkFormat ldrFormat = _context->getSwapchain()->swapchainImageFormat();
    const VkFormat hdrFormat = _context->getSwapchain()->drawImage().imageFormat;
    const VkFormat depthFormat = _context->getSwapchain()->depthImage().imageFormat;

    GraphicsPipelineCreateInfo base{};
    base.vertexShaderPath = _context->getAssets()->shaderPath(k_debug_vert);
    base.fragmentShaderPath = _context->getAssets()->shaderPath(k_debug_frag);
    base.setLayouts = {};

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(DebugDrawPushConstants);
    base.pushConstants = {pcr};

    auto make_cfg = [depthFormat](VkFormat colorFormat, bool depth_test) {
        return [colorFormat, depthFormat, depth_test](PipelineBuilder &b) {
            b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
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
            b.set_color_attachment_format(colorFormat);
            b.set_depth_format(depthFormat);
        };
    };

    GraphicsPipelineCreateInfo ldrDepth = base;
    ldrDepth.configure = make_cfg(ldrFormat, true);
    _context->pipelines->createGraphicsPipeline(k_ldr_depth, ldrDepth);

    GraphicsPipelineCreateInfo ldrOverlay = base;
    ldrOverlay.configure = make_cfg(ldrFormat, false);
    _context->pipelines->createGraphicsPipeline(k_ldr_overlay, ldrOverlay);

    GraphicsPipelineCreateInfo hdrDepth = base;
    hdrDepth.configure = make_cfg(hdrFormat, true);
    _context->pipelines->createGraphicsPipeline(k_hdr_depth, hdrDepth);

    GraphicsPipelineCreateInfo hdrOverlay = base;
    hdrOverlay.configure = make_cfg(hdrFormat, false);
    _context->pipelines->createGraphicsPipeline(k_hdr_overlay, hdrOverlay);
}

void DebugDrawPass::cleanup()
{
    _deletionQueue.flush();
}

void DebugDrawPass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void DebugDrawPass::register_graph(RenderGraph *graph,
                                   RGImageHandle target_color,
                                   RGImageHandle depth,
                                   bool is_ldr_target)
{
    if (!graph || !_context || !target_color.valid())
    {
        return;
    }

    if (!_context->debug_draw || !_context->debug_draw->settings().enabled)
    {
        return;
    }

    // Always declare depth so depth-tested lines can be drawn when present.
    // Pipelines are compatible even if the overlay variant disables depth testing.
    graph->add_pass(
        "DebugDraw",
        RGPassType::Graphics,
        [target_color, depth](RGPassBuilder &builder, EngineContext *) {
            builder.write_color(target_color);
            if (depth.valid())
            {
                builder.write_depth(depth, false /*load existing depth*/);
            }
        },
        [this, is_ldr_target](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            (void)res;
            draw_debug(cmd, ctx, res, is_ldr_target);
        });
}

void DebugDrawPass::draw_debug(VkCommandBuffer cmd,
                               EngineContext *ctx,
                               const RGPassResources &,
                               bool is_ldr_target)
{
    EngineContext *ctxLocal = ctx ? ctx : _context;
    if (!ctxLocal || !ctxLocal->currentFrame || !ctxLocal->getDevice() || !ctxLocal->getResources() || !ctxLocal->pipelines)
    {
        return;
    }

    DebugDrawSystem *dd = ctxLocal->debug_draw;
    if (!dd || !dd->settings().enabled)
    {
        return;
    }

    WorldVec3 origin_world{0.0, 0.0, 0.0};
    if (ctxLocal->scene)
    {
        origin_world = ctxLocal->scene->get_world_origin();
    }

    DebugDrawSystem::LineVertexLists lists = dd->build_line_vertices(origin_world);
    if (lists.vertices.empty())
    {
        return;
    }

    const VkDeviceSize bytes = static_cast<VkDeviceSize>(lists.vertices.size() * sizeof(DebugDrawVertex));
    if (bytes == 0)
    {
        return;
    }

    ResourceManager *rm = ctxLocal->getResources();
    DeviceManager *dm = ctxLocal->getDevice();

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

    ctxLocal->currentFrame->_deletionQueue.push_function([rm, vb]() {
        rm->destroy_buffer(vb);
    });

    VkBufferDeviceAddressInfo addrInfo{.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    addrInfo.buffer = vb.buffer;
    VkDeviceAddress addr = vkGetBufferDeviceAddress(dm->device(), &addrInfo);

    DebugDrawPushConstants pc{};
    pc.viewproj = ctxLocal->getSceneData().viewproj;
    pc.vertex_buffer = addr;

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport vp{0.f, 0.f, (float)extent.width, (float)extent.height, 0.f, 1.f};
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);

    const char *depthPipeName = is_ldr_target ? k_ldr_depth : k_hdr_depth;
    const char *overlayPipeName = is_ldr_target ? k_ldr_overlay : k_hdr_overlay;

    if (lists.depth_vertex_count > 0)
    {
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;
        if (ctxLocal->pipelines->getGraphics(depthPipeName, pipeline, layout))
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
            vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
            vkCmdDraw(cmd, lists.depth_vertex_count, 1, 0, 0);
        }
    }

    if (lists.overlay_vertex_count > 0)
    {
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;
        if (ctxLocal->pipelines->getGraphics(overlayPipeName, pipeline, layout))
        {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
            vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
            vkCmdDraw(cmd, lists.overlay_vertex_count, 1, lists.depth_vertex_count, 0);
        }
    }
}
