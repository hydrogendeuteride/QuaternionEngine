#include "tonemap.h"

#include <core/context.h>
#include <core/descriptor/descriptors.h>
#include <core/descriptor/manager.h>
#include <core/pipeline/manager.h>
#include <core/assets/manager.h>
#include <core/device/device.h>
#include <core/device/swapchain.h>
#include <core/device/resource.h>
#include <core/pipeline/sampler.h>
#include <render/graph/graph.h>
#include <render/graph/resources.h>

#include "core/frame/resources.h"

struct TonemapPush
{
    float exposure;
    int mode;
    int bloomEnabled;
    float bloomThreshold;
    float bloomIntensity;
};

void TonemapPass::init(EngineContext *context)
{
    _context = context;

    _inputSetLayout = _context->getDescriptorLayouts()->singleImageLayout();

    const VkFormat ldrFormat =
        (_context && _context->getSwapchain())
            ? _context->getSwapchain()->swapchainImageFormat()
            : VK_FORMAT_B8G8R8A8_UNORM;

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("tonemap.frag.spv");
    info.setLayouts = { _inputSetLayout };

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(TonemapPush);
    info.pushConstants = { pcr };

    info.configure = [ldrFormat](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.disable_blending();
        b.set_color_attachment_format(ldrFormat);
    };

    _context->pipelines->createGraphicsPipeline("tonemap", info);

    MaterialPipeline mp{};
    _context->pipelines->getMaterialPipeline("tonemap", mp);
    _pipeline = mp.pipeline;
    _pipelineLayout = mp.layout;
}

void TonemapPass::cleanup()
{
    _deletionQueue.flush();
}

void TonemapPass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

RGImageHandle TonemapPass::register_graph(RenderGraph *graph, RGImageHandle hdrInput)
{
    if (!graph || !hdrInput.valid()) return {};

    const VkFormat ldrFormat =
        (_context && _context->getSwapchain())
            ? _context->getSwapchain()->swapchainImageFormat()
            : VK_FORMAT_B8G8R8A8_UNORM;

    RGImageDesc desc{};
    desc.name = "ldr.tonemap";
    desc.format = ldrFormat;
    desc.extent = _context->getDrawExtent();
    desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                 | VK_IMAGE_USAGE_SAMPLED_BIT
                 | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    RGImageHandle ldr = graph->create_image(desc);

    graph->add_pass(
        "Tonemap",
        RGPassType::Graphics,
        [hdrInput, ldr](RGPassBuilder &builder, EngineContext *) {
            builder.read(hdrInput, RGImageUsage::SampledFragment);
            builder.write_color(ldr, true /*clear*/);
        },
        [this, hdrInput](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            draw_tonemap(cmd, ctx, res, hdrInput);
        }
    );

    return ldr;
}

void TonemapPass::draw_tonemap(VkCommandBuffer cmd, EngineContext *ctx, const RGPassResources &res,
                               RGImageHandle hdrInput)
{
    if (!ctx || !ctx->currentFrame) return;
    VkDevice device = ctx->getDevice()->device();

    VkImageView hdrView = res.image_view(hdrInput);
    if (hdrView == VK_NULL_HANDLE) return;

    VkDescriptorSet set = ctx->currentFrame->_frameDescriptors.allocate(device, _inputSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, hdrView, ctx->getSamplers()->defaultLinear(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, set);

    ctx->pipelines->getGraphics("tonemap", _pipeline, _pipelineLayout);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &set, 0, nullptr);

    TonemapPush push{};
    push.exposure = _exposure;
    push.mode = _mode;
    push.bloomEnabled = _bloomEnabled ? 1 : 0;
    push.bloomThreshold = _bloomThreshold;
    push.bloomIntensity = _bloomIntensity;
    vkCmdPushConstants(cmd, _pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(TonemapPush), &push);

    VkExtent2D extent = ctx->getDrawExtent();
    VkViewport vp{0.f, 0.f, (float)extent.width, (float)extent.height, 0.f, 1.f};
    VkRect2D sc{{0,0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

