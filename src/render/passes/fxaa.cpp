#include "fxaa.h"

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

void FxaaPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines)
    {
        return;
    }

    _inputSetLayout = _context->getDescriptorLayouts()->singleImageLayout();

    const VkFormat ldrFormat =
        (_context && _context->getSwapchain())
            ? _context->getSwapchain()->swapchainImageFormat()
            : VK_FORMAT_B8G8R8A8_UNORM;

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("fxaa.frag.spv");
    info.setLayouts = { _inputSetLayout };

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(FxaaPush);
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

    _context->pipelines->createGraphicsPipeline("fxaa", info);
}

void FxaaPass::cleanup()
{
    _deletionQueue.flush();
}

void FxaaPass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

RGImageHandle FxaaPass::register_graph(RenderGraph *graph, RGImageHandle ldrInput)
{
    if (!graph || !ldrInput.valid() || !_context)
    {
        return {};
    }

    // If disabled, simply bypass and return the input image.
    if (!_enabled)
    {
        return ldrInput;
    }

    const VkFormat ldrFormat =
        (_context && _context->getSwapchain())
            ? _context->getSwapchain()->swapchainImageFormat()
            : VK_FORMAT_B8G8R8A8_UNORM;

    RGImageDesc desc{};
    desc.name = "ldr.fxaa";
    desc.format = ldrFormat;
    desc.extent = _context->getDrawExtent();
    desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                 | VK_IMAGE_USAGE_SAMPLED_BIT
                 | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    RGImageHandle aaOutput = graph->create_image(desc);

    graph->add_pass(
        "FXAA",
        RGPassType::Graphics,
        [ldrInput, aaOutput](RGPassBuilder &builder, EngineContext *) {
            builder.read(ldrInput, RGImageUsage::SampledFragment);
            builder.write_color(aaOutput, true /*clear*/);
        },
        [this, ldrInput](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            draw_fxaa(cmd, ctx, res, ldrInput);
        });

    return aaOutput;
}

void FxaaPass::draw_fxaa(VkCommandBuffer cmd, EngineContext *ctx, const RGPassResources &res,
                         RGImageHandle ldrInput)
{
    if (!ctx || !ctx->currentFrame) return;
    DeviceManager *deviceManager = ctx->getDevice();
    DescriptorManager *descriptorLayouts = ctx->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctx->pipelines;
    if (!deviceManager || !descriptorLayouts || !pipelineManager) return;

    VkImageView srcView = res.image_view(ldrInput);
    if (srcView == VK_NULL_HANDLE) return;

    VkDevice device = deviceManager->device();

    VkDescriptorSet set = ctx->currentFrame->_frameDescriptors.allocate(device, _inputSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, srcView, ctx->getSamplers()->defaultLinear(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, set);

    VkPipeline pipeline{};
    VkPipelineLayout layout{};
    if (!pipelineManager->getGraphics("fxaa", pipeline, layout))
    {
        return;
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &set, 0, nullptr);

    VkExtent2D extent = ctx->getDrawExtent();

    FxaaPush push{};
    push.inverse_width = extent.width > 0 ? 1.0f / static_cast<float>(extent.width) : 0.0f;
    push.inverse_height = extent.height > 0 ? 1.0f / static_cast<float>(extent.height) : 0.0f;
    push.edge_threshold = _edge_threshold;
    push.edge_threshold_min = _edge_threshold_min;
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(FxaaPush), &push);

    VkViewport vp{0.f, 0.f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.f, 1.f};
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
