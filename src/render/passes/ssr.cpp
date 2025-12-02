#include "ssr.h"

#include "raytracing.h"
#include "core/frame/resources.h"
#include "core/descriptor/manager.h"
#include "core/descriptor/descriptors.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "core/pipeline/sampler.h"

#include "render/graph/graph.h"
#include "render/pipelines.h"

void SSRPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines)
    {
        return;
    }

    VkDevice device = _context->getDevice()->device();

    // Set 1 layout: HDR + G-Buffer inputs (all sampled images).
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // hdrColor
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // posTex
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // normalTex
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // albedoTex
        _inputSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    // Graphics pipelines: fullscreen triangle, no depth, HDR color attachment.
    GraphicsPipelineCreateInfo baseInfo{};
    baseInfo.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    baseInfo.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set = 0 (sceneData UBO + optional TLAS)
        _inputSetLayout                                         // set = 1 (HDR + GBuffer)
    };

    baseInfo.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.disable_blending();
        if (_context && _context->getSwapchain())
        {
            b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
        }
    };

    // Non-RT variant (pure screen-space reflections).
    GraphicsPipelineCreateInfo infoNoRT = baseInfo;
    infoNoRT.fragmentShaderPath = _context->getAssets()->shaderPath("ssr.frag.spv");
    _context->pipelines->createGraphicsPipeline("ssr.nort", infoNoRT);

    // RT-assisted variant (SSR + ray-query fallback using TLAS).
    GraphicsPipelineCreateInfo infoRT = baseInfo;
    infoRT.fragmentShaderPath = _context->getAssets()->shaderPath("ssr_rt.frag.spv");
    _context->pipelines->createGraphicsPipeline("ssr.rt", infoRT);
}

void SSRPass::cleanup()
{
    if (_context && _context->getDevice() && _inputSetLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _inputSetLayout, nullptr);
        _inputSetLayout = VK_NULL_HANDLE;
    }

    _deletionQueue.flush();
}

void SSRPass::execute(VkCommandBuffer)
{
    // Executed via render graph; nothing to do here.
}

void SSRPass::register_graph(RenderGraph *graph,
                             RGImageHandle hdrInput,
                             RGImageHandle gbufPos,
                             RGImageHandle gbufNorm,
                             RGImageHandle gbufAlbedo,
                             RGImageHandle hdrOutput)
{
    if (!graph || !hdrInput.valid() || !hdrOutput.valid())
    {
        return;
    }

    graph->add_pass(
        "SSR",
        RGPassType::Graphics,
        [hdrInput, gbufPos, gbufNorm, gbufAlbedo, hdrOutput](RGPassBuilder &builder, EngineContext *)
        {
            // Read current HDR lighting + G-Buffer; write to an HDR output.
            builder.read(hdrInput, RGImageUsage::SampledFragment);
            if (gbufPos.valid())
            {
                builder.read(gbufPos, RGImageUsage::SampledFragment);
            }
            if (gbufNorm.valid())
            {
                builder.read(gbufNorm, RGImageUsage::SampledFragment);
            }
            if (gbufAlbedo.valid())
            {
                builder.read(gbufAlbedo, RGImageUsage::SampledFragment);
            }
            builder.write_color(hdrOutput, false /*load existing contents*/);
        },
        [this, hdrInput, gbufPos, gbufNorm, gbufAlbedo](VkCommandBuffer cmd,
                                                        const RGPassResources &res,
                                                        EngineContext *ctx)
        {
            draw_ssr(cmd, ctx, res, hdrInput, gbufPos, gbufNorm, gbufAlbedo);
        });
}

void SSRPass::draw_ssr(VkCommandBuffer cmd,
                       EngineContext *context,
                       const RGPassResources &resources,
                       RGImageHandle hdrInput,
                       RGImageHandle gbufPos,
                       RGImageHandle gbufNorm,
                       RGImageHandle gbufAlbedo)
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager) return;

    VkImageView hdrView = resources.image_view(hdrInput);
    VkImageView posView = resources.image_view(gbufPos);
    VkImageView normView = resources.image_view(gbufNorm);
    VkImageView albedoView = resources.image_view(gbufAlbedo);
    if (hdrView == VK_NULL_HANDLE || posView == VK_NULL_HANDLE ||
        normView == VK_NULL_HANDLE || albedoView == VK_NULL_HANDLE)
    {
        return;
    }

    // Choose RT variant only if TLAS is valid; otherwise fall back to non-RT.
    const bool haveRTFeatures = deviceManager->supportsAccelerationStructure();
    const VkAccelerationStructureKHR tlas = (ctxLocal->ray ? ctxLocal->ray->tlas() : VK_NULL_HANDLE);
    const VkDeviceAddress tlasAddr = (ctxLocal->ray ? ctxLocal->ray->tlasAddress() : 0);
    const bool useRT = haveRTFeatures && (tlas != VK_NULL_HANDLE) && (tlasAddr != 0);

    const char *pipeName = useRT ? "ssr.rt" : "ssr.nort";
    if (!pipelineManager->getGraphics(pipeName, _pipeline, _pipelineLayout))
    {
        // Try the other variant as a fallback.
        const char *fallback = useRT ? "ssr.nort" : "ssr.rt";
        if (!pipelineManager->getGraphics(fallback, _pipeline, _pipelineLayout))
        {
            return;
        }
    }

    // Scene UBO (set=0, binding=0) – mirror LightingPass behavior.
    AllocatedBuffer sceneBuf = resourceManager->create_buffer(
        sizeof(GPUSceneData),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, sceneBuf]()
    {
        resourceManager->destroy_buffer(sceneBuf);
    });

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(deviceManager->allocator(), sceneBuf.allocation, &allocInfo);
    auto *sceneUniformData = static_cast<GPUSceneData *>(allocInfo.pMappedData);
    *sceneUniformData = ctxLocal->getSceneData();
    vmaFlushAllocation(deviceManager->allocator(), sceneBuf.allocation, 0, sizeof(GPUSceneData));

    VkDescriptorSet globalSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    {
        DescriptorWriter writer;
        writer.write_buffer(0, sceneBuf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        if (useRT)
        {
            writer.write_acceleration_structure(1, tlas);
        }
        writer.update_set(deviceManager->device(), globalSet);
    }

    // Input set (set=1): HDR color + G-Buffer textures.
    VkDescriptorSet inputSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _inputSetLayout);
    {
        DescriptorWriter writer;
        writer.write_image(0, hdrView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(1, posView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(2, normView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(3, albedoView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(deviceManager->device(), inputSet);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 1, 1, &inputSet, 0, nullptr);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport vp{0.f, 0.f, (float)extent.width, (float)extent.height, 0.f, 1.f};
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);

    vkCmdDraw(cmd, 3, 1, 0, 0);
}
