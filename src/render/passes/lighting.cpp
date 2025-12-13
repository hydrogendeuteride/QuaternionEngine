#include "lighting.h"

#include "core/frame/resources.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/context.h"
#include "core/util/initializers.h"
#include "core/device/resource.h"
#include "render/pipelines.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "core/descriptor/descriptors.h"
#include "core/config.h"

#include "vk_mem_alloc.h"
#include "core/pipeline/sampler.h"
#include "core/device/swapchain.h"
#include "render/graph/graph.h"
#include <array>
#include <cstring>

#include "core/assets/ibl_manager.h"
#include "core/raytracing/raytracing.h"

void LightingPass::init(EngineContext *context)
{
    _context = context;

    // Placeholder empty set layout to keep array sizes stable if needed
    {
        VkDescriptorSetLayoutCreateInfo info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        info.bindingCount = 0; info.pBindings = nullptr;
        vkCreateDescriptorSetLayout(_context->getDevice()->device(), &info, nullptr, &_emptySetLayout);
    }

    // Build descriptor layout for GBuffer inputs
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _gBufferInputDescriptorLayout = builder.build(
            _context->getDevice()->device(), VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr, VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    // Shadow map descriptor layout (set = 2, updated per-frame). Use array of cascades
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kShadowCascadeCount);
        _shadowDescriptorLayout = builder.build(
            _context->getDevice()->device(), VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr, VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    // Build lighting pipelines (RT and non-RT) through PipelineManager
    // Ensure IBL layout exists (moved to IBLManager)
    VkDescriptorSetLayout iblLayout = _emptySetLayout;
    if (_context->ibl && _context->ibl->ensureLayout())
        iblLayout = _context->ibl->descriptorLayout();

    VkDescriptorSetLayout layouts[] = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set=0
        _gBufferInputDescriptorLayout,                          // set=1
        _shadowDescriptorLayout,                                // set=2
        iblLayout                                               // set=3
    };

    GraphicsPipelineCreateInfo baseInfo{};
    baseInfo.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    baseInfo.setLayouts.assign(std::begin(layouts), std::end(layouts));
    baseInfo.configure = [this](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.enable_blending_alphablend();
        b.disable_depthtest();
        b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
    };

    // Non-RT variant (no TLAS required)
    auto infoNoRT = baseInfo;
    infoNoRT.fragmentShaderPath = _context->getAssets()->shaderPath("deferred_lighting_nort.frag.spv");
    _context->pipelines->createGraphicsPipeline("deferred_lighting.nort", infoNoRT);

    // RT variant (requires GL_EXT_ray_query and TLAS bound at set=0,binding=1)
    auto infoRT = baseInfo;
    infoRT.fragmentShaderPath = _context->getAssets()->shaderPath("deferred_lighting.frag.spv");
    _context->pipelines->createGraphicsPipeline("deferred_lighting.rt", infoRT);

    _deletionQueue.push_function([&]() {
        // Pipelines are owned by PipelineManager; only destroy our local descriptor set layout
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _gBufferInputDescriptorLayout, nullptr);
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _shadowDescriptorLayout, nullptr);
        if (_emptySetLayout) vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _emptySetLayout, nullptr);
    });

    // Create tiny fallback textures for IBL (grey 2D and RG LUT)
    // so shaders can safely sample even when IBL isn't loaded.
    {
        const uint32_t pixel = 0xFF333333u; // RGBA8 grey
        _fallbackIbl2D = _context->getResources()->create_image(&pixel, VkExtent3D{1,1,1},
                                                                VK_FORMAT_R8G8B8A8_UNORM,
                                                                VK_IMAGE_USAGE_SAMPLED_BIT);
    }
    {
        // 1x1 RG UNORM for BRDF LUT fallback
        const uint16_t rg = 0x0000u; // R=0,G=0
        _fallbackBrdfLut2D = _context->getResources()->create_image(
            &rg, VkExtent3D{1,1,1}, VK_FORMAT_R8G8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);
    }
}

void LightingPass::execute(VkCommandBuffer)
{
    // Lighting is executed via the render graph now.
}

void LightingPass::register_graph(RenderGraph *graph,
                                  RGImageHandle drawHandle,
                                  RGImageHandle gbufferPosition,
                                  RGImageHandle gbufferNormal,
                                  RGImageHandle gbufferAlbedo,
                                  RGImageHandle gbufferExtra,
                                  std::span<RGImageHandle> shadowCascades)
{
    if (!graph || !drawHandle.valid() || !gbufferPosition.valid() || !gbufferNormal.valid() || !gbufferAlbedo.valid() ||
        !gbufferExtra.valid())
    {
        return;
    }

    graph->add_pass(
        "Lighting",
        RGPassType::Graphics,
        [drawHandle, gbufferPosition, gbufferNormal, gbufferAlbedo, gbufferExtra, shadowCascades](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(gbufferPosition, RGImageUsage::SampledFragment);
            builder.read(gbufferNormal, RGImageUsage::SampledFragment);
            builder.read(gbufferAlbedo, RGImageUsage::SampledFragment);
            builder.read(gbufferExtra, RGImageUsage::SampledFragment);
            for (size_t i = 0; i < shadowCascades.size(); ++i)
            {
                if (shadowCascades[i].valid()) builder.read(shadowCascades[i], RGImageUsage::SampledFragment);
            }

            builder.write_color(drawHandle);
        },
        [this, drawHandle, gbufferPosition, gbufferNormal, gbufferAlbedo, gbufferExtra, shadowCascades](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
        {
            draw_lighting(cmd, ctx, res, drawHandle, gbufferPosition, gbufferNormal, gbufferAlbedo, gbufferExtra, shadowCascades);
        });
}

void LightingPass::draw_lighting(VkCommandBuffer cmd,
                                 EngineContext *context,
                                 const RGPassResources &resources,
                                 RGImageHandle drawHandle,
                                 RGImageHandle gbufferPosition,
                                 RGImageHandle gbufferNormal,
                                 RGImageHandle gbufferAlbedo,
                                 RGImageHandle gbufferExtra,
                                 std::span<RGImageHandle> shadowCascades)
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager) return;

    VkImageView drawView = resources.image_view(drawHandle);
    if (drawView == VK_NULL_HANDLE) return;

    VkImageView posView = resources.image_view(gbufferPosition);
    VkImageView nrmView = resources.image_view(gbufferNormal);
    VkImageView albView = resources.image_view(gbufferAlbedo);
    VkImageView extView = resources.image_view(gbufferExtra);
    if (posView == VK_NULL_HANDLE || nrmView == VK_NULL_HANDLE || albView == VK_NULL_HANDLE || extView == VK_NULL_HANDLE)
    {
        return;
    }

    // Choose RT only if TLAS is valid; otherwise fall back to non-RT.
    const bool haveRTFeatures = ctxLocal->getDevice()->supportsAccelerationStructure();
    const VkAccelerationStructureKHR tlas = (ctxLocal->ray ? ctxLocal->ray->tlas() : VK_NULL_HANDLE);
    const VkDeviceAddress tlasAddr = (ctxLocal->ray ? ctxLocal->ray->tlasAddress() : 0);
    const bool useRT =
        haveRTFeatures &&
        ctxLocal->shadowSettings.enabled &&
        (ctxLocal->shadowSettings.mode != 0u) &&
        (tlas != VK_NULL_HANDLE) &&
        (tlasAddr != 0);

    const char* pipeName = useRT ? "deferred_lighting.rt" : "deferred_lighting.nort";
    if (!pipelineManager->getGraphics(pipeName, _pipeline, _pipelineLayout))
    {
        // Try the other variant as a fallback
        const char* fallback = useRT ? "deferred_lighting.nort" : "deferred_lighting.rt";
        if (!pipelineManager->getGraphics(fallback, _pipeline, _pipelineLayout))
            return; // Neither pipeline is ready
    }

    // Dynamic rendering is handled by the RenderGraph using the declared draw attachment.

    AllocatedBuffer gpuSceneDataBuffer = resourceManager->create_buffer(
        sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, gpuSceneDataBuffer]()
    {
        resourceManager->destroy_buffer(gpuSceneDataBuffer);
    });

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(deviceManager->allocator(), gpuSceneDataBuffer.allocation, &allocInfo);
    auto *sceneUniformData = static_cast<GPUSceneData *>(allocInfo.pMappedData);
    *sceneUniformData = ctxLocal->getSceneData();
    vmaFlushAllocation(deviceManager->allocator(), gpuSceneDataBuffer.allocation, 0, sizeof(GPUSceneData));

    VkDescriptorSet globalDescriptor = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    DescriptorWriter writer;
    writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    // Only write TLAS when using the RT pipeline and we have a valid TLAS
    if (useRT)
    {
        writer.write_acceleration_structure(1, tlas);
    }
    writer.update_set(deviceManager->device(), globalDescriptor);

    // Allocate and write GBuffer descriptor set for this frame (set = 1).
    VkDescriptorSet gbufferSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _gBufferInputDescriptorLayout);
    {
        DescriptorWriter gbw;
        gbw.write_image(0, posView, ctxLocal->getSamplers()->defaultLinear(),
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        gbw.write_image(1, nrmView, ctxLocal->getSamplers()->defaultLinear(),
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        gbw.write_image(2, albView, ctxLocal->getSamplers()->defaultLinear(),
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        gbw.write_image(3, extView, ctxLocal->getSamplers()->defaultLinear(),
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        gbw.update_set(deviceManager->device(), gbufferSet);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalDescriptor, 0,
                            nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 1, 1,
                            &gbufferSet, 0, nullptr);

    // Allocate and write shadow descriptor set for this frame (set = 2).
    // When RT is enabled, TLAS is bound in the global set at (set=0, binding=1)
    // via DescriptorManager::gpuSceneDataLayout(). See docs/RayTracing.md.
    VkDescriptorSet shadowSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _shadowDescriptorLayout);
    {
        const uint32_t cascadeCount = std::min<uint32_t>(kShadowCascadeCount, static_cast<uint32_t>(shadowCascades.size()));
        std::array<VkDescriptorImageInfo, kShadowCascadeCount> infos{};
        for (uint32_t i = 0; i < cascadeCount; ++i)
        {
            infos[i].sampler = ctxLocal->getSamplers()->shadowLinearClamp();
            infos[i].imageView = resources.image_view(shadowCascades[i]);
            infos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        }
        VkWriteDescriptorSet write{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = shadowSet;
        write.dstBinding = 0;
        write.descriptorCount = cascadeCount;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = infos.data();
        vkUpdateDescriptorSets(deviceManager->device(), 1, &write, 0, nullptr);
    }
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 2, 1, &shadowSet, 0, nullptr);

    // IBL descriptor set (set = 3). Use loaded IBL if present, otherwise fall back to black.
    VkImageView specView = _fallbackIbl2D.imageView;
    VkImageView brdfView = _fallbackBrdfLut2D.imageView;
    VkBuffer shBuf = VK_NULL_HANDLE; VkDeviceSize shSize = sizeof(glm::vec4)*9;
    if (ctxLocal->ibl)
    {
        if (ctxLocal->ibl->specular().imageView) specView = ctxLocal->ibl->specular().imageView;
        if (ctxLocal->ibl->brdf().imageView)     brdfView = ctxLocal->ibl->brdf().imageView;
        if (ctxLocal->ibl->hasSH())              shBuf = ctxLocal->ibl->shBuffer().buffer;
    }
    // If SH missing, create a zero buffer for this frame
    AllocatedBuffer shZero{};
    if (shBuf == VK_NULL_HANDLE)
    {
        shZero = resourceManager->create_buffer(shSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
        std::memset(shZero.info.pMappedData, 0, shSize);
        vmaFlushAllocation(deviceManager->allocator(), shZero.allocation, 0, shSize);
        shBuf = shZero.buffer;
        ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, shZero]() { resourceManager->destroy_buffer(shZero); });
    }
    // Allocate from IBL layout (must exist because pipeline was created with it)
    VkDescriptorSetLayout iblSetLayout = (ctxLocal->ibl ? ctxLocal->ibl->descriptorLayout() : _emptySetLayout);
    VkDescriptorSet iblSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), iblSetLayout);
    {
        DescriptorWriter w;
        w.write_image(0, specView, ctxLocal->getSamplers()->defaultLinear(),
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        w.write_image(1, brdfView, ctxLocal->getSamplers()->defaultLinear(),
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        w.write_buffer(2, shBuf, shSize, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        w.update_set(deviceManager->device(), iblSet);
    }
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 3, 1, &iblSet, 0, nullptr);

    VkViewport viewport{};
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = static_cast<float>(ctxLocal->getDrawExtent().width);
    viewport.height = static_cast<float>(ctxLocal->getDrawExtent().height);
    viewport.minDepth = 0.f;
    viewport.maxDepth = 1.f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = {ctxLocal->getDrawExtent().width, ctxLocal->getDrawExtent().height};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdDraw(cmd, 3, 1, 0, 0);

    // RenderGraph ends rendering.
}

void LightingPass::cleanup()
{
    if (_context && _context->getResources())
    {
        if (_fallbackIbl2D.image)
        {
            _context->getResources()->destroy_image(_fallbackIbl2D);
            _fallbackIbl2D = {};
        }
        if (_fallbackBrdfLut2D.image)
        {
            _context->getResources()->destroy_image(_fallbackBrdfLut2D);
            _fallbackBrdfLut2D = {};
        }
    }

    _deletionQueue.flush();
    fmt::print("LightingPass::cleanup()\n");
}
