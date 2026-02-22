#include "rocket_plume.h"

#include "core/frame/resources.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "core/assets/texture_cache.h"
#include "core/pipeline/sampler.h"

#include "render/graph/graph.h"
#include "render/graph/resources.h"
#include "render/pipelines.h"

#include "vk_scene.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace
{
    struct GPURocketPlume
    {
        glm::mat4 world_to_plume;

        glm::vec4 shape;          // x length, y nozzleRadius, z expansionAngleRad, w radiusExp
        glm::vec4 emission0;      // rgb coreColor, w intensity
        glm::vec4 emission1;      // rgb plumeColor, w coreStrength
        glm::vec4 params;         // x coreLength, y radialFalloff, z axialFalloff, w softAbsorption
        glm::vec4 noise_shock;    // x noiseStrength, y noiseScale, z noiseSpeed, w shockStrength
        glm::vec4 shock_misc;     // x shockFrequency, y unused, z unused, w unused
    };

    struct PlumePush
    {
        glm::ivec4 misc; // x steps, y plumeCount
    };
}

void RocketPlumePass::update_noise_texture(EngineContext *ctxLocal)
{
    if (!ctxLocal || !ctxLocal->textures || !ctxLocal->getSamplers() || !ctxLocal->getAssets())
    {
        return;
    }

    std::string desired = ctxLocal->rocketPlumeNoiseTexturePath;
    if (desired.empty())
    {
        desired = "vfx/simplex.ktx2";
    }

    if (_noiseHandle != TextureCache::InvalidHandle && desired == _noisePath)
    {
        return;
    }

    TextureCache *cache = ctxLocal->textures;
    if (_noiseHandle != TextureCache::InvalidHandle)
    {
        cache->unpin(_noiseHandle);
        _noiseHandle = TextureCache::InvalidHandle;
    }

    TextureCache::TextureKey key{};
    key.kind = TextureCache::TextureKey::SourceKind::FilePath;
    key.path = ctxLocal->getAssets()->assetPath(desired);
    key.srgb = false;
    key.mipmapped = true;
    key.channels = TextureCache::TextureKey::ChannelsHint::R;
    std::string id = std::string("RocketPlumeNoise:") + key.path;
    key.hash = texcache::fnv1a64(id);

    VkSampler sampler = ctxLocal->getSamplers()->defaultLinear();
    _noiseHandle = cache->request(key, sampler);
    cache->pin(_noiseHandle);
    _noisePath = desired;
}

void RocketPlumePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getResources() || !_context->getAssets())
    {
        return;
    }

    VkDevice device = _context->getDevice()->device();

    // Set 1 layout: HDR input, gbuffer position, plume SSBO, noise texture.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // hdrInput
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // posTex
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);         // plumes
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // noiseTex
        _inputSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("rocket_plume.frag.spv");
    info.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set = 0 (sceneData UBO)
        _inputSetLayout                                         // set = 1 (inputs + instances)
    };

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(PlumePush);
    info.pushConstants = {pcr};

    info.configure = [this](PipelineBuilder &b)
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

    _context->pipelines->createGraphicsPipeline("rocket_plume", info);

    // Request/pin the configured noise texture for plume breakup.
    update_noise_texture(_context);
}

void RocketPlumePass::cleanup()
{
    if (_context && _context->textures && _noiseHandle != TextureCache::InvalidHandle)
    {
        _context->textures->unpin(_noiseHandle);
        _noiseHandle = TextureCache::InvalidHandle;
        _noisePath.clear();
    }

    if (_context && _context->getDevice() && _inputSetLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _inputSetLayout, nullptr);
        _inputSetLayout = VK_NULL_HANDLE;
    }

    _deletionQueue.flush();
}

void RocketPlumePass::execute(VkCommandBuffer)
{
    // Executed via render graph; nothing to do here.
}

RGImageHandle RocketPlumePass::register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos)
{
    if (!graph || !hdrInput.valid() || !gbufPos.valid())
    {
        return hdrInput;
    }

    if (!_context || !_context->enableRocketPlumes)
    {
        return hdrInput;
    }

    EngineContext *ctxLocal = _context;
    if (!ctxLocal || !ctxLocal->currentFrame || !ctxLocal->getResources() || !ctxLocal->getDevice())
    {
        return hdrInput;
    }

    // Shader-side camera/gbuffer positions are in render-local space (floating-origin shifted).
    // Convert user-facing world->plume transforms to local->plume each frame.
    const glm::vec3 origin_local = glm::vec3(ctxLocal->origin_world);

    std::vector<GPURocketPlume> instances;
    instances.reserve(EngineContext::MAX_ROCKET_PLUMES);
    for (uint32_t i = 0; i < EngineContext::MAX_ROCKET_PLUMES; ++i)
    {
        const RocketPlumeSettings &ps = ctxLocal->rocketPlumes[i];
        if (!ps.enabled)
        {
            continue;
        }

        GPURocketPlume gpu{};
        gpu.world_to_plume = ps.worldToPlume;
        gpu.world_to_plume[3] += ps.worldToPlume * glm::vec4(origin_local, 0.0f);
        gpu.shape = glm::vec4(std::max(0.0f, ps.length),
                              std::max(0.0f, ps.nozzleRadius),
                              ps.expansionAngleRad,
                              std::max(0.0f, ps.radiusExp));
        gpu.emission0 = glm::vec4(glm::max(ps.coreColor, glm::vec3(0.0f)), std::max(0.0f, ps.intensity));
        gpu.emission1 = glm::vec4(glm::max(ps.plumeColor, glm::vec3(0.0f)), std::max(0.0f, ps.coreStrength));
        gpu.params = glm::vec4(std::max(0.0f, ps.coreLength),
                               std::max(0.0f, ps.radialFalloff),
                               std::max(0.0f, ps.axialFalloff),
                               std::max(0.0f, ps.softAbsorption));
        gpu.noise_shock = glm::vec4(std::max(0.0f, ps.noiseStrength),
                                    std::max(0.001f, ps.noiseScale),
                                    ps.noiseSpeed,
                                    std::max(0.0f, ps.shockStrength));
        gpu.shock_misc = glm::vec4(std::max(0.0f, ps.shockFrequency), 0.0f, 0.0f, 0.0f);

        instances.push_back(gpu);
    }

    const uint32_t plumeCount = static_cast<uint32_t>(instances.size());
    if (plumeCount == 0)
    {
        return hdrInput;
    }

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();

    const VkDeviceSize plumeBufSize = static_cast<VkDeviceSize>(instances.size()) * sizeof(GPURocketPlume);
    AllocatedBuffer plumeBuf = resourceManager->create_buffer(
        static_cast<size_t>(plumeBufSize),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);

    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, plumeBuf]() {
        resourceManager->destroy_buffer(plumeBuf);
    });

    if (plumeBuf.info.pMappedData)
    {
        std::memcpy(plumeBuf.info.pMappedData, instances.data(), static_cast<size_t>(plumeBufSize));
        vmaFlushAllocation(deviceManager->allocator(), plumeBuf.allocation, 0, plumeBufSize);
    }

    RGImageDesc desc{};
    desc.name = "hdr.rocket_plume";
    desc.format = (_context && _context->getSwapchain()) ? _context->getSwapchain()->drawImage().imageFormat : VK_FORMAT_R16G16B16A16_SFLOAT;
    desc.extent = _context->getDrawExtent();
    desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    RGImageHandle hdrOutput = graph->create_image(desc);

    const VkBuffer plumeVk = plumeBuf.buffer;
    const VkDeviceSize plumeSize = plumeBufSize;
    const uint32_t count = plumeCount;
    const RGImageHandle hdrIn = hdrInput;

    graph->add_pass(
        "RocketPlume",
        RGPassType::Graphics,
        [hdrIn, gbufPos, hdrOutput, plumeVk, plumeSize](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(hdrIn, RGImageUsage::SampledFragment);
            builder.read(gbufPos, RGImageUsage::SampledFragment);
            builder.read_buffer(plumeVk, RGBufferUsage::StorageRead, plumeSize, "rocket_plume.instances");
            builder.write_color(hdrOutput, false /*load*/);
        },
        [this, hdrIn, gbufPos, plumeVk, plumeSize, count](VkCommandBuffer cmd,
                                                          const RGPassResources &res,
                                                          EngineContext *ctx)
        {
            draw_plumes(cmd, ctx, res, hdrIn, gbufPos, plumeVk, plumeSize, count);
        });

    return hdrOutput;
}

void RocketPlumePass::draw_plumes(VkCommandBuffer cmd,
                                  EngineContext *context,
                                  const RGPassResources &resources,
                                  RGImageHandle hdrInput,
                                  RGImageHandle gbufPos,
                                  VkBuffer plumeBuffer,
                                  VkDeviceSize plumeBufferSize,
                                  uint32_t plumeCount)
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
    if (hdrView == VK_NULL_HANDLE || posView == VK_NULL_HANDLE) return;

    if (plumeBuffer == VK_NULL_HANDLE || plumeBufferSize == 0 || plumeCount == 0)
    {
        return;
    }

    if (!pipelineManager->getGraphics("rocket_plume", _pipeline, _pipelineLayout))
    {
        return;
    }

    // Scene UBO (set=0, binding=0)
    AllocatedBuffer sceneBuf = resourceManager->create_buffer(
        sizeof(GPUSceneData),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, sceneBuf]()
    {
        resourceManager->destroy_buffer(sceneBuf);
    });

    auto *sceneUniformData = static_cast<GPUSceneData *>(sceneBuf.info.pMappedData);
    if (sceneUniformData)
    {
        *sceneUniformData = ctxLocal->getSceneData();
        vmaFlushAllocation(deviceManager->allocator(), sceneBuf.allocation, 0, sizeof(GPUSceneData));
    }

    VkDescriptorSet globalSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    {
        DescriptorWriter writer;
        writer.write_buffer(0, sceneBuf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        writer.update_set(deviceManager->device(), globalSet);
    }

    VkDescriptorSet inputSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _inputSetLayout);
    {
        update_noise_texture(ctxLocal);

        VkImageView noiseView = VK_NULL_HANDLE;
        if (ctxLocal->textures && _noiseHandle != TextureCache::InvalidHandle)
        {
            ctxLocal->textures->markUsed(_noiseHandle, ctxLocal->frameIndex);
            noiseView = ctxLocal->textures->imageView(_noiseHandle);
        }
        if (noiseView == VK_NULL_HANDLE && ctxLocal->getAssets())
        {
            // Fallback to a neutral 0.5 value (flat normal) to avoid biasing noise math.
            noiseView = ctxLocal->getAssets()->fallbackFlatNormalView();
        }
        if (noiseView == VK_NULL_HANDLE)
        {
            return;
        }

        DescriptorWriter writer;
        writer.write_image(0, hdrView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(1, posView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_buffer(2, plumeBuffer, static_cast<size_t>(plumeBufferSize), 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        writer.write_image(3, noiseView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(deviceManager->device(), inputSet);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 1, 1, &inputSet, 0, nullptr);

    PlumePush push{};
    push.misc = glm::ivec4(std::clamp(ctxLocal->rocketPlumeSteps, 8, 256),
                           static_cast<int>(plumeCount),
                           0,
                           0);
    vkCmdPushConstants(cmd, _pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport vp{0.f, 0.f, (float)extent.width, (float)extent.height, 0.f, 1.f};
    VkRect2D sc{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
