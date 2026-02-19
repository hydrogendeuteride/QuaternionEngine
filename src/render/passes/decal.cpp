#include "decal.h"

#include <algorithm>

#include "core/context.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/frame/resources.h"
#include "core/descriptor/manager.h"
#include "core/descriptor/descriptors.h"
#include "core/pipeline/manager.h"
#include "core/pipeline/sampler.h"
#include "core/assets/manager.h"
#include "core/assets/texture_cache.h"
#include "scene/vk_scene.h"
#include "render/graph/graph.h"
#include "render/pipelines.h"

#include "vk_mem_alloc.h"
#include <glm/gtc/quaternion.hpp>

namespace
{
    struct DecalPushConstants
    {
        glm::vec4 axis_x;
        glm::vec4 axis_y;
        glm::vec4 axis_z;
        glm::vec4 center_extent_x;
        glm::vec4 extent_yz_shape_opacity;
        glm::vec4 tint_normal;
    };
    static_assert(sizeof(DecalPushConstants) == 96);
}

void DecalPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() ||
        !_context->getSwapchain() || !_context->pipelines || !_context->getResources())
    {
        return;
    }

    // Set 1: GBuffer position input.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _gbufferInputLayout = builder.build(
            _context->getDevice()->device(),
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    // Set 2: decal material textures (albedo + normal).
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _decalMaterialLayout = builder.build(
            _context->getDevice()->device(),
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    VkPushConstantRange pushRange{};
    pushRange.offset = 0;
    pushRange.size = sizeof(DecalPushConstants);
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayout layouts[] = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set=0
        _gbufferInputLayout,                                    // set=1
        _decalMaterialLayout                                    // set=2
    };

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("decal.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("decal.frag.spv");
    info.setLayouts.assign(std::begin(layouts), std::end(layouts));
    info.pushConstants = {pushRange};
    info.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.enable_blending_alphablend();
        b.set_color_write_mask(VK_COLOR_COMPONENT_R_BIT |
                               VK_COLOR_COMPONENT_G_BIT |
                               VK_COLOR_COMPONENT_B_BIT);
        b.disable_depthtest();

        VkFormat formats[] = {
            _context->getSwapchain()->gBufferNormal().imageFormat,
            _context->getSwapchain()->gBufferAlbedo().imageFormat,
        };
        b.set_color_attachment_formats(std::span<VkFormat>(formats, 2));
    };
    _context->pipelines->createGraphicsPipeline("decal.gbuffer", info);

    const uint32_t albedoPixel = 0xFFFFFFFFu;
    _fallbackAlbedo = _context->getResources()->create_image(
        &albedoPixel,
        VkExtent3D{1, 1, 1},
        VK_FORMAT_R8G8B8A8_UNORM,
        VK_IMAGE_USAGE_SAMPLED_BIT);

    const uint8_t normalPixel[4] = {128u, 128u, 255u, 255u};
    _fallbackNormal = _context->getResources()->create_image(
        normalPixel,
        VkExtent3D{1, 1, 1},
        VK_FORMAT_R8G8B8A8_UNORM,
        VK_IMAGE_USAGE_SAMPLED_BIT);
}

void DecalPass::execute(VkCommandBuffer)
{
    // Executed through the render graph.
}

void DecalPass::register_graph(RenderGraph *graph,
                               RGImageHandle gbufferPosition,
                               RGImageHandle gbufferNormal,
                               RGImageHandle gbufferAlbedo)
{
    if (!graph || !gbufferPosition.valid() || !gbufferNormal.valid() || !gbufferAlbedo.valid())
    {
        return;
    }

    graph->add_pass(
        "Decal",
        RGPassType::Graphics,
        [gbufferPosition, gbufferNormal, gbufferAlbedo](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(gbufferPosition, RGImageUsage::SampledFragment);
            builder.write_color(gbufferNormal, false);
            builder.write_color(gbufferAlbedo, false);
        },
        [this, gbufferPosition, gbufferNormal, gbufferAlbedo](VkCommandBuffer cmd,
                                                              const RGPassResources &resources,
                                                              EngineContext *ctx)
        {
            draw_decals(cmd, ctx, resources, gbufferPosition, gbufferNormal, gbufferAlbedo);
        });
}

void DecalPass::draw_decals(VkCommandBuffer cmd,
                            EngineContext *context,
                            const RGPassResources &resources,
                            RGImageHandle gbufferPosition,
                            RGImageHandle gbufferNormal,
                            RGImageHandle gbufferAlbedo)
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    SamplerManager *samplers = ctxLocal->getSamplers();
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager || !samplers) return;

    const DrawContext &dc = ctxLocal->getMainDrawContext();
    if (dc.Decals.empty()) return;

    VkImageView posView = resources.image_view(gbufferPosition);
    VkImageView nrmView = resources.image_view(gbufferNormal);
    VkImageView albView = resources.image_view(gbufferAlbedo);
    if (posView == VK_NULL_HANDLE || nrmView == VK_NULL_HANDLE || albView == VK_NULL_HANDLE)
    {
        return;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    if (!pipelineManager->getGraphics("decal.gbuffer", pipeline, pipelineLayout))
    {
        return;
    }

    AllocatedBuffer gpuSceneDataBuffer = resourceManager->create_buffer(
        sizeof(GPUSceneData),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
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

    VkDescriptorSet globalSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    {
        DescriptorWriter writer;
        writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        writer.update_set(deviceManager->device(), globalSet);
    }

    VkDescriptorSet gbufferSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), _gbufferInputLayout);
    {
        DescriptorWriter writer;
        writer.write_image(0, posView, samplers->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(deviceManager->device(), gbufferSet);
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &globalSet, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1, &gbufferSet, 0, nullptr);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport viewport{0.f, 0.f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.f, 1.f};
    VkRect2D scissor{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    for (const DecalDraw &decal : dc.Decals)
    {
        if (decal.opacity <= 0.0f)
        {
            continue;
        }

        VkImageView albedoView = _fallbackAlbedo.imageView;
        VkImageView normalView = _fallbackNormal.imageView;
        if (ctxLocal->textures)
        {
            if (decal.albedoHandle != TextureCache::InvalidHandle)
            {
                if (VkImageView view = ctxLocal->textures->imageView(decal.albedoHandle); view != VK_NULL_HANDLE)
                {
                    albedoView = view;
                }
                ctxLocal->textures->markUsed(decal.albedoHandle, ctxLocal->frameIndex);
            }
            if (decal.normalHandle != TextureCache::InvalidHandle)
            {
                if (VkImageView view = ctxLocal->textures->imageView(decal.normalHandle); view != VK_NULL_HANDLE)
                {
                    normalView = view;
                }
                ctxLocal->textures->markUsed(decal.normalHandle, ctxLocal->frameIndex);
            }
        }

        VkDescriptorSet materialSet = ctxLocal->currentFrame->_frameDescriptors.allocate(
            deviceManager->device(), _decalMaterialLayout);
        {
            DescriptorWriter writer;
            writer.write_image(0, albedoView, samplers->defaultLinear(),
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.write_image(1, normalView, samplers->defaultLinear(),
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
            writer.update_set(deviceManager->device(), materialSet);
        }
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 2, 1, &materialSet, 0, nullptr);

        const glm::quat rotation = glm::normalize(decal.rotation);
        const glm::mat3 basis = glm::mat3_cast(rotation);
        const glm::vec3 half_extents = glm::max(decal.half_extents, glm::vec3(1.0e-3f));

        bool fullscreenFallback = false;
        if (ctxLocal->scene)
        {
            const glm::vec3 camera_local = ctxLocal->scene->get_camera_local_position();
            const glm::vec3 rel = camera_local - decal.center_local;
            const glm::vec3 camLocal(
                glm::dot(rel, basis[0]) / half_extents.x,
                glm::dot(rel, basis[1]) / half_extents.y,
                glm::dot(rel, basis[2]) / half_extents.z);

            if (decal.shape == DecalShape::Sphere)
            {
                fullscreenFallback = glm::dot(camLocal, camLocal) <= 1.0f;
            }
            else
            {
                const glm::vec3 a = glm::abs(camLocal);
                fullscreenFallback = (a.x <= 1.0f && a.y <= 1.0f && a.z <= 1.0f);
            }
        }

        DecalPushConstants push{};
        push.axis_x = glm::vec4(basis[0], 0.0f);
        push.axis_y = glm::vec4(basis[1], 0.0f);
        push.axis_z = glm::vec4(basis[2], 0.0f);
        push.center_extent_x = glm::vec4(decal.center_local, half_extents.x);
        // mode:
        // 0 = box proxy volume, 1 = sphere proxy volume,
        // 2 = box fullscreen fallback, 3 = sphere fullscreen fallback.
        const float mode = ((decal.shape == DecalShape::Sphere) ? 1.0f : 0.0f)
                           + (fullscreenFallback ? 2.0f : 0.0f);
        push.extent_yz_shape_opacity = glm::vec4(
            half_extents.y,
            half_extents.z,
            mode,
            decal.opacity);
        push.tint_normal = glm::vec4(decal.tint, decal.normalStrength);

        vkCmdPushConstants(cmd, pipelineLayout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(DecalPushConstants), &push);
        vkCmdDraw(cmd, fullscreenFallback ? 3u : 36u, 1, 0, 0);

        if (ctxLocal->stats)
        {
            ctxLocal->stats->drawcall_count++;
            ctxLocal->stats->triangle_count += fullscreenFallback ? 1 : 12;
        }
    }
}

void DecalPass::cleanup()
{
    if (!_context || !_context->getDevice() || !_context->getResources()) return;

    if (_gbufferInputLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _gbufferInputLayout, nullptr);
        _gbufferInputLayout = VK_NULL_HANDLE;
    }
    if (_decalMaterialLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _decalMaterialLayout, nullptr);
        _decalMaterialLayout = VK_NULL_HANDLE;
    }

    if (_fallbackAlbedo.image)
    {
        _context->getResources()->destroy_image(_fallbackAlbedo);
        _fallbackAlbedo = {};
    }
    if (_fallbackNormal.image)
    {
        _context->getResources()->destroy_image(_fallbackNormal);
        _fallbackNormal = {};
    }
}
