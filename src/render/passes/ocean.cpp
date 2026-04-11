#include "ocean.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>

#include "core/assets/ibl_manager.h"
#include "core/assets/manager.h"
#include "core/assets/texture_cache.h"
#include "core/context.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/frame/resources.h"
#include "core/pipeline/manager.h"
#include "core/pipeline/sampler.h"
#include "core/world.h"
#include "render/graph/graph.h"
#include "render/pipelines.h"
#include "scene/planet/planet_system.h"
#include "scene/vk_scene.h"

namespace
{
    constexpr uint32_t k_transmittance_lut_width = 256;
    constexpr uint32_t k_transmittance_lut_height = 64;

    struct OceanPushConstants
    {
        glm::mat4 world_matrix{1.0f};
        glm::vec4 body_center_radius{0.0f};
        glm::vec4 shell_params{0.0f};
        glm::vec4 atmosphere_center_radius{0.0f};
        glm::vec4 atmosphere_params{0.0f};
        glm::vec4 beta_rayleigh{0.0f};
        glm::vec4 beta_mie{0.0f};
        glm::vec4 beta_absorption{0.0f};
        VkDeviceAddress vertex_buffer = 0;
    };

    struct OceanAtmosphereLutPush
    {
        glm::vec4 radii_heights{0.0f};
        glm::ivec4 misc{0};
    };

    bool vec3_finite(const glm::vec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
}

void OceanPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getSwapchain() || !_context->ibl || !_context->ibl->ensureLayout())
    {
        return;
    }

    DeviceManager *device = _context->getDevice();
    VkDevice vk_device = device->device();

    DescriptorLayoutBuilder materialBuilder;
    materialBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    materialBuilder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    materialBuilder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    materialBuilder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    materialBuilder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    materialBuilder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    materialBuilder.add_binding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    _materialSetLayout = materialBuilder.build(vk_device,
                                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                               nullptr,
                                               VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);

    DescriptorLayoutBuilder atmosphereBuilder;
    atmosphereBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    _atmosphereSetLayout = atmosphereBuilder.build(vk_device,
                                                   VK_SHADER_STAGE_FRAGMENT_BIT,
                                                   nullptr,
                                                   VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);

    VkPushConstantRange push_range{};
    push_range.offset = 0;
    push_range.size = sizeof(OceanPushConstants);
    push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("ocean.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("ocean.frag.spv");
    info.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(),
        _materialSetLayout,
        _atmosphereSetLayout,
        _context->ibl->descriptorLayout()
    };
    info.pushConstants = {push_range};
    info.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
        b.set_depth_format(_context->getSwapchain()->depthImage().imageFormat);
    };
    _context->pipelines->createGraphicsPipeline("ocean.surface", info);

    const uint32_t black = 0x00000000u;
    const uint64_t black64 = 0ull;
    _fallbackIbl2D = _context->getResources()->create_image(&black,
                                                            VkExtent3D{1, 1, 1},
                                                            VK_FORMAT_R8G8B8A8_UNORM,
                                                            VK_IMAGE_USAGE_SAMPLED_BIT);
    _fallbackBrdf2D = _context->getResources()->create_image(&black,
                                                             VkExtent3D{1, 1, 1},
                                                             VK_FORMAT_R8G8_UNORM,
                                                             VK_IMAGE_USAGE_SAMPLED_BIT);
    _fallbackTransmittanceLut = _context->getResources()->create_image(&black64,
                                                                       VkExtent3D{1, 1, 1},
                                                                       VK_FORMAT_R16G16B16A16_SFLOAT,
                                                                       VK_IMAGE_USAGE_SAMPLED_BIT);

    ComputePipelineCreateInfo lut_info{};
    lut_info.shaderPath = _context->getAssets()->shaderPath("atmosphere/transmittance_lut.comp.spv");
    lut_info.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
    lut_info.pushConstantSize = sizeof(OceanAtmosphereLutPush);
    lut_info.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
    _context->pipelines->createComputePipeline("ocean.transmittance_lut", lut_info);
    _context->pipelines->createComputeInstance("ocean.transmittance_lut", "ocean.transmittance_lut");
}

void OceanPass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void OceanPass::register_graph(RenderGraph *graph,
                               RGImageHandle drawHandle,
                               RGImageHandle depthHandle,
                               RGImageHandle transmittanceLut)
{
    if (!graph || !drawHandle.valid() || !depthHandle.valid())
    {
        return;
    }

    if (_context && _context->enableAtmosphere && !transmittanceLut.valid())
    {
        RGImageDesc lut_desc{};
        lut_desc.name = "ocean.lut.transmittance";
        lut_desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
        lut_desc.extent = VkExtent2D{k_transmittance_lut_width, k_transmittance_lut_height};
        lut_desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        transmittanceLut = graph->create_image(lut_desc);

        graph->add_pass(
            "OceanLUT.Transmittance",
            RGPassType::Compute,
            [transmittanceLut](RGPassBuilder &builder, EngineContext *)
            {
                builder.write(transmittanceLut, RGImageUsage::ComputeWrite);
            },
            [this, transmittanceLut](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
            {
                EngineContext *ctx_local = ctx ? ctx : _context;
                if (!ctx_local || !ctx_local->pipelines)
                {
                    return;
                }

                VkImageView lut_view = resources.image_view(transmittanceLut);
                if (lut_view == VK_NULL_HANDLE)
                {
                    return;
                }

                ctx_local->pipelines->setComputeInstanceStorageImage("ocean.transmittance_lut", 0, lut_view);

                float planet_radius_m = 0.0f;
                if (ctx_local->scene)
                {
                    if (PlanetSystem *planets = ctx_local->scene->get_planet_system(); planets && planets->enabled())
                    {
                        const PlanetSystem::PlanetBody *picked = nullptr;
                        double best_dist2 = 0.0;
                        const std::string &want_name = ctx_local->atmosphere.bodyName;
                        const WorldVec3 cam_world = ctx_local->scene->getMainCamera().position_world;
                        for (const PlanetSystem::PlanetBody &body : planets->bodies())
                        {
                            if (!body.visible || body.radius_m <= 0.0)
                            {
                                continue;
                            }

                            if (!want_name.empty())
                            {
                                if (body.name == want_name)
                                {
                                    picked = &body;
                                    break;
                                }
                                continue;
                            }

                            const WorldVec3 delta_world = cam_world - body.center_world;
                            const double dist2 = glm::dot(delta_world, delta_world);
                            if (!picked || dist2 < best_dist2)
                            {
                                picked = &body;
                                best_dist2 = dist2;
                            }
                        }

                        if (picked)
                        {
                            planet_radius_m = static_cast<float>(picked->radius_m);
                        }
                    }
                }

                const AtmosphereSettings &settings = ctx_local->atmosphere;
                const float atmosphere_height = std::max(0.0f, settings.atmosphereHeightM);
                const float atmosphere_radius = (planet_radius_m > 0.0f && atmosphere_height > 0.0f)
                    ? (planet_radius_m + atmosphere_height)
                    : 0.0f;

                OceanAtmosphereLutPush push{};
                push.radii_heights = glm::vec4(
                    planet_radius_m,
                    atmosphere_radius,
                    std::max(1.0f, settings.rayleighScaleHeightM),
                    std::max(1.0f, settings.mieScaleHeightM));
                push.misc = glm::ivec4(std::clamp(settings.lightSteps, 2, 256), 0, 0, 0);

                ComputeDispatchInfo dispatch = ComputeManager::createDispatch2D(
                    k_transmittance_lut_width,
                    k_transmittance_lut_height);
                dispatch.pushConstants = &push;
                dispatch.pushConstantSize = sizeof(push);
                ctx_local->pipelines->dispatchComputeInstance(cmd, "ocean.transmittance_lut", dispatch);
            });
    }

    graph->add_pass(
        "Ocean",
        RGPassType::Graphics,
        [drawHandle, depthHandle, transmittanceLut](RGPassBuilder &builder, EngineContext *ctx)
        {
            builder.write_color(drawHandle);
            builder.write_depth(depthHandle, false /* load existing depth */);
            if (transmittanceLut.valid())
            {
                builder.read(transmittanceLut, RGImageUsage::SampledFragment);
            }

            if (ctx)
            {
                const DrawContext &dc = ctx->getMainDrawContext();
                std::unordered_set<VkBuffer> index_set;
                std::unordered_set<VkBuffer> vertex_set;
                index_set.reserve(dc.OceanSurfaces.size());
                vertex_set.reserve(dc.OceanSurfaces.size());
                for (const OceanRenderObject &draw : dc.OceanSurfaces)
                {
                    if (draw.surface.indexBuffer)
                    {
                        index_set.insert(draw.surface.indexBuffer);
                    }
                    if (draw.surface.vertexBuffer)
                    {
                        vertex_set.insert(draw.surface.vertexBuffer);
                    }
                }
                for (VkBuffer buffer : index_set)
                {
                    builder.read_buffer(buffer, RGBufferUsage::IndexRead, 0, "ocean.index");
                }
                for (VkBuffer buffer : vertex_set)
                {
                    builder.read_buffer(buffer, RGBufferUsage::StorageRead, 0, "ocean.vertex");
                }
            }
        },
        [this, drawHandle, depthHandle, transmittanceLut](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
        {
            draw_ocean(cmd, ctx, resources, drawHandle, depthHandle, transmittanceLut);
        });
}

void OceanPass::draw_ocean(VkCommandBuffer cmd,
                           EngineContext *context,
                           const RGPassResources &resources,
                           RGImageHandle,
                           RGImageHandle,
                           RGImageHandle transmittanceLut) const
{
    EngineContext *ctx_local = context ? context : _context;
    if (!ctx_local || !ctx_local->currentFrame)
    {
        return;
    }

    ResourceManager *resource_manager = ctx_local->getResources();
    DeviceManager *device_manager = ctx_local->getDevice();
    PipelineManager *pipeline_manager = ctx_local->pipelines;
    if (!resource_manager || !device_manager || !pipeline_manager)
    {
        return;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (!pipeline_manager->getGraphics("ocean.surface", pipeline, layout))
    {
        return;
    }

    const auto &draws = ctx_local->getMainDrawContext().OceanSurfaces;
    if (draws.empty())
    {
        return;
    }

    VkDescriptorSet global_descriptor = ctx_local->getOrCreateSceneDataDescriptor();
    if (global_descriptor == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet ibl_set = VK_NULL_HANDLE;
    {
        VkImageView spec_view = ctx_local->ibl->specular().imageView ? ctx_local->ibl->specular().imageView
                                                                     : _fallbackIbl2D.imageView;
        VkImageView brdf_view = ctx_local->ibl->brdf().imageView ? ctx_local->ibl->brdf().imageView
                                                                 : _fallbackBrdf2D.imageView;
        VkBuffer sh_buffer = ctx_local->ibl->hasSH() ? ctx_local->ibl->shBuffer().buffer : VK_NULL_HANDLE;
        VkDeviceSize sh_size = sizeof(glm::vec4) * 9;

        AllocatedBuffer sh_zero{};
        if (sh_buffer == VK_NULL_HANDLE)
        {
            sh_zero = resource_manager->create_buffer(sh_size,
                                                      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                      VMA_MEMORY_USAGE_CPU_TO_GPU);
            std::memset(sh_zero.info.pMappedData, 0, static_cast<size_t>(sh_size));
            vmaFlushAllocation(device_manager->allocator(), sh_zero.allocation, 0, sh_size);
            sh_buffer = sh_zero.buffer;
            ctx_local->currentFrame->_deletionQueue.push_function([resource_manager, sh_zero]()
            {
                resource_manager->destroy_buffer(sh_zero);
            });
        }

        ibl_set = ctx_local->currentFrame->_frameDescriptors.allocate(device_manager->device(),
                                                                      ctx_local->ibl->descriptorLayout());
        DescriptorWriter ibl_writer;
        ibl_writer.write_image(0,
                               spec_view,
                               ctx_local->getSamplers()->defaultLinear(),
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        ibl_writer.write_image(1,
                               brdf_view,
                               ctx_local->getSamplers()->defaultLinear(),
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        ibl_writer.write_buffer(2, sh_buffer, sh_size, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        ibl_writer.update_set(device_manager->device(), ibl_set);
    }

    VkDescriptorSet atmosphere_set = VK_NULL_HANDLE;
    {
        VkImageView lut_view = transmittanceLut.valid() ? resources.image_view(transmittanceLut) : VK_NULL_HANDLE;
        if (lut_view == VK_NULL_HANDLE)
        {
            lut_view = _fallbackTransmittanceLut.imageView;
        }

        atmosphere_set = ctx_local->currentFrame->_frameDescriptors.allocate(device_manager->device(),
                                                                             _atmosphereSetLayout);
        DescriptorWriter atmosphere_writer;
        atmosphere_writer.write_image(0,
                                      lut_view,
                                      ctx_local->getSamplers()->linearClampEdge(),
                                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        atmosphere_writer.update_set(device_manager->device(), atmosphere_set);
    }

    glm::vec3 atmosphere_center_local{0.0f};
    float atmosphere_planet_radius = 0.0f;
    if (ctx_local->scene)
    {
        if (PlanetSystem *planets = ctx_local->scene->get_planet_system(); planets && planets->enabled())
        {
            const std::string &want_name = ctx_local->atmosphere.bodyName;
            const PlanetSystem::PlanetBody *picked = nullptr;
            double best_dist2 = 0.0;
            const WorldVec3 cam_world = ctx_local->scene->getMainCamera().position_world;
            for (const PlanetSystem::PlanetBody &body : planets->bodies())
            {
                if (!body.visible || body.radius_m <= 0.0)
                {
                    continue;
                }

                if (!want_name.empty())
                {
                    if (body.name == want_name)
                    {
                        picked = &body;
                        break;
                    }
                    continue;
                }

                const WorldVec3 delta_world = cam_world - body.center_world;
                const double dist2 = glm::dot(delta_world, delta_world);
                if (!picked || dist2 < best_dist2)
                {
                    picked = &body;
                    best_dist2 = dist2;
                }
            }

            if (picked)
            {
                atmosphere_center_local = world_to_local(picked->center_world, ctx_local->scene->get_world_origin());
                atmosphere_planet_radius = static_cast<float>(picked->radius_m);
            }
        }
    }

    const AtmosphereSettings &atmosphere = ctx_local->atmosphere;
    const float atmosphere_height = std::max(0.0f, atmosphere.atmosphereHeightM);
    const float atmosphere_radius = (ctx_local->enableAtmosphere && atmosphere_planet_radius > 0.0f && atmosphere_height > 0.0f)
        ? (atmosphere_planet_radius + atmosphere_height)
        : 0.0f;
    const glm::vec3 beta_rayleigh = (ctx_local->enableAtmosphere && vec3_finite(atmosphere.rayleighScattering))
        ? glm::max(atmosphere.rayleighScattering, glm::vec3(0.0f))
        : glm::vec3(0.0f);
    const glm::vec3 beta_mie = (ctx_local->enableAtmosphere && vec3_finite(atmosphere.mieScattering))
        ? glm::max(atmosphere.mieScattering, glm::vec3(0.0f))
        : glm::vec3(0.0f);
    const glm::vec3 absorption_color = vec3_finite(atmosphere.absorptionColor)
        ? glm::clamp(atmosphere.absorptionColor, glm::vec3(0.0f), glm::vec3(1.0f))
        : glm::vec3(1.0f);
    const glm::vec3 beta_absorption = absorption_color *
        ((ctx_local->enableAtmosphere && std::isfinite(atmosphere.absorptionStrength))
             ? std::max(0.0f, atmosphere.absorptionStrength)
             : 0.0f);
    const float atmosphere_intensity = ctx_local->enableAtmosphere ? std::max(0.0f, atmosphere.intensity) : 0.0f;

    VkExtent2D extent = ctx_local->getDrawExtent();
    VkViewport viewport{0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{{0, 0}, extent};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &global_descriptor, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 2, 1, &atmosphere_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 3, 1, &ibl_set, 0, nullptr);

    VkDescriptorSet last_material_set = VK_NULL_HANDLE;
    VkBuffer last_index_buffer = VK_NULL_HANDLE;

    for (const OceanRenderObject &draw : draws)
    {
        const RenderObject &surface = draw.surface;
        if (!surface.material || surface.material->materialSet == VK_NULL_HANDLE ||
            surface.indexBuffer == VK_NULL_HANDLE || surface.vertexBufferAddress == 0)
        {
            continue;
        }

        if (surface.material->materialSet != last_material_set)
        {
            last_material_set = surface.material->materialSet;
            vkCmdBindDescriptorSets(cmd,
                                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    layout,
                                    1,
                                    1,
                                    &last_material_set,
                                    0,
                                    nullptr);
            if (ctx_local->textures)
            {
                ctx_local->textures->markSetUsed(last_material_set, ctx_local->frameIndex);
            }
        }

        if (surface.indexBuffer != last_index_buffer)
        {
            last_index_buffer = surface.indexBuffer;
            vkCmdBindIndexBuffer(cmd, surface.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }

        OceanPushConstants push{};
        push.world_matrix = surface.transform;
        push.body_center_radius = glm::vec4(draw.body_center_local, draw.sea_level_radius);
        push.shell_params = glm::vec4(draw.shell_offset,
                                      atmosphere_radius > atmosphere_planet_radius ? 1.0f : 0.0f,
                                      0.0f,
                                      0.0f);
        push.atmosphere_center_radius = glm::vec4(atmosphere_center_local, atmosphere_planet_radius);
        push.atmosphere_params = glm::vec4(atmosphere_radius,
                                           std::max(1.0f, atmosphere.rayleighScaleHeightM),
                                           std::max(1.0f, atmosphere.mieScaleHeightM),
                                           atmosphere_intensity);
        push.beta_rayleigh = glm::vec4(beta_rayleigh, 0.0f);
        push.beta_mie = glm::vec4(beta_mie, 0.0f);
        push.beta_absorption = glm::vec4(beta_absorption, 0.0f);
        push.vertex_buffer = surface.vertexBufferAddress;
        vkCmdPushConstants(cmd,
                           layout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0,
                           sizeof(OceanPushConstants),
                           &push);

        vkCmdDrawIndexed(cmd, surface.indexCount, 1, surface.firstIndex, 0, 0);
        if (ctx_local->stats)
        {
            ctx_local->stats->drawcall_count++;
            ctx_local->stats->triangle_count += surface.indexCount / 3;
        }
    }
}

void OceanPass::cleanup()
{
    if (_context && _context->getResources())
    {
        if (_fallbackIbl2D.image)
        {
            _context->getResources()->destroy_image(_fallbackIbl2D);
        }
        if (_fallbackBrdf2D.image)
        {
            _context->getResources()->destroy_image(_fallbackBrdf2D);
        }
        if (_fallbackTransmittanceLut.image)
        {
            _context->getResources()->destroy_image(_fallbackTransmittanceLut);
        }
    }

    if (_context && _context->getDevice())
    {
        VkDevice device = _context->getDevice()->device();
        if (_materialSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _materialSetLayout, nullptr);
            _materialSetLayout = VK_NULL_HANDLE;
        }
        if (_atmosphereSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _atmosphereSetLayout, nullptr);
            _atmosphereSetLayout = VK_NULL_HANDLE;
        }
    }

    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("ocean.transmittance_lut");
        _context->pipelines->destroyComputePipeline("ocean.transmittance_lut");
    }
}
