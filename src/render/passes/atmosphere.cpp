#include "atmosphere.h"

#include "core/context.h"
#include "core/frame/resources.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/assets/manager.h"
#include "core/assets/ktx_loader.h"
#include "core/pipeline/manager.h"
#include "core/pipeline/sampler.h"
#include "core/world.h"

#include "render/graph/graph.h"
#include "render/graph/resources.h"
#include "render/pipelines.h"

#include "scene/vk_scene.h"
#include "scene/planet/planet_system.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>

#include <glm/gtc/packing.hpp>

namespace
{
    constexpr uint32_t k_transmittance_lut_width = 256;
    constexpr uint32_t k_transmittance_lut_height = 64;

    struct AtmospherePush
    {
        glm::vec4 planet_center_radius; // xyz: planet center (local), w: planet radius (m)
        glm::vec4 atmosphere_params;    // x: atmosphere radius (m), y: rayleigh H (m), z: mie H (m), w: mie g
        glm::vec4 beta_rayleigh;        // rgb: betaR (1/m), w: atmosphere intensity
        glm::vec4 beta_mie;             // rgb: betaM (1/m), w: absorption strength (1/m)
        glm::vec4 jitter_params;        // x: jitter strength (0..1), y: planet snap (m), z: time (sec), w: cloud overlay rotation (rad)
        glm::vec4 cloud_layer;          // x: base height (m), y: thickness (m), z: densityScale, w: coverage
        glm::vec4 cloud_params;         // x: noiseScale, y: detailScale, z: windSpeedMps, w: windAngleRad
        glm::ivec4 misc;                // x: view steps, y: packed absorption color (RGBA8), z: cloud steps, w: flags
    };

    struct AtmosphereLutPush
    {
        glm::vec4 radii_heights; // x: planet radius (m), y: atmosphere radius (m), z: rayleigh H (m), w: mie H (m)
        glm::ivec4 misc;         // x: integration steps, yzw: reserved
    };

    static_assert(sizeof(AtmospherePush) % 16 == 0);
    static_assert(sizeof(AtmospherePush) == 128);
    static_assert(sizeof(AtmosphereLutPush) % 16 == 0);

    static bool vec3_finite(const glm::vec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    static bool find_atmosphere_body(const EngineContext &ctx,
                                     const SceneManager &scene,
                                     const PlanetSystem &planets,
                                     glm::vec3 &out_center_local,
                                     float &out_radius_m)
    {
        const auto &bodies = planets.bodies();
        if (bodies.empty())
        {
            return false;
        }

        const std::string &want = ctx.atmosphere.bodyName;
        const PlanetSystem::PlanetBody *picked = nullptr;

        auto body_ok = [](const PlanetSystem::PlanetBody &b) {
            return b.visible && b.radius_m > 0.0;
        };

        if (!want.empty())
        {
            for (const PlanetSystem::PlanetBody &b : bodies)
            {
                if (b.name == want && body_ok(b))
                {
                    picked = &b;
                    break;
                }
            }
        }

        if (!picked)
        {
            const WorldVec3 camW = scene.getMainCamera().position_world;
            double best_d2 = 0.0;
            for (const PlanetSystem::PlanetBody &b : bodies)
            {
                if (!body_ok(b))
                {
                    continue;
                }
                const WorldVec3 d = camW - b.center_world;
                const double d2 = glm::dot(d, d);
                if (!picked || d2 < best_d2)
                {
                    picked = &b;
                    best_d2 = d2;
                }
            }
        }

        if (!picked)
        {
            return false;
        }

        const WorldVec3 originW = scene.get_world_origin();
        out_center_local = world_to_local(picked->center_world, originW);
        out_radius_m = static_cast<float>(picked->radius_m);
        return std::isfinite(out_radius_m) && out_radius_m > 0.0f;
    }
} // namespace

void AtmospherePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getResources() || !_context->getAssets())
    {
        return;
    }

    VkDevice device = _context->getDevice()->device();

    // Set 1 layout: HDR input + gbuffer position + transmittance LUT + cloud overlay texture.
    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // hdrInput
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // posTex
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // transmittanceLut
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER); // cloudOverlayTex
        _inputSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere.frag.spv");
    info.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set = 0
        _inputSetLayout                                         // set = 1
    };

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(AtmospherePush);
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

    _context->pipelines->createGraphicsPipeline("atmosphere", info);

    // Transmittance / optical-depth LUT compute pipeline (used to remove per-pixel sun raymarch).
    {
        ComputePipelineCreateInfo ci{};
        ci.shaderPath = _context->getAssets()->shaderPath("atmosphere_transmittance_lut.comp.spv");
        ci.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
        ci.pushConstantSize = sizeof(AtmosphereLutPush);
        ci.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("atmosphere.transmittance_lut", ci);
        _context->pipelines->createComputeInstance("atmosphere.transmittance_lut", "atmosphere.transmittance_lut");
    }

    // Cloud overlay texture (coverage map).
    if (ResourceManager *rm = _context->getResources())
    {
        // Loaded lazily again in register_graph() if the path changes at runtime.
        if (_cloudOverlayTex.image == VK_NULL_HANDLE)
        {
            _cloudOverlayLoadedPath = _context->planetClouds.overlayTexturePath;
            const std::string overlayAbs = _context->getAssets()->assetPath(_cloudOverlayLoadedPath);
            ktxutil::Ktx2D ktx{};
            if (!overlayAbs.empty() && ktxutil::load_ktx2_2d(overlayAbs.c_str(), ktx))
            {
                _cloudOverlayTex = rm->create_image_compressed_layers(
                    ktx.bytes.data(),
                    ktx.bytes.size(),
                    ktx.fmt,
                    ktx.mipLevels,
                    1,
                    ktx.copies,
                    VK_IMAGE_USAGE_SAMPLED_BIT);
            }
        }
    }
}

void AtmospherePass::cleanup()
{
    if (_context && _context->getResources())
    {
        ResourceManager *rm = _context->getResources();
        if (_cloudOverlayTex.image != VK_NULL_HANDLE)
        {
            rm->destroy_image(_cloudOverlayTex);
            _cloudOverlayTex = {};
        }
    }
    _cloudOverlayLoadedPath.clear();

    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("atmosphere.transmittance_lut");
        _context->pipelines->destroyComputePipeline("atmosphere.transmittance_lut");
    }

    if (_context && _context->getDevice() && _inputSetLayout)
    {
        vkDestroyDescriptorSetLayout(_context->getDevice()->device(), _inputSetLayout, nullptr);
        _inputSetLayout = VK_NULL_HANDLE;
    }
}

void AtmospherePass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

RGImageHandle AtmospherePass::register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos)
{
    if (!graph || !hdrInput.valid() || !gbufPos.valid())
    {
        return hdrInput;
    }
    if (!_context || !(_context->enableAtmosphere || _context->enablePlanetClouds))
    {
        return hdrInput;
    }

    // Transmittance / optical-depth LUT (Rayleigh + Mie), written by a small compute pass.
    RGImageDesc lutDesc{};
    lutDesc.name = "atmosphere.lut.transmittance";
    lutDesc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
    lutDesc.extent = VkExtent2D{k_transmittance_lut_width, k_transmittance_lut_height};
    lutDesc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    RGImageHandle transmittanceLut = graph->create_image(lutDesc);

    graph->add_pass(
        "AtmosphereLUT.Transmittance",
        RGPassType::Compute,
        [transmittanceLut](RGPassBuilder &builder, EngineContext *)
        {
            builder.write(transmittanceLut, RGImageUsage::ComputeWrite);
        },
        [this, transmittanceLut](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
        {
            EngineContext *ctxLocal = ctx ? ctx : _context;
            if (!ctxLocal || !ctxLocal->pipelines)
            {
                return;
            }

            VkImageView lutView = res.image_view(transmittanceLut);
            if (lutView == VK_NULL_HANDLE)
            {
                return;
            }

            ctxLocal->pipelines->setComputeInstanceStorageImage("atmosphere.transmittance_lut", 0, lutView);

            // Resolve active planet radius for LUT parameterization.
            glm::vec3 planet_center_local{0.0f, 0.0f, 0.0f};
            float planet_radius_m = 0.0f;
            if (ctxLocal->scene)
            {
                if (PlanetSystem *planets = ctxLocal->scene->get_planet_system(); planets && planets->enabled())
                {
                    (void)find_atmosphere_body(*ctxLocal, *ctxLocal->scene, *planets, planet_center_local, planet_radius_m);
                }
            }

            const AtmosphereSettings &s = ctxLocal->atmosphere;
            float atm_height = std::max(0.0f, s.atmosphereHeightM);
            float atm_radius = (planet_radius_m > 0.0f && atm_height > 0.0f) ? (planet_radius_m + atm_height) : 0.0f;
            float Hr = std::max(1.0f, s.rayleighScaleHeightM);
            float Hm = std::max(1.0f, s.mieScaleHeightM);

            int lutSteps = std::clamp(s.lightSteps, 2, 256);

            AtmosphereLutPush pc{};
            pc.radii_heights = glm::vec4(planet_radius_m, atm_radius, Hr, Hm);
            pc.misc = glm::ivec4(lutSteps, 0, 0, 0);

            ComputeDispatchInfo di = ComputeManager::createDispatch2D(k_transmittance_lut_width, k_transmittance_lut_height);
            di.pushConstants = &pc;
            di.pushConstantSize = sizeof(pc);

            ctxLocal->pipelines->dispatchComputeInstance(cmd, "atmosphere.transmittance_lut", di);
        });

    RGImageDesc desc{};
    desc.name = "hdr.atmosphere";
    desc.format = (_context && _context->getSwapchain()) ? _context->getSwapchain()->drawImage().imageFormat : VK_FORMAT_R32G32B32A32_SFLOAT;
    desc.extent = _context->getDrawExtent();
    desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    RGImageHandle hdrOutput = graph->create_image(desc);

    graph->add_pass(
        "Atmosphere",
        RGPassType::Graphics,
        [hdrInput, gbufPos, hdrOutput, transmittanceLut](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(hdrInput, RGImageUsage::SampledFragment);
            builder.read(gbufPos, RGImageUsage::SampledFragment);
            builder.read(transmittanceLut, RGImageUsage::SampledFragment);
            builder.write_color(hdrOutput, false /*load*/);
        },
        [this, hdrInput, gbufPos, transmittanceLut](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
        {
            draw_atmosphere(cmd, ctx, res, hdrInput, gbufPos, transmittanceLut);
        });

    return hdrOutput;
}

void AtmospherePass::draw_atmosphere(VkCommandBuffer cmd,
                                     EngineContext *context,
                                     const RGPassResources &resources,
                                     RGImageHandle hdrInput,
                                     RGImageHandle gbufPos,
                                     RGImageHandle transmittanceLut)
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
    VkImageView lutView = resources.image_view(transmittanceLut);
    if (hdrView == VK_NULL_HANDLE || posView == VK_NULL_HANDLE || lutView == VK_NULL_HANDLE) return;

    // Reload overlay texture on demand if the user changed the path.
    if (ctxLocal->enablePlanetClouds)
    {
        const std::string &wantRel = ctxLocal->planetClouds.overlayTexturePath;
        if (wantRel != _cloudOverlayLoadedPath)
        {
            if (_cloudOverlayTex.image != VK_NULL_HANDLE)
            {
                resourceManager->destroy_image(_cloudOverlayTex);
                _cloudOverlayTex = {};
            }

            _cloudOverlayLoadedPath = wantRel;

            const std::string wantAbs = ctxLocal->getAssets()->assetPath(wantRel);
            ktxutil::Ktx2D ktx{};
            if (!wantAbs.empty() && ktxutil::load_ktx2_2d(wantAbs.c_str(), ktx))
            {
                _cloudOverlayTex = resourceManager->create_image_compressed_layers(
                    ktx.bytes.data(),
                    ktx.bytes.size(),
                    ktx.fmt,
                    ktx.mipLevels,
                    1,
                    ktx.copies,
                    VK_IMAGE_USAGE_SAMPLED_BIT);
            }
        }
    }

    VkImageView overlayView = _cloudOverlayTex.imageView;
    if (AssetManager *assets = ctxLocal->getAssets())
    {
        if (overlayView == VK_NULL_HANDLE) overlayView = assets->fallbackBlackView();
    }
    if (overlayView == VK_NULL_HANDLE) return;

    // Global scene UBO (set = 0).
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
    {
        DescriptorWriter writer;
        writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        writer.update_set(deviceManager->device(), globalDescriptor);
    }

    // Input set (set = 1).
    VkDescriptorSet inputSet = ctxLocal->currentFrame->_frameDescriptors.allocate(deviceManager->device(), _inputSetLayout);
    {
        DescriptorWriter writer;
        writer.write_image(0, hdrView, ctxLocal->getSamplers()->defaultLinear(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(1, posView, ctxLocal->getSamplers()->defaultNearest(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(2, lutView, ctxLocal->getSamplers()->linearClampEdge(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.write_image(3, overlayView, ctxLocal->getSamplers()->linearRepeatClampEdge(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        writer.update_set(deviceManager->device(), inputSet);
    }

    if (!pipelineManager->getGraphics("atmosphere", _pipeline, _pipelineLayout))
    {
        return;
    }

    // Resolve the active planet for atmosphere parameters (or disable if not found).
    glm::vec3 planet_center_local{0.0f, 0.0f, 0.0f};
    float planet_radius_m = 0.0f;
    if (ctxLocal->scene)
    {
        if (PlanetSystem *planets = ctxLocal->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctxLocal, *ctxLocal->scene, *planets, planet_center_local, planet_radius_m);
        }
    }

    const AtmosphereSettings &s = ctxLocal->atmosphere;
    const PlanetCloudSettings &c = ctxLocal->planetClouds;

    const float dt_sec = (ctxLocal->scene) ? std::clamp(ctxLocal->scene->getDeltaTime(), 0.0f, 0.1f) : 0.0f;
    if (std::isfinite(dt_sec) && dt_sec > 0.0f)
    {
        _time_sec += dt_sec;
        if (!std::isfinite(_time_sec))
        {
            _time_sec = 0.0f;
        }
        else if (_time_sec > 1.0e6f)
        {
            _time_sec = std::fmod(_time_sec, 1.0e6f);
        }
    }

    float atm_height = std::max(0.0f, s.atmosphereHeightM);
    const bool atmosphereEnabled = ctxLocal->enableAtmosphere;
    float atm_radius = (atmosphereEnabled && planet_radius_m > 0.0f && atm_height > 0.0f) ? (planet_radius_m + atm_height) : 0.0f;

    float Hr = std::max(1.0f, s.rayleighScaleHeightM);
    float Hm = std::max(1.0f, s.mieScaleHeightM);

    glm::vec3 betaR = atmosphereEnabled && vec3_finite(s.rayleighScattering) ? glm::max(s.rayleighScattering, glm::vec3(0.0f)) : glm::vec3(0.0f);
    glm::vec3 betaM = atmosphereEnabled && vec3_finite(s.mieScattering) ? glm::max(s.mieScattering, glm::vec3(0.0f)) : glm::vec3(0.0f);

    float mieG = std::clamp(s.mieG, -0.99f, 0.99f);
    float intensity = atmosphereEnabled ? std::max(0.0f, s.intensity) : 0.0f;
    float jitterStrength = std::clamp(s.jitterStrength, 0.0f, 1.0f);
    float planetSnapM = std::max(0.0f, s.planetSurfaceSnapM);

    int viewSteps = std::clamp(s.viewSteps, 4, 64);

    const bool cloudsEnabled = ctxLocal->enablePlanetClouds && (_cloudOverlayTex.imageView != VK_NULL_HANDLE);
    float cloudBaseM = std::max(0.0f, c.baseHeightM);
    float cloudThicknessM = std::max(0.0f, c.thicknessM);
    float cloudDensityScale = std::max(0.0f, c.densityScale);
    float cloudCoverage = std::clamp(c.coverage, 0.0f, 0.999f);
    float cloudOverlayRot = cloudsEnabled ? c.overlayRotationRad : 0.0f;
    float cloudNoiseScale = std::max(0.001f, c.noiseScale);
    float cloudDetailScale = std::max(0.001f, c.detailScale);
    float cloudWindSpeed = c.windSpeed;
    float cloudWindAngle = c.windAngleRad;
    int cloudSteps = std::clamp(c.cloudSteps, 4, 128);

    uint32_t flags = 0u;
    if (atmosphereEnabled)
    {
        flags |= 1u;
    }
    if (cloudsEnabled)
    {
        flags |= 2u;
    }
    if (cloudsEnabled && c.overlayFlipV)
    {
        flags |= 4u;
    }

    glm::vec3 absorptionColor = vec3_finite(s.absorptionColor)
        ? glm::clamp(s.absorptionColor, glm::vec3(0.0f), glm::vec3(1.0f))
        : glm::vec3(1.0f);
    float absorptionStrength = (atmosphereEnabled && std::isfinite(s.absorptionStrength))
        ? std::max(0.0f, s.absorptionStrength)
        : 0.0f;

    const uint32_t packed_absorption_color = glm::packUnorm4x8(glm::vec4(absorptionColor, 1.0f));
    const int packed_absorption_color_bits = std::bit_cast<int32_t>(packed_absorption_color);

    AtmospherePush pc{};
    pc.planet_center_radius = glm::vec4(planet_center_local, planet_radius_m);
    pc.atmosphere_params = glm::vec4(atm_radius, Hr, Hm, mieG);
    pc.beta_rayleigh = glm::vec4(betaR, intensity);
    pc.beta_mie = glm::vec4(betaM, absorptionStrength);
    pc.jitter_params = glm::vec4(jitterStrength, planetSnapM, _time_sec, cloudOverlayRot);
    pc.cloud_layer = glm::vec4(cloudBaseM, cloudThicknessM, cloudDensityScale, cloudCoverage);
    pc.cloud_params = glm::vec4(cloudNoiseScale, cloudDetailScale, cloudWindSpeed, cloudWindAngle);
    pc.misc = glm::ivec4(viewSteps, packed_absorption_color_bits, cloudSteps, static_cast<int>(flags));

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalDescriptor, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 1, 1, &inputSet, 0, nullptr);
    vkCmdPushConstants(cmd, _pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport vp{0.f, 0.f, float(extent.width), float(extent.height), 0.f, 1.f};
    VkRect2D sc{{0,0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
