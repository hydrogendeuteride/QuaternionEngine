#include "atmosphere.h"

#include "core/assets/ktx_loader.h"
#include "core/assets/manager.h"
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

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>

#include <glm/gtc/packing.hpp>

namespace
{
    constexpr uint32_t k_transmittance_lut_width = 256;
    constexpr uint32_t k_transmittance_lut_height = 64;
    constexpr uint32_t k_misc_flags_mask = 0xFFu;
    constexpr uint32_t k_misc_noise_blend_shift = 8u;
    constexpr uint32_t k_misc_detail_erode_shift = 16u;
    constexpr uint32_t k_misc_jitter_frame_shift = 24u;
    constexpr uint32_t k_flag_cloud_noise_3d = 8u;
    constexpr uint32_t k_flag_jitter_blue_noise = 16u;

    struct AtmospherePush
    {
        glm::vec4 planet_center_radius;
        glm::vec4 atmosphere_params;
        glm::vec4 beta_rayleigh;
        glm::vec4 beta_mie;
        glm::vec4 jitter_params;
        glm::vec4 cloud_layer;
        glm::vec4 cloud_params;
        glm::vec4 cloud_color;
        glm::ivec4 misc;
    };

    struct AtmosphereLutPush
    {
        glm::vec4 radii_heights;
        glm::ivec4 misc;
    };

    struct CloudTemporalPush
    {
        glm::mat4 previous_view_proj;
        glm::vec4 origin_delta_blend;
        glm::vec4 viewport_params;
        glm::ivec4 misc;
    };

    static_assert(sizeof(AtmospherePush) == 144);
    static_assert(sizeof(CloudTemporalPush) == 112);

    static bool vec3_finite(const glm::vec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    static VkExtent2D half_extent(VkExtent2D extent)
    {
        extent.width = std::max(1u, (extent.width + 1u) / 2u);
        extent.height = std::max(1u, (extent.height + 1u) / 2u);
        return extent;
    }

    static bool nearly_equal(float a, float b, float eps = 1.0e-3f)
    {
        return std::abs(a - b) <= eps;
    }

    static bool nearly_equal_vec4(const glm::vec4 &a, const glm::vec4 &b, float eps = 1.0e-3f)
    {
        return nearly_equal(a.x, b.x, eps) && nearly_equal(a.y, b.y, eps) &&
               nearly_equal(a.z, b.z, eps) && nearly_equal(a.w, b.w, eps);
    }

    static bool nearly_equal_mat4(const glm::mat4 &a, const glm::mat4 &b, float eps = 1.0e-5f)
    {
        for (int c = 0; c < 4; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                if (!nearly_equal(a[c][r], b[c][r], eps))
                {
                    return false;
                }
            }
        }
        return true;
    }

    static void set_fullscreen_viewport(VkCommandBuffer cmd, VkExtent2D extent)
    {
        VkViewport vp{0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f};
        VkRect2D sc{{0, 0}, extent};
        vkCmdSetViewport(cmd, 0, 1, &vp);
        vkCmdSetScissor(cmd, 0, 1, &sc);
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

        auto body_ok = [](const PlanetSystem::PlanetBody &body) {
            return body.visible && body.radius_m > 0.0;
        };

        if (!want.empty())
        {
            for (const PlanetSystem::PlanetBody &body : bodies)
            {
                if (body.name == want && body_ok(body))
                {
                    picked = &body;
                    break;
                }
            }
        }

        if (!picked)
        {
            const WorldVec3 cam_world = scene.getMainCamera().position_world;
            double best_d2 = 0.0;
            for (const PlanetSystem::PlanetBody &body : bodies)
            {
                if (!body_ok(body))
                {
                    continue;
                }

                const WorldVec3 delta_world = cam_world - body.center_world;
                const double dist2 = glm::dot(delta_world, delta_world);
                if (!picked || dist2 < best_d2)
                {
                    picked = &body;
                    best_d2 = dist2;
                }
            }
        }

        if (!picked)
        {
            return false;
        }

        out_center_local = world_to_local(picked->center_world, scene.get_world_origin());
        out_radius_m = static_cast<float>(picked->radius_m);
        return std::isfinite(out_radius_m) && out_radius_m > 0.0f;
    }

    static AtmospherePush build_atmosphere_push(const EngineContext &ctx,
                                                const glm::vec3 &planet_center_local,
                                                float planet_radius_m,
                                                bool jitter_noise_ready,
                                                bool cloud_overlay_ready,
                                                bool cloud_noise_ready,
                                                bool cloud_noise_3d_ready)
    {
        const AtmosphereSettings &s = ctx.atmosphere;
        const PlanetCloudSettings &c = ctx.planetClouds;

        const bool atmosphere_enabled = ctx.enableAtmosphere;
        const bool clouds_enabled = ctx.enablePlanetClouds && cloud_overlay_ready;
        const bool cloud_noise_available = cloud_noise_ready;
        const bool cloud_detail_noise_available = cloud_noise_ready || cloud_noise_3d_ready;

        const float atm_height = std::max(0.0f, s.atmosphereHeightM);
        const float atm_radius = (atmosphere_enabled && planet_radius_m > 0.0f && atm_height > 0.0f)
            ? (planet_radius_m + atm_height)
            : 0.0f;
        const float rayleigh_height = std::max(1.0f, s.rayleighScaleHeightM);
        const float mie_height = std::max(1.0f, s.mieScaleHeightM);

        const glm::vec3 beta_rayleigh = atmosphere_enabled && vec3_finite(s.rayleighScattering)
            ? glm::max(s.rayleighScattering, glm::vec3(0.0f))
            : glm::vec3(0.0f);
        const glm::vec3 beta_mie = atmosphere_enabled && vec3_finite(s.mieScattering)
            ? glm::max(s.mieScattering, glm::vec3(0.0f))
            : glm::vec3(0.0f);

        const float mie_g = std::clamp(s.mieG, -0.99f, 0.99f);
        const float intensity = atmosphere_enabled ? std::max(0.0f, s.intensity) : 0.0f;
        const float jitter_strength = std::clamp(s.jitterStrength, 0.0f, 1.0f);
        const float planet_snap_m = std::max(0.0f, s.planetSurfaceSnapM);
        const int view_steps = std::clamp(s.viewSteps, 4, 64);

        const float cloud_base_m = std::max(0.0f, c.baseHeightM);
        const float cloud_thickness_m = std::max(0.0f, c.thicknessM);
        const float cloud_density_scale = std::max(0.0f, c.densityScale);
        const glm::vec3 cloud_color = vec3_finite(c.color)
            ? glm::clamp(c.color, glm::vec3(0.0f), glm::vec3(1.0f))
            : glm::vec3(1.0f);
        const float cloud_coverage = std::clamp(c.coverage, 0.0f, 0.999f);
        const float cloud_overlay_rot = clouds_enabled ? c.overlayRotationRad : 0.0f;
        const float cloud_overlay_sin = std::sin(cloud_overlay_rot);
        const float cloud_overlay_cos = std::cos(cloud_overlay_rot);
        const float cloud_noise_scale = std::max(0.001f, c.noiseScale);
        const float cloud_detail_scale = std::max(0.001f, c.detailScale);
        const float cloud_noise_blend = (clouds_enabled && cloud_noise_available) ? std::clamp(c.noiseBlend, 0.0f, 1.0f) : 0.0f;
        const float cloud_detail_erode = (clouds_enabled && cloud_detail_noise_available) ? std::clamp(c.detailErode, 0.0f, 1.0f) : 0.0f;
        const float cloud_wind_speed = c.windSpeed;
        const float cloud_wind_angle = c.windAngleRad;
        const int cloud_steps = std::clamp(c.cloudSteps, 4, 128);

        uint32_t flags = 0u;
        if (atmosphere_enabled)
        {
            flags |= 1u;
        }
        if (clouds_enabled)
        {
            flags |= 2u;
        }
        if (clouds_enabled && c.overlayFlipV)
        {
            flags |= 4u;
        }
        if (clouds_enabled && cloud_noise_3d_ready)
        {
            flags |= k_flag_cloud_noise_3d;
        }
        if (jitter_noise_ready)
        {
            flags |= k_flag_jitter_blue_noise;
        }

        const glm::vec3 absorption_color = vec3_finite(s.absorptionColor)
            ? glm::clamp(s.absorptionColor, glm::vec3(0.0f), glm::vec3(1.0f))
            : glm::vec3(1.0f);
        const float absorption_strength = (atmosphere_enabled && std::isfinite(s.absorptionStrength))
            ? std::max(0.0f, s.absorptionStrength)
            : 0.0f;

        const uint32_t packed_absorption_color = glm::packUnorm4x8(glm::vec4(absorption_color, 1.0f));
        const int packed_absorption_color_bits = std::bit_cast<int32_t>(packed_absorption_color);
        const uint32_t packed_noise_blend = static_cast<uint32_t>(std::lround(cloud_noise_blend * 255.0f));
        const uint32_t packed_detail_erode = static_cast<uint32_t>(std::lround(cloud_detail_erode * 255.0f));
        const uint32_t packed_jitter_frame = ctx.frameIndex & 0xFFu;
        uint32_t packed_misc_w = (flags & k_misc_flags_mask);
        packed_misc_w |= ((packed_noise_blend & 0xFFu) << k_misc_noise_blend_shift);
        packed_misc_w |= ((packed_detail_erode & 0xFFu) << k_misc_detail_erode_shift);
        packed_misc_w |= ((packed_jitter_frame & 0xFFu) << k_misc_jitter_frame_shift);

        AtmospherePush push{};
        push.planet_center_radius = glm::vec4(planet_center_local, planet_radius_m);
        push.atmosphere_params = glm::vec4(atm_radius, rayleigh_height, mie_height, mie_g);
        push.beta_rayleigh = glm::vec4(beta_rayleigh, intensity);
        push.beta_mie = glm::vec4(beta_mie, absorption_strength);
        push.jitter_params = glm::vec4(jitter_strength, planet_snap_m, cloud_overlay_sin, cloud_overlay_cos);
        push.cloud_layer = glm::vec4(cloud_base_m, cloud_thickness_m, cloud_density_scale, cloud_coverage);
        push.cloud_params = glm::vec4(cloud_noise_scale, cloud_detail_scale, cloud_wind_speed, cloud_wind_angle);
        push.cloud_color = glm::vec4(cloud_color, 1.0f);
        push.misc = glm::ivec4(view_steps, packed_absorption_color_bits, cloud_steps, std::bit_cast<int32_t>(packed_misc_w));
        return push;
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

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _lowResSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _temporalSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _upscaleSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _compositeSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    const std::string fullscreen_vertex = _context->getAssets()->shaderPath("atmosphere.vert.spv");

    {
        std::array<uint8_t, 8> white3d{};
        white3d.fill(0xFFu);
        _cloudNoiseFallback3D = _context->getResources()->create_image(
            white3d.data(),
            VkExtent3D{2, 2, 2},
            VK_FORMAT_R8_UNORM,
            VK_IMAGE_USAGE_SAMPLED_BIT,
            false);
    }

    const VkFormat hdr_format = _context->getSwapchain()
        ? _context->getSwapchain()->drawImage().imageFormat
        : VK_FORMAT_R16G16B16A16_SFLOAT;
    auto configure_cloud_outputs = [](PipelineBuilder &builder)
    {
        std::array<VkFormat, 2> cloud_formats{VK_FORMAT_R16G16B16A16_SFLOAT, VK_FORMAT_R32G32_SFLOAT};
        builder.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        builder.set_polygon_mode(VK_POLYGON_MODE_FILL);
        builder.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        builder.set_multisampling_none();
        builder.disable_depthtest();
        builder.disable_blending();
        builder.set_color_attachment_formats(std::span<VkFormat>(cloud_formats));
    };

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere_cloud_lowres.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _lowResSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(AtmospherePush);
        info.pushConstants = {push_range};
        info.configure = configure_cloud_outputs;
        _context->pipelines->createGraphicsPipeline("atmosphere.cloud_lowres", info);
    }

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere_cloud_temporal.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _temporalSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(CloudTemporalPush);
        info.pushConstants = {push_range};
        info.configure = configure_cloud_outputs;
        _context->pipelines->createGraphicsPipeline("atmosphere.cloud_temporal", info);
    }

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere_cloud_upscale.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _upscaleSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(AtmospherePush);
        info.pushConstants = {push_range};
        info.configure = configure_cloud_outputs;
        _context->pipelines->createGraphicsPipeline("atmosphere.cloud_upscale", info);
    }

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _compositeSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(AtmospherePush);
        info.pushConstants = {push_range};
        info.configure = [hdr_format](PipelineBuilder &builder)
        {
            builder.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
            builder.set_polygon_mode(VK_POLYGON_MODE_FILL);
            builder.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
            builder.set_multisampling_none();
            builder.disable_depthtest();
            builder.disable_blending();
            builder.set_color_attachment_format(hdr_format);
        };
        _context->pipelines->createGraphicsPipeline("atmosphere.composite", info);
    }

    {
        ComputePipelineCreateInfo info{};
        info.shaderPath = _context->getAssets()->shaderPath("atmosphere_transmittance_lut.comp.spv");
        info.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
        info.pushConstantSize = sizeof(AtmosphereLutPush);
        info.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("atmosphere.transmittance_lut", info);
        _context->pipelines->createComputeInstance("atmosphere.transmittance_lut", "atmosphere.transmittance_lut");
    }

    ensure_cloud_textures(_context);
}

void AtmospherePass::cleanup()
{
    release_history_images();

    if (_context && _context->getResources())
    {
        ResourceManager *resources = _context->getResources();
        if (_cloudOverlayTex.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudOverlayTex);
            _cloudOverlayTex = {};
        }
        if (_cloudNoiseTex.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudNoiseTex);
            _cloudNoiseTex = {};
        }
        if (_jitterNoiseTex.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_jitterNoiseTex);
            _jitterNoiseTex = {};
        }
        if (_cloudNoiseTex3D.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudNoiseTex3D);
            _cloudNoiseTex3D = {};
        }
        if (_cloudNoiseFallback3D.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudNoiseFallback3D);
            _cloudNoiseFallback3D = {};
        }
    }

    _cloudOverlayLoadedPath.clear();
    _cloudNoiseLoadedPath.clear();
    _jitterNoiseLoadedPath.clear();
    _cloudNoise3DLoadedPath.clear();
    _historySnapshot = {};
    _historyValid = false;

    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("atmosphere.transmittance_lut");
        _context->pipelines->destroyComputePipeline("atmosphere.transmittance_lut");
    }

    if (_context && _context->getDevice())
    {
        VkDevice device = _context->getDevice()->device();
        if (_lowResSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _lowResSetLayout, nullptr);
            _lowResSetLayout = VK_NULL_HANDLE;
        }
        if (_temporalSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _temporalSetLayout, nullptr);
            _temporalSetLayout = VK_NULL_HANDLE;
        }
        if (_upscaleSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _upscaleSetLayout, nullptr);
            _upscaleSetLayout = VK_NULL_HANDLE;
        }
        if (_compositeSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _compositeSetLayout, nullptr);
            _compositeSetLayout = VK_NULL_HANDLE;
        }
    }
}

void AtmospherePass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void AtmospherePass::preload_cloud_textures()
{
    ensure_cloud_textures(_context);
}

void AtmospherePass::ensure_cloud_textures(EngineContext *context)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->getResources() || !ctx->getAssets())
    {
        return;
    }

    ResourceManager *resources = ctx->getResources();

    auto reload_cloud_tex_2d = [&](AllocatedImage &dst, std::string &loaded_path, const std::string &wanted_path)
    {
        if (wanted_path == loaded_path)
        {
            return;
        }

        if (dst.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(dst);
            dst = {};
        }

        loaded_path = wanted_path;
        if (wanted_path.empty())
        {
            return;
        }

        const std::string absolute_path = ctx->getAssets()->assetPath(wanted_path);
        ktxutil::Ktx2D ktx{};
        if (!absolute_path.empty() && ktxutil::load_ktx2_2d(absolute_path.c_str(), ktx))
        {
            dst = resources->create_image_compressed_layers(
                ktx.bytes.data(),
                ktx.bytes.size(),
                ktx.fmt,
                ktx.mipLevels,
                1,
                ktx.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    };

    auto reload_cloud_tex_3d = [&](AllocatedImage &dst, std::string &loaded_path, const std::string &wanted_path)
    {
        if (wanted_path == loaded_path)
        {
            return;
        }

        if (dst.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(dst);
            dst = {};
        }

        loaded_path = wanted_path;
        if (wanted_path.empty())
        {
            return;
        }

        const std::string absolute_path = ctx->getAssets()->assetPath(wanted_path);
        ktxutil::Ktx3D ktx{};
        if (!absolute_path.empty() && ktxutil::load_ktx2_3d(absolute_path.c_str(), ktx))
        {
            dst = resources->create_image_compressed_layers(
                ktx.bytes.data(),
                ktx.bytes.size(),
                ktx.fmt,
                ktx.mipLevels,
                1,
                ktx.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    };

    auto reload_jitter_tex_3d = [&](AllocatedImage &dst, std::string &loaded_path, const std::string &wanted_path)
    {
        if (wanted_path == loaded_path)
        {
            return;
        }

        if (dst.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(dst);
            dst = {};
        }

        loaded_path = wanted_path;
        if (wanted_path.empty())
        {
            return;
        }

        const std::string absolute_path = ctx->getAssets()->assetPath(wanted_path);
        ktxutil::Ktx3D ktx{};
        if (!absolute_path.empty() && ktxutil::load_ktx2_3d(absolute_path.c_str(), ktx))
        {
            dst = resources->create_image_compressed_layers(
                ktx.bytes.data(),
                ktx.bytes.size(),
                ktx.fmt,
                ktx.mipLevels,
                1,
                ktx.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    };

    reload_jitter_tex_3d(_jitterNoiseTex, _jitterNoiseLoadedPath, ctx->atmosphere.jitterTexturePath);
    reload_cloud_tex_2d(_cloudOverlayTex, _cloudOverlayLoadedPath, ctx->planetClouds.overlayTexturePath);
    reload_cloud_tex_2d(_cloudNoiseTex, _cloudNoiseLoadedPath, ctx->planetClouds.noiseTexturePath);
    reload_cloud_tex_3d(_cloudNoiseTex3D, _cloudNoise3DLoadedPath, ctx->planetClouds.noiseTexture3DPath);
}

void AtmospherePass::release_history_images()
{
    if (!_context || !_context->getResources())
    {
        return;
    }

    ResourceManager *resources = _context->getResources();
    for (AllocatedImage &image : _cloudLightingHistory)
    {
        if (image.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(image);
        }
        image = {};
    }
    for (AllocatedImage &image : _cloudSegmentHistory)
    {
        if (image.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(image);
        }
        image = {};
    }

    _cloudLightingHistoryState = {};
    _cloudSegmentHistoryState = {};
    _historyActiveIndex = 0;
    _historyValid = false;
}

void AtmospherePass::ensure_history_images(VkExtent2D extent)
{
    if (!_context || !_context->getResources())
    {
        return;
    }

    const bool need_recreate = [&]() {
        for (uint32_t i = 0; i < 2; ++i)
        {
            if (_cloudLightingHistory[i].image == VK_NULL_HANDLE || _cloudSegmentHistory[i].image == VK_NULL_HANDLE)
            {
                return true;
            }
            if (_cloudLightingHistory[i].imageExtent.width != extent.width ||
                _cloudLightingHistory[i].imageExtent.height != extent.height ||
                _cloudSegmentHistory[i].imageExtent.width != extent.width ||
                _cloudSegmentHistory[i].imageExtent.height != extent.height)
            {
                return true;
            }
        }
        return false;
    }();

    if (!need_recreate)
    {
        return;
    }

    release_history_images();

    ResourceManager *resources = _context->getResources();
    const VkExtent3D image_extent{extent.width, extent.height, 1u};
    const VkImageUsageFlags usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

    for (uint32_t i = 0; i < 2; ++i)
    {
        _cloudLightingHistory[i] = resources->create_image(image_extent, VK_FORMAT_R16G16B16A16_SFLOAT, usage);
        _cloudSegmentHistory[i] = resources->create_image(image_extent, VK_FORMAT_R32G32_SFLOAT, usage);
    }
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

    ensure_cloud_textures(_context);

    RGImageDesc lut_desc{};
    lut_desc.name = "atmosphere.lut.transmittance";
    lut_desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
    lut_desc.extent = VkExtent2D{k_transmittance_lut_width, k_transmittance_lut_height};
    lut_desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    RGImageHandle transmittance_lut = graph->create_image(lut_desc);

    graph->add_pass(
        "AtmosphereLUT.Transmittance",
        RGPassType::Compute,
        [transmittance_lut](RGPassBuilder &builder, EngineContext *)
        {
            builder.write(transmittance_lut, RGImageUsage::ComputeWrite);
        },
        [this, transmittance_lut](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
        {
            EngineContext *ctx_local = ctx ? ctx : _context;
            if (!ctx_local || !ctx_local->pipelines)
            {
                return;
            }

            VkImageView lut_view = resources.image_view(transmittance_lut);
            if (lut_view == VK_NULL_HANDLE)
            {
                return;
            }

            ctx_local->pipelines->setComputeInstanceStorageImage("atmosphere.transmittance_lut", 0, lut_view);

            glm::vec3 planet_center_local{0.0f};
            float planet_radius_m = 0.0f;
            if (ctx_local->scene)
            {
                if (PlanetSystem *planets = ctx_local->scene->get_planet_system(); planets && planets->enabled())
                {
                    (void)find_atmosphere_body(*ctx_local, *ctx_local->scene, *planets, planet_center_local, planet_radius_m);
                }
            }

            const AtmosphereSettings &settings = ctx_local->atmosphere;
            const float atmosphere_height = std::max(0.0f, settings.atmosphereHeightM);
            const float atmosphere_radius = (planet_radius_m > 0.0f && atmosphere_height > 0.0f)
                ? (planet_radius_m + atmosphere_height)
                : 0.0f;

            AtmosphereLutPush push{};
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

            ctx_local->pipelines->dispatchComputeInstance(cmd, "atmosphere.transmittance_lut", dispatch);
        });

    glm::vec3 planet_center_local{0.0f};
    float planet_radius_m = 0.0f;
    if (_context->scene)
    {
        if (PlanetSystem *planets = _context->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*_context, *_context->scene, *planets, planet_center_local, planet_radius_m);
        }
    }

    const bool cloud_overlay_ready = (_cloudOverlayTex.imageView != VK_NULL_HANDLE);
    const VkExtent2D draw_extent = _context->getDrawExtent();
    const VkExtent2D low_extent = half_extent(draw_extent);
    const bool run_cloud_pipeline = _context->enablePlanetClouds && cloud_overlay_ready;

    if (run_cloud_pipeline)
    {
        ensure_history_images(low_extent);
    }

    HistorySnapshot next_snapshot{};
    next_snapshot.valid = run_cloud_pipeline;
    next_snapshot.drawExtent = draw_extent;
    next_snapshot.lowExtent = low_extent;
    next_snapshot.originRevision = _context->origin_revision;
    next_snapshot.projection = _context->getSceneData().proj;
    next_snapshot.planetCenterRadius = glm::vec4(planet_center_local, planet_radius_m);
    next_snapshot.sunDirection = _context->getSceneData().sunlightDirection;
    next_snapshot.sunColor = _context->getSceneData().sunlightColor;
    next_snapshot.ambientColor = _context->getSceneData().ambientColor;
    next_snapshot.bodyName = _context->atmosphere.bodyName;
    next_snapshot.jitterPath = _context->atmosphere.jitterTexturePath;
    next_snapshot.overlayPath = _context->planetClouds.overlayTexturePath;
    next_snapshot.noisePath = _context->planetClouds.noiseTexturePath;
    next_snapshot.noise3DPath = _context->planetClouds.noiseTexture3DPath;
    next_snapshot.cloudBaseM = _context->planetClouds.baseHeightM;
    next_snapshot.cloudThicknessM = _context->planetClouds.thicknessM;
    next_snapshot.cloudDensityScale = _context->planetClouds.densityScale;
    next_snapshot.cloudColor = glm::vec4(_context->planetClouds.color, 1.0f);
    next_snapshot.cloudCoverage = _context->planetClouds.coverage;
    next_snapshot.cloudNoiseScale = _context->planetClouds.noiseScale;
    next_snapshot.cloudDetailScale = _context->planetClouds.detailScale;
    next_snapshot.cloudNoiseBlend = _context->planetClouds.noiseBlend;
    next_snapshot.cloudDetailErode = _context->planetClouds.detailErode;
    next_snapshot.cloudWindSpeed = _context->planetClouds.windSpeed;
    next_snapshot.cloudWindAngle = _context->planetClouds.windAngleRad;
    next_snapshot.cloudOverlayRotation = _context->planetClouds.overlayRotationRad;
    next_snapshot.cloudOverlayFlipV = _context->planetClouds.overlayFlipV;

    bool history_reset = !_historyValid || !_historySnapshot.valid || !run_cloud_pipeline;
    if (!history_reset)
    {
        history_reset = _historySnapshot.drawExtent.width != next_snapshot.drawExtent.width ||
                        _historySnapshot.drawExtent.height != next_snapshot.drawExtent.height ||
                        _historySnapshot.lowExtent.width != next_snapshot.lowExtent.width ||
                        _historySnapshot.lowExtent.height != next_snapshot.lowExtent.height ||
                        _historySnapshot.originRevision != next_snapshot.originRevision ||
                        !nearly_equal_mat4(_historySnapshot.projection, next_snapshot.projection) ||
                        !nearly_equal_vec4(_historySnapshot.planetCenterRadius, next_snapshot.planetCenterRadius, 1.0e-2f) ||
                        !nearly_equal_vec4(_historySnapshot.sunDirection, next_snapshot.sunDirection, 1.0e-3f) ||
                        !nearly_equal_vec4(_historySnapshot.sunColor, next_snapshot.sunColor, 2.0e-2f) ||
                        !nearly_equal_vec4(_historySnapshot.ambientColor, next_snapshot.ambientColor, 2.0e-2f) ||
                        _historySnapshot.bodyName != next_snapshot.bodyName ||
                        _historySnapshot.jitterPath != next_snapshot.jitterPath ||
                        _historySnapshot.overlayPath != next_snapshot.overlayPath ||
                        _historySnapshot.noisePath != next_snapshot.noisePath ||
                        _historySnapshot.noise3DPath != next_snapshot.noise3DPath ||
                        !nearly_equal(_historySnapshot.cloudBaseM, next_snapshot.cloudBaseM) ||
                        !nearly_equal(_historySnapshot.cloudThicknessM, next_snapshot.cloudThicknessM) ||
                        !nearly_equal(_historySnapshot.cloudDensityScale, next_snapshot.cloudDensityScale) ||
                        !nearly_equal_vec4(_historySnapshot.cloudColor, next_snapshot.cloudColor, 2.0e-3f) ||
                        !nearly_equal(_historySnapshot.cloudCoverage, next_snapshot.cloudCoverage) ||
                        !nearly_equal(_historySnapshot.cloudNoiseScale, next_snapshot.cloudNoiseScale) ||
                        !nearly_equal(_historySnapshot.cloudDetailScale, next_snapshot.cloudDetailScale) ||
                        !nearly_equal(_historySnapshot.cloudNoiseBlend, next_snapshot.cloudNoiseBlend) ||
                        !nearly_equal(_historySnapshot.cloudDetailErode, next_snapshot.cloudDetailErode) ||
                        !nearly_equal(_historySnapshot.cloudWindSpeed, next_snapshot.cloudWindSpeed) ||
                        !nearly_equal(_historySnapshot.cloudWindAngle, next_snapshot.cloudWindAngle) ||
                        !nearly_equal(_historySnapshot.cloudOverlayRotation, next_snapshot.cloudOverlayRotation) ||
                        _historySnapshot.cloudOverlayFlipV != next_snapshot.cloudOverlayFlipV;
    }

    const glm::dvec3 origin_delta_world = _context->origin_world - _previousOriginWorld;
    const glm::vec3 origin_delta_local = glm::vec3(origin_delta_world);
    const glm::mat4 previous_view_proj = _previousViewProj;

    RGImageHandle cloud_lighting_resolved{};
    RGImageHandle cloud_segment_resolved{};

    if (run_cloud_pipeline)
    {
        const uint32_t previous_index = _historyActiveIndex;
        const uint32_t next_index = _historyActiveIndex ^ 1u;

        auto import_history = [&](const char *name,
                                  const AllocatedImage &image,
                                  const HistoryImageState &state) -> RGImageHandle
        {
            if (image.image == VK_NULL_HANDLE || image.imageView == VK_NULL_HANDLE)
            {
                return {};
            }

            RGImportedImageDesc desc{};
            desc.name = name;
            desc.image = image.image;
            desc.imageView = image.imageView;
            desc.format = image.imageFormat;
            desc.extent = VkExtent2D{image.imageExtent.width, image.imageExtent.height};
            desc.currentLayout = state.layout;
            desc.currentStage = state.stage;
            desc.currentAccess = state.access;
            return graph->import_image(desc);
        };

        RGImageDesc low_lighting_desc{};
        low_lighting_desc.name = "atmosphere.cloud.lowres_lighting";
        low_lighting_desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
        low_lighting_desc.extent = low_extent;
        low_lighting_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        RGImageHandle cloud_lighting_current = graph->create_image(low_lighting_desc);

        RGImageDesc low_segment_desc{};
        low_segment_desc.name = "atmosphere.cloud.lowres_segment";
        low_segment_desc.format = VK_FORMAT_R32G32_SFLOAT;
        low_segment_desc.extent = low_extent;
        low_segment_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        RGImageHandle cloud_segment_current = graph->create_image(low_segment_desc);

        RGImageHandle cloud_lighting_history_prev = import_history(
            "atmosphere.cloud.history_prev_lighting",
            _cloudLightingHistory[previous_index],
            _cloudLightingHistoryState[previous_index]);
        RGImageHandle cloud_segment_history_prev = import_history(
            "atmosphere.cloud.history_prev_segment",
            _cloudSegmentHistory[previous_index],
            _cloudSegmentHistoryState[previous_index]);
        RGImageHandle cloud_lighting_history_next = import_history(
            "atmosphere.cloud.history_next_lighting",
            _cloudLightingHistory[next_index],
            _cloudLightingHistoryState[next_index]);
        RGImageHandle cloud_segment_history_next = import_history(
            "atmosphere.cloud.history_next_segment",
            _cloudSegmentHistory[next_index],
            _cloudSegmentHistoryState[next_index]);

        graph->add_pass(
            "Atmosphere.CloudLowRes",
            RGPassType::Graphics,
            [gbufPos, transmittance_lut, cloud_lighting_current, cloud_segment_current](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(gbufPos, RGImageUsage::SampledFragment);
                builder.read(transmittance_lut, RGImageUsage::SampledFragment);
                builder.write_color(cloud_lighting_current, true);
                builder.write_color(cloud_segment_current, true);
            },
            [this, low_extent, gbufPos, transmittance_lut, cloud_lighting_current, cloud_segment_current](
                VkCommandBuffer cmd,
                const RGPassResources &resources,
                EngineContext *ctx)
            {
                draw_cloud_low_res(
                    cmd,
                    ctx,
                    resources,
                    low_extent,
                    gbufPos,
                    transmittance_lut,
                    cloud_lighting_current,
                    cloud_segment_current);
            });

        graph->add_pass(
            "Atmosphere.CloudTemporal",
            RGPassType::Graphics,
            [cloud_lighting_current,
             cloud_segment_current,
             cloud_lighting_history_prev,
             cloud_segment_history_prev,
             cloud_lighting_history_next,
             cloud_segment_history_next](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(cloud_lighting_current, RGImageUsage::SampledFragment);
                builder.read(cloud_segment_current, RGImageUsage::SampledFragment);
                if (cloud_lighting_history_prev.valid()) builder.read(cloud_lighting_history_prev, RGImageUsage::SampledFragment);
                if (cloud_segment_history_prev.valid()) builder.read(cloud_segment_history_prev, RGImageUsage::SampledFragment);
                builder.write_color(cloud_lighting_history_next, false);
                builder.write_color(cloud_segment_history_next, false);
            },
            [this,
             low_extent,
             cloud_lighting_current,
             cloud_segment_current,
             cloud_lighting_history_prev,
             cloud_segment_history_prev,
             cloud_lighting_history_next,
             cloud_segment_history_next,
             previous_view_proj,
             origin_delta_local,
             history_reset,
             run_cloud_pipeline](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
            {
                draw_cloud_temporal(
                    cmd,
                    ctx,
                    resources,
                    low_extent,
                    cloud_lighting_current,
                    cloud_segment_current,
                    cloud_lighting_history_prev,
                    cloud_segment_history_prev,
                    cloud_lighting_history_next,
                    cloud_segment_history_next,
                    previous_view_proj,
                    origin_delta_local,
                    run_cloud_pipeline && !history_reset);
            });

        RGImageDesc resolved_lighting_desc{};
        resolved_lighting_desc.name = "atmosphere.cloud.resolved_lighting";
        resolved_lighting_desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
        resolved_lighting_desc.extent = draw_extent;
        resolved_lighting_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        cloud_lighting_resolved = graph->create_image(resolved_lighting_desc);

        RGImageDesc resolved_segment_desc{};
        resolved_segment_desc.name = "atmosphere.cloud.resolved_segment";
        resolved_segment_desc.format = VK_FORMAT_R32G32_SFLOAT;
        resolved_segment_desc.extent = draw_extent;
        resolved_segment_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        cloud_segment_resolved = graph->create_image(resolved_segment_desc);

        graph->add_pass(
            "Atmosphere.CloudUpscale",
            RGPassType::Graphics,
            [gbufPos,
             cloud_lighting_history_next,
             cloud_segment_history_next,
             cloud_lighting_resolved,
             cloud_segment_resolved](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(gbufPos, RGImageUsage::SampledFragment);
                builder.read(cloud_lighting_history_next, RGImageUsage::SampledFragment);
                builder.read(cloud_segment_history_next, RGImageUsage::SampledFragment);
                builder.write_color(cloud_lighting_resolved, true);
                builder.write_color(cloud_segment_resolved, true);
            },
            [this,
             draw_extent,
             gbufPos,
             cloud_lighting_history_next,
             cloud_segment_history_next,
             cloud_lighting_resolved,
             cloud_segment_resolved](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
            {
                draw_cloud_upscale(
                    cmd,
                    ctx,
                    resources,
                    draw_extent,
                    gbufPos,
                    cloud_lighting_history_next,
                    cloud_segment_history_next,
                    cloud_lighting_resolved,
                    cloud_segment_resolved);
            });

        _cloudLightingHistoryState[previous_index] = {
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
        _cloudSegmentHistoryState[previous_index] = {
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
        _cloudLightingHistoryState[next_index] = {
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT};
        _cloudSegmentHistoryState[next_index] = {
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT};
        _historyActiveIndex = next_index;
        _historyValid = true;
    }
    else
    {
        _historyValid = false;
    }

    _previousViewProj = _context->getSceneData().viewproj;
    _previousOriginWorld = _context->origin_world;
    _historySnapshot = next_snapshot;

    RGImageDesc output_desc{};
    output_desc.name = "hdr.atmosphere";
    output_desc.format = _context->getSwapchain()
        ? _context->getSwapchain()->drawImage().imageFormat
        : VK_FORMAT_R16G16B16A16_SFLOAT;
    output_desc.extent = draw_extent;
    output_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    RGImageHandle hdr_output = graph->create_image(output_desc);

    graph->add_pass(
        "Atmosphere.Composite",
        RGPassType::Graphics,
        [hdrInput, gbufPos, transmittance_lut, cloud_lighting_resolved, cloud_segment_resolved, hdr_output](
            RGPassBuilder &builder,
            EngineContext *)
        {
            builder.read(hdrInput, RGImageUsage::SampledFragment);
            builder.read(gbufPos, RGImageUsage::SampledFragment);
            builder.read(transmittance_lut, RGImageUsage::SampledFragment);
            if (cloud_lighting_resolved.valid()) builder.read(cloud_lighting_resolved, RGImageUsage::SampledFragment);
            if (cloud_segment_resolved.valid()) builder.read(cloud_segment_resolved, RGImageUsage::SampledFragment);
            builder.write_color(hdr_output, false);
        },
        [this, draw_extent, hdrInput, gbufPos, transmittance_lut, cloud_lighting_resolved, cloud_segment_resolved](
            VkCommandBuffer cmd,
            const RGPassResources &resources,
            EngineContext *ctx)
        {
            draw_composite(
                cmd,
                ctx,
                resources,
                draw_extent,
                hdrInput,
                gbufPos,
                transmittance_lut,
                cloud_lighting_resolved,
                cloud_segment_resolved);
        });

    return hdr_output;
}

void AtmospherePass::draw_cloud_low_res(VkCommandBuffer cmd,
                                        EngineContext *context,
                                        const RGPassResources &resources,
                                        VkExtent2D extent,
                                        RGImageHandle gbufPos,
                                        RGImageHandle transmittanceLut,
                                        RGImageHandle cloudLighting,
                                        RGImageHandle cloudSegment)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    VkImageView pos_view = resources.image_view(gbufPos);
    VkImageView lut_view = resources.image_view(transmittanceLut);
    VkImageView cloud_lighting_view = resources.image_view(cloudLighting);
    VkImageView cloud_segment_view = resources.image_view(cloudSegment);
    if (pos_view == VK_NULL_HANDLE || lut_view == VK_NULL_HANDLE ||
        cloud_lighting_view == VK_NULL_HANDLE || cloud_segment_view == VK_NULL_HANDLE)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    VkImageView overlay_view = _cloudOverlayTex.imageView != VK_NULL_HANDLE ? _cloudOverlayTex.imageView : assets->fallbackBlackView();
    VkImageView noise_view = _cloudNoiseTex.imageView != VK_NULL_HANDLE ? _cloudNoiseTex.imageView : assets->fallbackWhiteView();
    VkImageView noise_view_3d = _cloudNoiseTex3D.imageView != VK_NULL_HANDLE ? _cloudNoiseTex3D.imageView : _cloudNoiseFallback3D.imageView;
    VkImageView jitter_view = _jitterNoiseTex.imageView != VK_NULL_HANDLE ? _jitterNoiseTex.imageView : _cloudNoiseFallback3D.imageView;
    if (overlay_view == VK_NULL_HANDLE || noise_view == VK_NULL_HANDLE || noise_view_3d == VK_NULL_HANDLE ||
        jitter_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _lowResSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, pos_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, lut_view, ctx->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, overlay_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, noise_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(4, noise_view_3d, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(5, jitter_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.cloud_lowres", _cloudLowResPipeline, _cloudLowResPipelineLayout))
    {
        return;
    }

    glm::vec3 planet_center_local{0.0f};
    float planet_radius_m = 0.0f;
    if (ctx->scene)
    {
        if (PlanetSystem *planets = ctx->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctx, *ctx->scene, *planets, planet_center_local, planet_radius_m);
        }
    }

    const AtmospherePush push = build_atmosphere_push(
        *ctx,
        planet_center_local,
        planet_radius_m,
        _jitterNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudOverlayTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex3D.imageView != VK_NULL_HANDLE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudLowResPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudLowResPipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudLowResPipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _cloudLowResPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void AtmospherePass::draw_cloud_temporal(VkCommandBuffer cmd,
                                         EngineContext *context,
                                         const RGPassResources &resources,
                                         VkExtent2D extent,
                                         RGImageHandle cloudLightingCurrent,
                                         RGImageHandle cloudSegmentCurrent,
                                         RGImageHandle cloudLightingHistoryPrev,
                                         RGImageHandle cloudSegmentHistoryPrev,
                                         RGImageHandle cloudLightingHistoryNext,
                                         RGImageHandle cloudSegmentHistoryNext,
                                         const glm::mat4 &previousViewProj,
                                         const glm::vec3 &originDeltaLocal,
                                         bool historyValid)
{
    (void)cloudLightingHistoryNext;
    (void)cloudSegmentHistoryNext;

    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    VkImageView cloud_lighting_current_view = resources.image_view(cloudLightingCurrent);
    VkImageView cloud_segment_current_view = resources.image_view(cloudSegmentCurrent);
    if (cloud_lighting_current_view == VK_NULL_HANDLE || cloud_segment_current_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkImageView cloud_lighting_history_prev_view = resources.image_view(cloudLightingHistoryPrev);
    VkImageView cloud_segment_history_prev_view = resources.image_view(cloudSegmentHistoryPrev);
    if (cloud_lighting_history_prev_view == VK_NULL_HANDLE)
    {
        cloud_lighting_history_prev_view = assets->fallbackBlackView();
    }
    if (cloud_segment_history_prev_view == VK_NULL_HANDLE)
    {
        cloud_segment_history_prev_view = assets->fallbackBlackView();
    }
    if (cloud_lighting_history_prev_view == VK_NULL_HANDLE || cloud_segment_history_prev_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _temporalSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, cloud_lighting_current_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, cloud_segment_current_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, cloud_lighting_history_prev_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, cloud_segment_history_prev_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.cloud_temporal", _cloudTemporalPipeline, _cloudTemporalPipelineLayout))
    {
        return;
    }

    CloudTemporalPush push{};
    push.previous_view_proj = previousViewProj;
    push.origin_delta_blend = glm::vec4(originDeltaLocal, 0.85f);
    push.viewport_params = glm::vec4(
        static_cast<float>(extent.width),
        static_cast<float>(extent.height),
        1.0f / static_cast<float>(std::max(1u, extent.width)),
        1.0f / static_cast<float>(std::max(1u, extent.height)));
    push.misc = glm::ivec4(historyValid ? 1 : 0, 0, 0, 0);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudTemporalPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudTemporalPipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudTemporalPipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _cloudTemporalPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void AtmospherePass::draw_cloud_upscale(VkCommandBuffer cmd,
                                        EngineContext *context,
                                        const RGPassResources &resources,
                                        VkExtent2D extent,
                                        RGImageHandle gbufPos,
                                        RGImageHandle cloudLightingLowRes,
                                        RGImageHandle cloudSegmentLowRes,
                                        RGImageHandle cloudLightingResolved,
                                        RGImageHandle cloudSegmentResolved)
{
    (void)cloudLightingResolved;
    (void)cloudSegmentResolved;

    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    VkImageView pos_view = resources.image_view(gbufPos);
    VkImageView cloud_lighting_lowres_view = resources.image_view(cloudLightingLowRes);
    VkImageView cloud_segment_lowres_view = resources.image_view(cloudSegmentLowRes);
    if (pos_view == VK_NULL_HANDLE || cloud_lighting_lowres_view == VK_NULL_HANDLE || cloud_segment_lowres_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _upscaleSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, pos_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, cloud_lighting_lowres_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, cloud_segment_lowres_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.cloud_upscale", _cloudUpscalePipeline, _cloudUpscalePipelineLayout))
    {
        return;
    }

    glm::vec3 planet_center_local{0.0f};
    float planet_radius_m = 0.0f;
    if (ctx->scene)
    {
        if (PlanetSystem *planets = ctx->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctx, *ctx->scene, *planets, planet_center_local, planet_radius_m);
        }
    }

    const AtmospherePush push = build_atmosphere_push(
        *ctx,
        planet_center_local,
        planet_radius_m,
        _jitterNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudOverlayTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex3D.imageView != VK_NULL_HANDLE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudUpscalePipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudUpscalePipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudUpscalePipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _cloudUpscalePipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void AtmospherePass::draw_composite(VkCommandBuffer cmd,
                                    EngineContext *context,
                                    const RGPassResources &resources,
                                    VkExtent2D extent,
                                    RGImageHandle hdrInput,
                                    RGImageHandle gbufPos,
                                    RGImageHandle transmittanceLut,
                                    RGImageHandle cloudLightingResolved,
                                    RGImageHandle cloudSegmentResolved)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    VkImageView hdr_view = resources.image_view(hdrInput);
    VkImageView pos_view = resources.image_view(gbufPos);
    VkImageView lut_view = resources.image_view(transmittanceLut);
    if (hdr_view == VK_NULL_HANDLE || pos_view == VK_NULL_HANDLE || lut_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkImageView overlay_view = _cloudOverlayTex.imageView != VK_NULL_HANDLE ? _cloudOverlayTex.imageView : assets->fallbackBlackView();
    VkImageView noise_view = _cloudNoiseTex.imageView != VK_NULL_HANDLE ? _cloudNoiseTex.imageView : assets->fallbackWhiteView();
    VkImageView noise_view_3d = _cloudNoiseTex3D.imageView != VK_NULL_HANDLE ? _cloudNoiseTex3D.imageView : _cloudNoiseFallback3D.imageView;
    VkImageView jitter_view = _jitterNoiseTex.imageView != VK_NULL_HANDLE ? _jitterNoiseTex.imageView : _cloudNoiseFallback3D.imageView;
    VkImageView cloud_lighting_resolved_view = resources.image_view(cloudLightingResolved);
    VkImageView cloud_segment_resolved_view = resources.image_view(cloudSegmentResolved);
    if (cloud_lighting_resolved_view == VK_NULL_HANDLE)
    {
        cloud_lighting_resolved_view = assets->fallbackBlackView();
    }
    if (cloud_segment_resolved_view == VK_NULL_HANDLE)
    {
        cloud_segment_resolved_view = assets->fallbackBlackView();
    }
    if (overlay_view == VK_NULL_HANDLE || noise_view == VK_NULL_HANDLE || noise_view_3d == VK_NULL_HANDLE ||
        jitter_view == VK_NULL_HANDLE ||
        cloud_lighting_resolved_view == VK_NULL_HANDLE || cloud_segment_resolved_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _compositeSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, hdr_view, ctx->getSamplers()->defaultLinear(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, pos_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, lut_view, ctx->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, overlay_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(4, noise_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(5, cloud_lighting_resolved_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(6, cloud_segment_resolved_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(7, noise_view_3d, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(8, jitter_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.composite", _compositePipeline, _compositePipelineLayout))
    {
        return;
    }

    glm::vec3 planet_center_local{0.0f};
    float planet_radius_m = 0.0f;
    if (ctx->scene)
    {
        if (PlanetSystem *planets = ctx->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctx, *ctx->scene, *planets, planet_center_local, planet_radius_m);
        }
    }

    const AtmospherePush push = build_atmosphere_push(
        *ctx,
        planet_center_local,
        planet_radius_m,
        _jitterNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudOverlayTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex3D.imageView != VK_NULL_HANDLE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _compositePipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _compositePipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _compositePipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _compositePipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
