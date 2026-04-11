#pragma once

#include "render/passes/atmosphere.h"

#include "core/assets/ktx_loader.h"
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

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <span>

#include <glm/gtc/packing.hpp>

namespace atmosphere::detail
{
    constexpr uint32_t k_transmittance_lut_width = 256;
    constexpr uint32_t k_transmittance_lut_height = 64;
    constexpr uint32_t k_misc_flags_mask = 0xFFu;
    constexpr uint32_t k_misc_noise_blend_shift = 8u;
    constexpr uint32_t k_misc_detail_erode_shift = 16u;
    constexpr uint32_t k_misc_jitter_frame_shift = 24u;
    constexpr uint32_t k_flag_cloud_noise_3d = 8u;
    constexpr uint32_t k_flag_jitter_blue_noise = 16u;

    struct AtmosphereBodySelection
    {
        const PlanetSystem::PlanetBody *body = nullptr;
        glm::vec3 center_local{0.0f};
        float radius_m = 0.0f;
    };

    struct AtmospherePush
    {
        glm::vec4 planet_center_radius;
        glm::vec4 atmosphere_params;
        glm::vec4 beta_rayleigh;
        glm::vec4 beta_mie;
        glm::vec4 jitter_params;
        glm::vec4 terrain_params;
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

    static_assert(sizeof(AtmospherePush) == 160);
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

    static std::string resolve_optional_face_texture_path(AssetManager &assets,
                                                          std::string_view dir,
                                                          planet::CubeFace face)
    {
        if (dir.empty())
        {
            return {};
        }

        std::string rel = std::string(dir) + "/" + planet::cube_face_name(face) + ".ktx2";
        std::string abs_path = assets.assetPath(rel);
        if (std::filesystem::exists(abs_path))
        {
            return abs_path;
        }

        rel = std::string(dir) + "/" + planet::cube_face_name(face) + ".png";
        abs_path = assets.assetPath(rel);
        if (std::filesystem::exists(abs_path))
        {
            return abs_path;
        }

        return {};
    }

    static bool find_atmosphere_body(const EngineContext &ctx,
                                     const SceneManager &scene,
                                     const PlanetSystem &planets,
                                     AtmosphereBodySelection &out_selection)
    {
        out_selection = {};

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

        out_selection.body = picked;
        out_selection.center_local = world_to_local(picked->center_world, scene.get_world_origin());
        out_selection.radius_m = static_cast<float>(picked->radius_m);
        return std::isfinite(out_selection.radius_m) && out_selection.radius_m > 0.0f;
    }

    static AtmospherePush build_atmosphere_push(const EngineContext &ctx,
                                                const AtmosphereBodySelection &body_selection,
                                                bool jitter_noise_ready,
                                                bool cloud_overlay_ready,
                                                bool cloud_noise_ready,
                                                bool cloud_noise_3d_ready)
    {
        const AtmosphereSettings &s = ctx.atmosphere;
        const PlanetCloudSettings &c = ctx.planetClouds;
        const PlanetSystem::PlanetBody *body = body_selection.body;
        const glm::vec3 &planet_center_local = body_selection.center_local;
        const float planet_radius_m = body_selection.radius_m;

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
        const bool terrain_height_enabled = body && body->terrain &&
            (!body->terrain_height_dir.empty()) &&
            (body->terrain_height_max_m > 0.0 || body->terrain_height_offset_m > 0.0);
        const float terrain_height_scale_m = terrain_height_enabled
            ? static_cast<float>(std::max(0.0, body->terrain_height_max_m))
            : 0.0f;
        const float terrain_height_offset_m = terrain_height_enabled
            ? static_cast<float>(std::max(0.0, body->terrain_height_offset_m))
            : 0.0f;
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
        push.terrain_params = glm::vec4(terrain_height_scale_m, terrain_height_offset_m, 0.0f, 0.0f);
        push.cloud_layer = glm::vec4(cloud_base_m, cloud_thickness_m, cloud_density_scale, cloud_coverage);
        push.cloud_params = glm::vec4(cloud_noise_scale, cloud_detail_scale, cloud_wind_speed, cloud_wind_angle);
        push.cloud_color = glm::vec4(cloud_color, 1.0f);
        push.misc = glm::ivec4(view_steps, packed_absorption_color_bits, cloud_steps, std::bit_cast<int32_t>(packed_misc_w));
        return push;
    }
} // namespace atmosphere::detail
