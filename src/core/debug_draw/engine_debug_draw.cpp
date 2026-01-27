#include "engine_debug_draw.h"

#include "core/context.h"
#include "core/debug_draw/debug_draw.h"
#include "core/picking/picking_system.h"

#include "render/renderpass.h"
#include "render/passes/particles.h"
#include "scene/mesh_bvh.h"
#include "scene/vk_scene.h"

#include <algorithm>
#include <cmath>

void debug_draw_engine_layers(DebugDrawSystem *dd,
                              const WorldVec3 &origin_world,
                              EngineContext *context,
                              SceneManager *scene,
                              PickingSystem *picking,
                              RenderPassManager *render_pass_manager)
{
    if (!dd || !context)
    {
        return;
    }

    const uint32_t layer_mask = dd->settings().layer_mask;
    auto layer_on = [layer_mask](DebugDrawLayer layer) {
        return (layer_mask & static_cast<uint32_t>(layer)) != 0u;
    };

    // Picking: BVH root bounds + picked surface bounds (if available)
    if (layer_on(DebugDrawLayer::Picking) && picking && picking->debug_draw_bvh())
    {
        const auto &pick = picking->last_pick();
        if (pick.valid && pick.mesh)
        {
            const glm::mat4 &M = pick.worldTransform;
            auto obb_from_local_aabb = [&](const glm::vec3 &center_local, const glm::vec3 &half_extents) {
                const glm::vec3 c = center_local;
                const glm::vec3 e = glm::max(half_extents, glm::vec3(0.0f));
                const glm::vec3 corners_local[8] = {
                    c + glm::vec3(-e.x, -e.y, -e.z),
                    c + glm::vec3(+e.x, -e.y, -e.z),
                    c + glm::vec3(-e.x, +e.y, -e.z),
                    c + glm::vec3(+e.x, +e.y, -e.z),
                    c + glm::vec3(-e.x, -e.y, +e.z),
                    c + glm::vec3(+e.x, -e.y, +e.z),
                    c + glm::vec3(-e.x, +e.y, +e.z),
                    c + glm::vec3(+e.x, +e.y, +e.z),
                };
                std::array<WorldVec3, 8> corners_world{};
                for (int i = 0; i < 8; ++i)
                {
                    const glm::vec3 p_local = glm::vec3(M * glm::vec4(corners_local[i], 1.0f));
                    corners_world[i] = local_to_world(p_local, origin_world);
                }
                return corners_world;
            };

            if (pick.surfaceIndex < pick.mesh->surfaces.size())
            {
                const Bounds &b = pick.mesh->surfaces[pick.surfaceIndex].bounds;
                dd->add_obb_corners(obb_from_local_aabb(b.origin, b.extents),
                                    glm::vec4(1.0f, 1.0f, 0.0f, 0.75f),
                                    0.0f,
                                    DebugDepth::AlwaysOnTop,
                                    DebugDrawLayer::Picking);
            }

            if (pick.mesh->bvh && !pick.mesh->bvh->nodes.empty())
            {
                const auto &root = pick.mesh->bvh->nodes[0];
                const glm::vec3 bmin(root.bounds.min.x, root.bounds.min.y, root.bounds.min.z);
                const glm::vec3 bmax(root.bounds.max.x, root.bounds.max.y, root.bounds.max.z);
                const glm::vec3 c = (bmin + bmax) * 0.5f;
                const glm::vec3 e = (bmax - bmin) * 0.5f;
                dd->add_obb_corners(obb_from_local_aabb(c, e),
                                    glm::vec4(0.0f, 1.0f, 1.0f, 0.75f),
                                    0.0f,
                                    DebugDepth::AlwaysOnTop,
                                    DebugDrawLayer::Picking);
            }
        }
    }

    // Lights: spheres + spot cones
    if (scene && layer_on(DebugDrawLayer::Lights))
    {
        for (const auto &pl: scene->getPointLights())
        {
            dd->add_sphere(pl.position_world,
                           pl.radius,
                           glm::vec4(pl.color, 0.35f),
                           0.0f,
                           DebugDepth::AlwaysOnTop,
                           DebugDrawLayer::Lights);
        }
        for (const auto &sl: scene->getSpotLights())
        {
            dd->add_cone(sl.position_world,
                         glm::dvec3(sl.direction),
                         sl.radius,
                         sl.outer_angle_deg,
                         glm::vec4(sl.color, 0.35f),
                         0.0f,
                         DebugDepth::AlwaysOnTop,
                         DebugDrawLayer::Lights);
            dd->add_sphere(sl.position_world,
                           0.15f,
                           glm::vec4(sl.color, 0.9f),
                           0.0f,
                           DebugDepth::AlwaysOnTop,
                           DebugDrawLayer::Lights);
        }
    }

    // Particles: emitter + spawn radius + emission cone
    if (layer_on(DebugDrawLayer::Particles) && render_pass_manager)
    {
        if (auto *particles = render_pass_manager->getPass<ParticlePass>())
        {
            for (const auto &sys: particles->systems())
            {
                if (!sys.enabled || sys.count == 0) continue;

                const WorldVec3 emitter_world = local_to_world(sys.params.emitter_pos_local, origin_world);
                glm::vec4 c = sys.params.color;
                c.a = 0.5f;

                dd->add_sphere(emitter_world,
                               std::max(0.05f, sys.params.spawn_radius * 0.5f),
                               c,
                               0.0f,
                               DebugDepth::AlwaysOnTop,
                               DebugDrawLayer::Particles);

                dd->add_circle(emitter_world,
                               glm::dvec3(sys.params.emitter_dir_local),
                               sys.params.spawn_radius,
                               glm::vec4(1.0f, 0.6f, 0.1f, 0.35f),
                               0.0f,
                               DebugDepth::AlwaysOnTop,
                               DebugDrawLayer::Particles);

                dd->add_cone(emitter_world,
                             glm::dvec3(sys.params.emitter_dir_local),
                             std::max(0.5f, sys.params.spawn_radius * 3.0f),
                             sys.params.cone_angle_degrees,
                             glm::vec4(1.0f, 0.6f, 0.1f, 0.35f),
                             0.0f,
                             DebugDepth::AlwaysOnTop,
                             DebugDrawLayer::Particles);
            }
        }
    }

    // Volumetrics: volume AABB + wind vector
    if (scene && context->enableVolumetrics && layer_on(DebugDrawLayer::Volumetrics))
    {
        const glm::vec3 cam_local = scene->get_camera_local_position();
        for (uint32_t i = 0; i < EngineContext::MAX_VOXEL_VOLUMES; ++i)
        {
            const auto &vs = context->voxelVolumes[i];
            if (!vs.enabled) continue;

            glm::vec3 center_local = vs.volumeCenterLocal;
            if (vs.followCameraXZ)
            {
                center_local.x += cam_local.x;
                center_local.z += cam_local.z;
            }

            const WorldVec3 center_world = local_to_world(center_local, origin_world);
            dd->add_aabb(center_world,
                         vs.volumeHalfExtents,
                         glm::vec4(0.4f, 0.8f, 1.0f, 0.35f),
                         0.0f,
                         DebugDepth::AlwaysOnTop,
                         DebugDrawLayer::Volumetrics);

            const float wind_len = glm::length(vs.windVelocityLocal);
            if (std::isfinite(wind_len) && wind_len > 1.0e-4f)
            {
                dd->add_ray(center_world,
                            glm::dvec3(vs.windVelocityLocal),
                            std::clamp(wind_len, 0.5f, 25.0f),
                            glm::vec4(0.2f, 1.0f, 0.2f, 0.9f),
                            0.0f,
                            DebugDepth::AlwaysOnTop,
                            DebugDrawLayer::Volumetrics);
            }
        }
    }
}
