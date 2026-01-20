#include "physics_debug_draw.h"

#include "core/context.h"
#include "core/debug_draw/debug_draw.h"
#include "physics/physics_world.h"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace
{
    glm::vec3 safe_normalize(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        const float len2 = glm::dot(v, v);
        if (!std::isfinite(len2) || len2 <= 1.0e-12f)
        {
            return fallback;
        }
        return v * (1.0f / std::sqrt(len2));
    }
}

void debug_draw_physics_colliders(DebugDrawSystem *dd,
                                 const WorldVec3 &origin_world,
                                 const Physics::PhysicsWorld *physics,
                                 const PhysicsDebugSettings &settings)
{
    if (!dd || !physics)
    {
        return;
    }

    const DebugDepth depth = settings.overlay ? DebugDepth::AlwaysOnTop : DebugDepth::DepthTested;
    const float alpha = std::clamp(settings.alpha, 0.0f, 1.0f);
    const int max_bodies = std::max(0, settings.max_bodies);

    auto color_for = [alpha](const Physics::PhysicsWorld::DebugBodyView &b) {
        glm::vec4 c{0.2f, 0.9f, 0.3f, 1.0f}; // dynamic
        if (b.motion_type == Physics::MotionType::Static) c = glm::vec4(0.6f, 0.6f, 0.6f, 1.0f);
        if (b.motion_type == Physics::MotionType::Kinematic) c = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);
        if (!b.is_active && b.motion_type != Physics::MotionType::Static) c = glm::vec4(0.15f, 0.35f, 0.15f, 1.0f);
        if (b.is_sensor) c = glm::vec4(1.0f, 0.35f, 0.9f, 1.0f);
        c.a *= alpha;
        return c;
    };

    int drawn = 0;
    physics->for_each_debug_body([&](const Physics::PhysicsWorld::DebugBodyView &b) {
        if (max_bodies > 0 && drawn >= max_bodies) return;
        if (settings.active_only && !b.is_active) return;
        if (!settings.include_sensors && b.is_sensor) return;
        if (b.motion_type == Physics::MotionType::Static && !settings.include_static) return;
        if (b.motion_type == Physics::MotionType::Kinematic && !settings.include_kinematic) return;
        if (b.motion_type == Physics::MotionType::Dynamic && !settings.include_dynamic) return;

        const glm::vec4 color = color_for(b);
        const glm::vec3 body_pos_local = b.position;
        const glm::quat body_rot_local = b.rotation;

        auto draw_primitive = [&](const auto &prim, const glm::vec3 &pos_local, const glm::quat &rot_local) {
            using T = std::decay_t<decltype(prim)>;
            const WorldVec3 center_world = local_to_world(pos_local, origin_world);

            if constexpr (std::is_same_v<T, Physics::BoxShape>)
            {
                dd->add_obb(center_world, rot_local, prim.half_extents, color, 0.0f, depth, DebugDrawLayer::Physics);
            }
            else if constexpr (std::is_same_v<T, Physics::SphereShape>)
            {
                dd->add_sphere(center_world, prim.radius, color, 0.0f, depth, DebugDrawLayer::Physics);
            }
            else if constexpr (std::is_same_v<T, Physics::CapsuleShape>)
            {
                const glm::vec3 axis = safe_normalize(rot_local * glm::vec3(0.0f, 1.0f, 0.0f),
                                                      glm::vec3(0.0f, 1.0f, 0.0f));
                const glm::vec3 p0_local = pos_local - axis * prim.half_height;
                const glm::vec3 p1_local = pos_local + axis * prim.half_height;
                dd->add_capsule(local_to_world(p0_local, origin_world),
                                local_to_world(p1_local, origin_world),
                                prim.radius,
                                color,
                                0.0f,
                                depth,
                                DebugDrawLayer::Physics);
            }
            else if constexpr (std::is_same_v<T, Physics::CylinderShape>)
            {
                dd->add_cylinder(center_world,
                                 glm::dvec3(rot_local * glm::vec3(0.0f, 1.0f, 0.0f)),
                                 prim.radius,
                                 prim.half_height,
                                 color,
                                 0.0f,
                                 depth,
                                 DebugDrawLayer::Physics);
            }
            else if constexpr (std::is_same_v<T, Physics::TaperedCylinderShape>)
            {
                dd->add_tapered_cylinder(center_world,
                                         glm::dvec3(rot_local * glm::vec3(0.0f, 1.0f, 0.0f)),
                                         prim.half_height,
                                         prim.top_radius,
                                         prim.bottom_radius,
                                         color,
                                         0.0f,
                                         depth,
                                         DebugDrawLayer::Physics);
            }
            else if constexpr (std::is_same_v<T, Physics::PlaneShape>)
            {
                const glm::vec3 n0 = safe_normalize(prim.normal, glm::vec3(0.0f, 1.0f, 0.0f));
                const glm::vec3 n = safe_normalize(rot_local * n0, glm::vec3(0.0f, 1.0f, 0.0f));
                const glm::vec3 point_local = pos_local + n * prim.offset;
                dd->add_plane_patch(local_to_world(point_local, origin_world),
                                    glm::dvec3(n),
                                    25.0f,
                                    color,
                                    0.0f,
                                    depth,
                                    DebugDrawLayer::Physics);
            }
        };

        std::visit([&](const auto &shape) {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, Physics::CompoundShape>)
            {
                for (const Physics::CompoundShapeChild &child: shape.children)
                {
                    const glm::vec3 child_pos = body_pos_local + (body_rot_local * child.position);
                    const glm::quat child_rot = body_rot_local * child.rotation;
                    std::visit([&](const auto &prim) { draw_primitive(prim, child_pos, child_rot); }, child.shape);
                }
            }
            else
            {
                draw_primitive(shape, body_pos_local, body_rot_local);
            }
        }, b.shape.shape);

        drawn++;
    });
}

