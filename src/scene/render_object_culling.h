#pragma once

#include <algorithm>
#include <cmath>

#include <glm/glm.hpp>

#include "scene/frustum_culling.h"
#include "scene/vk_scene.h"

namespace scene::culling
{
    struct VisibilityBounds
    {
        glm::vec3 center_local{0.0f};
        glm::mat3 basis{1.0f};
        float sphere_radius_local = 0.0f;
    };

    inline VisibilityBounds compute_visibility_bounds(const RenderObject &obj)
    {
        VisibilityBounds bounds{};
        bounds.center_local = glm::vec3(obj.transform * glm::vec4(obj.bounds.origin, 1.0f));
        bounds.basis = glm::mat3(obj.transform);
        const float max_scale = std::max({
            glm::length(bounds.basis[0]),
            glm::length(bounds.basis[1]),
            glm::length(bounds.basis[2]),
        });
        bounds.sphere_radius_local = obj.bounds.sphereRadius * max_scale;
        return bounds;
    }

    inline bool intersects_view_frustum(const RenderObject &obj,
                                        const VisibilityBounds &bounds,
                                        const frustum::PlaneSet &planes)
    {
        if (obj.bounds.type == BoundsType::Sphere && bounds.sphere_radius_local > 0.0f)
        {
            return frustum::intersects_sphere(planes, bounds.center_local, bounds.sphere_radius_local);
        }

        // BoundsType also drives picking behavior. Rendering falls back to the
        // conservative oriented box unless the draw is explicitly sphere-bounded.
        return frustum::intersects_obb(planes, bounds.center_local, bounds.basis, obj.bounds.extents);
    }

    inline bool is_occluded_by_planet(const VisibilityBounds &bounds,
                                      const glm::vec3 &camera_local,
                                      const GPUSceneData &scene_data)
    {
        if (!(bounds.sphere_radius_local > 0.0f))
        {
            return false;
        }

        const glm::vec3 to_object = bounds.center_local - camera_local;
        const float object_dist = glm::length(to_object);
        if (!(object_dist > bounds.sphere_radius_local))
        {
            return false;
        }

        const float object_center_tangent = std::sqrt(std::max(
            object_dist * object_dist - bounds.sphere_radius_local * bounds.sphere_radius_local,
            0.0f));

        const uint32_t occluder_count = std::min(scene_data.lightCounts.z, 4u);
        for (uint32_t i = 0; i < occluder_count; ++i)
        {
            const glm::vec4 occluder = scene_data.planetOccluders[i];
            const float occluder_radius = occluder.w;
            if (!(occluder_radius > 0.0f))
            {
                continue;
            }

            const glm::vec3 occluder_center = glm::vec3(occluder);
            const glm::vec3 to_occluder = occluder_center - camera_local;
            const float occluder_dist = glm::length(to_occluder);
            if (!(occluder_dist > occluder_radius))
            {
                continue;
            }

            const float object_front_center = object_dist - bounds.sphere_radius_local;
            const float occluder_near_center = occluder_dist - occluder_radius;
            if (object_front_center <= occluder_near_center)
            {
                continue;
            }

            const float angular_sep_cos = glm::clamp(
                glm::dot(to_object, to_occluder) / (object_dist * occluder_dist),
                -1.0f,
                1.0f);
            const float angular_sep = std::acos(angular_sep_cos);
            const float planet_angular_radius = std::asin(glm::clamp(occluder_radius / occluder_dist, 0.0f, 1.0f));
            const float object_angular_radius =
                std::asin(glm::clamp(bounds.sphere_radius_local / object_dist, 0.0f, 1.0f));
            const float outer_angle = angular_sep + object_angular_radius;

            if (outer_angle > planet_angular_radius)
            {
                continue;
            }

            const float boundary_cos = std::cos(outer_angle);
            const float perpendicular_dist = occluder_dist * std::sin(outer_angle);
            const float chord = std::sqrt(std::max(
                occluder_radius * occluder_radius - perpendicular_dist * perpendicular_dist,
                0.0f));
            const float occluder_near_boundary = occluder_dist * boundary_cos - chord;

            if (object_center_tangent >= occluder_near_boundary)
            {
                return true;
            }
        }

        return false;
    }

    inline bool intersects_bounding_sphere(const VisibilityBounds &bounds,
                                           const glm::vec3 &sphere_center,
                                           float sphere_radius)
    {
        if (!(sphere_radius > 0.0f) || !(bounds.sphere_radius_local >= 0.0f))
        {
            return false;
        }

        const glm::vec3 delta = bounds.center_local - sphere_center;
        const float combined_radius = sphere_radius + bounds.sphere_radius_local;
        return glm::dot(delta, delta) <= combined_radius * combined_radius;
    }
} // namespace scene::culling
