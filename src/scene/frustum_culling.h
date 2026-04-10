#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace scene::frustum
{
    struct Plane
    {
        glm::vec3 normal{0.0f};
        float d = 0.0f;
    };

    using PlaneSet = std::array<Plane, 6>;

    inline PlaneSet extract_clip_planes(const glm::mat4 &clip_from_space)
    {
        // Homogeneous clip half-spaces for Vulkan clip coordinates:
        //   -w <= x <= w
        //   -w <= y <= w
        //    0 <= z <= w
        static constexpr std::array<glm::vec4, 6> clip_halfspaces{{
            glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
            glm::vec4(-1.0f, 0.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, -1.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, 0.0f, 1.0f, 0.0f),
            glm::vec4(0.0f, 0.0f, -1.0f, 1.0f),
        }};

        PlaneSet planes{};
        const glm::mat4 clip_from_space_t = glm::transpose(clip_from_space);
        for (size_t i = 0; i < planes.size(); ++i)
        {
            const glm::vec4 plane_eq = clip_from_space_t * clip_halfspaces[i];
            planes[i].normal = glm::vec3(plane_eq);
            planes[i].d = plane_eq.w;
        }
        return planes;
    }

    inline bool intersects_sphere(const PlaneSet &planes, const glm::vec3 &center, float radius)
    {
        const float safe_radius = std::max(radius, 0.0f);
        for (const Plane &plane : planes)
        {
            const float normal_len = glm::length(plane.normal);
            if (!(normal_len > 0.0f))
            {
                continue;
            }

            const float signed_distance = glm::dot(plane.normal, center) + plane.d;
            if (signed_distance < -(safe_radius * normal_len))
            {
                return false;
            }
        }
        return true;
    }

    inline bool intersects_obb(const PlaneSet &planes,
                               const glm::vec3 &center,
                               const glm::mat3 &basis,
                               const glm::vec3 &extents)
    {
        for (const Plane &plane : planes)
        {
            const float projected_radius =
                std::abs(glm::dot(plane.normal, basis[0])) * extents.x +
                std::abs(glm::dot(plane.normal, basis[1])) * extents.y +
                std::abs(glm::dot(plane.normal, basis[2])) * extents.z;

            const float signed_distance = glm::dot(plane.normal, center) + plane.d;
            if (signed_distance < -projected_radius)
            {
                return false;
            }
        }
        return true;
    }
} // namespace scene::frustum
