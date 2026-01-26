#pragma once

#include <glm/vec3.hpp>

#include <cmath>

// Authoritative world-space coordinates are stored as double precision.
using WorldVec3 = glm::dvec3;

inline glm::vec3 world_to_local(const WorldVec3 &world, const WorldVec3 &origin_world)
{
    const WorldVec3 local_d = world - origin_world;
    return glm::vec3(static_cast<float>(local_d.x),
                     static_cast<float>(local_d.y),
                     static_cast<float>(local_d.z));
}

inline glm::dvec3 world_to_local_d(const WorldVec3 &world, const WorldVec3 &origin_world)
{
    return world - origin_world;
}

inline WorldVec3 local_to_world(const glm::vec3 &local, const WorldVec3 &origin_world)
{
    return origin_world + WorldVec3(local);
}

inline WorldVec3 local_to_world_d(const glm::dvec3 &local, const WorldVec3 &origin_world)
{
    return origin_world + WorldVec3(local);
}

inline bool is_zero(const glm::dvec3 &v)
{
    return v.x == 0.0 && v.y == 0.0 && v.z == 0.0;
}

inline WorldVec3 snap_world(const WorldVec3 &p, double grid_size)
{
    if (grid_size <= 0.0)
    {
        return p;
    }

    auto snap = [grid_size](double v) {
        return std::round(v / grid_size) * grid_size;
    };

    return WorldVec3(snap(p.x), snap(p.y), snap(p.z));
}
