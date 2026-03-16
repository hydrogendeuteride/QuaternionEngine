#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_segments.hpp"

#include <glm/vec3.hpp>

namespace Game::OrbitPlotUtil
{
    glm::dvec3 eval_segment_local_position(const orbitsim::TrajectorySegment &segment, double t_s);

    double meters_per_px_at_world(const glm::dvec3 &camera_world,
                                   double tan_half_fov,
                                   double viewport_height_px,
                                   const WorldVec3 &point_world);
} // namespace Game::OrbitPlotUtil
