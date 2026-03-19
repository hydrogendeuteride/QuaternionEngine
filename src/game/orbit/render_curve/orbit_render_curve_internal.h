#pragma once

/// Shared inline helpers for orbit_render_curve split files.
/// Used by orbit_render_curve.cpp, orbit_render_curve_render.cpp, and orbit_render_curve_pick.cpp.

#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/orbit_plot_util.h"

#include <cmath>

namespace Game
{
    /// Compute the end time of a trajectory segment (t0 + dt).
    inline double segment_end_time(const orbitsim::TrajectorySegment &segment)
    {
        return segment.t0_s + segment.dt_s;
    }

    /// Evaluate the Hermite-interpolated world position of a trajectory segment at time t_s.
    /// Adds reference_body_world and align_delta_world offsets to the local (BCI) position.
    inline WorldVec3 eval_segment_world_position(const orbitsim::TrajectorySegment &segment,
                                                  const double t_s,
                                                  const WorldVec3 &reference_body_world,
                                                  const WorldVec3 &align_delta_world)
    {
        return reference_body_world + OrbitPlotUtil::eval_segment_local_position(segment, t_s) + align_delta_world;
    }

    /// Test whether a world-space point lies inside the frustum with an NDC margin.
    /// Projects the point to clip space and checks against expanded bounds.
    /// Returns true (accept) when the frustum is invalid (culling disabled).
    inline bool frustum_contains_point_margin(const OrbitRenderCurve::FrustumContext &frustum,
                                               const WorldVec3 &point_world,
                                               const double margin_ratio)
    {
        if (!frustum.valid)
        {
            return true;
        }

        const double safe_margin = std::isfinite(margin_ratio) ? std::max(0.0, margin_ratio) : 0.0;

        const glm::vec3 point_local = world_to_local(point_world, frustum.origin_world);
        const glm::vec4 clip = frustum.viewproj * glm::vec4(point_local, 1.0f);
        if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w))
        {
            return false;
        }

        const double w = static_cast<double>(clip.w);
        const double aw = std::abs(w);
        if (!(aw > 1.0e-6))
        {
            return false;
        }

        const double x = static_cast<double>(clip.x);
        const double y = static_cast<double>(clip.y);
        const double z = static_cast<double>(clip.z);

        const double xy_bound = (1.0 + safe_margin) * aw;
        const double z_min = -safe_margin * aw;
        const double z_max = (1.0 + safe_margin) * aw;

        return (x >= -xy_bound && x <= xy_bound) &&
               (y >= -xy_bound && y <= xy_bound) &&
               (z >= z_min && z <= z_max);
    }

    /// Accept a line segment if either endpoint or its midpoint is inside the frustum.
    /// Conservative test: may accept segments partially outside, but never rejects fully-visible ones.
    inline bool frustum_accept_segment_margin(const OrbitRenderCurve::FrustumContext &frustum,
                                               const WorldVec3 &a_world,
                                               const WorldVec3 &b_world,
                                               const double margin_ratio)
    {
        if (!frustum.valid)
        {
            return true;
        }

        if (frustum_contains_point_margin(frustum, a_world, margin_ratio) ||
            frustum_contains_point_margin(frustum, b_world, margin_ratio))
        {
            return true;
        }

        const WorldVec3 mid_world = 0.5 * (a_world + b_world);
        return frustum_contains_point_margin(frustum, mid_world, margin_ratio);
    }
} // namespace Game
