#pragma once

/// Shared inline helpers for orbit_render_curve split files.
/// Used by orbit_render_curve.cpp, orbit_render_curve_render.cpp, and orbit_render_curve_pick.cpp.

#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/orbit_plot_util.h"

#include <algorithm>
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

    inline bool clip_point_is_finite(const glm::vec4 &clip)
    {
        return std::isfinite(clip.x) &&
               std::isfinite(clip.y) &&
               std::isfinite(clip.z) &&
               std::isfinite(clip.w);
    }

    inline glm::vec4 project_world_to_clip(const OrbitRenderCurve::FrustumContext &frustum,
                                           const WorldVec3 &point_world)
    {
        const glm::vec3 point_local = world_to_local(point_world, frustum.origin_world);
        return frustum.viewproj * glm::vec4(point_local, 1.0f);
    }

    inline double frustum_safe_margin(const double margin_ratio)
    {
        return std::isfinite(margin_ratio) ? std::max(0.0, margin_ratio) : 0.0;
    }

    inline void clip_plane_distances_margin(const glm::vec4 &clip,
                                            const double margin_ratio,
                                            double (&distances)[6])
    {
        const double x = static_cast<double>(clip.x);
        const double y = static_cast<double>(clip.y);
        const double z = static_cast<double>(clip.z);
        const double w = static_cast<double>(clip.w);

        const double xy_extent = (1.0 + margin_ratio) * w;
        const double near_extent = margin_ratio * w;
        const double far_extent = (1.0 + margin_ratio) * w;

        distances[0] = x + xy_extent;      // left:   x >= -(1 + margin) * w
        distances[1] = -x + xy_extent;     // right:  x <=  (1 + margin) * w
        distances[2] = y + xy_extent;      // bottom: y >= -(1 + margin) * w
        distances[3] = -y + xy_extent;     // top:    y <=  (1 + margin) * w
        distances[4] = z + near_extent;    // near:   z >= -margin * w
        distances[5] = -z + far_extent;    // far:    z <=  (1 + margin) * w
    }

    /// Test whether a world-space point lies inside the frustum with an NDC margin.
    /// Projects the point to clip space and checks against expanded Vulkan clip bounds.
    /// Returns true (accept) when the frustum is invalid (culling disabled).
    inline bool frustum_contains_point_margin(const OrbitRenderCurve::FrustumContext &frustum,
                                               const WorldVec3 &point_world,
                                               const double margin_ratio)
    {
        if (!frustum.valid)
        {
            return true;
        }

        const glm::vec4 clip = project_world_to_clip(frustum, point_world);
        if (!clip_point_is_finite(clip) || !(static_cast<double>(clip.w) > 1.0e-6))
        {
            return false;
        }

        double distances[6]{};
        clip_plane_distances_margin(clip, frustum_safe_margin(margin_ratio), distances);
        for (const double distance : distances)
        {
            if (distance < 0.0)
            {
                return false;
            }
        }
        return true;
    }

    /// Accept a line segment unless both endpoints are fully outside one homogeneous clip plane.
    /// This keeps long segments that cross the view even when endpoints and midpoint are off-screen.
    inline bool frustum_accept_segment_margin(const OrbitRenderCurve::FrustumContext &frustum,
                                               const WorldVec3 &a_world,
                                               const WorldVec3 &b_world,
                                               const double margin_ratio)
    {
        if (!frustum.valid)
        {
            return true;
        }

        const glm::vec4 clip_a = project_world_to_clip(frustum, a_world);
        const glm::vec4 clip_b = project_world_to_clip(frustum, b_world);
        if (!clip_point_is_finite(clip_a) || !clip_point_is_finite(clip_b))
        {
            return false;
        }

        double distances_a[6]{};
        double distances_b[6]{};
        const double safe_margin = frustum_safe_margin(margin_ratio);
        clip_plane_distances_margin(clip_a, safe_margin, distances_a);
        clip_plane_distances_margin(clip_b, safe_margin, distances_b);

        constexpr double kClipPlaneDistanceEpsilon = 1.0e-7;
        for (std::size_t i = 0; i < 6; ++i)
        {
            if (distances_a[i] < -kClipPlaneDistanceEpsilon &&
                distances_b[i] < -kClipPlaneDistanceEpsilon)
            {
                return false;
            }
        }
        return true;
    }
} // namespace Game
