#pragma once

/// Shared inline helpers for orbit_render_curve split files.
/// Used by orbit_render_curve.cpp, orbit_render_curve_render.cpp, and orbit_render_curve_pick.cpp.

#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/orbit_plot_util.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace Game
{
    /// Compute the end time of a trajectory segment (t0 + dt).
    inline double segment_end_time(const orbitsim::TrajectorySegment &segment)
    {
        return segment.t0_s + segment.dt_s;
    }

    inline bool anchor_times_are_sorted_and_finite(const std::span<const double> anchor_times_s)
    {
        double previous_time_s = 0.0;
        bool has_previous = false;
        for (const double anchor_time_s : anchor_times_s)
        {
            if (!std::isfinite(anchor_time_s))
            {
                return false;
            }

            if (has_previous && anchor_time_s < previous_time_s)
            {
                return false;
            }

            previous_time_s = anchor_time_s;
            has_previous = true;
        }

        return true;
    }

    /// Return a finite, nondecreasing anchor-time span suitable for binary searches.
    /// Already-normalized caller spans are returned without allocation.
    inline std::span<const double> normalized_anchor_times(
            const std::span<const double> anchor_times_s,
            std::vector<double> &scratch_anchor_times_s)
    {
        if (anchor_times_s.empty() || anchor_times_are_sorted_and_finite(anchor_times_s))
        {
            return anchor_times_s;
        }

        scratch_anchor_times_s.clear();
        scratch_anchor_times_s.reserve(anchor_times_s.size());
        for (const double anchor_time_s : anchor_times_s)
        {
            if (std::isfinite(anchor_time_s))
            {
                scratch_anchor_times_s.push_back(anchor_time_s);
            }
        }

        std::sort(scratch_anchor_times_s.begin(), scratch_anchor_times_s.end());
        scratch_anchor_times_s.erase(
                std::unique(scratch_anchor_times_s.begin(), scratch_anchor_times_s.end()),
                scratch_anchor_times_s.end());
        return std::span<const double>(scratch_anchor_times_s.data(), scratch_anchor_times_s.size());
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

    inline bool vec3_is_finite(const glm::dvec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
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

    inline bool frustum_accept_clip_control_points_margin(
            const std::array<glm::vec4, 4> &clip_points,
            const double margin_ratio)
    {
        double distances[4][6]{};
        for (std::size_t point_index = 0; point_index < clip_points.size(); ++point_index)
        {
            const glm::vec4 &clip = clip_points[point_index];
            if (!clip_point_is_finite(clip))
            {
                return false;
            }
            clip_plane_distances_margin(clip, margin_ratio, distances[point_index]);
        }

        constexpr double kClipPlaneDistanceEpsilon = 1.0e-7;
        for (std::size_t plane_index = 0; plane_index < 6; ++plane_index)
        {
            bool all_outside_plane = true;
            for (std::size_t point_index = 0; point_index < clip_points.size(); ++point_index)
            {
                if (distances[point_index][plane_index] >= -kClipPlaneDistanceEpsilon)
                {
                    all_outside_plane = false;
                    break;
                }
            }

            if (all_outside_plane)
            {
                return false;
            }
        }

        return true;
    }

    /// Conservatively accept a Hermite curve interval if its cubic Bezier control hull
    /// is not fully outside any homogeneous clip plane.
    inline bool frustum_accept_hermite_interval_margin(
            const OrbitRenderCurve::FrustumContext &frustum,
            const orbitsim::TrajectorySegment &segment,
            const double t0_s,
            const double t1_s,
            const WorldVec3 &reference_body_world,
            const WorldVec3 &align_delta_world,
            const double margin_ratio)
    {
        if (!frustum.valid)
        {
            return true;
        }

        const double dt_s = t1_s - t0_s;
        if (!std::isfinite(dt_s) || !(dt_s > 0.0))
        {
            return false;
        }

        const orbitsim::State state0 = orbitsim::trajectory_segment_state_at(segment, t0_s);
        const orbitsim::State state1 = orbitsim::trajectory_segment_state_at(segment, t1_s);
        if (!vec3_is_finite(state0.position_m) ||
            !vec3_is_finite(state0.velocity_mps) ||
            !vec3_is_finite(state1.position_m) ||
            !vec3_is_finite(state1.velocity_mps))
        {
            return false;
        }

        const WorldVec3 p0 = reference_body_world + state0.position_m + align_delta_world;
        const WorldVec3 p3 = reference_body_world + state1.position_m + align_delta_world;
        const WorldVec3 p1 = p0 + (state0.velocity_mps * (dt_s / 3.0));
        const WorldVec3 p2 = p3 - (state1.velocity_mps * (dt_s / 3.0));

        return frustum_accept_clip_control_points_margin(
                std::array<glm::vec4, 4>{
                        project_world_to_clip(frustum, p0),
                        project_world_to_clip(frustum, p1),
                        project_world_to_clip(frustum, p2),
                        project_world_to_clip(frustum, p3),
                },
                frustum_safe_margin(margin_ratio));
    }
} // namespace Game
