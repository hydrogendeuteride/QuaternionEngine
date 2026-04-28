#pragma once

#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace Game
{
    constexpr double kContinuityMinTimeEpsilonS = 1.0e-6;
    constexpr double kContinuityMinPosToleranceM = 1.0e-3;
    constexpr double kContinuityMinVelToleranceMps = 1.0e-6;

    enum class TrajectoryBoundarySide
    {
        // At t == segment boundary, return the state from the segment ending at t.
        Before,
        // At t == segment boundary, return the state from the segment starting at t.
        After,
        // Position-only callers should not interpret boundary velocity.
        ContinuousPositionOnly,
    };

    inline bool finite_vec3(const glm::dvec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    inline bool finite_state(const orbitsim::State &state)
    {
        return finite_vec3(state.position_m) && finite_vec3(state.velocity_mps);
    }

    inline double prediction_segment_end_time(const orbitsim::TrajectorySegment &segment)
    {
        return segment.t0_s + segment.dt_s;
    }

    inline double prediction_segment_span_s(const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        if (segments.empty())
        {
            return 0.0;
        }

        const double t0_s = segments.front().t0_s;
        const double t1_s = prediction_segment_end_time(segments.back());
        if (!std::isfinite(t0_s) || !std::isfinite(t1_s) || !(t1_s > t0_s))
        {
            return 0.0;
        }

        return t1_s - t0_s;
    }

    inline double continuity_time_epsilon_s(const double reference_time_s)
    {
        return std::max(kContinuityMinTimeEpsilonS, std::abs(reference_time_s) * 1.0e-12);
    }

    inline double continuity_pos_tolerance_m(const orbitsim::State &a, const orbitsim::State &b)
    {
        const double scale_m =
                std::max({1.0, glm::length(glm::dvec3(a.position_m)), glm::length(glm::dvec3(b.position_m))});
        return std::max(kContinuityMinPosToleranceM, scale_m * 1.0e-10);
    }

    inline double continuity_vel_tolerance_mps(const orbitsim::State &a, const orbitsim::State &b)
    {
        const double scale_mps =
                std::max({1.0, glm::length(glm::dvec3(a.velocity_mps)), glm::length(glm::dvec3(b.velocity_mps))});
        return std::max(kContinuityMinVelToleranceMps, scale_mps * 1.0e-10);
    }

    inline bool states_are_continuous(const orbitsim::State &a, const orbitsim::State &b)
    {
        if (!finite_state(a) || !finite_state(b))
        {
            return false;
        }

        const double pos_gap_m = glm::length(glm::dvec3(a.position_m - b.position_m));
        const double vel_gap_mps = glm::length(glm::dvec3(a.velocity_mps - b.velocity_mps));
        return pos_gap_m <= continuity_pos_tolerance_m(a, b) &&
               vel_gap_mps <= continuity_vel_tolerance_mps(a, b);
    }

    inline bool states_are_position_continuous(const orbitsim::State &a, const orbitsim::State &b)
    {
        if (!finite_state(a) || !finite_state(b))
        {
            return false;
        }

        const double pos_gap_m = glm::length(glm::dvec3(a.position_m - b.position_m));
        return pos_gap_m <= continuity_pos_tolerance_m(a, b);
    }

    inline bool segment_allows_velocity_discontinuity(const orbitsim::TrajectorySegment &segment)
    {
        return (segment.flags & (orbitsim::kTrajectorySegmentFlagImpulseBoundary |
                                 orbitsim::kTrajectorySegmentFlagBurnBoundary)) != 0u;
    }

    inline bool validate_trajectory_segment_continuity(const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        if (segments.empty())
        {
            return false;
        }

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            const orbitsim::TrajectorySegment &segment = segments[i];
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.t0_s) || !std::isfinite(segment.dt_s) ||
                !finite_state(segment.start) || !finite_state(segment.end))
            {
                return false;
            }

            if (i == 0)
            {
                continue;
            }

            const orbitsim::TrajectorySegment &prev = segments[i - 1];
            const double prev_t1_s = prediction_segment_end_time(prev);
            if (std::abs(segment.t0_s - prev_t1_s) > continuity_time_epsilon_s(prev_t1_s))
            {
                return false;
            }

            if (segment_allows_velocity_discontinuity(segment))
            {
                if (!states_are_position_continuous(prev.end, segment.start))
                {
                    return false;
                }
                continue;
            }

            if (!states_are_continuous(prev.end, segment.start))
            {
                return false;
            }
        }

        return true;
    }

    inline bool trajectory_segments_cover_window(const std::vector<orbitsim::TrajectorySegment> &segments,
                                                 const double sim_time_s,
                                                 const double required_duration_s)
    {
        if (segments.empty() || !std::isfinite(sim_time_s))
        {
            return false;
        }

        const double end_time_s = prediction_segment_end_time(segments.back());
        const double start_epsilon_s = continuity_time_epsilon_s(sim_time_s);
        const double end_epsilon_s = continuity_time_epsilon_s(end_time_s);
        const double required_end_s = sim_time_s + std::max(0.0, required_duration_s);
        return segments.front().t0_s <= (sim_time_s + start_epsilon_s) &&
               end_time_s >= (required_end_s - end_epsilon_s);
    }

    bool eval_segment_state(const orbitsim::TrajectorySegment &segment,
                            double t_s,
                            orbitsim::State &out_state);

    bool sample_trajectory_segment_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                         double t_s,
                                         orbitsim::State &out_state,
                                         TrajectoryBoundarySide boundary_side = TrajectoryBoundarySide::Before);

    std::vector<orbitsim::TrajectorySegment> slice_trajectory_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            double t0_s,
            double t1_s);

    std::vector<orbitsim::TrajectorySample> resample_segments_uniform(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            std::size_t sample_count);
} // namespace Game
