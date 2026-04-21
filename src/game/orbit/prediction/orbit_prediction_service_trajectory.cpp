#include "game/orbit/prediction/orbit_prediction_service_internal.h"

namespace Game
{
    bool build_maneuver_preview(const orbitsim::State &ship_state,
                                const double node_time_s,
                                OrbitPredictionService::ManeuverNodePreview &out_preview)
    {
        out_preview.valid = false;
        if (!std::isfinite(node_time_s))
        {
            return false;
        }

        if (!finite_vec3(ship_state.position_m) || !finite_vec3(ship_state.velocity_mps))
        {
            return false;
        }

        out_preview.inertial_position_m = ship_state.position_m;
        out_preview.inertial_velocity_mps = ship_state.velocity_mps;
        out_preview.valid = true;
        return true;
    }

    std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_body_ephemeris(
            const orbitsim::CelestialEphemeris &ephemeris,
            const orbitsim::BodyId body_id)
    {
        std::vector<orbitsim::TrajectorySegment> out;
        if (ephemeris.empty() || body_id == orbitsim::kInvalidBodyId)
        {
            return out;
        }

        std::size_t body_index = 0;
        if (!ephemeris.body_index_for_id(body_id, &body_index))
        {
            return out;
        }

        out.reserve(ephemeris.segments.size());
        for (const orbitsim::CelestialEphemerisSegment &segment : ephemeris.segments)
        {
            if (!(segment.dt_s > 0.0) || body_index >= segment.start.size() || body_index >= segment.end.size())
            {
                continue;
            }

            const orbitsim::State &start = segment.start[body_index];
            const orbitsim::State &end = segment.end[body_index];
            if (!finite_state(start) || !finite_state(end))
            {
                continue;
            }

            out.push_back(orbitsim::TrajectorySegment{
                    .t0_s = segment.t0_s,
                    .dt_s = segment.dt_s,
                    .start = start,
                    .end = end,
                    .flags = 0u,
            });
        }

        return out;
    }

    bool eval_segment_state(const orbitsim::TrajectorySegment &segment,
                            const double t_s,
                            orbitsim::State &out_state)
    {
        out_state = orbitsim::trajectory_segment_state_at(segment, t_s);
        return finite_state(out_state);
    }

    bool sample_trajectory_segment_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                         const double t_s,
                                         orbitsim::State &out_state)
    {
        out_state = {};
        if (segments.empty() || !std::isfinite(t_s))
        {
            return false;
        }

        const auto it = std::lower_bound(segments.begin(),
                                         segments.end(),
                                         t_s,
                                         [](const orbitsim::TrajectorySegment &segment, const double query_t_s) {
                                             return prediction_segment_end_time(segment) < query_t_s;
                                         });
        if (it == segments.end())
        {
            return eval_segment_state(segments.back(), prediction_segment_end_time(segments.back()), out_state);
        }

        return eval_segment_state(*it, t_s, out_state);
    }

    void append_or_merge_planned_boundary_state(std::vector<PlannedSegmentBoundaryState> &states,
                                                const double t_s,
                                                const orbitsim::State &state_before,
                                                const orbitsim::State &state_after,
                                                const std::uint32_t flags)
    {
        if (!std::isfinite(t_s) || !finite_state(state_before) || !finite_state(state_after))
        {
            return;
        }

        constexpr double kBoundaryMergeToleranceS = 1.0e-9;
        if (!states.empty() && std::abs(states.back().t_s - t_s) <= kBoundaryMergeToleranceS)
        {
            states.back().state_after = state_after;
            states.back().flags |= flags;
            return;
        }

        states.push_back(PlannedSegmentBoundaryState{
                .t_s = t_s,
                .state_before = state_before,
                .state_after = state_after,
                .flags = flags,
        });
    }

    std::vector<orbitsim::TrajectorySegment> split_trajectory_segments_at_known_boundaries(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::vector<PlannedSegmentBoundaryState> &boundaries)
    {
        if (segments.empty() || boundaries.empty())
        {
            return segments;
        }

        constexpr double kBoundaryEpsilonS = 1.0e-9;
        std::vector<orbitsim::TrajectorySegment> out;
        out.reserve(segments.size() + boundaries.size());

        std::size_t boundary_index = 0;
        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = prediction_segment_end_time(segment);
            if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) || !(seg_t1_s > seg_t0_s))
            {
                out.push_back(segment);
                continue;
            }

            double cursor_t_s = seg_t0_s;
            orbitsim::State cursor_state = segment.start;
            std::uint32_t cursor_flags = segment.flags;

            while (boundary_index < boundaries.size() &&
                   boundaries[boundary_index].t_s <= (seg_t0_s + kBoundaryEpsilonS))
            {
                if (std::abs(boundaries[boundary_index].t_s - seg_t0_s) <= kBoundaryEpsilonS)
                {
                    cursor_state = boundaries[boundary_index].state_after;
                    cursor_flags = boundaries[boundary_index].flags;
                }
                ++boundary_index;
            }

            std::size_t local_boundary_index = boundary_index;
            while (local_boundary_index < boundaries.size())
            {
                const PlannedSegmentBoundaryState &boundary = boundaries[local_boundary_index];
                if (!(boundary.t_s < (seg_t1_s - kBoundaryEpsilonS)))
                {
                    break;
                }
                if (!(boundary.t_s > (cursor_t_s + kBoundaryEpsilonS)))
                {
                    cursor_state = boundary.state_after;
                    cursor_flags = boundary.flags;
                    ++local_boundary_index;
                    continue;
                }

                const double part_dt_s = boundary.t_s - cursor_t_s;
                if (part_dt_s > kBoundaryEpsilonS)
                {
                    out.push_back(orbitsim::TrajectorySegment{
                            .t0_s = cursor_t_s,
                            .dt_s = part_dt_s,
                            .start = cursor_state,
                            .end = boundary.state_before,
                            .flags = cursor_flags,
                    });
                }

                cursor_t_s = boundary.t_s;
                cursor_state = boundary.state_after;
                cursor_flags = boundary.flags;
                ++local_boundary_index;
            }

            const double tail_dt_s = seg_t1_s - cursor_t_s;
            if (tail_dt_s > kBoundaryEpsilonS)
            {
                out.push_back(orbitsim::TrajectorySegment{
                        .t0_s = cursor_t_s,
                        .dt_s = tail_dt_s,
                        .start = cursor_state,
                        .end = segment.end,
                        .flags = cursor_flags,
                });
            }

            boundary_index = local_boundary_index;
        }

        return out;
    }

    std::vector<orbitsim::TrajectorySegment> slice_trajectory_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const double t0_s,
            const double t1_s)
    {
        std::vector<orbitsim::TrajectorySegment> out;
        if (segments.empty() || !std::isfinite(t0_s) || !std::isfinite(t1_s) || !(t1_s > t0_s))
        {
            return out;
        }

        out.reserve(segments.size());
        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = prediction_segment_end_time(segment);
            if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) || !(seg_t1_s > seg_t0_s))
            {
                continue;
            }

            const double overlap_t0_s = std::max(seg_t0_s, t0_s);
            const double overlap_t1_s = std::min(seg_t1_s, t1_s);
            if (!(overlap_t1_s > overlap_t0_s))
            {
                continue;
            }

            orbitsim::State start_state = segment.start;
            orbitsim::State end_state = segment.end;
            if (overlap_t0_s > seg_t0_s && !eval_segment_state(segment, overlap_t0_s, start_state))
            {
                continue;
            }
            if (overlap_t1_s < seg_t1_s && !eval_segment_state(segment, overlap_t1_s, end_state))
            {
                continue;
            }

            std::uint32_t flags = segment.flags;
            if (overlap_t0_s > seg_t0_s || overlap_t1_s < seg_t1_s)
            {
                flags |= orbitsim::kTrajectorySegmentFlagForcedBoundary;
            }

            out.push_back(orbitsim::TrajectorySegment{
                    .t0_s = overlap_t0_s,
                    .dt_s = overlap_t1_s - overlap_t0_s,
                    .start = start_state,
                    .end = end_state,
                    .flags = flags,
            });
        }

        return out;
    }
} // namespace Game
