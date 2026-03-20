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

    std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_samples(
            const std::vector<orbitsim::TrajectorySample> &samples)
    {
        std::vector<orbitsim::TrajectorySegment> out;
        if (samples.size() < 2)
        {
            return out;
        }

        out.reserve(samples.size() - 1);
        for (std::size_t i = 1; i < samples.size(); ++i)
        {
            const orbitsim::TrajectorySample &a = samples[i - 1];
            const orbitsim::TrajectorySample &b = samples[i];
            const double dt_s = b.t_s - a.t_s;
            if (!(dt_s > 0.0) || !std::isfinite(dt_s))
            {
                continue;
            }

            orbitsim::State start{};
            start.position_m = a.position_m;
            start.velocity_mps = a.velocity_mps;
            orbitsim::State end{};
            end.position_m = b.position_m;
            end.velocity_mps = b.velocity_mps;

            out.push_back(orbitsim::TrajectorySegment{
                    .t0_s = a.t_s,
                    .dt_s = dt_s,
                    .start = start,
                    .end = end,
                    .flags = 0u,
            });
        }

        return out;
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
        out_state = segment.start;
        if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s) || !std::isfinite(t_s))
        {
            return finite_vec3(out_state.position_m) && finite_vec3(out_state.velocity_mps);
        }

        double u = (t_s - segment.t0_s) / segment.dt_s;
        if (!std::isfinite(u))
        {
            u = 0.0;
        }
        u = std::clamp(u, 0.0, 1.0);

        const double u2 = u * u;
        const double u3 = u2 * u;
        const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
        const double h10 = u3 - (2.0 * u2) + u;
        const double h01 = (-2.0 * u3) + (3.0 * u2);
        const double h11 = u3 - u2;
        const double dh00 = (6.0 * u2) - (6.0 * u);
        const double dh10 = (3.0 * u2) - (4.0 * u) + 1.0;
        const double dh01 = (-6.0 * u2) + (6.0 * u);
        const double dh11 = (3.0 * u2) - (2.0 * u);

        const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
        const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
        const glm::dvec3 m0 = glm::dvec3(segment.start.velocity_mps) * segment.dt_s;
        const glm::dvec3 m1 = glm::dvec3(segment.end.velocity_mps) * segment.dt_s;
        const glm::dvec3 pos = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
        const glm::dvec3 vel = ((dh00 * p0) + (dh10 * m0) + (dh01 * p1) + (dh11 * m1)) / segment.dt_s;
        if (!finite_vec3(pos) || !finite_vec3(vel))
        {
            return false;
        }

        out_state = orbitsim::make_state(pos, vel);
        return true;
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
                                                const orbitsim::State &state_after)
    {
        if (!std::isfinite(t_s) || !finite_state(state_before) || !finite_state(state_after))
        {
            return;
        }

        constexpr double kBoundaryMergeToleranceS = 1.0e-9;
        if (!states.empty() && std::abs(states.back().t_s - t_s) <= kBoundaryMergeToleranceS)
        {
            states.back().state_after = state_after;
            return;
        }

        states.push_back(PlannedSegmentBoundaryState{
                .t_s = t_s,
                .state_before = state_before,
                .state_after = state_after,
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

            while (boundary_index < boundaries.size() &&
                   boundaries[boundary_index].t_s <= (seg_t0_s + kBoundaryEpsilonS))
            {
                if (std::abs(boundaries[boundary_index].t_s - seg_t0_s) <= kBoundaryEpsilonS)
                {
                    cursor_state = boundaries[boundary_index].state_after;
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
                            .flags = segment.flags,
                    });
                }

                cursor_t_s = boundary.t_s;
                cursor_state = boundary.state_after;
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
                        .flags = segment.flags,
                });
            }

            boundary_index = local_boundary_index;
        }

        return out;
    }
} // namespace Game
