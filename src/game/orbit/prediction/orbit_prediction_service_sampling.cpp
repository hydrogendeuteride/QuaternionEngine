#include "game/orbit/prediction/orbit_prediction_service_internal.h"

namespace Game
{
    std::vector<orbitsim::TrajectorySample> resample_segments_uniform(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const std::size_t sample_count)
    {
        if (segments.empty() || sample_count < 2)
        {
            return {};
        }

        const double t0 = segments.front().t0_s;
        const double t1 = prediction_segment_end_time(segments.back());
        if (!std::isfinite(t0) || !std::isfinite(t1) || !(t1 > t0))
        {
            return {};
        }

        const double dt = (t1 - t0) / static_cast<double>(sample_count - 1);

        std::vector<orbitsim::TrajectorySample> out;
        out.reserve(sample_count);
        std::size_t segment_index = 0;

        for (std::size_t i = 0; i < sample_count; ++i)
        {
            const double t = (i < sample_count - 1)
                                     ? (t0 + static_cast<double>(i) * dt)
                                     : t1;
            while ((segment_index + 1) < segments.size() &&
                   prediction_segment_end_time(segments[segment_index]) < t)
            {
                ++segment_index;
            }

            const orbitsim::TrajectorySegment &segment =
                    (segment_index < segments.size()) ? segments[segment_index] : segments.back();
            orbitsim::State state{};
            if (eval_segment_state(segment, t, state))
            {
                out.push_back(orbitsim::TrajectorySample{
                        .t_s = t,
                        .position_m = state.position_m,
                        .velocity_mps = state.velocity_mps,
                });
            }
        }

        return out;
    }

    std::vector<orbitsim::TrajectorySample> resample_ephemeris_uniform(
            const orbitsim::CelestialEphemeris &ephemeris,
            const orbitsim::BodyId body_id,
            const double t0_s,
            const double t1_s,
            const std::size_t sample_count)
    {
        if (body_id == orbitsim::kInvalidBodyId || !(t1_s > t0_s) || sample_count < 2)
        {
            return {};
        }

        const double dt = (t1_s - t0_s) / static_cast<double>(sample_count - 1);

        std::vector<orbitsim::TrajectorySample> out;
        out.reserve(sample_count);

        for (std::size_t i = 0; i < sample_count; ++i)
        {
            const double t = (i < sample_count - 1)
                                     ? (t0_s + static_cast<double>(i) * dt)
                                     : t1_s;
            const orbitsim::State state = ephemeris.body_state_at_by_id(body_id, t);
            if (!finite_vec3(state.position_m) || !finite_vec3(state.velocity_mps))
            {
                continue;
            }

            out.push_back(orbitsim::TrajectorySample{
                    .t_s = t,
                    .position_m = state.position_m,
                    .velocity_mps = state.velocity_mps,
            });
        }

        return out;
    }
} // namespace Game
