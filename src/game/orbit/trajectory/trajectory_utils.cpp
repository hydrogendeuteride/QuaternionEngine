#include "game/orbit/trajectory/trajectory_utils.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    bool eval_segment_state(const orbitsim::TrajectorySegment &segment,
                            const double t_s,
                            orbitsim::State &out_state)
    {
        out_state = orbitsim::trajectory_segment_state_at(segment, t_s);
        return finite_state(out_state);
    }

    bool sample_trajectory_segment_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                         const double t_s,
                                         orbitsim::State &out_state,
                                         const TrajectoryBoundarySide boundary_side)
    {
        out_state = {};
        if (segments.empty() || !std::isfinite(t_s))
        {
            return false;
        }

        const bool prefer_after = boundary_side == TrajectoryBoundarySide::After ||
                                  boundary_side == TrajectoryBoundarySide::ContinuousPositionOnly;
        const auto it = prefer_after
                                ? std::upper_bound(segments.begin(),
                                                   segments.end(),
                                                   t_s,
                                                   [](const double query_t_s,
                                                      const orbitsim::TrajectorySegment &segment) {
                                                       return query_t_s < prediction_segment_end_time(segment);
                                                   })
                                : std::lower_bound(segments.begin(),
                                                   segments.end(),
                                                   t_s,
                                                   [](const orbitsim::TrajectorySegment &segment,
                                                      const double query_t_s) {
                                                       return prediction_segment_end_time(segment) < query_t_s;
                                                   });
        if (it == segments.end())
        {
            return eval_segment_state(segments.back(), prediction_segment_end_time(segments.back()), out_state);
        }

        return eval_segment_state(*it, t_s, out_state);
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
} // namespace Game
