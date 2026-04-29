#pragma once

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/prediction/prediction_diagnostics_util.h"
#include "game/orbit/trajectory/trajectory_utils.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"
#include "game/states/gameplay/prediction/prediction_trajectory_sampler.h"

#include "orbitsim/math.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace Game::PredictionCacheInternal
{
    using CancelCheck = std::function<bool()>;

    inline constexpr orbitsim::SpacecraftId kPlayerDisplayTargetSpacecraftId =
            PredictionTrajectorySampler::kPlayerDisplayTargetSpacecraftId;

    inline bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
                                                 const double query_time_s,
                                                 orbitsim::State &out_state)
    {
        out_state = {};
        if (trajectory.empty() || !std::isfinite(query_time_s))
        {
            return false;
        }

        const auto it_hi = std::lower_bound(trajectory.cbegin(),
                                            trajectory.cend(),
                                            query_time_s,
                                            [](const orbitsim::TrajectorySample &sample, const double t_s) {
                                                return sample.t_s < t_s;
                                            });
        const std::size_t i_hi = static_cast<std::size_t>(std::distance(trajectory.cbegin(), it_hi));
        if (i_hi == 0)
        {
            out_state = orbitsim::make_state(trajectory.front().position_m, trajectory.front().velocity_mps);
            return true;
        }
        if (i_hi >= trajectory.size())
        {
            out_state = orbitsim::make_state(trajectory.back().position_m, trajectory.back().velocity_mps);
            return true;
        }

        const orbitsim::TrajectorySample &a = trajectory[i_hi - 1];
        const orbitsim::TrajectorySample &b = trajectory[i_hi];
        const orbitsim::State sampled = OrbitPredictionMath::sample_pair_state(a, b, query_time_s);
        if (!finite_vec3(sampled.position_m) || !finite_vec3(sampled.velocity_mps))
        {
            return false;
        }

        out_state = sampled;
        return true;
    }

    inline bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                                  const double query_time_s,
                                                  orbitsim::State &out_state,
                                                  const TrajectoryBoundarySide boundary_side =
                                                          TrajectoryBoundarySide::Before)
    {
        return sample_trajectory_segment_state(segments, query_time_s, out_state, boundary_side);
    }

    inline orbitsim::SpacecraftStateLookup build_player_lookup(
            const std::vector<orbitsim::TrajectorySample> &trajectory)
    {
        if (trajectory.size() < 2)
        {
            return nullptr;
        }

        return [&trajectory](const orbitsim::SpacecraftId spacecraft_id, const double t_s)
                -> std::optional<orbitsim::State> {
            if (spacecraft_id != kPlayerDisplayTargetSpacecraftId)
            {
                return std::nullopt;
            }

            orbitsim::State sampled{};
            if (!sample_prediction_inertial_state(trajectory, t_s, sampled))
            {
                return std::nullopt;
            }
            return sampled;
        };
    }

    inline orbitsim::SpacecraftStateLookup build_player_lookup(
            const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        if (segments.empty())
        {
            return nullptr;
        }

        return [&segments](const orbitsim::SpacecraftId spacecraft_id, const double t_s)
                -> std::optional<orbitsim::State> {
            if (spacecraft_id != kPlayerDisplayTargetSpacecraftId)
            {
                return std::nullopt;
            }

            orbitsim::State sampled{};
            if (!sample_prediction_inertial_state(segments, t_s, sampled, TrajectoryBoundarySide::After))
            {
                return std::nullopt;
            }
            return sampled;
        };
    }

    inline std::vector<double> collect_maneuver_node_times(const PredictionSolverTrajectoryCache &solver)
    {
        std::vector<double> out;
        out.reserve(solver.maneuver_previews.size());
        for (const auto &preview : solver.maneuver_previews)
        {
            if (std::isfinite(preview.t_s))
            {
                out.push_back(preview.t_s);
            }
        }
        return out;
    }

    inline std::vector<double> collect_maneuver_node_times(const OrbitPredictionCache &cache)
    {
        return collect_maneuver_node_times(cache.solver);
    }

    inline std::vector<orbitsim::TrajectorySample> sample_prediction_segments(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            std::size_t total_sample_budget,
            const std::vector<double> & /*node_times_s*/ = {})
    {
        return resample_segments_uniform(segments, std::max<std::size_t>(total_sample_budget, 2));
    }

    inline void append_unique_prediction_sample(std::vector<orbitsim::TrajectorySample> &dst,
                                                const orbitsim::TrajectorySample &sample)
    {
        if (!dst.empty() && std::abs(dst.back().t_s - sample.t_s) <= 1.0e-9)
        {
            dst.back() = sample;
            return;
        }
        dst.push_back(sample);
    }

    inline std::vector<orbitsim::TrajectorySample> collect_segment_boundary_samples(
            const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        std::vector<orbitsim::TrajectorySample> out;
        if (segments.empty())
        {
            return out;
        }

        out.reserve(segments.size() + 1u);
        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.t0_s))
            {
                continue;
            }

            append_unique_prediction_sample(out,
                                            orbitsim::TrajectorySample{
                                                    .t_s = segment.t0_s,
                                                    .position_m = segment.start.position_m,
                                                    .velocity_mps = segment.start.velocity_mps,
                                            });

            append_unique_prediction_sample(out,
                                            orbitsim::TrajectorySample{
                                                    .t_s = prediction_segment_end_time(segment),
                                                    .position_m = segment.end.position_m,
                                                    .velocity_mps = segment.end.velocity_mps,
                                            });
        }

        return out;
    }

    inline bool slice_trajectory_segments_from_cursor(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const double t0_s,
            const double t1_s,
            std::size_t &cursor,
            std::vector<orbitsim::TrajectorySegment> &out_segments)
    {
        out_segments.clear();
        if (segments.empty() || !std::isfinite(t0_s) || !std::isfinite(t1_s) || !(t1_s > t0_s))
        {
            return true;
        }

        cursor = std::min(cursor, segments.size());
        while (cursor < segments.size())
        {
            const orbitsim::TrajectorySegment &segment = segments[cursor];
            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = prediction_segment_end_time(segment);
            if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) || !(seg_t1_s > seg_t0_s))
            {
                ++cursor;
                continue;
            }
            if (seg_t1_s <= t0_s)
            {
                ++cursor;
                continue;
            }
            break;
        }

        out_segments.reserve(8u);
        std::size_t i = cursor;
        for (; i < segments.size(); ++i)
        {
            const orbitsim::TrajectorySegment &segment = segments[i];
            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = prediction_segment_end_time(segment);
            if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) || !(seg_t1_s > seg_t0_s))
            {
                continue;
            }
            if (seg_t0_s >= t1_s)
            {
                break;
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
                return false;
            }
            if (overlap_t1_s < seg_t1_s && !eval_segment_state(segment, overlap_t1_s, end_state))
            {
                return false;
            }

            std::uint32_t flags = segment.flags;
            if (overlap_t0_s > seg_t0_s || overlap_t1_s < seg_t1_s)
            {
                flags |= orbitsim::kTrajectorySegmentFlagForcedBoundary;
            }

            out_segments.push_back(orbitsim::TrajectorySegment{
                    .t0_s = overlap_t0_s,
                    .dt_s = overlap_t1_s - overlap_t0_s,
                    .start = start_state,
                    .end = end_state,
                    .flags = flags,
            });

            if (seg_t1_s >= t1_s)
            {
                break;
            }
        }

        const double cursor_advance_epsilon_s = continuity_time_epsilon_s(t1_s);
        while (cursor < segments.size() &&
               prediction_segment_end_time(segments[cursor]) <= (t1_s + cursor_advance_epsilon_s))
        {
            ++cursor;
        }

        return true;
    }

    inline void update_derived_diagnostics(OrbitPredictionDerivedDiagnostics *diagnostics,
                                           const PredictionDisplayFrameCache &display,
                                           const PredictionDerivedStatus status)
    {
        if (!diagnostics)
        {
            return;
        }

        diagnostics->status = status;
        diagnostics->frame_segment_count =
                diagnostics->frame_base.accepted_segments > 0 ? diagnostics->frame_base.accepted_segments
                                                              : display.trajectory_segments_frame.size();
        diagnostics->frame_segment_count_planned =
                diagnostics->frame_planned.accepted_segments > 0 ? diagnostics->frame_planned.accepted_segments
                                                                 : display.trajectory_segments_frame_planned.size();
        diagnostics->frame_sample_count = display.trajectory_frame.size();
        diagnostics->frame_sample_count_planned = display.trajectory_frame_planned.size();
    }

    inline void update_derived_diagnostics(OrbitPredictionDerivedDiagnostics *diagnostics,
                                           const OrbitPredictionCache &cache,
                                           const PredictionDerivedStatus status)
    {
        update_derived_diagnostics(diagnostics, cache.display, status);
    }

    inline void accumulate_stage_diagnostics(OrbitPredictionService::AdaptiveStageDiagnostics &dst,
                                             const OrbitPredictionService::AdaptiveStageDiagnostics &src)
    {
        const double dst_dt_weight = static_cast<double>(dst.accepted_segments);
        const double src_dt_weight = static_cast<double>(src.accepted_segments);
        const bool dst_has_dt = dst_dt_weight > 0.0;
        const bool src_has_dt = src_dt_weight > 0.0;

        dst.requested_duration_s += src.requested_duration_s;
        dst.covered_duration_s += src.covered_duration_s;
        dst.accepted_segments += src.accepted_segments;
        dst.rejected_splits += src.rejected_splits;
        dst.forced_boundary_splits += src.forced_boundary_splits;
        dst.frame_resegmentation_count += src.frame_resegmentation_count;

        if (src_has_dt)
        {
            dst.min_dt_s = dst_has_dt ? std::min(dst.min_dt_s, src.min_dt_s) : src.min_dt_s;
            dst.max_dt_s = dst_has_dt ? std::max(dst.max_dt_s, src.max_dt_s) : src.max_dt_s;
        }

        if ((dst_dt_weight + src_dt_weight) > 0.0)
        {
            const double dst_avg = std::isfinite(dst.avg_dt_s) ? dst.avg_dt_s : 0.0;
            const double src_avg = std::isfinite(src.avg_dt_s) ? src.avg_dt_s : 0.0;
            dst.avg_dt_s = ((dst_avg * dst_dt_weight) + (src_avg * src_dt_weight)) / (dst_dt_weight + src_dt_weight);
        }

        dst.hard_cap_hit = dst.hard_cap_hit || src.hard_cap_hit;
        dst.cancelled = dst.cancelled || src.cancelled;
        dst.cache_reused = dst.cache_reused || src.cache_reused;
        dst.maneuver_apply_failed_count += src.maneuver_apply_failed_count;
        if (dst.maneuver_apply_failed_node_id < 0 && src.maneuver_apply_failed_node_id >= 0)
        {
            dst.maneuver_apply_failed_node_id = src.maneuver_apply_failed_node_id;
        }
    }

    orbitsim::FrameSegmentTransformOptions build_frame_segment_transform_options(
            const orbitsim::TrajectoryFrameSpec &frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &inertial_segments,
            const CancelCheck &cancel_requested = {});
} // namespace Game::PredictionCacheInternal
