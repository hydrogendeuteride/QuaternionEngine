#pragma once

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include "orbitsim/math.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace Game::PredictionCacheInternal
{
    inline constexpr orbitsim::SpacecraftId kPlayerDisplayTargetSpacecraftId =
            static_cast<orbitsim::SpacecraftId>(0x7000'0001u);

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
                                                 orbitsim::State &out_state)
    {
        return sample_trajectory_segment_state(segments, query_time_s, out_state);
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
            if (!sample_prediction_inertial_state(segments, t_s, sampled))
            {
                return std::nullopt;
            }
            return sampled;
        };
    }

    inline std::vector<double> collect_maneuver_node_times(const OrbitPredictionCache &cache)
    {
        std::vector<double> out;
        out.reserve(cache.maneuver_previews.size());
        for (const auto &preview : cache.maneuver_previews)
        {
            if (std::isfinite(preview.t_s))
            {
                out.push_back(preview.t_s);
            }
        }
        return out;
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
                                           const OrbitPredictionCache &cache,
                                           const PredictionDerivedStatus status)
    {
        if (!diagnostics)
        {
            return;
        }

        diagnostics->status = status;
        diagnostics->frame_segment_count =
                diagnostics->frame_base.accepted_segments > 0 ? diagnostics->frame_base.accepted_segments
                                                              : cache.trajectory_segments_frame.size();
        diagnostics->frame_segment_count_planned =
                diagnostics->frame_planned.accepted_segments > 0 ? diagnostics->frame_planned.accepted_segments
                                                                 : cache.trajectory_segments_frame_planned.size();
        diagnostics->frame_sample_count = cache.trajectory_frame.size();
        diagnostics->frame_sample_count_planned = cache.trajectory_frame_planned.size();
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
    }

    inline orbitsim::FrameSegmentTransformOptions build_frame_segment_transform_options(
            const orbitsim::TrajectoryFrameSpec &frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &inertial_segments,
            const CancelCheck &cancel_requested = {})
    {
        orbitsim::FrameSegmentTransformOptions out{};
        const double start_time_s = inertial_segments.empty() ? 0.0 : inertial_segments.front().t0_s;
        const double end_time_s =
                inertial_segments.empty() ? 0.0 : prediction_segment_end_time(inertial_segments.back());
        const double horizon_s = std::max(0.0, end_time_s - start_time_s);
        const bool sensitive = frame_spec.type == orbitsim::TrajectoryFrameType::Synodic ||
                               frame_spec.type == orbitsim::TrajectoryFrameType::LVLH;

        out.min_dt_s = OrbitPredictionTuning::kAdaptiveFrameTransformMinDtS;
        out.max_dt_s = sensitive
                               ? std::min(OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtSensitiveS,
                                          OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtS)
                               : OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtS;
        out.soft_max_segments =
                std::max<std::size_t>(OrbitPredictionTuning::kAdaptiveFrameTransformSoftMaxSegments,
                                      inertial_segments.size());
        out.hard_max_segments =
                std::max<std::size_t>(OrbitPredictionTuning::kAdaptiveFrameTransformHardMaxSegments,
                                      out.soft_max_segments);
        out.tolerance.pos_near_m = OrbitPredictionTuning::kAdaptiveFrameTransformPosTolNearM;
        out.tolerance.pos_far_m = OrbitPredictionTuning::kAdaptiveFrameTransformPosTolFarM;
        out.tolerance.vel_near_mps = OrbitPredictionTuning::kAdaptiveFrameTransformVelTolNearMps;
        out.tolerance.vel_far_mps = OrbitPredictionTuning::kAdaptiveFrameTransformVelTolFarMps;
        out.tolerance.rel_pos_floor = OrbitPredictionTuning::kAdaptiveFrameTransformRelPosFloor;
        out.tolerance.rel_vel_floor = OrbitPredictionTuning::kAdaptiveFrameTransformRelVelFloor;
        if (sensitive)
        {
            out.tolerance.pos_near_m *= 0.5;
            out.tolerance.pos_far_m *= 0.25;
            out.tolerance.vel_near_mps *= 0.5;
            out.tolerance.vel_far_mps *= 0.25;
        }
        out.cancel_requested = cancel_requested;
        return out;
    }

    inline bool rebuild_prediction_frame_cache(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested = {},
            OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
            const bool build_planned_render_curve = true)
    {
        if (diagnostics)
        {
            *diagnostics = {};
        }

        cache.trajectory_frame.clear();
        cache.trajectory_frame_planned.clear();
        cache.trajectory_segments_frame.clear();
        cache.trajectory_segments_frame_planned.clear();
        cache.render_curve_frame.clear();
        cache.render_curve_frame_planned.clear();
        cache.resolved_frame_spec = {};
        cache.resolved_frame_spec_valid = false;
        cache.trajectory_analysis_bci.clear();
        cache.trajectory_segments_analysis_bci.clear();
        cache.analysis_cache_body_id = orbitsim::kInvalidBodyId;
        cache.analysis_cache_valid = false;
        cache.metrics_valid = false;

        const auto &base_ephemeris = cache.resolved_shared_ephemeris();
        const auto &base_bodies = cache.resolved_massive_bodies();
        const auto &base_samples = cache.resolved_trajectory_inertial();
        const auto &base_segments = cache.resolved_trajectory_segments_inertial();

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            cache.trajectory_frame = base_samples;
            cache.trajectory_frame_planned = cache.trajectory_inertial_planned;
            cache.trajectory_segments_frame = base_segments;
            cache.trajectory_segments_frame_planned = cache.trajectory_segments_inertial_planned;
            if (!validate_trajectory_segment_continuity(cache.trajectory_segments_frame))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (!cache.trajectory_segments_frame_planned.empty() &&
                !validate_trajectory_segment_continuity(cache.trajectory_segments_frame_planned))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (diagnostics)
            {
                diagnostics->frame_base = make_stage_diagnostics_from_segments(
                        cache.trajectory_segments_frame,
                        prediction_segment_span_s(base_segments));
                diagnostics->frame_planned = make_stage_diagnostics_from_segments(
                        cache.trajectory_segments_frame_planned,
                        prediction_segment_span_s(cache.trajectory_segments_inertial_planned));
            }
        }
        else
        {
            if (!base_ephemeris || base_ephemeris->empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingEphemeris);
                return false;
            }

            const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
            const std::size_t base_sample_budget = std::max<std::size_t>(base_samples.size(), 2);
            const std::size_t planned_sample_budget =
                    cache.trajectory_inertial_planned.size() >= 2 ? cache.trajectory_inertial_planned.size()
                                                                  : base_sample_budget;
            const std::vector<double> node_times = collect_maneuver_node_times(cache);

            const orbitsim::FrameSegmentTransformOptions base_opt =
                    build_frame_segment_transform_options(
                            resolved_frame_spec,
                            base_segments,
                            cancel_requested);
            orbitsim::FrameSegmentTransformDiagnostics base_frame_diag{};
            cache.trajectory_segments_frame = orbitsim::transform_trajectory_segments_to_frame_spec(
                    base_segments,
                    *base_ephemeris,
                    base_bodies,
                    resolved_frame_spec,
                    base_opt,
                    player_lookup,
                    &base_frame_diag);
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (cache.trajectory_segments_frame.empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameTransformFailed);
                return false;
            }
            if (!validate_trajectory_segment_continuity(cache.trajectory_segments_frame))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (diagnostics)
            {
                diagnostics->frame_base = make_stage_diagnostics_from_adaptive(
                        base_frame_diag,
                        prediction_segment_span_s(base_segments));
                diagnostics->frame_base.accepted_segments = cache.trajectory_segments_frame.size();
                diagnostics->frame_base.covered_duration_s = prediction_segment_span_s(cache.trajectory_segments_frame);
                diagnostics->frame_base.frame_resegmentation_count = base_frame_diag.frame_resegmentation_count;
            }
            cache.trajectory_frame = sample_prediction_segments(cache.trajectory_segments_frame, base_sample_budget);
            if (cache.trajectory_frame.size() < 2)
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameSamplesUnavailable);
                return false;
            }

            if (!cache.trajectory_segments_inertial_planned.empty())
            {
                const orbitsim::FrameSegmentTransformOptions planned_opt =
                        build_frame_segment_transform_options(
                                resolved_frame_spec,
                                cache.trajectory_segments_inertial_planned,
                                cancel_requested);
                orbitsim::FrameSegmentTransformDiagnostics planned_frame_diag{};
                cache.trajectory_segments_frame_planned = orbitsim::transform_trajectory_segments_to_frame_spec(
                        cache.trajectory_segments_inertial_planned,
                        *base_ephemeris,
                        base_bodies,
                        resolved_frame_spec,
                        planned_opt,
                        player_lookup,
                        &planned_frame_diag);
                if (cancel_requested && cancel_requested())
                {
                    update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                    return false;
                }
                if (!cache.trajectory_segments_frame_planned.empty())
                {
                    if (!validate_trajectory_segment_continuity(cache.trajectory_segments_frame_planned))
                    {
                        update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                        return false;
                    }
                    if (diagnostics)
                    {
                        diagnostics->frame_planned = make_stage_diagnostics_from_adaptive(
                                planned_frame_diag,
                                prediction_segment_span_s(cache.trajectory_segments_inertial_planned));
                        diagnostics->frame_planned.accepted_segments = cache.trajectory_segments_frame_planned.size();
                        diagnostics->frame_planned.covered_duration_s =
                                prediction_segment_span_s(cache.trajectory_segments_frame_planned);
                        diagnostics->frame_planned.frame_resegmentation_count =
                                planned_frame_diag.frame_resegmentation_count;
                    }
                    cache.trajectory_frame_planned = sample_prediction_segments(
                            cache.trajectory_segments_frame_planned,
                            planned_sample_budget,
                            node_times);
                }
            }
        }

        if (cache.trajectory_frame.size() < 2 || cache.trajectory_segments_frame.empty())
        {
            update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingSolverData);
            return false;
        }

        cache.render_curve_frame = OrbitRenderCurve::build(cache.trajectory_segments_frame);
        cache.render_curve_frame_planned =
                (build_planned_render_curve && !cache.trajectory_segments_frame_planned.empty())
                        ? OrbitRenderCurve::build(cache.trajectory_segments_frame_planned)
                        : OrbitRenderCurve{};
        cache.resolved_frame_spec = resolved_frame_spec;
        cache.resolved_frame_spec_valid = true;
        update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Success);
        return true;
    }

    inline bool rebuild_prediction_planned_frame_cache(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested = {},
            OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
            const bool build_planned_render_curve = true)
    {
        if (diagnostics)
        {
            *diagnostics = {};
        }

        cache.trajectory_frame_planned.clear();
        cache.trajectory_segments_frame_planned.clear();
        cache.render_curve_frame_planned.clear();
        cache.resolved_frame_spec = {};
        cache.resolved_frame_spec_valid = false;

        if (cache.trajectory_segments_inertial_planned.empty())
        {
            cache.resolved_frame_spec = resolved_frame_spec;
            cache.resolved_frame_spec_valid = true;
            update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Success);
            return true;
        }

        const auto &base_ephemeris = cache.resolved_shared_ephemeris();
        const auto &base_bodies = cache.resolved_massive_bodies();
        const auto &base_samples = cache.resolved_trajectory_inertial();
        const std::size_t base_sample_budget = std::max<std::size_t>(base_samples.size(), 2);
        const std::size_t planned_sample_budget =
                cache.trajectory_inertial_planned.size() >= 2 ? cache.trajectory_inertial_planned.size()
                                                              : base_sample_budget;
        const std::vector<double> node_times = collect_maneuver_node_times(cache);

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            cache.trajectory_segments_frame_planned = cache.trajectory_segments_inertial_planned;
            if (!validate_trajectory_segment_continuity(cache.trajectory_segments_frame_planned))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }

            cache.trajectory_frame_planned =
                    cache.trajectory_inertial_planned.size() >= 2
                            ? cache.trajectory_inertial_planned
                            : sample_prediction_segments(cache.trajectory_segments_frame_planned,
                                                         planned_sample_budget,
                                                         node_times);
            if (diagnostics)
            {
                diagnostics->frame_planned = make_stage_diagnostics_from_segments(
                        cache.trajectory_segments_frame_planned,
                        prediction_segment_span_s(cache.trajectory_segments_inertial_planned));
            }
        }
        else
        {
            if (!base_ephemeris || base_ephemeris->empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingEphemeris);
                return false;
            }

            const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
            const orbitsim::FrameSegmentTransformOptions planned_opt =
                    build_frame_segment_transform_options(
                            resolved_frame_spec,
                            cache.trajectory_segments_inertial_planned,
                            cancel_requested);
            orbitsim::FrameSegmentTransformDiagnostics planned_frame_diag{};
            cache.trajectory_segments_frame_planned = orbitsim::transform_trajectory_segments_to_frame_spec(
                    cache.trajectory_segments_inertial_planned,
                    *base_ephemeris,
                    base_bodies,
                    resolved_frame_spec,
                    planned_opt,
                    player_lookup,
                    &planned_frame_diag);
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (cache.trajectory_segments_frame_planned.empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameTransformFailed);
                return false;
            }

            if (!cache.trajectory_segments_frame_planned.empty())
            {
                if (!validate_trajectory_segment_continuity(cache.trajectory_segments_frame_planned))
                {
                    update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                    return false;
                }
                if (diagnostics)
                {
                    diagnostics->frame_planned = make_stage_diagnostics_from_adaptive(
                            planned_frame_diag,
                            prediction_segment_span_s(cache.trajectory_segments_inertial_planned));
                    diagnostics->frame_planned.accepted_segments = cache.trajectory_segments_frame_planned.size();
                    diagnostics->frame_planned.covered_duration_s =
                            prediction_segment_span_s(cache.trajectory_segments_frame_planned);
                    diagnostics->frame_planned.frame_resegmentation_count = planned_frame_diag.frame_resegmentation_count;
                }
                cache.trajectory_frame_planned = sample_prediction_segments(
                        cache.trajectory_segments_frame_planned,
                        planned_sample_budget,
                        node_times);
            }
        }

        cache.render_curve_frame_planned =
                (build_planned_render_curve && !cache.trajectory_segments_frame_planned.empty())
                        ? OrbitRenderCurve::build(cache.trajectory_segments_frame_planned)
                        : OrbitRenderCurve{};
        cache.resolved_frame_spec = resolved_frame_spec;
        cache.resolved_frame_spec_valid = true;
        update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Success);
        return true;
    }

    inline bool build_prediction_streamed_planned_chunk(
            OrbitChunk &out_chunk,
            OrbitPredictionService::AdaptiveStageDiagnostics *out_stage_diagnostics,
            const OrbitPredictionCache &cache,
            const OrbitPredictionService::StreamedPlannedChunk &streamed_chunk,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested = {},
            OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
            const bool build_chunk_render_curve = false,
            const bool use_dense_chunk_samples = true)
    {
        out_chunk = {};
        if (out_stage_diagnostics)
        {
            *out_stage_diagnostics = {};
        }

        const OrbitPredictionService::PublishedChunk &published_chunk = streamed_chunk.published_chunk;
        if (!published_chunk.includes_planned_path ||
            streamed_chunk.trajectory_segments_inertial.empty() ||
            !std::isfinite(published_chunk.t0_s) ||
            !std::isfinite(published_chunk.t1_s) ||
            !(published_chunk.t1_s > published_chunk.t0_s))
        {
            update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingSolverData);
            return false;
        }

        std::vector<orbitsim::TrajectorySegment> frame_segments{};
        std::vector<orbitsim::TrajectorySample> frame_samples{};
        const std::size_t sample_budget =
                std::max<std::size_t>(streamed_chunk.trajectory_inertial.size(),
                                      std::max<std::size_t>(2u, streamed_chunk.trajectory_segments_inertial.size() + 1u));
        std::vector<double> node_times_s;
        node_times_s.reserve(streamed_chunk.maneuver_previews.size());
        for (const OrbitPredictionService::ManeuverNodePreview &preview : streamed_chunk.maneuver_previews)
        {
            if (std::isfinite(preview.t_s))
            {
                node_times_s.push_back(preview.t_s);
            }
        }

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            frame_segments = streamed_chunk.trajectory_segments_inertial;
            if (!validate_trajectory_segment_continuity(frame_segments))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }

            frame_samples =
                    use_dense_chunk_samples && streamed_chunk.trajectory_inertial.size() >= 2
                            ? streamed_chunk.trajectory_inertial
                            : (use_dense_chunk_samples
                                       ? sample_prediction_segments(frame_segments, sample_budget, node_times_s)
                                       : collect_segment_boundary_samples(frame_segments));
            if (out_stage_diagnostics)
            {
                *out_stage_diagnostics = make_stage_diagnostics_from_segments(
                        frame_segments,
                        prediction_segment_span_s(streamed_chunk.trajectory_segments_inertial));
            }
        }
        else
        {
            const auto &base_ephemeris = cache.resolved_shared_ephemeris();
            const auto &base_bodies = cache.resolved_massive_bodies();
            if (!base_ephemeris || base_ephemeris->empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::MissingEphemeris);
                return false;
            }

            const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
            const orbitsim::FrameSegmentTransformOptions frame_opt =
                    build_frame_segment_transform_options(
                            resolved_frame_spec,
                            streamed_chunk.trajectory_segments_inertial,
                            cancel_requested);
            orbitsim::FrameSegmentTransformDiagnostics frame_diag{};
            frame_segments = orbitsim::transform_trajectory_segments_to_frame_spec(
                    streamed_chunk.trajectory_segments_inertial,
                    *base_ephemeris,
                    base_bodies,
                    resolved_frame_spec,
                    frame_opt,
                    player_lookup,
                    &frame_diag);
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (frame_segments.empty())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameTransformFailed);
                return false;
            }
            if (!validate_trajectory_segment_continuity(frame_segments))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }

            frame_samples =
                    use_dense_chunk_samples
                            ? sample_prediction_segments(frame_segments, sample_budget, node_times_s)
                            : collect_segment_boundary_samples(frame_segments);
            if (out_stage_diagnostics)
            {
                *out_stage_diagnostics = make_stage_diagnostics_from_adaptive(
                        frame_diag,
                        prediction_segment_span_s(streamed_chunk.trajectory_segments_inertial));
                out_stage_diagnostics->accepted_segments = frame_segments.size();
                out_stage_diagnostics->covered_duration_s = prediction_segment_span_s(frame_segments);
                out_stage_diagnostics->frame_resegmentation_count = frame_diag.frame_resegmentation_count;
            }
        }

        if (frame_samples.size() < 2)
        {
            update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::FrameSamplesUnavailable);
            return false;
        }

        out_chunk.chunk_id = published_chunk.chunk_id;
        out_chunk.generation_id = generation_id;
        out_chunk.quality_state = published_chunk.quality_state;
        out_chunk.t0_s = published_chunk.t0_s;
        out_chunk.t1_s = published_chunk.t1_s;
        out_chunk.frame_samples = std::move(frame_samples);
        out_chunk.frame_segments = std::move(frame_segments);
        if (build_chunk_render_curve && !out_chunk.frame_segments.empty())
        {
            out_chunk.render_curve = OrbitRenderCurve::build(out_chunk.frame_segments);
        }
        out_chunk.valid = !out_chunk.frame_segments.empty();
        return out_chunk.valid;
    }

    inline bool rebuild_prediction_streamed_chunk_assembly(
            PredictionChunkAssembly &out_assembly,
            const OrbitPredictionCache &cache,
            const std::vector<OrbitPredictionService::StreamedPlannedChunk> &streamed_chunks,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested = {},
            OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
            const bool build_chunk_render_curves = false,
            const bool use_dense_chunk_samples = true)
    {
        out_assembly.clear();
        if (diagnostics)
        {
            *diagnostics = {};
        }

        out_assembly.chunks.reserve(streamed_chunks.size());
        OrbitPredictionService::AdaptiveStageDiagnostics planned_stage_diagnostics{};
        bool planned_stage_diagnostics_valid = false;
        std::size_t total_chunk_segment_count = 0;
        std::size_t total_chunk_sample_count = 0;

        for (const OrbitPredictionService::StreamedPlannedChunk &streamed_chunk : streamed_chunks)
        {
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (!streamed_chunk.published_chunk.includes_planned_path)
            {
                continue;
            }

            OrbitChunk chunk{};
            OrbitPredictionService::AdaptiveStageDiagnostics chunk_diagnostics{};
            if (!build_prediction_streamed_planned_chunk(chunk,
                                                        &chunk_diagnostics,
                                                        cache,
                                                        streamed_chunk,
                                                        generation_id,
                                                        resolved_frame_spec,
                                                        player_lookup_segments_inertial,
                                                        cancel_requested,
                                                        diagnostics,
                                                        build_chunk_render_curves,
                                                        use_dense_chunk_samples))
            {
                return false;
            }

            total_chunk_segment_count += chunk.frame_segments.size();
            total_chunk_sample_count += chunk.frame_samples.size();
            if (!planned_stage_diagnostics_valid)
            {
                planned_stage_diagnostics = chunk_diagnostics;
                planned_stage_diagnostics_valid = true;
            }
            else
            {
                accumulate_stage_diagnostics(planned_stage_diagnostics, chunk_diagnostics);
            }
            out_assembly.chunks.push_back(std::move(chunk));
        }

        std::sort(out_assembly.chunks.begin(),
                  out_assembly.chunks.end(),
                  [](const OrbitChunk &a, const OrbitChunk &b) { return a.chunk_id < b.chunk_id; });
        out_assembly.generation_id = generation_id;
        out_assembly.valid = !out_assembly.chunks.empty();
        if (diagnostics)
        {
            diagnostics->status =
                    out_assembly.valid ? PredictionDerivedStatus::Success : PredictionDerivedStatus::MissingSolverData;
            diagnostics->frame_segment_count = cache.trajectory_segments_frame.size();
            diagnostics->frame_segment_count_planned = total_chunk_segment_count;
            diagnostics->frame_sample_count = cache.trajectory_frame.size();
            diagnostics->frame_sample_count_planned = total_chunk_sample_count;
            if (planned_stage_diagnostics_valid)
            {
                diagnostics->frame_planned = planned_stage_diagnostics;
                diagnostics->frame_planned.accepted_segments = total_chunk_segment_count;
            }
        }
        return out_assembly.valid;
    }

    inline void clear_prediction_metrics(OrbitPredictionCache &cache, const orbitsim::BodyId analysis_body_id)
    {
        cache.altitude_km.clear();
        cache.speed_kmps.clear();
        cache.semi_major_axis_m = 0.0;
        cache.eccentricity = 0.0;
        cache.orbital_period_s = 0.0;
        cache.periapsis_alt_km = 0.0;
        cache.apoapsis_alt_km = std::numeric_limits<double>::infinity();
        cache.metrics_body_id = analysis_body_id;
        cache.metrics_valid = true;
    }

    inline void rebuild_prediction_metrics(
            OrbitPredictionCache &cache,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested = {})
    {
        clear_prediction_metrics(cache, analysis_body_id);

        const auto &base_ephemeris = cache.resolved_shared_ephemeris();
        const auto &base_bodies = cache.resolved_massive_bodies();
        const auto &base_samples = cache.resolved_trajectory_inertial();
        const auto &base_segments = cache.resolved_trajectory_segments_inertial();
        const auto analysis_it = std::find_if(base_bodies.begin(),
                                              base_bodies.end(),
                                              [analysis_body_id](const orbitsim::MassiveBody &body) {
                                                  return body.id == analysis_body_id;
                                              });
        if (analysis_it == base_bodies.end() || !(analysis_it->mass_kg > 0.0))
        {
            return;
        }

        const double mu_ref_m3_s2 = sim_config.gravitational_constant * analysis_it->mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return;
        }

        std::vector<orbitsim::TrajectorySample> rel_samples;
        if (cache.resolved_frame_spec_valid &&
            cache.resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
            cache.resolved_frame_spec.primary_body_id == analysis_body_id)
        {
            rel_samples = cache.trajectory_frame;
            cache.trajectory_segments_analysis_bci = cache.trajectory_segments_frame;
            cache.trajectory_analysis_bci = rel_samples;
            cache.analysis_cache_body_id = analysis_body_id;
            cache.analysis_cache_valid = true;
        }
        else if (cache.analysis_cache_valid && cache.analysis_cache_body_id == analysis_body_id)
        {
            rel_samples = cache.trajectory_analysis_bci;
        }
        else if (base_ephemeris && !base_ephemeris->empty())
        {
            const std::size_t sample_budget = std::max<std::size_t>(base_samples.size(), 2);
            const orbitsim::TrajectoryFrameSpec analysis_frame =
                    orbitsim::TrajectoryFrameSpec::body_centered_inertial(analysis_body_id);
            const orbitsim::FrameSegmentTransformOptions frame_opt =
                    build_frame_segment_transform_options(
                            analysis_frame,
                            base_segments,
                            cancel_requested);
            std::vector<orbitsim::TrajectorySegment> rel_segments =
                    orbitsim::transform_trajectory_segments_to_frame_spec(
                            base_segments,
                            *base_ephemeris,
                            base_bodies,
                            analysis_frame,
                            frame_opt);
            if (cancel_requested && cancel_requested())
            {
                return;
            }
            rel_samples = sample_prediction_segments(rel_segments, sample_budget);
            cache.trajectory_segments_analysis_bci = std::move(rel_segments);
            cache.trajectory_analysis_bci = rel_samples;
            cache.analysis_cache_body_id = analysis_body_id;
            cache.analysis_cache_valid = !cache.trajectory_analysis_bci.empty();
        }

        if (rel_samples.size() < 2)
        {
            return;
        }

        cache.altitude_km.reserve(rel_samples.size());
        cache.speed_kmps.reserve(rel_samples.size());
        for (const orbitsim::TrajectorySample &sample : rel_samples)
        {
            const double r_m = glm::length(sample.position_m);
            const double alt_km = (r_m - analysis_it->radius_m) * 1.0e-3;
            const double spd_kmps = glm::length(sample.velocity_mps) * 1.0e-3;
            cache.altitude_km.push_back(static_cast<float>(alt_km));
            cache.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitPredictionMath::OrbitalElementsEstimate elements =
                OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2,
                                                              rel_samples.front().position_m,
                                                              rel_samples.front().velocity_mps);
        if (!elements.valid)
        {
            return;
        }

        cache.semi_major_axis_m = elements.semi_major_axis_m;
        cache.eccentricity = elements.eccentricity;
        cache.orbital_period_s = elements.orbital_period_s;
        cache.periapsis_alt_km = (elements.periapsis_m - analysis_it->radius_m) * 1.0e-3;
        cache.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                        ? (elements.apoapsis_m - analysis_it->radius_m) * 1.0e-3
                                        : std::numeric_limits<double>::infinity();
    }

    inline bool rebuild_prediction_patch_chunks(
            PredictionChunkAssembly &out_assembly,
            const OrbitPredictionCache &cache,
            const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const uint64_t /*display_frame_key*/,
            const uint64_t /*display_frame_revision*/,
            const CancelCheck &cancel_requested = {},
            const std::vector<double> &node_times_s = {},
            OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
            const bool build_chunk_render_curves = false,
            const bool use_dense_chunk_samples = true)
    {
        out_assembly.clear();
        if (diagnostics)
        {
            *diagnostics = {};
        }
        (void) resolved_frame_spec;

        out_assembly.chunks.reserve(published_chunks.size());
        std::size_t segment_cursor = 0;
        double previous_chunk_t0_s = -std::numeric_limits<double>::infinity();
        std::size_t total_chunk_segment_count = 0;
        std::size_t total_chunk_sample_count = 0;
        double total_chunk_duration_s = 0.0;

        for (const OrbitPredictionService::PublishedChunk &published_chunk : published_chunks)
        {
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::Cancelled);
                return false;
            }

            if (!published_chunk.includes_planned_path ||
                !std::isfinite(published_chunk.t0_s) ||
                !std::isfinite(published_chunk.t1_s) ||
                !(published_chunk.t1_s > published_chunk.t0_s))
            {
                continue;
            }

            const double chunk_order_epsilon_s = continuity_time_epsilon_s(published_chunk.t0_s);
            if (published_chunk.t0_s < (previous_chunk_t0_s - chunk_order_epsilon_s))
            {
                segment_cursor = 0;
            }
            previous_chunk_t0_s = published_chunk.t0_s;

            std::vector<orbitsim::TrajectorySegment> clipped_segments{};
            if (!slice_trajectory_segments_from_cursor(cache.trajectory_segments_frame_planned,
                                                       published_chunk.t0_s,
                                                       published_chunk.t1_s,
                                                       segment_cursor,
                                                       clipped_segments))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (clipped_segments.empty() || !validate_trajectory_segment_continuity(clipped_segments))
            {
                update_derived_diagnostics(diagnostics, cache, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }

            std::vector<orbitsim::TrajectorySample> clipped_samples =
                    use_dense_chunk_samples
                            ? sample_prediction_segments(clipped_segments,
                                                         std::max<std::size_t>(2u, clipped_segments.size() + 1u),
                                                         node_times_s)
                            : collect_segment_boundary_samples(clipped_segments);

            OrbitChunk chunk{};
            chunk.chunk_id = published_chunk.chunk_id;
            chunk.generation_id = generation_id;
            chunk.quality_state = published_chunk.quality_state;
            chunk.t0_s = published_chunk.t0_s;
            chunk.t1_s = published_chunk.t1_s;
            chunk.frame_samples = std::move(clipped_samples);
            chunk.frame_segments = std::move(clipped_segments);
            if (build_chunk_render_curves && !chunk.frame_segments.empty())
            {
                chunk.render_curve = OrbitRenderCurve::build(chunk.frame_segments);
            }
            else
            {
                chunk.render_curve.clear();
            }
            chunk.valid = !chunk.frame_segments.empty();
            total_chunk_segment_count += chunk.frame_segments.size();
            total_chunk_sample_count += chunk.frame_samples.size();
            total_chunk_duration_s += prediction_segment_span_s(chunk.frame_segments);
            out_assembly.chunks.push_back(std::move(chunk));
        }

        out_assembly.generation_id = generation_id;
        out_assembly.valid = !out_assembly.chunks.empty();
        if (diagnostics && out_assembly.valid)
        {
            diagnostics->status = PredictionDerivedStatus::Success;
            diagnostics->frame_segment_count = cache.trajectory_segments_frame.size();
            diagnostics->frame_segment_count_planned = total_chunk_segment_count;
            diagnostics->frame_sample_count = cache.trajectory_frame.size();
            diagnostics->frame_sample_count_planned = total_chunk_sample_count;
            diagnostics->frame_planned.accepted_segments = total_chunk_segment_count;
            diagnostics->frame_planned.covered_duration_s = total_chunk_duration_s;
        }
        return out_assembly.valid;
    }

    inline void flatten_chunk_assembly_to_cache(OrbitPredictionCache &cache,
                                                const PredictionChunkAssembly &assembly,
                                                const bool build_render_curve = true)
    {
        cache.trajectory_frame_planned.clear();
        cache.trajectory_segments_frame_planned.clear();
        cache.render_curve_frame_planned.clear();

        if (!assembly.valid)
        {
            return;
        }

        std::size_t total_segment_count = 0;
        std::size_t total_sample_count = 0;
        for (const OrbitChunk &chunk : assembly.chunks)
        {
            total_segment_count += chunk.frame_segments.size();
            total_sample_count += chunk.frame_samples.size();
        }
        cache.trajectory_segments_frame_planned.reserve(total_segment_count);
        cache.trajectory_frame_planned.reserve(total_sample_count);

        for (const OrbitChunk &chunk : assembly.chunks)
        {
            cache.trajectory_segments_frame_planned.insert(cache.trajectory_segments_frame_planned.end(),
                                                           chunk.frame_segments.begin(),
                                                           chunk.frame_segments.end());

            for (const orbitsim::TrajectorySample &sample : chunk.frame_samples)
            {
                append_unique_prediction_sample(cache.trajectory_frame_planned, sample);
            }
        }

        if (build_render_curve && !cache.trajectory_segments_frame_planned.empty())
        {
            cache.render_curve_frame_planned = OrbitRenderCurve::build(cache.trajectory_segments_frame_planned);
        }
    }
} // namespace Game::PredictionCacheInternal
