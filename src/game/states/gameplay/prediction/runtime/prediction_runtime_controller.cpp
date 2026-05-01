#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

#include "core/util/logger.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/prediction_frame_controller.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"
#include "game/states/gameplay/prediction/runtime/prediction_request_factory.h"
#include "game/states/gameplay/prediction/runtime/prediction_result_applier.h"
#include "game/states/gameplay/prediction/runtime/prediction_solver_result_applier.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace Game
{
    namespace
    {
        PredictionTrackState *find_track_by_id(std::vector<PredictionTrackState> &tracks, const uint64_t track_id)
        {
            for (PredictionTrackState &track : tracks)
            {
                if (track.key.track_id() == track_id)
                {
                    return &track;
                }
            }
            return nullptr;
        }

        bool current_plan_active_for_track(const PredictionTrackState &track,
                                           const PredictionRuntimeContext &context)
        {
            return track.supports_maneuvers &&
                   context.maneuver_nodes_enabled &&
                   context.maneuver_plan &&
                   !context.maneuver_plan->nodes.empty();
        }

        bool active_maneuver_edit_for_track(const PredictionTrackState &track,
                                            const PredictionRuntimeContext &context,
                                            const bool current_plan_active)
        {
            return current_plan_active &&
                   context.maneuver_edit_in_progress &&
                   track.key == context.selection.active_subject &&
                   context.maneuver_live_preview_available;
        }

        void mark_prediction_request_submitted(PredictionTrackState &track,
                                               const uint64_t generation_id,
                                               const double now_s,
                                               const OrbitPredictionService::SolveQuality solve_quality,
                                               const bool request_has_maneuver_plan,
                                               const uint64_t plan_signature,
                                               const bool preview_request_active = false)
        {
            PredictionLifecycleReducer::mark_solver_request_submitted(
                    track,
                    PredictionSolverRequestSubmittedEvent{
                            generation_id,
                            solve_quality,
                            request_has_maneuver_plan,
                            plan_signature,
                            preview_request_active,
                            now_s,
                    });

            PredictionDragDebugTelemetry &debug = track.drag_debug;
            const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
            debug.last_request_tp = now_tp;
            debug.last_request_generation_id = generation_id;
            ++debug.request_count;
            if (debug.drag_active && PredictionDragDebugTelemetry::has_time(debug.last_drag_update_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.drag_to_request_ms_last,
                        debug.drag_to_request_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_drag_update_tp, now_tp));
            }
        }

        bool prediction_request_is_throttled(const PredictionTrackState &track, const bool interactive_request)
        {
            if (!interactive_request)
            {
                return false;
            }

            const PredictionDragDebugTelemetry &debug = track.drag_debug;
            const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
            if (debug.drag_active && PredictionDragDebugTelemetry::has_time(debug.drag_started_tp))
            {
                const double elapsed_drag_s =
                        std::chrono::duration<double>(now_tp - debug.drag_started_tp).count();
                if (elapsed_drag_s < OrbitPredictionTuning::kDragStalePreviewGraceS)
                {
                    return true;
                }
            }

            if (!PredictionDragDebugTelemetry::has_time(debug.last_request_tp))
            {
                return false;
            }

            const double elapsed_s = std::chrono::duration<double>(
                                             now_tp - debug.last_request_tp)
                                             .count();
            return elapsed_s < OrbitPredictionTuning::kDragRebuildMinIntervalS;
        }

        bool should_defer_solver_request_until_publish(const PredictionTrackState &track)
        {
            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot snapshot =
                    PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
            return PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot);
        }

        [[nodiscard]] bool solve_already_reported_horizon_shortfall(
                const OrbitPredictionService::AdaptiveStageDiagnostics &diag,
                const double required_ahead_s,
                const double coverage_epsilon_s)
        {
            if (!(required_ahead_s > 0.0) ||
                !std::isfinite(required_ahead_s) ||
                !(diag.requested_duration_s > 0.0) ||
                !std::isfinite(diag.requested_duration_s))
            {
                return false;
            }

            const double requested_duration_s = std::max(0.0, diag.requested_duration_s);
            const double covered_duration_s = std::max(0.0, diag.covered_duration_s);
            if (required_ahead_s > (requested_duration_s + coverage_epsilon_s))
            {
                return false;
            }

            if ((covered_duration_s + coverage_epsilon_s) >= required_ahead_s)
            {
                return false;
            }

            return diag.hard_cap_hit ||
                   (covered_duration_s + coverage_epsilon_s) < (requested_duration_s - coverage_epsilon_s);
        }

        [[nodiscard]] double prediction_coverage_refresh_lead_s(const double required_ahead_s, const float fixed_dt)
        {
            const double fixed_dt_lead_s =
                    std::max(0.0, static_cast<double>(fixed_dt)) *
                    OrbitPredictionTuning::kPredictionCoverageRefreshLeadFixedDtScale;
            const double lead_s =
                    std::max(OrbitPredictionTuning::kPredictionCoverageRefreshLeadS, fixed_dt_lead_s);
            if (!(required_ahead_s > 0.0) || !std::isfinite(required_ahead_s))
            {
                return lead_s;
            }

            return std::min(required_ahead_s, lead_s);
        }

        [[nodiscard]] double prediction_cache_end_time_s(const OrbitPredictionCache &cache)
        {
            double cache_end_s = std::numeric_limits<double>::quiet_NaN();

            const auto accumulate_samples_end = [&cache_end_s](const std::vector<orbitsim::TrajectorySample> &samples) {
                if (samples.empty())
                {
                    return;
                }

                const double sample_end_s = samples.back().t_s;
                if (std::isfinite(sample_end_s))
                {
                    cache_end_s = std::isfinite(cache_end_s)
                                          ? std::max(cache_end_s, sample_end_s)
                                          : sample_end_s;
                }
            };

            const auto accumulate_segments_end = [&cache_end_s](const std::vector<orbitsim::TrajectorySegment> &segments) {
                if (segments.empty())
                {
                    return;
                }

                const orbitsim::TrajectorySegment &last_segment = segments.back();
                const double segment_end_s = last_segment.t0_s + last_segment.dt_s;
                if (std::isfinite(segment_end_s))
                {
                    cache_end_s = std::isfinite(cache_end_s)
                                          ? std::max(cache_end_s, segment_end_s)
                                          : segment_end_s;
                }
            };

            accumulate_samples_end(cache.solver.resolved_trajectory_inertial());
            accumulate_segments_end(cache.solver.resolved_trajectory_segments_inertial());
            accumulate_samples_end(cache.solver.trajectory_inertial_planned);
            accumulate_segments_end(cache.solver.trajectory_segments_inertial_planned);
            return cache_end_s;
        }
    } // namespace

    void PredictionRuntimeController::clear_runtime(GameplayPredictionState &prediction)
    {
        for (PredictionTrackState &track : prediction.tracks)
        {
            PredictionLifecycleReducer::reset_runtime(track, PredictionRuntimeResetMode::Clean);
        }
        prediction.service.reset();
        prediction.derived_service.reset();
        prediction.dirty = false;
    }

    void PredictionRuntimeController::clear_visible_runtime(
            GameplayPredictionState &prediction,
            const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        (void) visible_subjects;
        for (PredictionTrackState &track : prediction.tracks)
        {
            PredictionLifecycleReducer::reset_runtime(track, PredictionRuntimeResetMode::Dirty);
        }
        prediction.derived_service.reset();
    }

    bool PredictionRuntimeController::poll_completed_results(GameplayPredictionState &prediction,
                                                             const PredictionRuntimeContext &context)
    {
        bool applied_result = false;
        while (auto completed = prediction.service.poll_completed())
        {
            apply_completed_solver_result(prediction, context, std::move(*completed));
            applied_result = true;
        }

        while (auto completed = prediction.derived_service.poll_completed())
        {
            const bool streaming_publish =
                    completed->publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming ||
                    completed->publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
            apply_completed_derived_result(prediction, context, std::move(*completed));
            applied_result = true;
            if (streaming_publish)
            {
                break;
            }
        }

        return applied_result;
    }

    bool PredictionRuntimeController::apply_completed_solver_result(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            OrbitPredictionService::Result result)
    {
        PredictionTrackState *track = find_track_by_id(prediction.tracks, result.track_id);
        if (!track)
        {
            return false;
        }

        PredictionSolverResultApplyResult applied =
                PredictionSolverResultApplier::apply_solver_result(*track, std::move(result), context);
        if (!applied.derived_request)
        {
            return true;
        }

        prediction.derived_service.request(*applied.derived_request);
        PredictionFrameController::mark_derived_request_submitted(*track, *applied.derived_request);
        return true;
    }

    bool PredictionRuntimeController::apply_completed_derived_result(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            OrbitPredictionDerivedService::Result result)
    {
        PredictionTrackState *track = find_track_by_id(prediction.tracks, result.track_id);
        if (!track)
        {
            return false;
        }

        const bool current_plan_active = current_plan_active_for_track(*track, context);
        PredictionResultApplier::apply_derived_result(
                *track,
                std::move(result),
                PredictionDerivedResultApplyContext{
                        .current_maneuver_plan_revision = context.maneuver_plan_revision,
                        .live_preview_active = context.maneuver_live_preview_available,
                        .current_plan_active = current_plan_active,
                        .current_plan_signature = current_plan_active ? context.maneuver_plan_signature : 0u,
                        .active_maneuver_edit =
                                active_maneuver_edit_for_track(*track, context, current_plan_active),
                });
        return true;
    }

    bool PredictionRuntimeController::should_rebuild_track(
            const GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            const PredictionTrackState &track,
            const double now_s,
            const float fixed_dt,
            const bool thrusting,
            const bool with_maneuvers)
    {
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        bool rebuild = lifecycle.dirty || !lifecycle.visible_cache_valid;

        if (!rebuild && thrusting)
        {
            const double dt_since_build_s = now_s - track.cache.identity.build_time_s;
            rebuild = dt_since_build_s >= prediction.thrust_refresh_s;
        }

        if (!rebuild && prediction.periodic_refresh_s > 0.0)
        {
            const double dt_since_build_s = now_s - track.cache.identity.build_time_s;
            rebuild = dt_since_build_s >= prediction.periodic_refresh_s;
        }

        if (!rebuild && PredictionRuntimeDetail::prediction_track_should_rebuild_from_await_full_refine(lifecycle))
        {
            rebuild = true;
        }

        if (rebuild || !track.cache.identity.valid || track.cache.solver.resolved_trajectory_inertial().empty())
        {
            return rebuild;
        }

        const double cache_end_s = prediction_cache_end_time_s(track.cache);
        if (!std::isfinite(cache_end_s))
        {
            return true;
        }

        const double required_ahead_s =
                context.required_window_s
                        ? context.required_window_s(track, now_s, with_maneuvers)
                        : 0.0;

        const double coverage_epsilon_s =
                std::max(1.0e-3, std::min(0.25, std::max(0.0, static_cast<double>(fixed_dt)) * 0.5));
        if ((cache_end_s - now_s + coverage_epsilon_s) >= required_ahead_s)
        {
            return false;
        }

        const double coverage_refresh_lead_s = prediction_coverage_refresh_lead_s(required_ahead_s, fixed_dt);
        if ((cache_end_s - now_s + coverage_epsilon_s) > coverage_refresh_lead_s)
        {
            return false;
        }

        if (track.solver_diagnostics.status == OrbitPredictionService::Status::Success &&
            solve_already_reported_horizon_shortfall(
                    track.solver_diagnostics.trajectory_base,
                    required_ahead_s,
                    coverage_epsilon_s))
        {
            return false;
        }

        return true;
    }

    bool PredictionRuntimeController::request_orbiter_prediction_async(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            PredictionTrackState &track,
            const WorldVec3 &subject_pos_world,
            const glm::dvec3 &subject_vel_world,
            const double now_s,
            const bool thrusting,
            const bool with_maneuvers,
            bool *out_throttled)
    {
        if (out_throttled)
        {
            *out_throttled = false;
        }

        PredictionOrbiterRequestBuildResult build =
                PredictionRequestFactory::build_orbiter_request(context,
                                                                track,
                                                                subject_pos_world,
                                                                subject_vel_world,
                                                                now_s,
                                                                thrusting,
                                                                with_maneuvers);
        if (!build.built)
        {
            return false;
        }

        if (prediction_request_is_throttled(track, build.interactive_request))
        {
            if (out_throttled)
            {
                *out_throttled = true;
            }
            return false;
        }

        const OrbitPredictionService::SolveQuality submitted_quality = build.request.solve_quality;
        const uint64_t submitted_track_id = build.request.track_id;
        const uint64_t submitted_plan_revision = build.request.maneuver_plan_revision;
        const bool submitted_plan_signature_valid = build.request.maneuver_plan_signature_valid;
        const uint64_t submitted_plan_signature = build.request.maneuver_plan_signature;
        const auto submitted_priority = build.request.priority;
        const bool submitted_preview_patch = build.request.preview_patch.active;
        const bool submitted_anchor_state_valid = build.request.preview_patch.anchor_state_valid;
        const bool submitted_anchor_state_trusted = build.request.preview_patch.anchor_state_trusted;
        const double submitted_anchor_time_s = build.request.preview_patch.anchor_time_s;
        const double submitted_future_window_s = build.request.future_window_s;
        const std::size_t submitted_maneuver_count = build.request.maneuver_impulses.size();
        const uint64_t generation_id = prediction.service.request(std::move(build.request));
        mark_prediction_request_submitted(track,
                                          generation_id,
                                          now_s,
                                          submitted_quality,
                                          submitted_plan_signature_valid,
                                          submitted_plan_signature,
                                          build.preview_request_active);
        if (track.supports_maneuvers)
        {
            Logger::debug("Maneuver prediction request: track={} gen={} plan_rev={} plan_sig={} plan_sig_valid={} "
                          "quality={} priority={} "
                          "preview={} anchor_t={:.6f} anchor_state={} anchor_trusted={} impulses={} "
                          "future_window={:.3f} "
                          "pending_solver={} pending_derived={} dirty={}",
                          submitted_track_id,
                          generation_id,
                          submitted_plan_revision,
                          submitted_plan_signature,
                          submitted_plan_signature_valid,
                          static_cast<int>(submitted_quality),
                          static_cast<int>(submitted_priority),
                          submitted_preview_patch,
                          submitted_anchor_time_s,
                          submitted_anchor_state_valid,
                          submitted_anchor_state_trusted,
                          submitted_maneuver_count,
                          submitted_future_window_s,
                          track.request_pending,
                          track.derived_request_pending,
                          track.dirty);
        }
        return true;
    }

    bool PredictionRuntimeController::request_celestial_prediction_async(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            PredictionTrackState &track,
            const double now_s)
    {
        OrbitPredictionService::Request request{};
        if (!PredictionRequestFactory::build_celestial_request(context, track, now_s, request))
        {
            return false;
        }

        const uint64_t generation_id = prediction.service.request(std::move(request));
        mark_prediction_request_submitted(
                track,
                generation_id,
                now_s,
                OrbitPredictionService::SolveQuality::Full,
                false,
                0u);
        return true;
    }

    void PredictionRuntimeController::update_orbiter_prediction_track(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            PredictionTrackState &track,
            const double now_s,
            const bool thrusting,
            const bool with_maneuvers)
    {
        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 subject_vel_world{0.0};
        glm::vec3 subject_vel_local{0.0f};
        if (!context.get_subject_world_state ||
            !context.get_subject_world_state(track.key, subject_pos_world, subject_vel_world, subject_vel_local))
        {
            PredictionLifecycleReducer::reset_runtime(track, PredictionRuntimeResetMode::Dirty);
            return;
        }

        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        const bool live_preview_pending_override =
                PredictionRuntimeDetail::prediction_track_live_preview_pending_override(lifecycle) &&
                track.supports_maneuvers &&
                track.key == prediction.selection.active_subject &&
                with_maneuvers &&
                context.maneuver_live_preview_available;
        if (!live_preview_pending_override &&
            should_defer_solver_request_until_publish(track))
        {
            return;
        }

        bool throttled = false;
        const bool requested = request_orbiter_prediction_async(
                prediction,
                context,
                track,
                subject_pos_world,
                subject_vel_world,
                now_s,
                thrusting,
                with_maneuvers,
                &throttled);
        PredictionLifecycleReducer::mark_rebuild_attempt_finished(track, requested);
        if (!requested && !throttled)
        {
            const bool has_cache_to_keep = track.cache.identity.valid || track.authoritative_cache.identity.valid;
            if (!has_cache_to_keep)
            {
                PredictionLifecycleReducer::reset_runtime(track, PredictionRuntimeResetMode::Dirty);
            }
        }
    }

    void PredictionRuntimeController::update_celestial_prediction_track(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            PredictionTrackState &track,
            const double now_s)
    {
        if (!context.orbital_scenario)
        {
            PredictionLifecycleReducer::reset_runtime(track, PredictionRuntimeResetMode::Dirty);
            return;
        }

        if (should_defer_solver_request_until_publish(track))
        {
            return;
        }

        const bool requested = request_celestial_prediction_async(prediction, context, track, now_s);
        PredictionLifecycleReducer::mark_rebuild_attempt_finished(track, requested);
        if (!requested)
        {
            PredictionLifecycleReducer::reset_runtime(track, PredictionRuntimeResetMode::Dirty);
        }
    }

    void PredictionRuntimeController::update_visible_tracks(
            GameplayPredictionState &prediction,
            const PredictionRuntimeContext &context,
            const std::vector<PredictionSubjectKey> &visible_subjects,
            const double now_s,
            const float fixed_dt)
    {
        for (PredictionTrackState &track : prediction.tracks)
        {
            if (!PredictionRuntimeDetail::contains_key(visible_subjects, track.key))
            {
                continue;
            }

            const bool with_maneuvers =
                    track.supports_maneuvers &&
                    context.maneuver_nodes_enabled &&
                    context.maneuver_plan &&
                    !context.maneuver_plan->nodes.empty();
            const bool thrusting =
                    context.subject_thrust_applied_this_tick
                            ? context.subject_thrust_applied_this_tick(track.key)
                            : false;
            const double track_reference_time_s = now_s;
            const bool rebuild =
                    should_rebuild_track(prediction,
                                         context,
                                         track,
                                         track_reference_time_s,
                                         fixed_dt,
                                         thrusting,
                                         with_maneuvers);
            if (!rebuild)
            {
                continue;
            }

            if (track.key.kind == PredictionSubjectKind::Celestial)
            {
                update_celestial_prediction_track(prediction, context, track, now_s);
                continue;
            }

            update_orbiter_prediction_track(prediction, context, track, track_reference_time_s, thrusting, with_maneuvers);
        }
    }
} // namespace Game
