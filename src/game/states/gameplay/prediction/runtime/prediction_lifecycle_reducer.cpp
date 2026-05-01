#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"

#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

namespace Game
{
    void PredictionLifecycleReducer::reset_runtime(PredictionTrackState &track,
                                                   const PredictionRuntimeResetMode mode)
    {
        track.clear_runtime();
        if (mode == PredictionRuntimeResetMode::Clean)
        {
            track.dirty = false;
        }
    }

    void PredictionLifecycleReducer::mark_dirty(PredictionTrackState &track)
    {
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        if (PredictionRuntimeDetail::prediction_track_should_mark_invalidated_while_pending(lifecycle))
        {
            track.invalidated_while_pending = true;
            return;
        }

        track.dirty = true;
    }

    void PredictionLifecycleReducer::mark_dirty_for_preview(PredictionTrackState &track)
    {
        track.dirty = true;
        track.invalidated_while_pending = track.request_pending || track.derived_request_pending;
    }

    void PredictionLifecycleReducer::mark_solver_request_submitted(
            PredictionTrackState &track,
            const PredictionSolverRequestSubmittedEvent &event)
    {
        track.latest_requested_generation_id = event.generation_id;
        if (event.solve_quality == OrbitPredictionService::SolveQuality::Full)
        {
            track.latest_requested_authoritative_generation_id = event.generation_id;
        }

        track.request_pending = true;
        track.derived_request_pending = false;
        track.pending_solve_quality = event.solve_quality;
        track.pending_solver_has_maneuver_plan = event.request_has_maneuver_plan;
        track.pending_solver_plan_signature =
                event.request_has_maneuver_plan ? event.maneuver_plan_signature : 0u;
        track.pending_derived_has_maneuver_plan = false;
        track.pending_derived_plan_signature = 0u;
        track.invalidated_while_pending = false;

        if (event.preview_request_active)
        {
            mark_preview_request_submitted(track, event.now_s);
        }
    }

    void PredictionLifecycleReducer::mark_rebuild_attempt_finished(PredictionTrackState &track,
                                                                   const bool requested)
    {
        track.dirty = !requested;
    }

    void PredictionLifecycleReducer::mark_derived_request_submitted(
            PredictionTrackState &track,
            const OrbitPredictionDerivedService::Request &request)
    {
        track.derived_request_pending = true;
        track.latest_requested_derived_generation_id = request.generation_id;
        track.latest_requested_derived_display_frame_key = request.display_frame_key;
        track.latest_requested_derived_display_frame_revision = request.display_frame_revision;
        track.latest_requested_derived_analysis_body_id = request.analysis_body_id;
        track.latest_requested_derived_publish_stage = request.solver_result.publish_stage;
        track.pending_derived_has_maneuver_plan = request.maneuver_plan_signature_valid;
        track.pending_derived_plan_signature =
                request.maneuver_plan_signature_valid ? request.maneuver_plan_signature : 0u;
    }

    void PredictionLifecycleReducer::mark_solver_result_accepted(
            PredictionTrackState &track,
            const OrbitPredictionService::SolveQuality solve_quality,
            const OrbitPredictionService::PublishStage publish_stage,
            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot &lifecycle_before)
    {
        if (PredictionRuntimeDetail::prediction_track_is_preview_streaming_publish(solve_quality, publish_stage))
        {
            mark_preview_streaming(track);
        }

        track.request_pending =
                PredictionRuntimeDetail::prediction_track_should_keep_request_pending_after_solver_publish(
                        solve_quality,
                        publish_stage);
        if (PredictionRuntimeDetail::prediction_track_should_promote_dirty_after_solver_publish(lifecycle_before))
        {
            track.dirty = true;
            track.invalidated_while_pending = false;
        }
        track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
    }

    void PredictionLifecycleReducer::mark_solver_result_rejected_for_rebuild(
            PredictionTrackState &track,
            const bool reset_derived_request,
            const bool clear_invalidated)
    {
        track.request_pending = false;
        track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        if (clear_invalidated)
        {
            track.invalidated_while_pending = false;
        }
        track.dirty = true;

        if (reset_derived_request)
        {
            reset_derived_request_state(track);
        }
    }

    void PredictionLifecycleReducer::mark_derived_result_completed(
            PredictionTrackState &track,
            const bool completes_latest,
            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot &lifecycle_before)
    {
        if (completes_latest)
        {
            track.derived_request_pending = false;
        }

        track.dirty = PredictionRuntimeDetail::prediction_track_should_keep_dirty_for_followup(lifecycle_before);
        track.invalidated_while_pending = false;
    }

    void PredictionLifecycleReducer::mark_derived_result_rejected_for_rebuild(
            PredictionTrackState &track,
            const bool complete_derived_request)
    {
        if (complete_derived_request)
        {
            track.derived_request_pending = false;
        }
        track.invalidated_while_pending = false;
        track.dirty = true;
    }

    void PredictionLifecycleReducer::enter_preview_drag(PredictionTrackState &track, const double now_s)
    {
        track.preview_state = PredictionPreviewRuntimeState::EnterDrag;
        track.preview_entered_at_s = now_s;
    }

    void PredictionLifecycleReducer::mark_preview_request_submitted(PredictionTrackState &track, const double now_s)
    {
        track.preview_state = PredictionPreviewRuntimeState::DragPreviewPending;
        track.preview_last_request_at_s = now_s;
    }

    void PredictionLifecycleReducer::mark_preview_streaming(PredictionTrackState &track)
    {
        track.preview_state = PredictionPreviewRuntimeState::PreviewStreaming;
    }

    void PredictionLifecycleReducer::mark_preview_publish_accepted(
            PredictionTrackState &track,
            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot &lifecycle_before,
            const OrbitPredictionService::PublishStage publish_stage,
            const bool live_preview_active_now)
    {
        track.preview_state = PredictionRuntimeDetail::prediction_track_preview_state_after_preview_publish(
                lifecycle_before,
                publish_stage,
                live_preview_active_now);
    }

    void PredictionLifecycleReducer::mark_preview_publish_rejected(PredictionTrackState &track)
    {
        if (track.preview_state != PredictionPreviewRuntimeState::AwaitFullRefine)
        {
            reset_preview(track);
        }
    }

    void PredictionLifecycleReducer::mark_final_publish_completed(PredictionTrackState &track,
                                                                  const bool live_preview_active_now)
    {
        track.preview_state = PredictionPreviewRuntimeState::Idle;
        if (PredictionRuntimeDetail::prediction_track_should_clear_preview_anchor_after_final_publish(
                    live_preview_active_now))
        {
            track.preview_anchor = {};
        }
    }

    void PredictionLifecycleReducer::await_full_refine(PredictionTrackState &track, const double now_s)
    {
        track.preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
        track.preview_last_anchor_refresh_at_s = now_s;
    }

    void PredictionLifecycleReducer::reset_preview(PredictionTrackState &track)
    {
        track.preview_state = PredictionPreviewRuntimeState::Idle;
        track.preview_anchor = {};
    }

    void PredictionLifecycleReducer::reset_derived_request_state(PredictionTrackState &track)
    {
        track.derived_request_pending = false;
        track.latest_requested_derived_generation_id = 0;
        track.latest_requested_derived_display_frame_key = 0;
        track.latest_requested_derived_display_frame_revision = 0;
        track.latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
        track.latest_requested_derived_publish_stage = OrbitPredictionService::PublishStage::Final;
    }

    void PredictionLifecycleReducer::clear_pending_maneuver_requests(PredictionTrackState &track,
                                                                     const bool mark_dirty_after_clear)
    {
        track.request_pending = false;
        track.derived_request_pending = false;
        track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        track.pending_solver_has_maneuver_plan = false;
        track.pending_solver_plan_signature = 0u;
        track.pending_derived_has_maneuver_plan = false;
        track.pending_derived_plan_signature = 0u;
        track.invalidated_while_pending = false;
        if (mark_dirty_after_clear)
        {
            track.dirty = true;
        }
    }

    void PredictionLifecycleReducer::clear_maneuver_scoped_pending_requests(PredictionTrackState &track)
    {
        if (track.request_pending && track.pending_solver_has_maneuver_plan)
        {
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        }
        if (track.derived_request_pending && track.pending_derived_has_maneuver_plan)
        {
            track.derived_request_pending = false;
        }

        track.pending_solver_has_maneuver_plan = false;
        track.pending_solver_plan_signature = 0u;
        track.pending_derived_has_maneuver_plan = false;
        track.pending_derived_plan_signature = 0u;
        track.invalidated_while_pending = false;
    }
} // namespace Game
