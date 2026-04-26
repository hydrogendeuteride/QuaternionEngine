#pragma once

#include "game/states/gameplay/gameplay_state.h"

#include <algorithm>
#include <chrono>

namespace Game::PredictionRuntimeDetail
{
    struct PredictionTrackLifecycleSnapshot
    {
        PredictionTrackLifecycleState state{PredictionTrackLifecycleState::Idle};
        PredictionPreviewRuntimeState preview_state{PredictionPreviewRuntimeState::Idle};
        bool visible_cache_valid{false};
        bool authoritative_cache_valid{false};
        bool preview_overlay_active{false};
        bool full_stream_overlay_active{false};
        bool dirty{false};
        bool invalidated_while_pending{false};
        bool request_pending{false};
        bool derived_request_pending{false};
        bool awaiting_authoritative_publish{false};
        bool needs_rebuild{false};
    };

    template<typename T>
    bool contains_key(const std::vector<T> &items, const T &needle)
    {
        return std::find(items.begin(), items.end(), needle) != items.end();
    }

    inline bool maneuver_drag_active(const ManeuverGizmoInteraction::State state)
    {
        return state == ManeuverGizmoInteraction::State::DragAxis;
    }

    inline OrbitPredictionService::RequestPriority classify_prediction_subject_priority(
            const PredictionSelectionState &selection,
            const PredictionSubjectKey key,
            const bool is_celestial)
    {
        if (selection.active_subject == key)
        {
            return OrbitPredictionService::RequestPriority::ActiveTrack;
        }

        for (const auto &overlay : selection.overlay_subjects)
        {
            if (overlay == key)
            {
                return OrbitPredictionService::RequestPriority::Overlay;
            }
        }

        return is_celestial
                       ? OrbitPredictionService::RequestPriority::BackgroundCelestial
                       : OrbitPredictionService::RequestPriority::BackgroundOrbiter;
    }

    inline OrbitPredictionService::RequestPriority classify_prediction_request_priority(
            const PredictionSelectionState &selection,
            const PredictionSubjectKey key,
            const bool is_celestial,
            const bool interactive)
    {
        OrbitPredictionService::RequestPriority priority =
                classify_prediction_subject_priority(selection, key, is_celestial);
        if (interactive && priority == OrbitPredictionService::RequestPriority::ActiveTrack)
        {
            priority = OrbitPredictionService::RequestPriority::ActiveInteractiveTrack;
        }
        return priority;
    }

    inline double elapsed_ms(const PredictionDragDebugTelemetry::TimePoint &start_tp,
                             const PredictionDragDebugTelemetry::TimePoint &end_tp)
    {
        return std::chrono::duration<double, std::milli>(end_tp - start_tp).count();
    }

    inline void update_last_and_peak(double &last_value, double &peak_value, const double sample)
    {
        last_value = std::max(0.0, sample);
        peak_value = std::max(peak_value, last_value);
    }

    inline uint64_t visible_generation_id(const PredictionTrackState &track)
    {
        return track.cache.valid ? track.cache.generation_id : 0u;
    }

    inline uint64_t authoritative_generation_id(const PredictionTrackState &track)
    {
        return track.authoritative_cache.valid ? track.authoritative_cache.generation_id : 0u;
    }

    inline bool latest_solver_generation_published(const PredictionTrackState &track)
    {
        return track.latest_requested_authoritative_generation_id <= authoritative_generation_id(track);
    }

    inline bool prediction_chunk_assembly_active(const PredictionChunkAssembly &assembly)
    {
        return assembly.valid && !assembly.chunks.empty();
    }

    inline PredictionTrackLifecycleSnapshot describe_prediction_track_lifecycle(const PredictionTrackState &track)
    {
        PredictionTrackLifecycleSnapshot out{};
        out.preview_state = track.preview_state;
        out.visible_cache_valid = track.cache.valid;
        out.authoritative_cache_valid = track.authoritative_cache.valid;
        out.preview_overlay_active = prediction_chunk_assembly_active(track.preview_overlay.chunk_assembly);
        out.full_stream_overlay_active = prediction_chunk_assembly_active(track.full_stream_overlay.chunk_assembly);
        out.dirty = track.dirty;
        out.invalidated_while_pending = track.invalidated_while_pending;
        out.request_pending = track.request_pending;
        out.derived_request_pending = track.derived_request_pending;
        out.awaiting_authoritative_publish = !latest_solver_generation_published(track);
        out.needs_rebuild = out.dirty || out.invalidated_while_pending || !out.visible_cache_valid;

        if (track.preview_state == PredictionPreviewRuntimeState::EnterDrag ||
            track.preview_state == PredictionPreviewRuntimeState::DragPreviewPending)
        {
            out.state = PredictionTrackLifecycleState::DragPreviewPending;
            return out;
        }

        if (track.preview_state == PredictionPreviewRuntimeState::PreviewStreaming)
        {
            out.state = PredictionTrackLifecycleState::PreviewStreaming;
            return out;
        }

        if (out.full_stream_overlay_active)
        {
            out.state = PredictionTrackLifecycleState::FullStreaming;
            return out;
        }

        if (out.request_pending)
        {
            out.state = PredictionTrackLifecycleState::FullSolvePending;
            return out;
        }

        if (out.derived_request_pending || out.awaiting_authoritative_publish)
        {
            out.state = PredictionTrackLifecycleState::FinalDerivedPending;
            return out;
        }

        if (track.preview_state == PredictionPreviewRuntimeState::AwaitFullRefine)
        {
            out.state = PredictionTrackLifecycleState::AwaitFullRefine;
            return out;
        }

        if (!out.visible_cache_valid && !out.authoritative_cache_valid)
        {
            out.state = PredictionTrackLifecycleState::Idle;
            return out;
        }

        if (out.dirty || out.invalidated_while_pending)
        {
            out.state = PredictionTrackLifecycleState::NeedsRebuild;
            return out;
        }

        out.state = PredictionTrackLifecycleState::Stable;
        return out;
    }

    inline bool prediction_track_should_defer_solver_request(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        if (snapshot.request_pending)
        {
            return true;
        }

        if (snapshot.derived_request_pending || snapshot.awaiting_authoritative_publish)
        {
            return !snapshot.dirty;
        }

        return false;
    }

    inline bool prediction_track_should_mark_invalidated_while_pending(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.request_pending;
    }

    inline bool prediction_track_should_rebuild_from_await_full_refine(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.state == PredictionTrackLifecycleState::AwaitFullRefine &&
               !snapshot.request_pending &&
               !snapshot.derived_request_pending;
    }

    inline bool prediction_track_live_preview_pending_override(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.dirty || snapshot.invalidated_while_pending;
    }

    inline bool prediction_track_live_preview_drag_pending_override(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return prediction_track_live_preview_pending_override(snapshot);
    }

    inline bool prediction_track_can_enter_preview_drag(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.preview_state == PredictionPreviewRuntimeState::Idle ||
               snapshot.preview_state == PredictionPreviewRuntimeState::AwaitFullRefine;
    }

    inline bool prediction_track_can_transition_to_await_full_refine(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.preview_state == PredictionPreviewRuntimeState::EnterDrag ||
               snapshot.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
               snapshot.preview_state == PredictionPreviewRuntimeState::PreviewStreaming;
    }

    inline bool prediction_track_preserves_await_full_refine(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.preview_state == PredictionPreviewRuntimeState::AwaitFullRefine;
    }

    inline bool prediction_track_preview_fallback_active(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.preview_state == PredictionPreviewRuntimeState::EnterDrag ||
               snapshot.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
               snapshot.preview_state == PredictionPreviewRuntimeState::PreviewStreaming;
    }

    inline bool prediction_track_preview_overlay_draw_active(const PredictionTrackLifecycleSnapshot &snapshot,
                                                             const bool preview_anchor_valid)
    {
        return preview_anchor_valid && prediction_track_preview_fallback_active(snapshot);
    }

    inline bool prediction_track_preview_pick_clamp_active(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
               snapshot.preview_state == PredictionPreviewRuntimeState::PreviewStreaming;
    }

    inline bool prediction_track_should_accept_preview_publish(const PredictionTrackLifecycleSnapshot &snapshot,
                                                               const bool live_preview_active_now,
                                                               const bool preview_anchor_valid)
    {
        return live_preview_active_now &&
               prediction_track_preview_overlay_draw_active(snapshot, preview_anchor_valid);
    }

    inline bool prediction_track_is_preview_streaming_publish(const OrbitPredictionService::SolveQuality solve_quality,
                                                              const OrbitPredictionService::PublishStage publish_stage)
    {
        return solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
               publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming;
    }

    inline bool prediction_track_is_full_streaming_publish(const OrbitPredictionService::SolveQuality solve_quality,
                                                           const OrbitPredictionService::PublishStage publish_stage)
    {
        return solve_quality == OrbitPredictionService::SolveQuality::Full &&
               publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
    }

    inline bool prediction_track_should_keep_request_pending_after_solver_publish(
            const OrbitPredictionService::SolveQuality solve_quality,
            const OrbitPredictionService::PublishStage publish_stage)
    {
        return prediction_track_is_full_streaming_publish(solve_quality, publish_stage);
    }

    inline bool prediction_track_should_keep_dirty_for_followup(const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.dirty || snapshot.invalidated_while_pending;
    }

    inline bool prediction_track_should_promote_dirty_after_solver_publish(
            const PredictionTrackLifecycleSnapshot &snapshot)
    {
        return snapshot.invalidated_while_pending;
    }

    inline PredictionPreviewRuntimeState prediction_track_preview_state_after_preview_publish(
            const PredictionTrackLifecycleSnapshot &snapshot,
            const OrbitPredictionService::PublishStage publish_stage,
            const bool live_preview_active_now)
    {
        if (publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming)
        {
            return PredictionPreviewRuntimeState::PreviewStreaming;
        }

        if (prediction_track_preserves_await_full_refine(snapshot))
        {
            return PredictionPreviewRuntimeState::AwaitFullRefine;
        }

        return live_preview_active_now
                       ? PredictionPreviewRuntimeState::PreviewStreaming
                       : PredictionPreviewRuntimeState::Idle;
    }

    inline bool prediction_track_should_clear_preview_anchor_after_final_publish(const bool live_preview_active_now)
    {
        return !live_preview_active_now;
    }

    inline const char *prediction_track_lifecycle_name(const PredictionTrackLifecycleState state)
    {
        switch (state)
        {
            case PredictionTrackLifecycleState::Idle: return "Idle";
            case PredictionTrackLifecycleState::NeedsRebuild: return "NeedsRebuild";
            case PredictionTrackLifecycleState::DragPreviewPending: return "DragPreviewPending";
            case PredictionTrackLifecycleState::PreviewStreaming: return "PreviewStreaming";
            case PredictionTrackLifecycleState::AwaitFullRefine: return "AwaitFullRefine";
            case PredictionTrackLifecycleState::FullSolvePending: return "FullSolvePending";
            case PredictionTrackLifecycleState::FullStreaming: return "FullStreaming";
            case PredictionTrackLifecycleState::FinalDerivedPending: return "FinalDerivedPending";
            case PredictionTrackLifecycleState::Stable: return "Stable";
        }
        return "Unknown";
    }
} // namespace Game::PredictionRuntimeDetail
