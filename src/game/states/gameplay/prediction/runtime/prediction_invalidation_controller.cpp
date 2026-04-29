#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"

#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <algorithm>

namespace Game
{
    namespace
    {
        bool contains_key(const std::vector<PredictionSubjectKey> &keys, const PredictionSubjectKey key)
        {
            return std::find(keys.begin(), keys.end(), key) != keys.end();
        }

        void clear_pending_maneuver_requests(PredictionTrackState &track)
        {
            track.request_pending = false;
            track.derived_request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track.pending_solver_has_maneuver_plan = false;
            track.pending_solver_plan_signature = 0u;
            track.pending_derived_has_maneuver_plan = false;
            track.pending_derived_plan_signature = 0u;
            track.invalidated_while_pending = false;
        }
    } // namespace

    bool PredictionInvalidationController::any_visible_track_dirty(
            const std::vector<PredictionTrackState> &tracks,
            const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        for (const PredictionTrackState &track : tracks)
        {
            if (contains_key(visible_subjects, track.key) && track.dirty)
            {
                return true;
            }
        }

        return false;
    }

    void PredictionInvalidationController::mark_visible_tracks_dirty(
            std::vector<PredictionTrackState> &tracks,
            const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        for (PredictionTrackState &track : tracks)
        {
            if (!contains_key(visible_subjects, track.key))
            {
                continue;
            }

            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                    PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
            if (PredictionRuntimeDetail::prediction_track_should_mark_invalidated_while_pending(lifecycle))
            {
                track.invalidated_while_pending = true;
                continue;
            }

            track.dirty = true;
        }
    }

    void PredictionInvalidationController::mark_track_dirty_for_preview(PredictionTrackState &track)
    {
        track.dirty = true;
        track.invalidated_while_pending = track.request_pending || track.derived_request_pending;
    }

    void PredictionInvalidationController::invalidate_maneuver_plan_revision(
            std::vector<PredictionTrackState> &tracks,
            OrbitPredictionService &prediction_service,
            OrbitPredictionDerivedService &derived_service,
            const uint64_t maneuver_plan_revision)
    {
        for (PredictionTrackState &track : tracks)
        {
            if (!track.supports_maneuvers)
            {
                continue;
            }

            const uint64_t track_id = track.key.track_id();
            prediction_service.invalidate_maneuver_plan_revision(track_id, maneuver_plan_revision);
            derived_service.invalidate_maneuver_plan_revision(track_id, maneuver_plan_revision);

            if (track.request_pending || track.derived_request_pending)
            {
                clear_pending_maneuver_requests(track);
                track.dirty = true;
            }
        }
    }

    void PredictionInvalidationController::clear_maneuver_prediction_artifacts(
            std::vector<PredictionTrackState> &tracks)
    {
        for (PredictionTrackState &track : tracks)
        {
            if (!track.supports_maneuvers)
            {
                continue;
            }

            clear_prediction_cache_planned_data(track.cache);
            clear_prediction_cache_planned_data(track.authoritative_cache);
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor = {};
            track.preview_overlay.clear();
            track.full_stream_overlay.clear();
            track.pick_cache.clear();

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
    }
} // namespace Game
