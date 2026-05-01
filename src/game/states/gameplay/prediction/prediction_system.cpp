#include "game/states/gameplay/prediction/prediction_system.h"

#include "game/states/gameplay/prediction/gameplay_prediction_state.h"
#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"
#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"

namespace Game
{
    OrbitPredictionDerivedService &PredictionSystem::derived_service()
    {
        return _prediction.derived_service;
    }

    const OrbitPredictionDerivedService &PredictionSystem::derived_service() const
    {
        return _prediction.derived_service;
    }

    void PredictionSystem::reset_solver_service()
    {
        _prediction.service.reset();
    }

    void PredictionSystem::reset_derived_service()
    {
        _prediction.derived_service.reset();
    }

    void PredictionSystem::reset_services()
    {
        reset_solver_service();
        reset_derived_service();
    }

    bool PredictionSystem::any_visible_track_dirty(const std::vector<PredictionSubjectKey> &visible_subjects) const
    {
        return PredictionInvalidationController::any_visible_track_dirty(_prediction.tracks, visible_subjects);
    }

    void PredictionSystem::sync_visible_dirty_flag(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        _prediction.dirty = any_visible_track_dirty(visible_subjects);
    }

    void PredictionSystem::mark_visible_tracks_dirty(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        PredictionInvalidationController::mark_visible_tracks_dirty(_prediction.tracks, visible_subjects);
    }

    void PredictionSystem::invalidate_maneuver_plan_revision(const uint64_t maneuver_plan_revision)
    {
        PredictionInvalidationController::invalidate_maneuver_plan_revision(_prediction.tracks,
                                                                           _prediction.service,
                                                                           _prediction.derived_service,
                                                                           maneuver_plan_revision);
    }

    void PredictionSystem::clear_maneuver_prediction_artifacts()
    {
        PredictionInvalidationController::clear_maneuver_prediction_artifacts(_prediction.tracks);
    }

    void PredictionSystem::mark_maneuver_preview_dirty(PredictionTrackState &track)
    {
        PredictionLifecycleReducer::mark_dirty_for_preview(track);
    }

    void PredictionSystem::await_maneuver_preview_full_refine(PredictionTrackState &track,
                                                              const double now_s)
    {
        PredictionLifecycleReducer::await_full_refine(track, now_s);
    }

    void PredictionSystem::clear_unapplied_maneuver_drag_preview(PredictionTrackState &track)
    {
        if (track.preview_state != PredictionPreviewRuntimeState::EnterDrag &&
            track.preview_state != PredictionPreviewRuntimeState::DragPreviewPending)
        {
            return;
        }

        PredictionLifecycleReducer::reset_preview(track);
        track.preview_overlay.clear();
        track.pick_cache.clear();
    }

    void PredictionSystem::clear_maneuver_live_preview_state()
    {
        for (PredictionTrackState &track : _prediction.tracks)
        {
            if (!track.supports_maneuvers)
            {
                continue;
            }

            PredictionLifecycleReducer::reset_preview(track);
            track.preview_overlay.clear();
            track.pick_cache.clear();
        }
    }
} // namespace Game
