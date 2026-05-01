#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"

#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"

#include <algorithm>

namespace Game
{
    namespace
    {
        bool contains_key(const std::vector<PredictionSubjectKey> &keys, const PredictionSubjectKey key)
        {
            return std::find(keys.begin(), keys.end(), key) != keys.end();
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

            PredictionLifecycleReducer::mark_dirty(track);
        }
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
                PredictionLifecycleReducer::clear_pending_maneuver_requests(track, true);
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
            PredictionLifecycleReducer::reset_preview(track);
            track.preview_overlay.clear();
            track.full_stream_overlay.clear();
            track.pick_cache.clear();

            PredictionLifecycleReducer::clear_maneuver_scoped_pending_requests(track);
        }
    }
} // namespace Game
