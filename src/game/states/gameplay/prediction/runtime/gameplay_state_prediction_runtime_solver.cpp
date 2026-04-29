#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/runtime/prediction_solver_result_applier.h"

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
    } // namespace

    void GameplayState::poll_completed_prediction_results()
    {
        bool applied_result = false;
        while (auto completed = _prediction.service.poll_completed())
        {
            apply_completed_prediction_result(std::move(*completed));
            applied_result = true;
        }

        // Keep streaming publishes visible across frames instead of immediately
        // draining a full-stream chunk and the final replacement in the same tick.
        while (auto completed = _prediction.derived_service.poll_completed())
        {
            const bool streaming_publish =
                    completed->publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming ||
                    completed->publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
            apply_completed_prediction_derived_result(std::move(*completed));
            applied_result = true;
            if (streaming_publish)
            {
                break;
            }
        }

        if (applied_result)
        {
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        PredictionTrackState *track = find_track_by_id(_prediction.tracks, result.track_id);
        if (!track)
        {
            return;
        }

        PredictionSolverResultApplyResult applied =
                PredictionSolverResultApplier::apply_solver_result(*track,
                                                                   std::move(result),
                                                                   build_prediction_runtime_context());
        if (!applied.derived_request)
        {
            return;
        }

        _prediction.derived_service.request(*applied.derived_request);
        mark_prediction_derived_request_submitted(*track, *applied.derived_request);
    }
} // namespace Game
