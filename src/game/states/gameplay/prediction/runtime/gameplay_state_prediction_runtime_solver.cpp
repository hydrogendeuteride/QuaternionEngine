#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

#include <utility>

namespace Game
{
    void GameplayState::poll_completed_prediction_results()
    {
        if (PredictionRuntimeController::poll_completed_results(_prediction,
                                                                build_prediction_runtime_context()))
        {
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        (void) PredictionRuntimeController::apply_completed_solver_result(_prediction,
                                                                          build_prediction_runtime_context(),
                                                                          std::move(result));
    }
} // namespace Game
