#include "game/states/gameplay/gameplay_state.h"

#include <utility>

namespace Game
{
    void GameplayState::poll_completed_prediction_results()
    {
        if (_prediction->poll_completed_results(build_prediction_host_context()))
        {
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        (void) _prediction->apply_completed_solver_result(build_prediction_host_context(), std::move(result));
    }
} // namespace Game
