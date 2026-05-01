#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

#include <utility>

namespace Game
{
    void GameplayState::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        (void) PredictionRuntimeController::apply_completed_derived_result(_prediction,
                                                                           build_prediction_runtime_context(),
                                                                           std::move(result));
    }
} // namespace Game
