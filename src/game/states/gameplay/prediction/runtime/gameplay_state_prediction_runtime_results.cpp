#include "game/states/gameplay/gameplay_state.h"

#include <utility>

namespace Game
{
    void GameplayState::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        (void) _prediction_system.apply_completed_derived_result(build_prediction_runtime_context(),
                                                                 std::move(result));
    }
} // namespace Game
