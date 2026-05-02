#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

#include <utility>

namespace Game
{
    void GameplayPredictionAdapter::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        (void) _prediction->apply_completed_derived_result(build_prediction_host_context(), std::move(result));
    }
} // namespace Game
