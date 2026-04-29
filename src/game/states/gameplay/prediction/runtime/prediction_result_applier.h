#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>

namespace Game
{
    struct PredictionDerivedResultApplyContext
    {
        uint64_t current_maneuver_plan_revision{0};
        bool live_preview_active{false};
        bool current_plan_active{false};
        uint64_t current_plan_signature{0};
        bool active_maneuver_edit{false};
    };

    class PredictionResultApplier
    {
    public:
        static void apply_derived_result(PredictionTrackState &track,
                                         OrbitPredictionDerivedService::Result result,
                                         const PredictionDerivedResultApplyContext &context);
    };
} // namespace Game
