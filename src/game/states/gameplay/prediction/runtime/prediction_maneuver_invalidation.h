#pragma once

#include "game/orbit/orbit_prediction_service.h"
#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>
#include <vector>

namespace Game::PredictionManeuverInvalidation
{
    void invalidate_maneuver_plan_revision(std::vector<PredictionTrackState> &tracks,
                                           OrbitPredictionService &prediction_service,
                                           OrbitPredictionDerivedService &derived_service,
                                           uint64_t maneuver_plan_revision);
}
