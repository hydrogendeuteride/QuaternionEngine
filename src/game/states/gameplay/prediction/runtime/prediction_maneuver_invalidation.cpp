#include "game/states/gameplay/prediction/runtime/prediction_maneuver_invalidation.h"

#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"

namespace Game::PredictionManeuverInvalidation
{
    void invalidate_maneuver_plan_revision(std::vector<PredictionTrackState> &tracks,
                                           OrbitPredictionService &prediction_service,
                                           OrbitPredictionDerivedService &derived_service,
                                           const uint64_t maneuver_plan_revision)
    {
        PredictionInvalidationController::invalidate_maneuver_plan_revision(tracks,
                                                                           prediction_service,
                                                                           derived_service,
                                                                           maneuver_plan_revision);
    }
}
