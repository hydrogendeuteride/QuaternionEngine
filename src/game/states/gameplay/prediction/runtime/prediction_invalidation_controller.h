#pragma once

#include "game/orbit/orbit_prediction_service.h"
#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>
#include <vector>

namespace Game
{
    class PredictionInvalidationController
    {
    public:
        static bool any_visible_track_dirty(const std::vector<PredictionTrackState> &tracks,
                                            const std::vector<PredictionSubjectKey> &visible_subjects);

        static void mark_visible_tracks_dirty(std::vector<PredictionTrackState> &tracks,
                                              const std::vector<PredictionSubjectKey> &visible_subjects);

        static void invalidate_maneuver_plan_revision(std::vector<PredictionTrackState> &tracks,
                                                      OrbitPredictionService &prediction_service,
                                                      OrbitPredictionDerivedService &derived_service,
                                                      uint64_t maneuver_plan_revision);

        static void clear_maneuver_prediction_artifacts(std::vector<PredictionTrackState> &tracks);
    };
} // namespace Game
