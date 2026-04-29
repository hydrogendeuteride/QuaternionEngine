#pragma once

#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <functional>

namespace Game
{
    struct PredictionMetricsBuilder
    {
        using CancelCheck = std::function<bool()>;

        static void clear(OrbitPredictionCache &cache, orbitsim::BodyId analysis_body_id);

        static void rebuild(
                const PredictionSolverTrajectoryCache &solver,
                const PredictionDisplayFrameCache &display,
                PredictionAnalysisCache &analysis,
                const orbitsim::GameSimulation::Config &sim_config,
                orbitsim::BodyId analysis_body_id,
                const CancelCheck &cancel_requested = {});

        static void rebuild(
                OrbitPredictionCache &cache,
                const orbitsim::GameSimulation::Config &sim_config,
                orbitsim::BodyId analysis_body_id,
                const CancelCheck &cancel_requested = {});
    };
} // namespace Game
