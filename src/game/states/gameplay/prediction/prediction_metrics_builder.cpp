#include "game/states/gameplay/prediction/prediction_metrics_builder.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

namespace Game
{
    void PredictionMetricsBuilder::clear(OrbitPredictionCache &cache, const orbitsim::BodyId analysis_body_id)
    {
        PredictionCacheInternal::clear_prediction_metrics(cache, analysis_body_id);
    }

    void PredictionMetricsBuilder::rebuild(
            const PredictionSolverTrajectoryCache &solver,
            const PredictionDisplayFrameCache &display,
            PredictionAnalysisCache &analysis,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested)
    {
        PredictionCacheInternal::rebuild_prediction_metrics(solver,
                                                            display,
                                                            analysis,
                                                            sim_config,
                                                            analysis_body_id,
                                                            cancel_requested);
    }

    void PredictionMetricsBuilder::rebuild(
            OrbitPredictionCache &cache,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested)
    {
        PredictionCacheInternal::rebuild_prediction_metrics(cache,
                                                            sim_config,
                                                            analysis_body_id,
                                                            cancel_requested);
    }
} // namespace Game
