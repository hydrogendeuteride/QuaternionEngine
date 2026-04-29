#pragma once

#include "game/states/gameplay/prediction/runtime/prediction_runtime_context.h"

namespace Game
{
    struct PredictionOrbiterRequestBuildResult
    {
        bool built{false};
        bool interactive_request{false};
        bool preview_request_active{false};
        OrbitPredictionService::Request request{};
    };

    class PredictionRequestFactory
    {
    public:
        static PredictionOrbiterRequestBuildResult build_orbiter_request(
                const PredictionRuntimeContext &context,
                PredictionTrackState &track,
                const WorldVec3 &subject_pos_world,
                const glm::dvec3 &subject_vel_world,
                double now_s,
                bool thrusting,
                bool with_maneuvers);

        static bool build_celestial_request(const PredictionRuntimeContext &context,
                                            const PredictionTrackState &track,
                                            double now_s,
                                            OrbitPredictionService::Request &out_request);

        static bool resolve_preview_anchor_state(const PredictionRuntimeContext &context,
                                                 const PredictionTrackState &track,
                                                 orbitsim::State &out_state);

        static bool resolve_preview_anchor_state(const PredictionRuntimeContext &context,
                                                 const PredictionTrackState &track,
                                                 orbitsim::State &out_state,
                                                 bool &out_trusted);
    };
} // namespace Game
