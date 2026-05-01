#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_state.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_context.h"

#include <vector>

namespace Game
{
    class PredictionRuntimeController
    {
    public:
        static void clear_runtime(GameplayPredictionState &prediction);

        static void clear_visible_runtime(GameplayPredictionState &prediction,
                                          const std::vector<PredictionSubjectKey> &visible_subjects);

        static bool poll_completed_results(GameplayPredictionState &prediction,
                                           const PredictionRuntimeContext &context);

        static bool apply_completed_solver_result(GameplayPredictionState &prediction,
                                                  const PredictionRuntimeContext &context,
                                                  OrbitPredictionService::Result result);

        static bool apply_completed_derived_result(GameplayPredictionState &prediction,
                                                   const PredictionRuntimeContext &context,
                                                   OrbitPredictionDerivedService::Result result);

        static bool should_rebuild_track(const GameplayPredictionState &prediction,
                                         const PredictionRuntimeContext &context,
                                         const PredictionTrackState &track,
                                         double now_s,
                                         float fixed_dt,
                                         bool thrusting,
                                         bool with_maneuvers);

        static bool request_orbiter_prediction_async(GameplayPredictionState &prediction,
                                                     const PredictionRuntimeContext &context,
                                                     PredictionTrackState &track,
                                                     const WorldVec3 &subject_pos_world,
                                                     const glm::dvec3 &subject_vel_world,
                                                     double now_s,
                                                     bool thrusting,
                                                     bool with_maneuvers,
                                                     bool *out_throttled = nullptr);

        static bool request_celestial_prediction_async(GameplayPredictionState &prediction,
                                                       const PredictionRuntimeContext &context,
                                                       PredictionTrackState &track,
                                                       double now_s);

        static void update_orbiter_prediction_track(GameplayPredictionState &prediction,
                                                    const PredictionRuntimeContext &context,
                                                    PredictionTrackState &track,
                                                    double now_s,
                                                    bool thrusting,
                                                    bool with_maneuvers);

        static void update_celestial_prediction_track(GameplayPredictionState &prediction,
                                                      const PredictionRuntimeContext &context,
                                                      PredictionTrackState &track,
                                                      double now_s);

        static void update_visible_tracks(GameplayPredictionState &prediction,
                                          const PredictionRuntimeContext &context,
                                          const std::vector<PredictionSubjectKey> &visible_subjects,
                                          double now_s,
                                          float fixed_dt);
    };
} // namespace Game
