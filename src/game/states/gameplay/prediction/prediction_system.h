#pragma once

#include "game/states/gameplay/prediction/gameplay_prediction_state.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_context.h"

#include <cstdint>
#include <vector>

namespace Game
{
    class PredictionSystem
    {
    public:
        explicit PredictionSystem(GameplayPredictionState &prediction)
            : _prediction(prediction)
        {
        }

        GameplayPredictionState &state();
        const GameplayPredictionState &state() const;

        OrbitPredictionService &service();
        const OrbitPredictionService &service() const;

        OrbitPredictionDerivedService &derived_service();
        const OrbitPredictionDerivedService &derived_service() const;

        void reset_solver_service();
        void reset_derived_service();
        void reset_services();

        bool any_visible_track_dirty(const std::vector<PredictionSubjectKey> &visible_subjects) const;
        void sync_visible_dirty_flag(const std::vector<PredictionSubjectKey> &visible_subjects);
        void mark_visible_tracks_dirty(const std::vector<PredictionSubjectKey> &visible_subjects);
        void invalidate_maneuver_plan_revision(uint64_t maneuver_plan_revision);
        void clear_maneuver_prediction_artifacts();

        void clear_runtime();
        void clear_visible_runtime(const std::vector<PredictionSubjectKey> &visible_subjects);
        bool poll_completed_results(const PredictionRuntimeContext &context);
        bool apply_completed_solver_result(const PredictionRuntimeContext &context,
                                           OrbitPredictionService::Result result);
        bool apply_completed_derived_result(const PredictionRuntimeContext &context,
                                            OrbitPredictionDerivedService::Result result);

        bool should_rebuild_track(const PredictionRuntimeContext &context,
                                  const PredictionTrackState &track,
                                  double now_s,
                                  float fixed_dt,
                                  bool thrusting,
                                  bool with_maneuvers) const;

        bool request_orbiter_prediction_async(const PredictionRuntimeContext &context,
                                              PredictionTrackState &track,
                                              const WorldVec3 &subject_pos_world,
                                              const glm::dvec3 &subject_vel_world,
                                              double now_s,
                                              bool thrusting,
                                              bool with_maneuvers,
                                              bool *out_throttled = nullptr);

        bool request_celestial_prediction_async(const PredictionRuntimeContext &context,
                                                PredictionTrackState &track,
                                                double now_s);

        void update_orbiter_prediction_track(const PredictionRuntimeContext &context,
                                             PredictionTrackState &track,
                                             double now_s,
                                             bool thrusting,
                                             bool with_maneuvers);

        void update_celestial_prediction_track(const PredictionRuntimeContext &context,
                                               PredictionTrackState &track,
                                               double now_s);

        void update_visible_tracks(const PredictionRuntimeContext &context,
                                   const std::vector<PredictionSubjectKey> &visible_subjects,
                                   double now_s,
                                   float fixed_dt);

    private:
        GameplayPredictionState &_prediction;
    };
} // namespace Game
