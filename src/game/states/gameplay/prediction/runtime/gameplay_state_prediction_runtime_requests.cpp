#include "game/states/gameplay/gameplay_state.h"

namespace Game
{
    PredictionRuntimeContext GameplayState::build_prediction_runtime_context() const
    {
        return _prediction->build_runtime_context(build_prediction_host_context());
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers,
                                                         bool *out_throttled)
    {
        return _prediction->request_orbiter_prediction_async(build_prediction_host_context(),
                                                             track,
                                                             subject_pos_world,
                                                             subject_vel_world,
                                                             now_s,
                                                             thrusting,
                                                             with_maneuvers,
                                                             out_throttled);
    }

    bool GameplayState::request_celestial_prediction_async(PredictionTrackState &track, const double now_s)
    {
        return _prediction->request_celestial_prediction_async(build_prediction_host_context(), track, now_s);
    }

    void GameplayState::update_orbiter_prediction_track(PredictionTrackState &track,
                                                        const double now_s,
                                                        const bool thrusting,
                                                        const bool with_maneuvers)
    {
        _prediction->update_orbiter_prediction_track(build_prediction_host_context(),
                                                     track,
                                                     now_s,
                                                     thrusting,
                                                     with_maneuvers);
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        _prediction->update_celestial_prediction_track(build_prediction_host_context(), track, now_s);
    }
} // namespace Game
