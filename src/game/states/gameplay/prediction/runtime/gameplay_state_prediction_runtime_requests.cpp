#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"

namespace Game
{
    PredictionRuntimeContext GameplayPredictionAdapter::build_prediction_runtime_context() const
    {
        return _prediction->build_runtime_context(_state.build_prediction_host_context());
    }

    bool GameplayPredictionAdapter::request_orbiter_prediction_async(PredictionTrackState &track,
                                                                     const WorldVec3 &subject_pos_world,
                                                                     const glm::dvec3 &subject_vel_world,
                                                                     const double now_s,
                                                                     const bool thrusting,
                                                                     const bool with_maneuvers,
                                                                     bool *out_throttled)
    {
        return _prediction->request_orbiter_prediction_async(_state.build_prediction_host_context(),
                                                             track,
                                                             subject_pos_world,
                                                             subject_vel_world,
                                                             now_s,
                                                             thrusting,
                                                             with_maneuvers,
                                                             out_throttled);
    }

    bool GameplayPredictionAdapter::request_celestial_prediction_async(PredictionTrackState &track, const double now_s)
    {
        return _prediction->request_celestial_prediction_async(_state.build_prediction_host_context(), track, now_s);
    }

    void GameplayPredictionAdapter::update_orbiter_prediction_track(PredictionTrackState &track,
                                                                    const double now_s,
                                                                    const bool thrusting,
                                                                    const bool with_maneuvers)
    {
        _prediction->update_orbiter_prediction_track(_state.build_prediction_host_context(),
                                                     track,
                                                     now_s,
                                                     thrusting,
                                                     with_maneuvers);
    }

    void GameplayPredictionAdapter::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        _prediction->update_celestial_prediction_track(_state.build_prediction_host_context(), track, now_s);
    }
} // namespace Game
