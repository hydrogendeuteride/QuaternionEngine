#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction/prediction_host_context_builder.h"

namespace Game
{
    PredictionRuntimeContext GameplayPredictionAdapter::build_prediction_runtime_context() const
    {
        return _state._prediction->build_runtime_context(PredictionHostContextBuilder(context()).build());
    }

    bool GameplayPredictionAdapter::request_orbiter_prediction_async(PredictionTrackState &track,
                                                                     const WorldVec3 &subject_pos_world,
                                                                     const glm::dvec3 &subject_vel_world,
                                                                     const double now_s,
                                                                     const bool thrusting,
                                                                     const bool with_maneuvers,
                                                                     bool *out_throttled)
    {
        return _state._prediction->request_orbiter_prediction_async(PredictionHostContextBuilder(context()).build(),
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
        return _state._prediction->request_celestial_prediction_async(
                PredictionHostContextBuilder(context()).build(),
                track,
                now_s);
    }

    void GameplayPredictionAdapter::update_orbiter_prediction_track(PredictionTrackState &track,
                                                                    const double now_s,
                                                                    const bool thrusting,
                                                                    const bool with_maneuvers)
    {
        _state._prediction->update_orbiter_prediction_track(PredictionHostContextBuilder(context()).build(),
                                                            track,
                                                            now_s,
                                                            thrusting,
                                                            with_maneuvers);
    }

    void GameplayPredictionAdapter::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        _state._prediction->update_celestial_prediction_track(
                PredictionHostContextBuilder(context()).build(),
                track,
                now_s);
    }
} // namespace Game
