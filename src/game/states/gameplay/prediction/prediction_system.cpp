#include "game/states/gameplay/prediction/prediction_system.h"

#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

#include <utility>

namespace Game
{
    GameplayPredictionState &PredictionSystem::state()
    {
        return _prediction;
    }

    const GameplayPredictionState &PredictionSystem::state() const
    {
        return _prediction;
    }

    OrbitPredictionService &PredictionSystem::service()
    {
        return _prediction.service;
    }

    const OrbitPredictionService &PredictionSystem::service() const
    {
        return _prediction.service;
    }

    OrbitPredictionDerivedService &PredictionSystem::derived_service()
    {
        return _prediction.derived_service;
    }

    const OrbitPredictionDerivedService &PredictionSystem::derived_service() const
    {
        return _prediction.derived_service;
    }

    void PredictionSystem::reset_solver_service()
    {
        _prediction.service.reset();
    }

    void PredictionSystem::reset_derived_service()
    {
        _prediction.derived_service.reset();
    }

    void PredictionSystem::reset_services()
    {
        reset_solver_service();
        reset_derived_service();
    }

    bool PredictionSystem::any_visible_track_dirty(const std::vector<PredictionSubjectKey> &visible_subjects) const
    {
        return PredictionInvalidationController::any_visible_track_dirty(_prediction.tracks, visible_subjects);
    }

    void PredictionSystem::sync_visible_dirty_flag(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        _prediction.dirty = any_visible_track_dirty(visible_subjects);
    }

    void PredictionSystem::mark_visible_tracks_dirty(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        PredictionInvalidationController::mark_visible_tracks_dirty(_prediction.tracks, visible_subjects);
    }

    void PredictionSystem::invalidate_maneuver_plan_revision(const uint64_t maneuver_plan_revision)
    {
        PredictionInvalidationController::invalidate_maneuver_plan_revision(_prediction.tracks,
                                                                           _prediction.service,
                                                                           _prediction.derived_service,
                                                                           maneuver_plan_revision);
    }

    void PredictionSystem::clear_maneuver_prediction_artifacts()
    {
        PredictionInvalidationController::clear_maneuver_prediction_artifacts(_prediction.tracks);
    }

    void PredictionSystem::clear_runtime()
    {
        PredictionRuntimeController::clear_runtime(_prediction);
    }

    void PredictionSystem::clear_visible_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        PredictionRuntimeController::clear_visible_runtime(_prediction, visible_subjects);
    }

    bool PredictionSystem::poll_completed_results(const PredictionRuntimeContext &context)
    {
        return PredictionRuntimeController::poll_completed_results(_prediction, context);
    }

    bool PredictionSystem::apply_completed_solver_result(const PredictionRuntimeContext &context,
                                                         OrbitPredictionService::Result result)
    {
        return PredictionRuntimeController::apply_completed_solver_result(_prediction,
                                                                         context,
                                                                         std::move(result));
    }

    bool PredictionSystem::apply_completed_derived_result(const PredictionRuntimeContext &context,
                                                          OrbitPredictionDerivedService::Result result)
    {
        return PredictionRuntimeController::apply_completed_derived_result(_prediction,
                                                                          context,
                                                                          std::move(result));
    }

    bool PredictionSystem::should_rebuild_track(const PredictionRuntimeContext &context,
                                                const PredictionTrackState &track,
                                                const double now_s,
                                                const float fixed_dt,
                                                const bool thrusting,
                                                const bool with_maneuvers) const
    {
        return PredictionRuntimeController::should_rebuild_track(_prediction,
                                                                 context,
                                                                 track,
                                                                 now_s,
                                                                 fixed_dt,
                                                                 thrusting,
                                                                 with_maneuvers);
    }

    bool PredictionSystem::request_orbiter_prediction_async(const PredictionRuntimeContext &context,
                                                            PredictionTrackState &track,
                                                            const WorldVec3 &subject_pos_world,
                                                            const glm::dvec3 &subject_vel_world,
                                                            const double now_s,
                                                            const bool thrusting,
                                                            const bool with_maneuvers,
                                                            bool *out_throttled)
    {
        return PredictionRuntimeController::request_orbiter_prediction_async(_prediction,
                                                                             context,
                                                                             track,
                                                                             subject_pos_world,
                                                                             subject_vel_world,
                                                                             now_s,
                                                                             thrusting,
                                                                             with_maneuvers,
                                                                             out_throttled);
    }

    bool PredictionSystem::request_celestial_prediction_async(const PredictionRuntimeContext &context,
                                                              PredictionTrackState &track,
                                                              const double now_s)
    {
        return PredictionRuntimeController::request_celestial_prediction_async(_prediction, context, track, now_s);
    }

    void PredictionSystem::update_orbiter_prediction_track(const PredictionRuntimeContext &context,
                                                           PredictionTrackState &track,
                                                           const double now_s,
                                                           const bool thrusting,
                                                           const bool with_maneuvers)
    {
        PredictionRuntimeController::update_orbiter_prediction_track(_prediction,
                                                                    context,
                                                                    track,
                                                                    now_s,
                                                                    thrusting,
                                                                    with_maneuvers);
    }

    void PredictionSystem::update_celestial_prediction_track(const PredictionRuntimeContext &context,
                                                             PredictionTrackState &track,
                                                             const double now_s)
    {
        PredictionRuntimeController::update_celestial_prediction_track(_prediction, context, track, now_s);
    }

    void PredictionSystem::update_visible_tracks(const PredictionRuntimeContext &context,
                                                 const std::vector<PredictionSubjectKey> &visible_subjects,
                                                 const double now_s,
                                                 const float fixed_dt)
    {
        PredictionRuntimeController::update_visible_tracks(_prediction,
                                                           context,
                                                           visible_subjects,
                                                           now_s,
                                                           fixed_dt);
    }
} // namespace Game
