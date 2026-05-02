#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

namespace Game
{
    PredictionRuntimeContext GameplayState::build_prediction_runtime_context() const
    {
        PredictionRuntimeContext context{};
        context.orbital_scenario = _orbitsim.get();
        context.selection = _prediction.selection;
        context.frame_selection = _prediction.frame_selection;
        context.analysis_selection = _prediction.analysis_selection;
        context.sampling_policy = _prediction.sampling_policy;
        context.maneuver_plan_windows = _maneuver.settings().plan_windows;
        context.maneuver_plan = &_maneuver.plan();
        context.maneuver_edit_preview = &_maneuver.edit_preview();
        context.maneuver_nodes_enabled = _maneuver.settings().nodes_enabled;
        context.maneuver_edit_in_progress =
                PredictionRuntimeDetail::maneuver_drag_active(_maneuver.gizmo_interaction().state) ||
                _maneuver.edit_preview().state != ManeuverNodeEditPreview::State::Idle;
        context.maneuver_live_preview_available = maneuver_live_preview_active(true);
        context.maneuver_plan_revision = _maneuver.revision();
        context.maneuver_plan_signature = current_maneuver_plan_signature();
        context.display_frame_revision = _prediction.display_frame_revision;
        context.active_maneuver_preview_anchor_node_id = active_maneuver_preview_anchor_node_id();
        context.current_sim_time_s = current_sim_time_s();
        context.reference_body_world = [this]() { return prediction_world_reference_body_world(); };
        context.get_subject_world_state = [this](const PredictionSubjectKey key,
                                                 WorldVec3 &out_pos_world,
                                                 glm::dvec3 &out_vel_world,
                                                 glm::vec3 &out_vel_local) {
            return get_prediction_subject_world_state(key, out_pos_world, out_vel_world, out_vel_local);
        };
        context.future_window_s = [this](const PredictionSubjectKey key) {
            return prediction_future_window_s(key);
        };
        context.required_window_s = [this](const PredictionTrackState &track,
                                           const double now_s,
                                           const bool with_maneuvers) {
            return prediction_required_window_s(track, now_s, with_maneuvers);
        };
        context.preview_exact_window_s = [this](const PredictionTrackState &track,
                                                const double now_s,
                                                const bool with_maneuvers) {
            return prediction_preview_exact_window_s(track, now_s, with_maneuvers);
        };
        context.refresh_preview_anchor = [this](PredictionTrackState &track,
                                                const double now_s,
                                                const bool with_maneuvers) {
            refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
        };
        context.subject_is_player = [this](const PredictionSubjectKey key) {
            return prediction_subject_is_player(key);
        };
        context.subject_thrust_applied_this_tick = [this](const PredictionSubjectKey key) {
            return prediction_subject_thrust_applied_this_tick(key);
        };
        context.resolve_maneuver_node_primary_body_id = [this](const ManeuverNode &node, const double query_time_s) {
            return resolve_maneuver_node_primary_body_id(node, query_time_s);
        };
        context.resolve_display_frame_spec = [this](const OrbitPredictionCache &cache, const double display_time_s) {
            return resolve_prediction_display_frame_spec(cache, display_time_s);
        };
        context.resolve_analysis_body_id = [this](const OrbitPredictionCache &cache,
                                                  const PredictionSubjectKey key,
                                                  const double query_time_s,
                                                  const orbitsim::BodyId preferred_body_id) {
            return resolve_prediction_analysis_body_id(cache, key, query_time_s, preferred_body_id);
        };
        context.player_effective_cache = [this]() -> const OrbitPredictionCache * {
            return effective_prediction_cache(player_prediction_track());
        };
        return context;
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers,
                                                         bool *out_throttled)
    {
        return PredictionRuntimeController::request_orbiter_prediction_async(_prediction,
                                                                             build_prediction_runtime_context(),
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
        return PredictionRuntimeController::request_celestial_prediction_async(_prediction,
                                                                               build_prediction_runtime_context(),
                                                                               track,
                                                                               now_s);
    }

    void GameplayState::update_orbiter_prediction_track(PredictionTrackState &track,
                                                        const double now_s,
                                                        const bool thrusting,
                                                        const bool with_maneuvers)
    {
        PredictionRuntimeController::update_orbiter_prediction_track(_prediction,
                                                                     build_prediction_runtime_context(),
                                                                     track,
                                                                     now_s,
                                                                     thrusting,
                                                                     with_maneuvers);
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        PredictionRuntimeController::update_celestial_prediction_track(_prediction,
                                                                       build_prediction_runtime_context(),
                                                                       track,
                                                                       now_s);
    }
} // namespace Game
