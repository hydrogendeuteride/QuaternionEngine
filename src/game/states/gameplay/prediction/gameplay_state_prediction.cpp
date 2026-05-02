#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/maneuver_prediction_bridge.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction/prediction_subject_state_provider.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    PredictionSubjectStateProvider GameplayPredictionAdapter::make_subject_state_provider() const
    {
        return PredictionSubjectStateProvider(PredictionSubjectStateProvider::Context{
                .orbit = _orbit,
                .world = _world,
                .physics = _physics.get(),
                .physics_context = _physics_context.get(),
                .scenario_config = _scenario_config,
                .orbital_physics = _orbital_physics,
                .time_warp = _time_warp,
        });
    }

    PredictionSubjectKey GameplayPredictionAdapter::player_prediction_subject_key() const
    {
        return make_subject_state_provider().player_subject_key();
    }

    std::vector<PredictionSubjectDescriptor> GameplayPredictionAdapter::build_prediction_subject_descriptors() const
    {
        return make_subject_state_provider().build_subject_descriptors();
    }

    PredictionHostContext GameplayPredictionAdapter::build_prediction_host_context(const GameStateContext *ctx) const
    {
        PredictionSubjectStateProvider subjects = make_subject_state_provider();
        GameplayPredictionAdapter prediction = *this;
        GameplayState *state = &_state;

        PredictionHostContext host{};
        host.orbital_scenario = _orbit.scenario_owner().get();
        host.subjects = subjects.build_subject_descriptors();
        host.player_subject = subjects.player_subject_key();
        host.current_sim_time_s = current_sim_time_s();
        host.last_sim_step_dt_s = _orbital_physics.last_sim_step_dt_s();
        host.fixed_delta_time_s = ctx ? ctx->fixed_delta_time() : 0.0f;
        host.interpolation_alpha = ctx ? std::clamp(ctx->interpolation_alpha(), 0.0f, 1.0f) : 0.0f;
        host.frame_delta_time_s = ctx ? ctx->delta_time() : 0.0f;
        host.debug_draw_enabled = _debug_draw_enabled;

        host.maneuver.plan = &_maneuver.plan();
        host.maneuver.edit_preview = &_maneuver.edit_preview();
        host.maneuver.gizmo_interaction = &_maneuver.gizmo_interaction();
        host.maneuver.plan_horizon = _maneuver.settings().plan_horizon;
        host.maneuver.plan_windows = _maneuver.settings().plan_windows;
        host.maneuver.nodes_enabled = _maneuver.settings().nodes_enabled;
        host.maneuver.live_preview_active = _maneuver.live_preview_active(true);
        host.maneuver.edit_in_progress =
                _maneuver.gizmo_interaction().state == ManeuverGizmoInteraction::State::DragAxis ||
                _maneuver.edit_preview().state != ManeuverNodeEditPreview::State::Idle;
        host.maneuver.revision = _maneuver.revision();
        host.maneuver.signature = current_maneuver_plan_signature();
        host.maneuver.active_preview_anchor_node_id = _maneuver.active_preview_anchor_node_id();

        host.reference_body_world = [subjects]() {
            return subjects.reference_body_world();
        };
        host.get_subject_world_state = [subjects](const PredictionSubjectKey key,
                                                  WorldVec3 &out_pos_world,
                                                  glm::dvec3 &out_vel_world,
                                                  glm::vec3 &out_vel_local) {
            return subjects.get_subject_world_state(key, out_pos_world, out_vel_world, out_vel_local);
        };
        host.render_subject_position_world = [subjects](const PredictionSubjectKey key, const float alpha_f) {
            return subjects.render_subject_position_world(key, alpha_f);
        };
        host.future_window_s = [prediction](const PredictionSubjectKey key) {
            return prediction.prediction_future_window_s(key);
        };
        host.required_window_s = [prediction](const PredictionTrackState &track,
                                              const double now_s,
                                              const bool with_maneuvers) {
            return prediction.prediction_required_window_s(track, now_s, with_maneuvers);
        };
        host.preview_exact_window_s = [prediction](const PredictionTrackState &track,
                                                   const double now_s,
                                                   const bool with_maneuvers) {
            return prediction.prediction_preview_exact_window_s(track, now_s, with_maneuvers);
        };
        host.refresh_preview_anchor = [prediction](PredictionTrackState &track,
                                                   const double now_s,
                                                   const bool with_maneuvers) {
            prediction.refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
        };
        host.subject_is_player = [subjects](const PredictionSubjectKey key) {
            return subjects.subject_is_player(key);
        };
        host.subject_thrust_applied_this_tick = [subjects](const PredictionSubjectKey key) {
            return subjects.subject_thrust_applied_this_tick(key);
        };
        host.resolve_maneuver_node_primary_body_id = [state](const ManeuverNode &node, const double query_time_s) {
            return ManeuverPredictionBridge::resolve_node_primary_body_id(*state, node, query_time_s);
        };
        host.resolve_display_frame_spec = [prediction](const OrbitPredictionCache &cache, const double display_time_s) {
            return prediction.resolve_prediction_display_frame_spec(cache, display_time_s);
        };
        host.resolve_analysis_body_id = [prediction](const OrbitPredictionCache &cache,
                                                     const PredictionSubjectKey key,
                                                     const double query_time_s,
                                                     const orbitsim::BodyId preferred_body_id) {
            return prediction.resolve_prediction_analysis_body_id(cache, key, query_time_s, preferred_body_id);
        };
        return host;
    }

    bool GameplayPredictionAdapter::get_player_world_state(WorldVec3 &out_pos_world,
                                                           glm::dvec3 &out_vel_world,
                                                           glm::vec3 &out_vel_local) const
    {
        return make_subject_state_provider().get_player_world_state(out_pos_world, out_vel_world, out_vel_local);
    }

    bool GameplayPredictionAdapter::get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                            WorldVec3 &out_pos_world,
                                                            glm::dvec3 &out_vel_world,
                                                            glm::vec3 &out_vel_local) const
    {
        return make_subject_state_provider().get_orbiter_world_state(
                orbiter,
                out_pos_world,
                out_vel_world,
                out_vel_local);
    }

    bool GameplayPredictionAdapter::get_prediction_subject_world_state(PredictionSubjectKey key,
                                                                       WorldVec3 &out_pos_world,
                                                                       glm::dvec3 &out_vel_world,
                                                                       glm::vec3 &out_vel_local) const
    {
        return make_subject_state_provider().get_subject_world_state(
                key,
                out_pos_world,
                out_vel_world,
                out_vel_local);
    }

    void GameplayPredictionAdapter::rebuild_prediction_subjects()
    {
        _prediction->sync_subjects(build_prediction_host_context());
    }

    std::vector<PredictionSubjectKey> GameplayPredictionAdapter::collect_visible_prediction_subjects() const
    {
        return _prediction->collect_visible_subjects();
    }

    double GameplayPredictionAdapter::prediction_future_window_s(const PredictionSubjectKey key) const
    {
        // Celestials and spacecraft use different minimum look-ahead windows.
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return std::max(0.0, _prediction->state().sampling_policy.celestial_min_window_s);
        }

        return std::max(0.0, _prediction->state().sampling_policy.orbiter_min_window_s);
    }

    double GameplayPredictionAdapter::maneuver_plan_horizon_s() const
    {
        return std::max(OrbitPredictionTuning::kPostNodeCoverageMinS,
                        std::max(0.0, _maneuver.settings().plan_horizon.horizon_s));
    }

    uint64_t GameplayPredictionAdapter::current_maneuver_plan_signature() const
    {
        const double plan_horizon_s = maneuver_plan_horizon_s();
        return _maneuver.plan_signature(plan_horizon_s);
    }

    PredictionTrackState *GameplayPredictionAdapter::find_prediction_track(PredictionSubjectKey key)
    {
        return _prediction->find_track(key);
    }

    const PredictionTrackState *GameplayPredictionAdapter::find_prediction_track(PredictionSubjectKey key) const
    {
        return _prediction->find_track(key);
    }

    PredictionTrackState *GameplayPredictionAdapter::active_prediction_track()
    {
        return _prediction->active_track();
    }

    const PredictionTrackState *GameplayPredictionAdapter::active_prediction_track() const
    {
        return _prediction->active_track();
    }

    PredictionTrackState *GameplayPredictionAdapter::player_prediction_track()
    {
        return _prediction->player_track(player_prediction_subject_key());
    }

    const PredictionTrackState *GameplayPredictionAdapter::player_prediction_track() const
    {
        return _prediction->player_track(player_prediction_subject_key());
    }

    OrbitPredictionCache *GameplayPredictionAdapter::effective_prediction_cache(PredictionTrackState *track)
    {
        return _prediction->effective_cache(track);
    }

    const OrbitPredictionCache *GameplayPredictionAdapter::effective_prediction_cache(const PredictionTrackState *track) const
    {
        return _prediction->effective_cache(track);
    }

    OrbitPredictionCache *GameplayPredictionAdapter::player_prediction_cache()
    {
        return _prediction->player_cache(player_prediction_subject_key());
    }

    const OrbitPredictionCache *GameplayPredictionAdapter::player_prediction_cache() const
    {
        return _prediction->player_cache(player_prediction_subject_key());
    }

    bool GameplayPredictionAdapter::prediction_subject_is_player(PredictionSubjectKey key) const
    {
        return make_subject_state_provider().subject_is_player(key);
    }

    bool GameplayPredictionAdapter::prediction_subject_supports_maneuvers(PredictionSubjectKey key) const
    {
        return make_subject_state_provider().subject_supports_maneuvers(key);
    }

    std::string GameplayPredictionAdapter::prediction_subject_label(PredictionSubjectKey key) const
    {
        // Build a UI label that reflects subject type and player ownership.
        const PredictionTrackState *track = find_prediction_track(key);
        if (!track)
        {
            return {};
        }

        if (key.kind == PredictionSubjectKind::Orbiter && prediction_subject_is_player(key))
        {
            return track->label + " (player)";
        }
        if (key.kind == PredictionSubjectKind::Celestial)
        {
            return track->label + " (celestial)";
        }
        return track->label;
    }

    glm::vec3 GameplayPredictionAdapter::prediction_subject_orbit_rgb(const PredictionSubjectKey key) const
    {
        return make_subject_state_provider().subject_orbit_rgb(key);
    }

    bool GameplayPredictionAdapter::prediction_subject_thrust_applied_this_tick(PredictionSubjectKey key) const
    {
        return make_subject_state_provider().subject_thrust_applied_this_tick(key);
    }

} // namespace Game
