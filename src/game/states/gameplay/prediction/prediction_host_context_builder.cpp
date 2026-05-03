#include "game/states/gameplay/prediction/prediction_host_context_builder.h"

#include "game/state/game_state.h"
#include "game/states/gameplay/maneuver/maneuver_system.h"
#include "game/states/gameplay/orbital_physics_system.h"
#include "game/states/gameplay/orbital_runtime_system.h"
#include "game/states/gameplay/prediction/prediction_frame_context_builder.h"
#include "game/states/gameplay/prediction/runtime/prediction_window_context_builder.h"

#include <algorithm>
#include <utility>

namespace Game
{
    PredictionHostContextBuilder::PredictionHostContextBuilder(GameplayPredictionContext context)
        : _context(std::move(context))
    {
    }

    PredictionSubjectStateProvider PredictionHostContextBuilder::make_subject_state_provider() const
    {
        return PredictionSubjectStateProvider(PredictionSubjectStateProvider::Context{
                .orbit = _context.orbit,
                .world = _context.world,
                .physics = _context.physics,
                .physics_context = _context.physics_context,
                .scenario_config = _context.scenario_config,
                .orbital_physics = _context.orbital_physics,
                .time_warp = _context.time_warp,
        });
    }

    PredictionHostContext PredictionHostContextBuilder::build(const GameStateContext *ctx) const
    {
        PredictionSubjectStateProvider subjects = make_subject_state_provider();
        GameplayPredictionContext context = _context;
        const ManeuverSystem &maneuver = _context.maneuver;

        PredictionHostContext host{};
        host.orbital_scenario = _context.orbit.scenario_owner().get();
        host.subjects = subjects.build_subject_descriptors();
        host.player_subject = subjects.player_subject_key();
        host.current_sim_time_s = _context.current_sim_time_s;
        host.last_sim_step_dt_s = _context.orbital_physics.last_sim_step_dt_s();
        host.fixed_delta_time_s = ctx ? ctx->fixed_delta_time() : 0.0f;
        host.interpolation_alpha = ctx ? std::clamp(ctx->interpolation_alpha(), 0.0f, 1.0f) : 0.0f;
        host.frame_delta_time_s = ctx ? ctx->delta_time() : 0.0f;
        host.debug_draw_enabled = _context.debug_draw_enabled;

        host.maneuver.plan = &maneuver.plan();
        host.maneuver.edit_preview = &maneuver.edit_preview();
        host.maneuver.gizmo_interaction = &maneuver.gizmo_interaction();
        host.maneuver.plan_horizon = maneuver.settings().plan_horizon;
        host.maneuver.plan_windows = maneuver.settings().plan_windows;
        host.maneuver.nodes_enabled = maneuver.settings().nodes_enabled;
        host.maneuver.live_preview_active = maneuver.live_preview_active(true);
        host.maneuver.edit_in_progress =
                maneuver.gizmo_interaction().state == ManeuverGizmoInteraction::State::DragAxis ||
                maneuver.edit_preview().state != ManeuverNodeEditPreview::State::Idle;
        host.maneuver.revision = maneuver.revision();
        host.maneuver.signature = PredictionWindowContextBuilder(_context).current_maneuver_plan_signature();
        host.maneuver.active_preview_anchor_node_id = maneuver.active_preview_anchor_node_id();

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
        host.future_window_s = [context](const PredictionSubjectKey key) {
            return PredictionWindowContextBuilder(context).future_window_s(key);
        };
        host.required_window_s = [context](const PredictionTrackState &track,
                                           const double now_s,
                                           const bool with_maneuvers) {
            return PredictionWindowContextBuilder(context).required_window_s(track, now_s, with_maneuvers);
        };
        host.preview_exact_window_s = [context](const PredictionTrackState &track,
                                                const double now_s,
                                                const bool with_maneuvers) {
            return PredictionWindowContextBuilder(context).preview_exact_window_s(track, now_s, with_maneuvers);
        };
        host.refresh_preview_anchor = [context](PredictionTrackState &track,
                                                const double now_s,
                                                const bool with_maneuvers) {
            PredictionWindowContextBuilder(context).refresh_preview_anchor(track, now_s, with_maneuvers);
        };
        host.subject_is_player = [subjects](const PredictionSubjectKey key) {
            return subjects.subject_is_player(key);
        };
        host.subject_thrust_applied_this_tick = [subjects](const PredictionSubjectKey key) {
            return subjects.subject_thrust_applied_this_tick(key);
        };
        host.resolve_maneuver_node_primary_body_id = [context](const ManeuverNode &node, const double query_time_s) {
            return context.resolve_maneuver_node_primary_body_id
                           ? context.resolve_maneuver_node_primary_body_id(node, query_time_s)
                           : orbitsim::kInvalidBodyId;
        };
        host.resolve_display_frame_spec = [context](const OrbitPredictionCache &cache, const double display_time_s) {
            return PredictionFrameContextBuilder(context).resolve_prediction_display_frame_spec(cache, display_time_s);
        };
        host.resolve_analysis_body_id = [context](const OrbitPredictionCache &cache,
                                                  const PredictionSubjectKey key,
                                                  const double query_time_s,
                                                  const orbitsim::BodyId preferred_body_id) {
            return PredictionFrameContextBuilder(context).resolve_prediction_analysis_body_id(
                    cache,
                    key,
                    query_time_s,
                    preferred_body_id);
        };
        return host;
    }
} // namespace Game
