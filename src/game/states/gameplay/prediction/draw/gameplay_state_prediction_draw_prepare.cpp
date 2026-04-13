#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    bool GameplayState::build_orbit_prediction_track_draw_context(
            PredictionTrackState &track,
            const Draw::PredictionGlobalDrawContext &global_ctx,
            Draw::PredictionTrackDrawContext &out)
    {
        out = {};
        out.track = &track;

        refresh_prediction_derived_cache(track, global_ctx.display_time_s);
        if (!prediction_track_has_current_derived_cache(track, global_ctx.display_time_s))
        {
            return false;
        }

        out.stable_cache = &track.cache;
        out.planned_cache = out.stable_cache;

        out.traj_base = &out.stable_cache->trajectory_frame;
        out.traj_planned = &out.planned_cache->trajectory_frame_planned;
        out.display_cache = out.planned_cache->resolved_frame_spec_valid ? out.planned_cache : out.stable_cache;

        if (!build_prediction_display_transform(
                    *out.display_cache,
                    out.ref_body_world,
                    out.frame_to_world,
                    global_ctx.display_time_s))
        {
            out.ref_body_world = prediction_frame_origin_world(*out.display_cache, global_ctx.display_time_s);
            out.frame_to_world = glm::dmat3(1.0);
        }

        out.t0_s = out.traj_base->front().t_s;
        out.t1_s = out.traj_base->back().t_s;
        if (!(out.t1_s > out.t0_s))
        {
            return false;
        }

        out.now_s = Draw::compute_prediction_now_s(global_ctx.display_time_s, out.t0_s, out.t1_s);
        if (!std::isfinite(out.now_s))
        {
            return false;
        }

        if (!get_prediction_subject_world_state(
                    track.key,
                    out.subject_pos_world_state,
                    out.subject_vel_world,
                    out.subject_vel_local))
        {
            return false;
        }

        out.subject_pos_world = out.subject_pos_world_state;
        if (track.key.kind == PredictionSubjectKind::Orbiter)
        {
            if (const Entity *entity = _world.entities().find(EntityId{track.key.value}))
            {
                out.subject_pos_world = entity->get_render_physics_center_of_mass_world(global_ctx.alpha_f);
            }
        }

        out.traj_base_segments = &out.stable_cache->trajectory_segments_frame;
        if (out.traj_base_segments->empty())
        {
            return false;
        }

        out.traj_planned_segments = &out.planned_cache->trajectory_segments_frame_planned;
        out.planned_window_segments =
                !out.stable_cache->trajectory_segments_frame_planned.empty()
                        ? &out.stable_cache->trajectory_segments_frame_planned
                        : (!out.planned_cache->trajectory_segments_frame_planned.empty()
                                   ? &out.planned_cache->trajectory_segments_frame_planned
                                   : out.traj_planned_segments);

        out.is_active = track.key == _prediction_selection.active_subject;
        out.active_player_track = out.is_active && prediction_subject_is_player(track.key);
        out.maneuver_drag_active =
                out.active_player_track &&
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis;

        if (out.is_active)
        {
            _orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(out.traj_base_segments->size());
            _orbit_plot_perf.solver_segments_planned = static_cast<uint32_t>(out.traj_planned_segments->size());
        }

        out.i_hi = Draw::lower_bound_sample_index(*out.traj_base, out.now_s);
        if (out.i_hi >= out.traj_base->size())
        {
            return false;
        }

        out.align_delta = Draw::compute_align_delta(*out.traj_base,
                                                    out.i_hi,
                                                    out.subject_pos_world,
                                                    out.now_s,
                                                    out.ref_body_world,
                                                    out.frame_to_world);
        out.direct_world_polyline = Draw::frame_spec_uses_direct_world_polyline(
                out.display_cache->resolved_frame_spec_valid ? out.display_cache->resolved_frame_spec
                                                             : _prediction_frame_selection.spec);

        out.draw_ctx.orbit_plot = global_ctx.orbit_plot;
        out.draw_ctx.ref_body_world = out.ref_body_world;
        out.draw_ctx.frame_to_world = out.frame_to_world;
        out.draw_ctx.align_delta = out.align_delta;
        out.draw_ctx.lod_camera = global_ctx.lod_camera;
        out.draw_ctx.render_frustum = global_ctx.render_frustum;
        out.draw_ctx.camera_world = global_ctx.camera_world;
        out.draw_ctx.tan_half_fov = global_ctx.tan_half_fov;
        out.draw_ctx.viewport_height_px = std::max(1.0, static_cast<double>(global_ctx.viewport_height_px));
        out.draw_ctx.render_error_px = global_ctx.render_error_px;
        out.draw_ctx.render_max_segments =
                static_cast<std::size_t>(std::max(1, _orbit_plot_budget.render_max_segments_cpu));
        out.draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost, 0.0f, 1.0f);

        out.identity_frame_transform = Draw::frame_transform_is_identity(out.frame_to_world);
        out.use_base_adaptive_curve = !out.stable_cache->render_curve_frame.empty();

        out.world_basis_draw_ctx = out.draw_ctx;
        if (!out.identity_frame_transform)
        {
            out.world_basis_draw_ctx.frame_to_world = glm::dmat3(1.0);
        }

        const glm::vec3 track_rgb = prediction_subject_orbit_rgb(track.key);
        out.track_color_full = Draw::scale_line_color(glm::vec4(track_rgb, 0.22f), global_ctx.line_alpha_scale);
        out.track_color_future = Draw::scale_line_color(glm::vec4(track_rgb, 0.80f), global_ctx.line_alpha_scale);
        out.track_color_plan = global_ctx.color_orbit_plan;
        if (out.maneuver_drag_active)
        {
            out.track_color_plan.a = out.track_color_future.a;
        }
        if (!out.is_active)
        {
            out.draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost * 0.35f, 0.0f, 1.0f);
        }
        out.world_basis_draw_ctx.line_overlay_boost = out.draw_ctx.line_overlay_boost;

        out.future_window_s = prediction_future_window_s(track.key);
        if (out.planned_window_segments && !out.planned_window_segments->empty() &&
            out.planned_cache && out.planned_cache->has_planned_frame_draw_data())
        {
            const double planned_segments_t0_s = out.planned_window_segments->front().t0_s;
            const double planned_segments_t1_s =
                    out.planned_window_segments->back().t0_s + out.planned_window_segments->back().dt_s;
            const PredictionTimeContext time_ctx =
                    build_prediction_time_context(track.key, out.now_s, planned_segments_t0_s, planned_segments_t1_s);
            out.planned_window_policy = resolve_prediction_window_policy(&track, time_ctx, true);
            out.planned_visual_window_s = out.planned_window_policy.visual_window_s;
            out.planned_exact_window_s = out.planned_window_policy.exact_window_s;
            out.planned_pick_window_s = out.planned_window_policy.pick_window_s;
        }
        return true;
    }
} // namespace Game
