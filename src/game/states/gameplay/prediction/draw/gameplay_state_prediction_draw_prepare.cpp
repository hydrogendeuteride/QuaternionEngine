#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include <algorithm>
#include <cmath>

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
        if (!track.cache.valid || track.cache.trajectory_frame.size() < 2)
        {
            return false;
        }

        out.stable_cache = &track.cache;
        out.preview_planned_cache = track.preview_overlay.cache.valid ? &track.preview_overlay.cache : nullptr;
        out.planned_cache = out.preview_planned_cache ? out.preview_planned_cache : out.stable_cache;
        out.planned_chunk_assembly =
                track.preview_overlay.chunk_assembly.valid ? &track.preview_overlay.chunk_assembly : nullptr;
        out.has_preview_planned_overlay = track.preview_overlay.valid();
        out.has_chunk_planned_overlay =
                out.planned_chunk_assembly && out.planned_chunk_assembly->valid &&
                !out.planned_chunk_assembly->chunks.empty();

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
                out.subject_pos_world = entity->get_render_position_world(global_ctx.alpha_f);
            }
        }

        out.traj_base_segments = &out.stable_cache->trajectory_segments_frame;
        if (out.traj_base_segments->empty())
        {
            return false;
        }

        out.traj_planned_segments = &out.planned_cache->trajectory_segments_frame_planned;
        out.planned_window_segments = !out.stable_cache->trajectory_segments_frame_planned.empty()
                                              ? &out.stable_cache->trajectory_segments_frame_planned
                                              : (!out.planned_cache->trajectory_segments_frame_planned.empty()
                                                         ? &out.planned_cache->trajectory_segments_frame_planned
                                                         : out.traj_planned_segments);

        out.is_active = track.key == _prediction_selection.active_subject;
        out.active_player_track = out.is_active && prediction_subject_is_player(track.key);
        out.maneuver_drag_active =
                out.active_player_track &&
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis;
        out.suppress_stale_planned_preview =
                out.maneuver_drag_active && track.preview_state == PredictionPreviewRuntimeState::EnterDrag;
        out.drag_anchor_valid = out.maneuver_drag_active && track.preview_anchor.valid &&
                                std::isfinite(track.preview_anchor.anchor_time_s);
        out.drag_anchor_time_s =
                out.drag_anchor_valid ? track.preview_anchor.anchor_time_s : std::numeric_limits<double>::quiet_NaN();

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
        out.use_persistent_gpu_roots =
                global_ctx.orbit_plot && global_ctx.orbit_plot->settings().gpu_generate_enabled &&
                !out.direct_world_polyline;
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
        if (!out.is_active)
        {
            out.draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost * 0.35f, 0.0f, 1.0f);
        }
        out.world_basis_draw_ctx.line_overlay_boost = out.draw_ctx.line_overlay_boost;

        out.future_window_s = prediction_future_window_s(track.key);
        if (out.active_player_track && track.preview_anchor.valid)
        {
            out.planned_visual_window_s = std::max(0.0, track.preview_anchor.visual_window_s);
            out.planned_pick_window_s = std::max(0.0, track.preview_anchor.pick_window_s);
        }
        else
        {
            const double default_planned_window_s = maneuver_plan_preview_window_s();
            out.planned_visual_window_s = default_planned_window_s;
            out.planned_pick_window_s = default_planned_window_s;
        }
        return true;
    }
} // namespace Game
