#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    bool GameplayState::build_orbit_prediction_global_draw_context(
            GameStateContext &ctx,
            Draw::PredictionGlobalDrawContext &out)
    {
        out.picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        out.orbit_plot = (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        Draw::reset_orbit_plot_state(out.picking, out.orbit_plot, _prediction->state().orbit_plot_perf, _prediction->state().enabled);

        if (!_prediction->state().enabled || !ctx.api || !_orbitsim)
        {
            return false;
        }

        if (!active_prediction_track())
        {
            return false;
        }

        out.alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        out.display_time_s = Draw::compute_prediction_display_time_s(_orbitsim->sim.time_s(),
                                                                     _last_sim_step_dt_s,
                                                                     ctx.fixed_delta_time(),
                                                                     out.alpha_f);
        if (!std::isfinite(out.display_time_s))
        {
            return false;
        }

        // DebugDrawSystem prunes commands during begin_frame, so keep velocity rays
        // alive slightly longer than the current dt.
        out.ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;
        out.line_alpha_scale = std::clamp(_prediction->state().line_alpha_scale, 0.1f, 8.0f);
        out.color_orbit_plan =
                Draw::scale_line_color(_prediction->state().draw_config.palette.orbit_planned, out.line_alpha_scale);

        out.camera_world = ctx.api->get_camera_position_d();
        float camera_fov_deg = 70.0f;
        out.viewport_height_px = 720.0f;
        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            out.camera_world = cam.position_world;
            camera_fov_deg = cam.fovDegrees;
            if (ctx.renderer->_logicalRenderExtent.height > 0)
            {
                out.viewport_height_px = static_cast<float>(ctx.renderer->_logicalRenderExtent.height);
            }
        }

        out.tan_half_fov = std::tan(glm::radians(static_cast<double>(camera_fov_deg)) * 0.5);
        out.lod_camera.camera_world = out.camera_world;
        out.lod_camera.tan_half_fov = out.tan_half_fov;
        out.lod_camera.viewport_height_px = std::max(1.0, static_cast<double>(out.viewport_height_px));

        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            out.render_frustum.valid = true;
            out.render_frustum.viewproj = ctx.renderer->_sceneManager->getSceneData().viewproj;
            out.render_frustum.origin_world = WorldVec3(out.camera_world);
        }

        out.render_error_px =
                (std::isfinite(_prediction->budget().render_error_px) && _prediction->budget().render_error_px > 0.0)
                        ? _prediction->budget().render_error_px
                        : 0.75;
        if (out.orbit_plot)
        {
            out.orbit_plot->settings().render_error_px = out.render_error_px;
        }

        return true;
    }

    // Build per-frame orbit visuals, pick data, and debug overlays from the latest prediction cache.
    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        Draw::PredictionGlobalDrawContext global_ctx{};
        if (!build_orbit_prediction_global_draw_context(ctx, global_ctx))
        {
            return;
        }

        std::vector<PredictionTrackState *> visible_tracks;
        visible_tracks.reserve(1 + _prediction->state().selection.overlay_subjects.size());
        for (PredictionSubjectKey key : collect_visible_prediction_subjects())
        {
            if (PredictionTrackState *track = find_prediction_track(key))
            {
                visible_tracks.push_back(track);
            }
        }
        if (visible_tracks.empty())
        {
            return;
        }

        for (PredictionTrackState *track : visible_tracks)
        {
            if (!track)
            {
                continue;
            }

            Draw::PredictionTrackDrawContext track_ctx{};
            if (!build_orbit_prediction_track_draw_context(*track, global_ctx, track_ctx))
            {
                continue;
            }

            draw_orbit_prediction_track_windows(track_ctx);

            if (global_ctx.picking && track_ctx.active_player_track)
            {
                emit_orbit_prediction_track_picks(global_ctx, track_ctx);
            }

            if (track_ctx.is_active &&
                _prediction->state().draw_velocity_ray &&
                _debug_draw_enabled &&
                track_ctx.track->key.kind == PredictionSubjectKind::Orbiter)
            {
                Draw::emit_velocity_ray(ctx.api,
                                        track_ctx.subject_pos_world,
                                        track_ctx.subject_vel_world,
                                        global_ctx.ttl_s,
                                        _prediction->state().draw_config.palette.velocity_ray);
            }
        }
    }
} // namespace Game
