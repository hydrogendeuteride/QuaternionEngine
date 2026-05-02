#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"
#include "game/states/gameplay/maneuver/maneuver_gizmo_controller.h"
#include "game/states/gameplay/maneuver/maneuver_commands.h"
#include "game/states/gameplay/maneuver/maneuver_ui_controller.h"
#include "game/states/gameplay/prediction/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include "core/engine.h"
#include "core/input/input_system.h"
#include "core/render_viewport.h"

#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace
    {
        bool compute_maneuver_camera_ray(const GameStateContext &ctx,
                                         const glm::vec2 &window_pos,
                                         ManeuverGizmoController::CameraRay &out_ray)
        {
            if (!ctx.renderer || !ctx.renderer->_sceneManager)
            {
                return false;
            }

            render::RenderViewportMetrics metrics{};
            glm::vec2 logical_pos{};
            if (!render::window_to_logical_render_pixels(*ctx.renderer, window_pos, logical_pos, &metrics))
            {
                return false;
            }

            const double width = static_cast<double>(metrics.logical_extent.width);
            const double height = static_cast<double>(metrics.logical_extent.height);
            if (!(width > 0.0) || !(height > 0.0))
            {
                return false;
            }

            const double ndc_x = (2.0 * static_cast<double>(logical_pos.x) / width) - 1.0;
            const double ndc_y = 1.0 - (2.0 * static_cast<double>(logical_pos.y) / height);

            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
            const double tan_half_fov = std::tan(fov_rad * 0.5);
            const double aspect = width / height;

            glm::dvec3 dir_camera(ndc_x * aspect * tan_half_fov, ndc_y * tan_half_fov, -1.0);
            const double dir_len2 = glm::dot(dir_camera, dir_camera);
            if (!(dir_len2 > 0.0) || !std::isfinite(dir_len2))
            {
                return false;
            }
            dir_camera /= std::sqrt(dir_len2);

            const glm::mat4 cam_rot = cam.getRotationMatrix();
            const glm::vec3 dir_camera_f(static_cast<float>(dir_camera.x),
                                         static_cast<float>(dir_camera.y),
                                         static_cast<float>(dir_camera.z));

            glm::vec3 dir_world_f = glm::vec3(cam_rot * glm::vec4(dir_camera_f, 0.0f));
            const float dir_world_len2 = glm::dot(dir_world_f, dir_world_f);
            if (!(dir_world_len2 > 0.0f) || !std::isfinite(dir_world_len2))
            {
                return false;
            }
            dir_world_f = glm::normalize(dir_world_f);

            out_ray.camera_world = WorldVec3(cam.position_world);
            out_ray.ray_origin_local = glm::dvec3(0.0, 0.0, 0.0);
            out_ray.ray_dir_local = glm::dvec3(dir_world_f);
            return true;
        }
    } // namespace

    bool ManeuverUiController::build_gizmo_view_context(const GameplayState &state,
                                                        const GameStateContext &ctx,
                                                        ManeuverGizmoViewContext &out_view)
    {
        // Gather the camera and render-space transforms needed to project world-space node markers into ImGui space.
        out_view = {};

        if (!ctx.renderer || !ctx.renderer->_sceneManager)
        {
            return false;
        }

        render::RenderViewportMetrics viewport{};
        if (!render::query_render_viewport_metrics(*ctx.renderer, viewport))
        {
            return false;
        }

        const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
        out_view.camera_world = cam.position_world;
        out_view.world_to_cam = glm::transpose(glm::dmat3(cam.getRotationMatrix()));
        out_view.letterbox_rect = ManeuverViewportRect{
            .x = viewport.letterbox_rect.offset.x,
            .y = viewport.letterbox_rect.offset.y,
            .width = viewport.letterbox_rect.extent.width,
            .height = viewport.letterbox_rect.extent.height,
        };

        out_view.logical_w = static_cast<double>(viewport.logical_extent.width);
        out_view.logical_h = static_cast<double>(viewport.logical_extent.height);
        if (!(out_view.logical_w > 0.0) || !(out_view.logical_h > 0.0))
        {
            return false;
        }

        const double fov_rad = glm::radians(static_cast<double>(cam.fovDegrees));
        out_view.tan_half_fov = std::tan(fov_rad * 0.5);
        out_view.aspect = out_view.logical_w / out_view.logical_h;
        if (!std::isfinite(out_view.tan_half_fov) || out_view.tan_half_fov <= 1.0e-8 ||
            !std::isfinite(out_view.aspect) || out_view.aspect <= 0.0)
        {
            return false;
        }

        out_view.draw_from_swap_x = viewport.draw_from_swap_x;
        out_view.draw_from_swap_y = viewport.draw_from_swap_y;
        out_view.window_from_draw_x = viewport.window_from_draw_x;
        out_view.window_from_draw_y = viewport.window_from_draw_y;
        // ImGui mouse coordinates live in window pixels, while projection starts from logical render space.
        // Cache the conversion chain once so projection and picking use exactly the same path.
        if (!std::isfinite(out_view.draw_from_swap_x) || !std::isfinite(out_view.draw_from_swap_y) ||
            !std::isfinite(out_view.window_from_draw_x) || !std::isfinite(out_view.window_from_draw_y))
        {
            return false;
        }

        const double analysis_time_s = state.current_sim_time_s();
        GameplayPredictionAdapter prediction(state);
        const PredictionTrackState *player_track = prediction.player_prediction_track();
        const OrbitPredictionCache *player_cache = prediction.effective_prediction_cache(player_track);
        const orbitsim::BodyId occluder_body_id =
                (state._prediction->state().analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI)
                    ? state._prediction->state().analysis_selection.spec.fixed_body_id
                    : ((player_track && player_cache)
                               ? prediction.resolve_prediction_analysis_body_id(*player_cache, player_track->key, analysis_time_s)
                               : orbitsim::kInvalidBodyId);
        const CelestialBodyInfo *ref_info = prediction.find_celestial_body_info(occluder_body_id);
        out_view.depth_occluder_valid = ref_info && std::isfinite(ref_info->radius_m) && ref_info->radius_m > 0.0;
        out_view.depth_occluder_center = out_view.depth_occluder_valid
                                             ? prediction.prediction_body_world_position(occluder_body_id, player_cache, analysis_time_s)
                                             : WorldVec3(0.0, 0.0, 0.0);
        out_view.depth_occluder_radius = out_view.depth_occluder_valid ? ref_info->radius_m : 0.0;
        return true;
    }

    bool ManeuverUiController::begin_axis_drag(GameplayState &state,
                                               GameStateContext &ctx,
                                               const int node_id,
                                               const ManeuverHandleAxis axis)
    {
        ManeuverNode *node = state._maneuver.plan().find_node(node_id);
        if (!node || !node->gizmo_valid || axis == ManeuverHandleAxis::None || axis == ManeuverHandleAxis::Hub)
        {
            return false;
        }
        if (!ManeuverUtil::resolve_axis(axis, node->basis_r_world, node->basis_t_world, node->basis_n_world).valid)
        {
            return false;
        }

        state._maneuver.cancel_edit_preview();

        const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
        const glm::vec2 mouse_pos_window(mouse_pos.x, mouse_pos.y);
        ManeuverGizmoController::CameraRay ray{};
        if (!compute_maneuver_camera_ray(ctx, mouse_pos_window, ray))
        {
            return false;
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (state._orbital_physics.last_sim_step_dt_s() > 0.0)
                    ? state._orbital_physics.last_sim_step_dt_s()
                    : static_cast<double>(ctx.fixed_delta_time());
        double drag_display_reference_time_s =
                state._orbit.scenario_owner() ? state._orbit.scenario_owner()->sim.time_s() : 0.0;
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            drag_display_reference_time_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }
        if (!ManeuverGizmoController::begin_axis_drag(state._maneuver.plan(),
                                                      state._maneuver.gizmo_interaction(),
                                                      node_id,
                                                      axis,
                                                      ray,
                                                      mouse_pos_window,
                                                      drag_display_reference_time_s))
        {
            return false;
        }
        GameplayPredictionAdapter prediction(state);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            prediction.refresh_prediction_preview_anchor(*track, state.current_sim_time_s(), true);

            PredictionDragDebugTelemetry &debug = track->drag_debug;
            const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
            const uint64_t next_drag_session_id = debug.drag_session_id + 1;
            debug.clear();
            debug.drag_session_id = next_drag_session_id;
            debug.drag_active = true;
            debug.drag_started_tp = now_tp;
            debug.last_drag_update_tp = now_tp;
        }
        return true;
    }

    void ManeuverUiController::apply_axis_drag(GameplayState &state,
                                               GameStateContext &ctx,
                                               ManeuverNode &node,
                                               const glm::vec2 &mouse_pos_window)
    {
        const auto drag_apply_start_tp = PredictionDragDebugTelemetry::Clock::now();

        ManeuverGizmoController::CameraRay ray{};
        if (!compute_maneuver_camera_ray(ctx, mouse_pos_window, ray))
        {
            return;
        }

        const InputModifiers mods = ctx.input ? ctx.input->modifiers() : InputModifiers{};
        const ManeuverGizmoController::DragUpdateResult update =
                ManeuverGizmoController::update_axis_drag(
                        state._maneuver.gizmo_interaction(),
                        node,
                        mouse_pos_window,
                        ray,
                        std::max(ImGui::GetIO().MouseDragThreshold,
                                 state._maneuver.settings().ui_config.scaled(2.0f)),
                        static_cast<double>(state._maneuver.settings().gizmo_style.drag_sensitivity_mps_per_m),
                        mods.ctrl,
                        mods.shift);
        if (update.clear_interaction)
        {
            state._maneuver.clear_gizmo_interaction();
            return;
        }
        if (!update.sampled)
        {
            return;
        }

        if (update.changed)
        {
            (void) state.apply_maneuver_command(ManeuverCommand::set_node_dv(node.id, update.dv_rtn_mps, true));
            node.total_dv_mps = ManeuverUtil::safe_length(node.dv_rtn_mps);
            state._maneuver.gizmo_interaction().applied_delta = true;

            GameplayPredictionAdapter prediction(state);
            if (PredictionTrackState *track = prediction.active_prediction_track())
            {
                state._prediction->mark_maneuver_preview_dirty(*track);
                prediction.sync_prediction_dirty_flag();
            }
        }

        GameplayPredictionAdapter prediction(state);
        if (PredictionTrackState *track = prediction.active_prediction_track())
        {
            PredictionDragDebugTelemetry &debug = track->drag_debug;
            const auto drag_apply_end_tp = PredictionDragDebugTelemetry::Clock::now();
            debug.drag_active = true;
            debug.last_drag_update_tp = drag_apply_end_tp;
            ++debug.drag_update_count;
            PredictionRuntimeDetail::update_last_and_peak(debug.drag_apply_ms_last,
                                 debug.drag_apply_ms_peak,
                                 std::chrono::duration<double, std::milli>(drag_apply_end_tp - drag_apply_start_tp).count());
        }
    }

} // namespace Game
