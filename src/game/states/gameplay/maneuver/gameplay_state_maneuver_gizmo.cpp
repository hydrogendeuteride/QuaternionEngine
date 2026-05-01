#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_gizmo_helpers.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"
#include "game/states/gameplay/maneuver/maneuver_commands.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include "core/device/images.h"
#include "core/engine.h"
#include "core/input/input_system.h"

#include "imgui.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace Game
{
    namespace Gizmo = ManeuverGizmoHelpers;

    namespace
    {
        using namespace ManeuverUtil;

        struct CameraRay
        {
            WorldVec3 camera_world{0.0, 0.0, 0.0};
            glm::dvec3 ray_origin_local{0.0, 0.0, 0.0};
            glm::dvec3 ray_dir_local{0.0, 0.0, -1.0};
        };

        bool compute_camera_ray(const GameStateContext &ctx, const glm::vec2 &window_pos, CameraRay &out_ray)
        {
            if (!ctx.renderer || !ctx.renderer->_sceneManager || !ctx.renderer->_swapchainManager)
            {
                return false;
            }

            int win_w = 0;
            int win_h = 0;
            int draw_w = 0;
            int draw_h = 0;

            if (!ctx.renderer->_window)
            {
                return false;
            }

            SDL_GetWindowSize(ctx.renderer->_window, &win_w, &win_h);
            SDL_Vulkan_GetDrawableSize(ctx.renderer->_window, &draw_w, &draw_h);

            glm::vec2 scale{1.0f, 1.0f};
            if (win_w > 0 && win_h > 0 && draw_w > 0 && draw_h > 0)
            {
                scale.x = static_cast<float>(draw_w) / static_cast<float>(win_w);
                scale.y = static_cast<float>(draw_h) / static_cast<float>(win_h);
            }

            const glm::vec2 drawable_pos{window_pos.x * scale.x, window_pos.y * scale.y};

            VkExtent2D drawable_extent{0, 0};
            if (draw_w > 0 && draw_h > 0)
            {
                drawable_extent.width = static_cast<uint32_t>(draw_w);
                drawable_extent.height = static_cast<uint32_t>(draw_h);
            }
            if ((drawable_extent.width == 0 || drawable_extent.height == 0) && ctx.renderer->_swapchainManager)
            {
                drawable_extent = ctx.renderer->_swapchainManager->windowExtent();
            }

            const VkExtent2D swap_extent = ctx.renderer->_swapchainManager->swapchainExtent();
            if (drawable_extent.width == 0 || drawable_extent.height == 0 ||
                swap_extent.width == 0 || swap_extent.height == 0)
            {
                return false;
            }

            const float sx = static_cast<float>(swap_extent.width) / static_cast<float>(drawable_extent.width);
            const float sy = static_cast<float>(swap_extent.height) / static_cast<float>(drawable_extent.height);
            const glm::vec2 swapchain_pos{drawable_pos.x * sx, drawable_pos.y * sy};

            VkExtent2D logical_extent = ctx.renderer->_logicalRenderExtent;
            if (logical_extent.width == 0 || logical_extent.height == 0)
            {
                logical_extent = VkExtent2D{1280, 720};
            }

            glm::vec2 logical_pos{};
            // Undo the renderer's letterboxing so the ray matches the scene camera rather than the swapchain surface.
            if (!vkutil::map_window_to_letterbox_src(swapchain_pos, logical_extent, swap_extent, logical_pos))
            {
                return false;
            }

            const double width = static_cast<double>(logical_extent.width);
            const double height = static_cast<double>(logical_extent.height);
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

        bool closest_param_ray_line(const glm::dvec3 &ray_origin_local,
                                    const glm::dvec3 &ray_dir_local,
                                    const glm::dvec3 &line_origin_local,
                                    const glm::dvec3 &line_dir_local,
                                    double &out_line_t)
        {
            out_line_t = 0.0;

            // Solve the closest-points problem between a ray and an infinite line by minimizing squared distance.
            // This is the 2x2 least-squares / normal-equations form for two skew lines, then clamped to the ray.
            const double uu = glm::dot(ray_dir_local, ray_dir_local);
            const double vv = glm::dot(line_dir_local, line_dir_local);
            if (!(uu > 0.0) || !(vv > 0.0) || !std::isfinite(uu) || !std::isfinite(vv))
            {
                return false;
            }

            const glm::dvec3 w0 = line_origin_local - ray_origin_local;
            const double uv = glm::dot(ray_dir_local, line_dir_local);
            const double uw = glm::dot(ray_dir_local, w0);
            const double vw = glm::dot(line_dir_local, w0);
            const double denom = uu * vv - uv * uv;

            double s = 0.0; // ray distance
            double t = 0.0; // line parameter
            if (std::abs(denom) > 1.0e-9 && std::isfinite(denom))
            {
                s = (uw * vv - vw * uv) / denom;
                t = (uw * uv - vw * uu) / denom;
                if (!std::isfinite(s) || !std::isfinite(t))
                {
                    return false;
                }
                if (s < 0.0)
                {
                    s = 0.0;
                    t = -vw / vv;
                }
            }
            else
            {
                // Nearly parallel: choose the axis parameter nearest to the ray origin.
                t = -vw / vv;
                if (!std::isfinite(t))
                {
                    return false;
                }
            }

            out_line_t = t;
            return std::isfinite(out_line_t);
        }

    } // namespace

    void GameplayState::clear_maneuver_gizmo_instances(GameStateContext &ctx)
    {
        (void) ctx;
        _maneuver_gizmo_interaction = {};
    }

    bool GameplayState::build_maneuver_gizmo_view_context(const GameStateContext &ctx,
                                                          ManeuverGizmoViewContext &out_view) const
    {
        // Gather the camera and render-space transforms needed to project world-space node markers into ImGui space.
        out_view = {};

        if (!ctx.renderer || !ctx.renderer->_sceneManager || !ctx.renderer->_swapchainManager || !ctx.renderer->_window)
        {
            return false;
        }

        int win_w = 0;
        int win_h = 0;
        int draw_w = 0;
        int draw_h = 0;
        SDL_GetWindowSize(ctx.renderer->_window, &win_w, &win_h);
        SDL_Vulkan_GetDrawableSize(ctx.renderer->_window, &draw_w, &draw_h);
        if (win_w <= 0 || win_h <= 0 || draw_w <= 0 || draw_h <= 0)
        {
            return false;
        }

        const VkExtent2D swap_extent = ctx.renderer->_swapchainManager->swapchainExtent();
        if (swap_extent.width == 0 || swap_extent.height == 0)
        {
            return false;
        }

        VkExtent2D logical_extent = ctx.renderer->_logicalRenderExtent;
        if (logical_extent.width == 0 || logical_extent.height == 0)
        {
            logical_extent = VkExtent2D{1280, 720};
        }

        const VkRect2D letterbox_rect = vkutil::compute_letterbox_rect(logical_extent, swap_extent);
        if (letterbox_rect.extent.width == 0 || letterbox_rect.extent.height == 0)
        {
            return false;
        }

        const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
        out_view.camera_world = cam.position_world;
        out_view.world_to_cam = glm::transpose(glm::dmat3(cam.getRotationMatrix()));
        out_view.letterbox_rect = ManeuverViewportRect{
            .x = letterbox_rect.offset.x,
            .y = letterbox_rect.offset.y,
            .width = letterbox_rect.extent.width,
            .height = letterbox_rect.extent.height,
        };

        out_view.logical_w = static_cast<double>(logical_extent.width);
        out_view.logical_h = static_cast<double>(logical_extent.height);
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

        out_view.draw_from_swap_x = static_cast<double>(draw_w) / static_cast<double>(swap_extent.width);
        out_view.draw_from_swap_y = static_cast<double>(draw_h) / static_cast<double>(swap_extent.height);
        out_view.window_from_draw_x = static_cast<double>(win_w) / static_cast<double>(draw_w);
        out_view.window_from_draw_y = static_cast<double>(win_h) / static_cast<double>(draw_h);
        // ImGui mouse coordinates live in window pixels, while projection starts from logical render space.
        // Cache the conversion chain once so projection and picking use exactly the same path.
        if (!std::isfinite(out_view.draw_from_swap_x) || !std::isfinite(out_view.draw_from_swap_y) ||
            !std::isfinite(out_view.window_from_draw_x) || !std::isfinite(out_view.window_from_draw_y))
        {
            return false;
        }

        const double analysis_time_s = current_sim_time_s();
        const PredictionTrackState *player_track = player_prediction_track();
        const OrbitPredictionCache *player_cache = effective_prediction_cache(player_track);
        const orbitsim::BodyId occluder_body_id =
                (_prediction.analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI)
                    ? _prediction.analysis_selection.spec.fixed_body_id
                    : ((player_track && player_cache)
                               ? resolve_prediction_analysis_body_id(*player_cache, player_track->key, analysis_time_s)
                               : orbitsim::kInvalidBodyId);
        const CelestialBodyInfo *ref_info = find_celestial_body_info(occluder_body_id);
        out_view.depth_occluder_valid = ref_info && std::isfinite(ref_info->radius_m) && ref_info->radius_m > 0.0;
        out_view.depth_occluder_center = out_view.depth_occluder_valid
                                             ? prediction_body_world_position(occluder_body_id, player_cache, analysis_time_s)
                                             : WorldVec3(0.0, 0.0, 0.0);
        out_view.depth_occluder_radius = out_view.depth_occluder_valid ? ref_info->radius_m : 0.0;
        return true;
    }

    bool GameplayState::begin_maneuver_axis_drag(GameStateContext &ctx,
                                                 const int node_id,
                                                 const ManeuverHandleAxis axis)
    {
        // Capture the initial closest-point parameter so drag updates can be expressed as delta-along-axis.
        if (axis == ManeuverHandleAxis::None || axis == ManeuverHandleAxis::Hub)
        {
            return false;
        }

        ManeuverNode *node = _maneuver_state.find_node(node_id);
        if (!node || !node->gizmo_valid)
        {
            return false;
        }

        glm::dvec3 axis_dir_world(0.0, 1.0, 0.0);
        int component = 1;
        double sign = 1.0;
        if (!Gizmo::resolve_maneuver_axis(*node, axis, axis_dir_world, component, sign))
        {
            return false;
        }

        cancel_maneuver_node_dv_edit_preview();

        const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
        const glm::vec2 mouse_pos_window(mouse_pos.x, mouse_pos.y);
        CameraRay ray{};
        if (!compute_camera_ray(ctx, mouse_pos_window, ray))
        {
            return false;
        }

        double start_t = 0.0;
        const glm::dvec3 axis_origin_local = glm::dvec3(node->position_world - ray.camera_world);
        if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world, start_t))
        {
            return false;
        }

        _maneuver_gizmo_interaction.state = ManeuverGizmoInteraction::State::DragAxis;
        _maneuver_gizmo_interaction.node_id = node_id;
        _maneuver_gizmo_interaction.axis = axis;
        _maneuver_gizmo_interaction.start_dv_rtn_mps = node->dv_rtn_mps;
        _maneuver_gizmo_interaction.start_dv_display_mps = dv_rtn_to_display_basis(*node);
        _maneuver_gizmo_interaction.start_axis_t_m = start_t;
        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double drag_display_reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : 0.0;
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            drag_display_reference_time_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }
        _maneuver_gizmo_interaction.drag_display_reference_time_s = drag_display_reference_time_s;
        _maneuver_gizmo_interaction.drag_basis_r_world = node->basis_r_world;
        _maneuver_gizmo_interaction.drag_basis_t_world = node->basis_t_world;
        _maneuver_gizmo_interaction.drag_basis_n_world = node->basis_n_world;
        // Keep both the visible handle basis and the authored RTN basis fixed for the whole drag session.
        _maneuver_gizmo_interaction.drag_maneuver_basis_r_world = node->maneuver_basis_r_world;
        _maneuver_gizmo_interaction.drag_maneuver_basis_t_world = node->maneuver_basis_t_world;
        _maneuver_gizmo_interaction.drag_maneuver_basis_n_world = node->maneuver_basis_n_world;
        _maneuver_gizmo_interaction.drag_start_mouse_window_pos = mouse_pos_window;
        _maneuver_gizmo_interaction.drag_last_sample_mouse_window_pos = mouse_pos_window;
        _maneuver_gizmo_interaction.drag_threshold_passed = false;
        _maneuver_gizmo_interaction.drag_display_snapshots.clear();
        _maneuver_gizmo_interaction.drag_display_snapshots.reserve(_maneuver_state.nodes.size());
        for (const ManeuverNode &candidate : _maneuver_state.nodes)
        {
            if (!candidate.gizmo_valid)
            {
                continue;
            }

            _maneuver_gizmo_interaction.drag_display_snapshots.push_back(ManeuverNodeDisplaySnapshot{
                    .node_id = candidate.id,
                    .position_world = candidate.position_world,
                    .basis_r_world = candidate.basis_r_world,
                    .basis_t_world = candidate.basis_t_world,
                    .basis_n_world = candidate.basis_n_world,
            });
        }
        _maneuver_gizmo_interaction.applied_delta = false;
        if (PredictionTrackState *track = active_prediction_track())
        {
            refresh_prediction_preview_anchor(*track, current_sim_time_s(), true);

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

    void GameplayState::apply_maneuver_axis_drag(GameStateContext &ctx,
                                                 ManeuverNode &node,
                                                 const glm::vec2 &mouse_pos_window)
    {
        const auto drag_apply_start_tp = PredictionDragDebugTelemetry::Clock::now();

        const glm::vec2 drag_from_start =
                mouse_pos_window - _maneuver_gizmo_interaction.drag_start_mouse_window_pos;
        if (!_maneuver_gizmo_interaction.drag_threshold_passed)
        {
            const float drag_threshold_px =
                    std::max(ImGui::GetIO().MouseDragThreshold, _maneuver_ui_config.scaled(2.0f));
            if (glm::dot(drag_from_start, drag_from_start) < (drag_threshold_px * drag_threshold_px))
            {
                return;
            }
            _maneuver_gizmo_interaction.drag_threshold_passed = true;
        }

        const glm::vec2 drag_since_last_sample =
                mouse_pos_window - _maneuver_gizmo_interaction.drag_last_sample_mouse_window_pos;
        if (glm::dot(drag_since_last_sample, drag_since_last_sample) <= 1.0e-8f)
        {
            return;
        }
        _maneuver_gizmo_interaction.drag_last_sample_mouse_window_pos = mouse_pos_window;

        const AxisResolveResult axis_result = resolve_axis(
                _maneuver_gizmo_interaction.axis,
                _maneuver_gizmo_interaction.drag_basis_r_world,
                _maneuver_gizmo_interaction.drag_basis_t_world,
                _maneuver_gizmo_interaction.drag_basis_n_world);
        if (!axis_result.valid)
        {
            _maneuver_gizmo_interaction = {};
            return;
        }
        const glm::dvec3 &axis_dir_world = axis_result.axis_dir_world;
        const int component = axis_result.component;
        const double sign = axis_result.sign;

        CameraRay ray{};
        if (!compute_camera_ray(ctx, mouse_pos_window, ray))
        {
            return;
        }

        double current_t = 0.0;
        WorldVec3 axis_origin_world = node.position_world;
        if (const ManeuverNodeDisplaySnapshot *drag_snapshot =
                    find_display_snapshot(_maneuver_gizmo_interaction.drag_display_snapshots, node.id))
        {
            axis_origin_world = drag_snapshot->position_world;
        }
        const glm::dvec3 axis_origin_local = glm::dvec3(axis_origin_world - ray.camera_world);
        if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world, current_t))
        {
            return;
        }

        const double delta_axis_m = current_t - _maneuver_gizmo_interaction.start_axis_t_m;
        double scale = static_cast<double>(std::max(0.00001f, _maneuver_gizmo_style.drag_sensitivity_mps_per_m));
        const InputModifiers mods = ctx.input->modifiers();
        if (mods.ctrl && !mods.shift)
        {
            scale *= 10.0;
        }
        else if (mods.shift && !mods.ctrl)
        {
            scale *= 0.1;
        }

        // The drag solver works in world meters along the selected axis; convert that signed motion into DV.
        const double delta_mps = delta_axis_m * scale * sign;
        glm::dvec3 dv_display_new = _maneuver_gizmo_interaction.start_dv_display_mps;
        if (component == 0)
        {
            dv_display_new.x = _maneuver_gizmo_interaction.start_dv_display_mps.x + delta_mps;
        }
        else if (component == 1)
        {
            dv_display_new.y = _maneuver_gizmo_interaction.start_dv_display_mps.y + delta_mps;
        }
        else
        {
            dv_display_new.z = _maneuver_gizmo_interaction.start_dv_display_mps.z + delta_mps;
        }

        const glm::dvec3 dv_world_new =
                compose_basis_vector(dv_display_new,
                                     _maneuver_gizmo_interaction.drag_basis_r_world,
                                     _maneuver_gizmo_interaction.drag_basis_t_world,
                                     _maneuver_gizmo_interaction.drag_basis_n_world);
        const glm::dvec3 dv_new =
                project_basis_vector(dv_world_new,
                                     _maneuver_gizmo_interaction.drag_maneuver_basis_r_world,
                                     _maneuver_gizmo_interaction.drag_maneuver_basis_t_world,
                                     _maneuver_gizmo_interaction.drag_maneuver_basis_n_world);

        if (finite3(dv_new) && safe_length(dv_new - node.dv_rtn_mps) > 1.0e-7)
        {
            (void) apply_maneuver_command(ManeuverCommand::set_node_dv(node.id, dv_new, true));
            node.total_dv_mps = safe_length(node.dv_rtn_mps);
            _maneuver_gizmo_interaction.applied_delta = true;

            if (PredictionTrackState *track = active_prediction_track())
            {
                _prediction_system.mark_maneuver_preview_dirty(*track);
                sync_prediction_dirty_flag();
            }
        }

        if (PredictionTrackState *track = active_prediction_track())
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

    void GameplayState::build_maneuver_gizmo_markers(const ManeuverGizmoViewContext &view,
                                                     const float overlay_size_px,
                                                     std::vector<ManeuverHubMarker> &out_hubs,
                                                     std::vector<ManeuverAxisMarker> &out_handles) const
    {
        // Draw hubs for every visible node, but only expose draggable axis handles for the selected node.
        out_hubs.clear();
        out_handles.clear();

        out_hubs.reserve(_maneuver_state.nodes.size());
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!node.gizmo_valid)
            {
                continue;
            }
            if (Gizmo::maneuver_gizmo_is_occluded(view, node.position_world))
            {
                continue;
            }

            ManeuverHubMarker hub{};
            hub.node_id = node.id;
            if (!Gizmo::project_maneuver_gizmo_point(view, node.position_world, hub.screen, hub.depth_m))
            {
                continue;
            }

            out_hubs.push_back(hub);
        }

        out_handles.reserve(6);
        const ManeuverNode *selected = _maneuver_state.find_node(_maneuver_state.selected_node_id);
        if (!selected || !selected->gizmo_valid || Gizmo::maneuver_gizmo_is_occluded(view, selected->position_world))
        {
            return;
        }

        glm::vec2 hub_screen{};
        double hub_depth_m = 0.0;
        if (!Gizmo::project_maneuver_gizmo_point(view, selected->position_world, hub_screen, hub_depth_m))
        {
            return;
        }

        double dist_m = safe_length(glm::dvec3(selected->position_world) - view.camera_world);
        if (!std::isfinite(dist_m) || dist_m <= 1.0)
        {
            dist_m = std::max(1.0, hub_depth_m);
        }

        const double meters_per_px = (2.0 * view.tan_half_fov * dist_m) / view.logical_h;
        const float axis_len_px = std::max(24.0f, overlay_size_px * 1.9f);
        const double axis_offset_m = meters_per_px * static_cast<double>(axis_len_px);

        struct AxisDesc
        {
            ManeuverHandleAxis axis{ManeuverHandleAxis::None};
            glm::dvec3 dir_world{0.0, 0.0, 0.0};
        };

        const AxisDesc axis_descs[6]{
            {ManeuverHandleAxis::TangentialPos, selected->basis_t_world},
            {ManeuverHandleAxis::TangentialNeg, -selected->basis_t_world},
            {ManeuverHandleAxis::RadialPos, selected->basis_r_world},
            {ManeuverHandleAxis::RadialNeg, -selected->basis_r_world},
            {ManeuverHandleAxis::NormalPos, selected->basis_n_world},
            {ManeuverHandleAxis::NormalNeg, -selected->basis_n_world},
        };

        for (const AxisDesc &desc : axis_descs)
        {
            const glm::dvec3 dir = normalized_or(desc.dir_world, glm::dvec3(0.0, 1.0, 0.0));
            const bool is_active_axis =
                    (_maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis) &&
                    (_maneuver_gizmo_interaction.node_id == selected->id) &&
                    (_maneuver_gizmo_interaction.axis == desc.axis);
            const double axis_offset_active_m = axis_offset_m * (is_active_axis ? 1.14 : 1.0);
            const WorldVec3 handle_world = selected->position_world + dir * axis_offset_active_m;
            if (Gizmo::maneuver_gizmo_is_occluded(view, handle_world))
            {
                continue;
            }

            ManeuverAxisMarker marker{};
            marker.node_id = selected->id;
            marker.axis = desc.axis;
            marker.hub_screen = hub_screen;
            marker.base_color = Gizmo::maneuver_axis_color(desc.axis);
            marker.label = Gizmo::maneuver_axis_label(_maneuver_gizmo_basis_mode, desc.axis);
            if (!Gizmo::project_maneuver_gizmo_point(view, handle_world, marker.handle_screen, marker.depth_m))
            {
                continue;
            }

            out_handles.push_back(marker);
        }
    }
} // namespace Game
