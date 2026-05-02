#include "game/states/gameplay/maneuver/maneuver_gizmo_controller.h"

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_gizmo_helpers.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace Gizmo = ManeuverGizmoHelpers;

    bool ManeuverGizmoController::closest_param_ray_line(const glm::dvec3 &ray_origin_local,
                                                         const glm::dvec3 &ray_dir_local,
                                                         const glm::dvec3 &line_origin_local,
                                                         const glm::dvec3 &line_dir_local,
                                                         double &out_line_t)
    {
        out_line_t = 0.0;

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

        double s = 0.0;
        double t = 0.0;
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
                t = -vw / vv;
            }
        }
        else
        {
            t = -vw / vv;
            if (!std::isfinite(t))
            {
                return false;
            }
        }

        out_line_t = t;
        return std::isfinite(out_line_t);
    }

    bool ManeuverGizmoController::begin_axis_drag(ManeuverPlanState &plan,
                                                  ManeuverGizmoInteraction &interaction,
                                                  const int node_id,
                                                  const ManeuverHandleAxis axis,
                                                  const CameraRay &ray,
                                                  const glm::vec2 &mouse_pos_window,
                                                  const double drag_display_reference_time_s)
    {
        if (axis == ManeuverHandleAxis::None || axis == ManeuverHandleAxis::Hub)
        {
            return false;
        }

        ManeuverNode *node = plan.find_node(node_id);
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

        double start_t = 0.0;
        const glm::dvec3 axis_origin_local = glm::dvec3(node->position_world - ray.camera_world);
        if (!closest_param_ray_line(ray.ray_origin_local, ray.ray_dir_local, axis_origin_local, axis_dir_world, start_t))
        {
            return false;
        }

        interaction.state = ManeuverGizmoInteraction::State::DragAxis;
        interaction.node_id = node_id;
        interaction.axis = axis;
        interaction.start_dv_rtn_mps = node->dv_rtn_mps;
        interaction.start_dv_display_mps = ManeuverUtil::dv_rtn_to_display_basis(*node);
        interaction.start_axis_t_m = start_t;
        interaction.drag_display_reference_time_s = drag_display_reference_time_s;
        interaction.drag_basis_r_world = node->basis_r_world;
        interaction.drag_basis_t_world = node->basis_t_world;
        interaction.drag_basis_n_world = node->basis_n_world;
        interaction.drag_maneuver_basis_r_world = node->maneuver_basis_r_world;
        interaction.drag_maneuver_basis_t_world = node->maneuver_basis_t_world;
        interaction.drag_maneuver_basis_n_world = node->maneuver_basis_n_world;
        interaction.drag_start_mouse_window_pos = mouse_pos_window;
        interaction.drag_last_sample_mouse_window_pos = mouse_pos_window;
        interaction.drag_threshold_passed = false;
        interaction.drag_display_snapshots.clear();
        interaction.drag_display_snapshots.reserve(plan.nodes.size());
        for (const ManeuverNode &candidate : plan.nodes)
        {
            if (!candidate.gizmo_valid)
            {
                continue;
            }

            interaction.drag_display_snapshots.push_back(ManeuverNodeDisplaySnapshot{
                    .node_id = candidate.id,
                    .position_world = candidate.position_world,
                    .basis_r_world = candidate.basis_r_world,
                    .basis_t_world = candidate.basis_t_world,
                    .basis_n_world = candidate.basis_n_world,
            });
        }
        interaction.applied_delta = false;
        return true;
    }

    ManeuverGizmoController::DragUpdateResult ManeuverGizmoController::update_axis_drag(
            ManeuverGizmoInteraction &interaction,
            const ManeuverNode &node,
            const glm::vec2 &mouse_pos_window,
            const CameraRay &ray,
            const float drag_threshold_px,
            const double drag_sensitivity_mps_per_m,
            const bool ctrl_down,
            const bool shift_down)
    {
        DragUpdateResult result{};

        const glm::vec2 drag_from_start = mouse_pos_window - interaction.drag_start_mouse_window_pos;
        if (!interaction.drag_threshold_passed)
        {
            if (glm::dot(drag_from_start, drag_from_start) < (drag_threshold_px * drag_threshold_px))
            {
                return result;
            }
            interaction.drag_threshold_passed = true;
        }

        const glm::vec2 drag_since_last_sample = mouse_pos_window - interaction.drag_last_sample_mouse_window_pos;
        if (glm::dot(drag_since_last_sample, drag_since_last_sample) <= 1.0e-8f)
        {
            return result;
        }
        interaction.drag_last_sample_mouse_window_pos = mouse_pos_window;

        const ManeuverUtil::AxisResolveResult axis_result = ManeuverUtil::resolve_axis(
                interaction.axis,
                interaction.drag_basis_r_world,
                interaction.drag_basis_t_world,
                interaction.drag_basis_n_world);
        if (!axis_result.valid)
        {
            result.clear_interaction = true;
            return result;
        }

        double current_t = 0.0;
        WorldVec3 axis_origin_world = node.position_world;
        if (const ManeuverNodeDisplaySnapshot *drag_snapshot =
                    ManeuverUtil::find_display_snapshot(interaction.drag_display_snapshots, node.id))
        {
            axis_origin_world = drag_snapshot->position_world;
        }
        const glm::dvec3 axis_origin_local = glm::dvec3(axis_origin_world - ray.camera_world);
        if (!closest_param_ray_line(ray.ray_origin_local,
                                    ray.ray_dir_local,
                                    axis_origin_local,
                                    axis_result.axis_dir_world,
                                    current_t))
        {
            return result;
        }

        const double delta_axis_m = current_t - interaction.start_axis_t_m;
        double scale = std::max(0.00001, drag_sensitivity_mps_per_m);
        if (ctrl_down && !shift_down)
        {
            scale *= 10.0;
        }
        else if (shift_down && !ctrl_down)
        {
            scale *= 0.1;
        }

        const double delta_mps = delta_axis_m * scale * axis_result.sign;
        glm::dvec3 dv_display_new = interaction.start_dv_display_mps;
        if (axis_result.component == 0)
        {
            dv_display_new.x = interaction.start_dv_display_mps.x + delta_mps;
        }
        else if (axis_result.component == 1)
        {
            dv_display_new.y = interaction.start_dv_display_mps.y + delta_mps;
        }
        else
        {
            dv_display_new.z = interaction.start_dv_display_mps.z + delta_mps;
        }

        const glm::dvec3 dv_world_new =
                ManeuverUtil::compose_basis_vector(dv_display_new,
                                                   interaction.drag_basis_r_world,
                                                   interaction.drag_basis_t_world,
                                                   interaction.drag_basis_n_world);
        const glm::dvec3 dv_new =
                ManeuverUtil::project_basis_vector(dv_world_new,
                                                  interaction.drag_maneuver_basis_r_world,
                                                  interaction.drag_maneuver_basis_t_world,
                                                  interaction.drag_maneuver_basis_n_world);

        result.sampled = true;
        if (ManeuverUtil::finite3(dv_new) && ManeuverUtil::safe_length(dv_new - node.dv_rtn_mps) > 1.0e-7)
        {
            result.changed = true;
            result.dv_rtn_mps = dv_new;
        }
        return result;
    }

    void ManeuverGizmoController::build_markers(const ManeuverPlanState &plan,
                                                const ManeuverGizmoInteraction &interaction,
                                                const ManeuverGizmoBasisMode basis_mode,
                                                const ManeuverGizmoViewContext &view,
                                                const float overlay_size_px,
                                                std::vector<ManeuverHubMarker> &out_hubs,
                                                std::vector<ManeuverAxisMarker> &out_handles)
    {
        out_hubs.clear();
        out_handles.clear();

        out_hubs.reserve(plan.nodes.size());
        for (const ManeuverNode &node : plan.nodes)
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
        const ManeuverNode *selected = plan.find_node(plan.selected_node_id);
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

        double dist_m = ManeuverUtil::safe_length(glm::dvec3(selected->position_world) - view.camera_world);
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
            const glm::dvec3 dir = ManeuverUtil::normalized_or(desc.dir_world, glm::dvec3(0.0, 1.0, 0.0));
            const bool is_active_axis =
                    interaction.state == ManeuverGizmoInteraction::State::DragAxis &&
                    interaction.node_id == selected->id &&
                    interaction.axis == desc.axis;
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
            marker.label = Gizmo::maneuver_axis_label(basis_mode, desc.axis);
            if (!Gizmo::project_maneuver_gizmo_point(view, handle_world, marker.handle_screen, marker.depth_m))
            {
                continue;
            }

            out_handles.push_back(marker);
        }
    }
} // namespace Game
