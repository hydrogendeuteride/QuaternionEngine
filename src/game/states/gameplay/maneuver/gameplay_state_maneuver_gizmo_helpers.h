#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_colors.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_util.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace Game::ManeuverGizmoHelpers
{
    inline const char *maneuver_axis_label(const ManeuverGizmoBasisMode basis_mode, const ManeuverHandleAxis axis)
    {
        return ManeuverUtil::axis_short_label(basis_mode, axis);
    }

    inline uint32_t maneuver_axis_color(const ManeuverHandleAxis axis)
    {
        switch (axis)
        {
            case ManeuverHandleAxis::TangentialPos:
            case ManeuverHandleAxis::TangentialNeg:
                return ManeuverColors::kAxisTangential;
            case ManeuverHandleAxis::RadialPos:
            case ManeuverHandleAxis::RadialNeg:
                return ManeuverColors::kAxisRadial;
            case ManeuverHandleAxis::NormalPos:
            case ManeuverHandleAxis::NormalNeg:
                return ManeuverColors::kAxisNormal;
            default:
                return ManeuverColors::kAxisDefault;
        }
    }

    inline bool maneuver_gizmo_is_occluded(const ManeuverGizmoViewContext &view, const WorldVec3 &point_world)
    {
        if (!view.depth_occluder_valid)
        {
            return false;
        }

        const glm::dvec3 center_to_cam = view.camera_world - glm::dvec3(view.depth_occluder_center);
        const double center_to_cam_len2 = glm::dot(center_to_cam, center_to_cam);
        const double radius2 = view.depth_occluder_radius * view.depth_occluder_radius;
        if (!std::isfinite(center_to_cam_len2) || center_to_cam_len2 <= radius2)
        {
            return false;
        }

        const glm::dvec3 cam_to_point = glm::dvec3(point_world) - view.camera_world;
        const double point_dist = ManeuverUtil::safe_length(cam_to_point);
        if (!std::isfinite(point_dist) || point_dist <= 1.0e-3)
        {
            return false;
        }

        const glm::dvec3 ray_dir = cam_to_point / point_dist;
        const glm::dvec3 oc = view.camera_world - glm::dvec3(view.depth_occluder_center);
        const double b = glm::dot(oc, ray_dir);
        const double c = glm::dot(oc, oc) - radius2;
        const double disc = b * b - c;
        if (!std::isfinite(disc) || disc <= 0.0)
        {
            return false;
        }

        const double t_near = -b - std::sqrt(disc);
        const double eps = std::max(1.0, view.depth_occluder_radius * 1.0e-6);
        return std::isfinite(t_near) && t_near > eps && t_near < (point_dist - eps);
    }

    inline bool project_maneuver_gizmo_point(const ManeuverGizmoViewContext &view,
                                             const WorldVec3 &point_world,
                                             glm::vec2 &out_screen,
                                             double &out_depth_m)
    {
        out_screen = glm::vec2(0.0f, 0.0f);
        out_depth_m = 0.0;

        const glm::dvec3 rel_world = glm::dvec3(point_world) - view.camera_world;
        const glm::dvec3 cam_space = view.world_to_cam * rel_world;
        const double depth_m = -cam_space.z;
        if (!std::isfinite(depth_m) || depth_m <= 1.0e-4)
        {
            return false;
        }

        const double ndc_x = cam_space.x / (depth_m * view.aspect * view.tan_half_fov);
        const double ndc_y = cam_space.y / (depth_m * view.tan_half_fov);
        if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y))
        {
            return false;
        }
        if (ndc_x < -1.2 || ndc_x > 1.2 || ndc_y < -1.2 || ndc_y > 1.2)
        {
            return false;
        }

        const double u = (ndc_x + 1.0) * 0.5;
        const double v = (1.0 - ndc_y) * 0.5;
        if (!std::isfinite(u) || !std::isfinite(v))
        {
            return false;
        }

        const double swap_x =
                static_cast<double>(view.letterbox_rect.x) + u * static_cast<double>(view.letterbox_rect.width);
        const double swap_y =
                static_cast<double>(view.letterbox_rect.y) + v * static_cast<double>(view.letterbox_rect.height);

        const double draw_x = swap_x * view.draw_from_swap_x;
        const double draw_y = swap_y * view.draw_from_swap_y;
        const double win_x = draw_x * view.window_from_draw_x;
        const double win_y = draw_y * view.window_from_draw_y;
        if (!std::isfinite(win_x) || !std::isfinite(win_y))
        {
            return false;
        }

        out_screen = glm::vec2(static_cast<float>(win_x), static_cast<float>(win_y));
        out_depth_m = depth_m;
        return true;
    }

    inline bool resolve_maneuver_axis(const ManeuverNode &node,
                                      const ManeuverHandleAxis axis,
                                      glm::dvec3 &out_axis_dir_world,
                                      int &out_component,
                                      double &out_sign)
    {
        const ManeuverUtil::AxisResolveResult r =
                ManeuverUtil::resolve_axis(axis, node.basis_r_world, node.basis_t_world, node.basis_n_world);
        out_axis_dir_world = r.axis_dir_world;
        out_component = r.component;
        out_sign = r.sign;
        return r.valid;
    }

    inline void find_maneuver_gizmo_hover(const std::vector<ManeuverHubMarker> &hubs,
                                          const std::vector<ManeuverAxisMarker> &handles,
                                          const glm::vec2 &mouse_pos,
                                          const float hub_hit_px2,
                                          const float axis_hit_px2,
                                          int &out_hovered_hub_idx,
                                          int &out_hovered_handle_idx)
    {
        out_hovered_hub_idx = -1;
        out_hovered_handle_idx = -1;

        float best_handle_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < handles.size(); ++i)
        {
            const glm::vec2 d = handles[i].handle_screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= axis_hit_px2 && d2 < best_handle_d2)
            {
                best_handle_d2 = d2;
                out_hovered_handle_idx = static_cast<int>(i);
            }
        }

        if (out_hovered_handle_idx >= 0)
        {
            return;
        }

        float best_hub_d2 = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < hubs.size(); ++i)
        {
            const glm::vec2 d = hubs[i].screen - mouse_pos;
            const float d2 = glm::dot(d, d);
            if (d2 <= hub_hit_px2 && d2 < best_hub_d2)
            {
                best_hub_d2 = d2;
                out_hovered_hub_idx = static_cast<int>(i);
            }
        }
    }
} // namespace Game::ManeuverGizmoHelpers
