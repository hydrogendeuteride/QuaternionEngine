#pragma once

#include "core/world.h"

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "orbitsim/maneuvers_types.hpp"

namespace Game
{
    struct ManeuverNode
    {
        int id{-1}; // unique ID (stable)
        double time_s{0.0}; // sim time (absolute)
        glm::dvec3 dv_rtn_mps{0.0, 0.0, 0.0}; // UI-authored maneuver-frame DV (Radial, Tangential/Prograde, Normal) [m/s]
        orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId}; // RTN primary body

        // Cached/derived values (updated by gameplay state)
        // These drive the in-world gizmo, debug overlay, and node execution preview.
        double total_dv_mps{0.0};
        WorldVec3 position_world{0.0, 0.0, 0.0};
        glm::dvec3 burn_direction_world{0.0, 0.0, 0.0};
        glm::dvec3 basis_r_world{1.0, 0.0, 0.0};
        glm::dvec3 basis_t_world{0.0, 1.0, 0.0};
        glm::dvec3 basis_n_world{0.0, 0.0, 1.0};
        float gizmo_scale_m{1.0f};
        bool gizmo_valid{false};
    };

    struct ManeuverPlanState
    {
        std::vector<ManeuverNode> nodes;
        int selected_node_id{-1};
        int next_node_id{0};

        // IDs stay stable even if nodes are time-sorted, so selection/interaction can follow the same node.
        ManeuverNode *find_node(const int id)
        {
            for (auto &n : nodes)
            {
                if (n.id == id)
                {
                    return &n;
                }
            }
            return nullptr;
        }

        const ManeuverNode *find_node(const int id) const
        {
            for (const auto &n : nodes)
            {
                if (n.id == id)
                {
                    return &n;
                }
            }
            return nullptr;
        }

        void sort_by_time()
        {
            std::sort(nodes.begin(), nodes.end(), [](const ManeuverNode &a, const ManeuverNode &b) {
                return a.time_s < b.time_s;
            });
        }

        // Convert the editor-facing plan into the orbitsim impulse plan consumed by prediction/runtime.
        orbitsim::ManeuverPlan to_orbitsim_plan(const orbitsim::SpacecraftId sc_id) const
        {
            orbitsim::ManeuverPlan plan{};
            plan.impulses.reserve(nodes.size());

            for (const auto &node : nodes)
            {
                orbitsim::ImpulseSegment imp{};
                imp.t_s = node.time_s;
                imp.primary_body_id = node.primary_body_id;
                imp.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                imp.spacecraft_id = sc_id;
                plan.impulses.push_back(imp);
            }

            orbitsim::sort_impulses_by_time(plan);
            return plan;
        }
    };

    enum class ManeuverHandleAxis
    {
        None = 0,
        Hub,
        ProgradePos,
        ProgradeNeg,
        RadialPos,
        RadialNeg,
        NormalPos,
        NormalNeg,
    };

    struct ManeuverGizmoStyle
    {
        glm::vec4 icon_color{0.30f, 0.80f, 1.00f, 0.90f};
        float icon_size_px{30.0f};
        float overlay_scale{1.0f};
        bool show_axis_labels{true};
        float drag_sensitivity_mps_per_m{0.0025f};
    };

    struct ManeuverGizmoInteraction
    {
        enum class State
        {
            Idle = 0,
            HoverAxis,
            DragAxis
        };

        // Captures drag state in RTN component space plus the world-space axis parameter used by the solver.
        State state{State::Idle};
        int node_id{-1};
        ManeuverHandleAxis axis{ManeuverHandleAxis::None};
        glm::dvec3 start_dv_rtn_mps{0.0, 0.0, 0.0};
        double start_axis_t_m{0.0};
        bool applied_delta{false};
    };

    struct ManeuverGizmoViewContext
    {
        // Cached projection state shared by screen-space marker generation, hit-testing, and picking.
        glm::dvec3 camera_world{0.0, 0.0, 0.0};
        glm::dmat3 world_to_cam{1.0};
        double logical_w{0.0};
        double logical_h{0.0};
        double aspect{1.0};
        double tan_half_fov{0.0};
        VkRect2D letterbox_rect{};
        double draw_from_swap_x{1.0};
        double draw_from_swap_y{1.0};
        double window_from_draw_x{1.0};
        double window_from_draw_y{1.0};
        bool depth_occluder_valid{false};
        WorldVec3 depth_occluder_center{0.0, 0.0, 0.0};
        double depth_occluder_radius{0.0};
    };

    struct ManeuverHubMarker
    {
        int node_id{-1};
        glm::vec2 screen{0.0f, 0.0f};
        double depth_m{0.0};
    };

    struct ManeuverAxisMarker
    {
        // Screen-space endpoints for the currently selected node's draggable axis handles.
        int node_id{-1};
        ManeuverHandleAxis axis{ManeuverHandleAxis::None};
        glm::vec2 hub_screen{0.0f, 0.0f};
        glm::vec2 handle_screen{0.0f, 0.0f};
        uint32_t base_color{0};
        const char *label{""};
        double depth_m{0.0};
    };
} // namespace Game
