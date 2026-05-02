#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include <glm/glm.hpp>

#include <vector>

namespace Game
{
    class ManeuverGizmoController
    {
    public:
        struct CameraRay
        {
            WorldVec3 camera_world{0.0, 0.0, 0.0};
            glm::dvec3 ray_origin_local{0.0, 0.0, 0.0};
            glm::dvec3 ray_dir_local{0.0, 0.0, -1.0};
        };

        struct DragUpdateResult
        {
            bool sampled{false};
            bool clear_interaction{false};
            bool changed{false};
            glm::dvec3 dv_rtn_mps{0.0, 0.0, 0.0};
        };

        static bool begin_axis_drag(ManeuverPlanState &plan,
                                    ManeuverGizmoInteraction &interaction,
                                    int node_id,
                                    ManeuverHandleAxis axis,
                                    const CameraRay &ray,
                                    const glm::vec2 &mouse_pos_window,
                                    double drag_display_reference_time_s);

        static DragUpdateResult update_axis_drag(ManeuverGizmoInteraction &interaction,
                                                 const ManeuverNode &node,
                                                 const glm::vec2 &mouse_pos_window,
                                                 const CameraRay &ray,
                                                 float drag_threshold_px,
                                                 double drag_sensitivity_mps_per_m,
                                                 bool ctrl_down,
                                                 bool shift_down);

        static void build_markers(const ManeuverPlanState &plan,
                                  const ManeuverGizmoInteraction &interaction,
                                  ManeuverGizmoBasisMode basis_mode,
                                  const ManeuverGizmoViewContext &view,
                                  float overlay_size_px,
                                  std::vector<ManeuverHubMarker> &out_hubs,
                                  std::vector<ManeuverAxisMarker> &out_handles);

    private:
        static bool closest_param_ray_line(const glm::dvec3 &ray_origin_local,
                                           const glm::dvec3 &ray_dir_local,
                                           const glm::dvec3 &line_origin_local,
                                           const glm::dvec3 &line_dir_local,
                                           double &out_line_t);
    };
} // namespace Game
