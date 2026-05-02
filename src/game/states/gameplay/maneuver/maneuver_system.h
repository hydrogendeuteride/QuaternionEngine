#pragma once

#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/maneuver/maneuver_commands.h"

#include <cstdint>

namespace Game
{
    struct ManeuverSystemSettings
    {
        ManeuverPlanHorizonSettings plan_horizon{};
        ManeuverPlanWindowSettings plan_windows{};
        bool live_preview_active{true};

        bool nodes_enabled{true};
        bool nodes_debug_draw{true};
        double timeline_window_s{3600.0};
        ManeuverGizmoBasisMode gizmo_basis_mode{ManeuverGizmoBasisMode::ProgradeOutwardNormal};
        ManeuverGizmoStyle gizmo_style{};
        ManeuverUIConfig ui_config{};
    };

    struct ManeuverRuntimeState
    {
        bool warp_to_time_active{false};
        double warp_to_time_target_s{0.0};
        int warp_to_time_restore_level{0};
        bool execute_node_armed{false};
        int execute_node_id{-1};

        void reset();
        void arm_warp_to_time(double target_s, int restore_level = 0);
        void disarm_warp_to_time();
        void arm_execute_node(int node_id);
        void disarm_execute_node();
    };

    class ManeuverSystem
    {
    public:
        [[nodiscard]] ManeuverSystemSettings &settings();
        [[nodiscard]] const ManeuverSystemSettings &settings() const;
        [[nodiscard]] ManeuverRuntimeState &runtime();
        [[nodiscard]] const ManeuverRuntimeState &runtime() const;
        [[nodiscard]] ManeuverPlanState &plan();
        [[nodiscard]] const ManeuverPlanState &plan() const;
        [[nodiscard]] ManeuverGizmoInteraction &gizmo_interaction();
        [[nodiscard]] const ManeuverGizmoInteraction &gizmo_interaction() const;
        [[nodiscard]] ManeuverNodeEditPreview &edit_preview();
        [[nodiscard]] const ManeuverNodeEditPreview &edit_preview() const;
        [[nodiscard]] uint64_t &revision();
        [[nodiscard]] uint64_t revision() const;

        void reset_session();
        uint64_t increment_revision();
        void set_revision(uint64_t revision);
        ManeuverCommandResult apply_command(const ManeuverCommand &command);

        [[nodiscard]] bool live_preview_active(bool with_maneuvers) const;
        [[nodiscard]] int active_preview_anchor_node_id() const;
        [[nodiscard]] uint64_t plan_signature(double plan_horizon_s) const;

    private:
        ManeuverSystemSettings _settings{};
        ManeuverRuntimeState _runtime{};
        ManeuverPlanState _plan{};
        ManeuverGizmoInteraction _gizmo_interaction{};
        ManeuverNodeEditPreview _edit_preview{};
        uint64_t _revision{0};
    };
} // namespace Game
