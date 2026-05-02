#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"

#include <vector>

struct ImDrawList;

namespace Game
{
    class GameplayState;
    struct GameStateContext;

    class ManeuverUiController
    {
    public:
        static void emit_node_debug_overlay(GameplayState &state, GameStateContext &ctx);
        static void draw_nodes_panel(GameplayState &state, GameStateContext &ctx);
        static void draw_imgui_gizmo(GameplayState &state, GameStateContext &ctx);

    private:
        static void update_ui_config(GameplayState &state, GameStateContext &ctx);
        static void draw_gizmo_markers(const GameplayState &state,
                                       ImDrawList *draw_list,
                                       const std::vector<ManeuverHubMarker> &hubs,
                                       const std::vector<ManeuverAxisMarker> &handles,
                                       int hovered_hub_idx,
                                       int hovered_handle_idx,
                                       float hub_hit_px,
                                       float axis_hit_px);
        static void draw_gizmo_hover_tooltip(const GameplayState &state,
                                             const std::vector<ManeuverAxisMarker> &handles,
                                             int hovered_handle_idx);
    };
} // namespace Game
