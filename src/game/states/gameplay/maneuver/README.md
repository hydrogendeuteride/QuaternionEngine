# Maneuver Folder Guide

This folder contains the maneuver-node editor used by `GameplayState`.

## What Lives Here

- `gameplay_state_maneuver_types.h`
  Shared data types for maneuver nodes, gizmo markers, interaction state, and view/projection cache.

- `gameplay_state_maneuver_nodes.cpp`
  Shared helper math for the maneuver feature.
  This file is also the translation unit that includes the three `.inc` files below.

- `gameplay_state_maneuver_nodes_panel.inc`
  ImGui panel UI.
  This is where the "Maneuver Nodes" window, timeline bar, node list, node detail editor, and buttons like `+Node`, `Warp to Node`, and `Execute` live.

- `gameplay_state_maneuver_nodes_gizmo.inc`
  In-world screen-space gizmo UI.
  This is where node hubs, axis handles, hover logic, drag logic, world-to-screen projection, and runtime cache refresh live.

- `gameplay_state_maneuver_nodes_runtime.inc`
  Runtime behavior for warp-to-node, actual burn execution, and debug overlay drawing.

## UI Breakdown

There are two different UI surfaces for maneuver nodes:

1. Panel UI
   Implemented in `gameplay_state_maneuver_nodes_panel.inc`.
   Use this file when changing window layout, node creation flow, timeline dragging, list selection, numeric DV editing, or button behavior.

2. Gizmo UI
   Implemented in `gameplay_state_maneuver_nodes_gizmo.inc`.
   Use this file when changing on-screen node markers, axis handles, hover hit-testing, drag sensitivity, projection math, or node marker placement on the orbit.

## How It Is Called

The entry points are declared on `GameplayState` in:

- `src/game/states/gameplay/gameplay_state.h`

The feature is driven from `GameplayState` like this:

- `on_update()`
  Refreshes maneuver runtime cache and emits debug overlay data.

- `on_fixed_update()`
  Updates warp-to-node state, executes armed nodes when their time arrives, and keeps prediction in sync.

- `on_draw_ui()`
  Draws both the panel UI and the gizmo UI.

Relevant call sites are currently in:

- `src/game/states/gameplay/gameplay_state.cpp`

## If You Want To Change...

- The floating window, timeline, node list, or buttons:
  Start in `gameplay_state_maneuver_nodes_panel.inc`.

- The in-world axis handles or click/drag behavior:
  Start in `gameplay_state_maneuver_nodes_gizmo.inc`.

- When a node executes, how warp behaves, or debug draw:
  Start in `gameplay_state_maneuver_nodes_runtime.inc`.

- What data a node stores or what state the gizmo caches:
  Start in `gameplay_state_maneuver_types.h`.

- Shared math helpers like RTN frame generation, trajectory sampling, or ray/line closest-point solve:
  Start in `gameplay_state_maneuver_nodes.cpp`.

## Notes About Structure

- The `.inc` files are not independent modules; they are included into `gameplay_state_maneuver_nodes.cpp`.
- The panel and gizmo both edit the same underlying node data: `ManeuverNode::dv_rtn_mps` and `ManeuverNode::time_s`.
- The gizmo depends on prediction data and render interpolation, so marker placement is rebuilt every frame in `refresh_maneuver_node_runtime_cache()`.
