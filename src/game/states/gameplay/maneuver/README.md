# Maneuver Folder Guide

This folder contains the maneuver-node editor used by `GameplayState`.

## What Lives Here

- `gameplay_state_maneuver_types.h`
  Shared data types for maneuver nodes, gizmo markers, interaction state, view/projection cache, and UI config.

- `gameplay_state_maneuver_colors.h`
  All color constants for gizmo axes, hub markers, drag feedback, orbit-pick highlights, timeline elements, basis-mode buttons, and debug-draw overlays.

- `gameplay_state_maneuver_util.h`
  Inline math helpers: basis composition/projection, RTN-to-display-basis conversion, and axis/basis-mode label strings.

- `gameplay_state_maneuver_nodes.cpp`
  Core logic: sim-time query, primary-body resolution, node removal, orbit-curve alignment delta, and the per-frame `refresh_maneuver_node_runtime_cache()` that rebuilds all derived node state (world positions, RTN/PON bases, burn direction, gizmo validity).

- `gameplay_state_maneuver_panel.cpp`
  ImGui panel UI — the "Maneuver Nodes" window, timeline bar, node list, node detail editor, and buttons like `+Node`, `Warp to Node`, and `Execute`.

- `gameplay_state_maneuver_gizmo.cpp`
  In-world screen-space gizmo UI — node hubs, axis handles, hover logic, drag logic, world-to-screen projection, and runtime cache refresh.

- `gameplay_state_maneuver_runtime.cpp`
  Runtime behavior — warp-to-node time-warp control, actual burn execution, and debug overlay drawing.

- `gameplay_state_maneuver_ui.cpp`
  DPI-aware UI config — computes effective scale from window/drawable size, display DPI, and user scale setting.

## UI Breakdown

There are two different UI surfaces for maneuver nodes:

1. Panel UI
   Implemented in `gameplay_state_maneuver_panel.cpp`.
   Use this file when changing window layout, node creation flow, timeline dragging, list selection, numeric DV editing, or button behavior.

2. Gizmo UI
   Implemented in `gameplay_state_maneuver_gizmo.cpp`.
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
  Start in `gameplay_state_maneuver_panel.cpp`.

- The in-world axis handles or click/drag behavior:
  Start in `gameplay_state_maneuver_gizmo.cpp`.

- When a node executes, how warp behaves, or debug draw:
  Start in `gameplay_state_maneuver_runtime.cpp`.

- DPI scaling or UI size settings:
  Start in `gameplay_state_maneuver_ui.cpp`.

- Color constants for gizmo, timeline, or debug overlays:
  Start in `gameplay_state_maneuver_colors.h`.

- What data a node stores or what state the gizmo caches:
  Start in `gameplay_state_maneuver_types.h`.

- Shared math helpers like basis composition/projection or label strings:
  Start in `gameplay_state_maneuver_util.h`.

- Core node logic like primary-body resolution, node removal, or runtime cache refresh:
  Start in `gameplay_state_maneuver_nodes.cpp`.

## Notes About Structure

- Each `.cpp` file is an independent translation unit (no `.inc` includes).
- The panel and gizmo both edit the same underlying node data: `ManeuverNode::dv_rtn_mps` and `ManeuverNode::time_s`.
- `dv_rtn_mps` stores maneuver-frame components in `Radial / Tangential / Normal` order.
- Execution and prediction always use the canonical true RTN frame built from the pre-burn primary-relative state.
- The gizmo can either expose that same RTN basis directly or a `Prograde / Outward / Normal` display basis that projects back into RTN for storage.
- The gizmo depends on prediction data and render interpolation, so marker placement is rebuilt every frame in `refresh_maneuver_node_runtime_cache()`.
