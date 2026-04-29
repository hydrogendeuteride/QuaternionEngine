# Maneuver Folder Guide

This folder contains the maneuver-node editor used by `GameplayState`.

## What Lives Here

### Headers

- `gameplay_state_maneuver_types.h`
  Shared data types for maneuver nodes, gizmo markers, interaction state, view/projection cache, and UI config.

- `gameplay_state_maneuver_colors.h`
  Color constants for gizmo axes, hub markers, drag feedback, orbit-pick highlights, timeline elements, basis-mode buttons (`kBasisModes`, `kBasisButtonActive/Hovered/Inactive`), and debug-draw overlays.

- `gameplay_state_maneuver_util.h`
  Shared inline helpers used across all `.cpp` files in this folder:
  - Math: `finite3`, `normalized_or`, `compute_maneuver_frame`, `clamp_sane`, re-exported `safe_length`
  - Basis: `compose_basis_vector`, `project_basis_vector`, `dv_rtn_to_display_basis`
  - Axis resolution: `AxisResolveResult` struct + `resolve_axis` (replaces duplicated switch statements)
  - Snapshot lookup: `find_display_snapshot`
  - Labels: `basis_mode_label`, `axis_short_label`

### Implementation Files

- `gameplay_state_maneuver_nodes.cpp`
  Core node coordination: sim-time query (`current_sim_time_s`), primary-body resolution (`resolve_maneuver_node_primary_body_id`), command side effects (`apply_maneuver_command`), node removal wrappers (`remove_maneuver_node`, `remove_maneuver_node_suffix`), and orbit-curve alignment delta (`compute_maneuver_align_delta`).

- `maneuver_plan_model.*`, `maneuver_commands.*`, `maneuver_controller.*`
  Plan mutation boundary. UI and gizmo code create commands, the controller applies them to `ManeuverPlanState`, and the returned result drives prediction dirty/artifact cleanup from `GameplayState`.

- `gameplay_state_maneuver_nodes_cache.cpp`
  Per-frame runtime cache rebuild: `refresh_maneuver_node_runtime_cache()` and its internal helpers. Rebuilds all derived node state (world positions, RTN/PON bases, burn direction, gizmo validity) from prediction data. Contains shared sampling helpers as templates (`sample_traj_world_at`, `sample_chunk_world_at`, `sample_tangent_world_from`).

- `gameplay_state_maneuver_panel.cpp`
  ImGui panel UI -- the "Maneuver Nodes" window, timeline bar, node list, node detail editor, and buttons like `+Node`, `Warp to Node`, and `Execute`.

- `gameplay_state_maneuver_gizmo.cpp`
  In-world screen-space gizmo UI -- node hubs, axis handles, hover logic, drag logic, world-to-screen projection, and runtime cache refresh. Uses `ManeuverUtil::resolve_axis` for axis direction mapping and delegates to `PredictionRuntimeDetail::update_last_and_peak` for streaming state.

- `gameplay_state_maneuver_runtime.cpp`
  Runtime behavior -- warp-to-node time-warp control, actual burn execution, and debug overlay drawing.

- `gameplay_state_maneuver_ui.cpp`
  UI config -- computes effective scale from window/drawable size and user scale setting.

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

- UI scaling or size settings:
  Start in `gameplay_state_maneuver_ui.cpp`.

- Color constants for gizmo, timeline, or debug overlays:
  Start in `gameplay_state_maneuver_colors.h`.

- What data a node stores or gizmo interaction state:
  Start in `gameplay_state_maneuver_types.h`.

- Shared math helpers, axis resolution, basis composition, or label strings:
  Start in `gameplay_state_maneuver_util.h`.

- Core node logic like primary-body resolution or node removal:
  Start in `gameplay_state_maneuver_nodes.cpp`.

- Runtime cache rebuild, trajectory/chunk sampling, or per-frame derived state:
  Start in `gameplay_state_maneuver_nodes_cache.cpp`.

## Notes About Structure

- Each `.cpp` file is an independent translation unit (no `.inc` includes).
- All `.cpp` files import shared helpers via `using namespace ManeuverUtil` from `gameplay_state_maneuver_util.h`.
- The panel and gizmo both edit the same underlying node data: `ManeuverNode::dv_rtn_mps` and `ManeuverNode::time_s`.
- `dv_rtn_mps` stores maneuver-frame components in `Radial / Tangential / Normal` order.
- Execution and prediction always use the canonical true RTN frame built from the pre-burn primary-relative state.
- The gizmo can either expose that same RTN basis directly or a `Prograde / Outward / Normal` display basis that projects back into RTN for storage.
- The gizmo depends on prediction data and render interpolation, so marker placement is rebuilt every frame in `refresh_maneuver_node_runtime_cache()`.
- Sampling helpers in `nodes_cache.cpp` use templated callables to access `GameplayState` private methods without exposing them.
