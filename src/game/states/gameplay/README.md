# Gameplay Folder Guide

This folder contains the `GameplayState` class and its supporting systems, implementing the main gameplay mode for the space combat game: orbital mechanics simulation, physics, time warp, scene setup, prediction display, maneuver planning, and the gameplay HUD.

## Folder Structure

```
gameplay/
  gameplay_state.h                # GameplayState class declaration (IGameState)
  gameplay_state.cpp              # lifecycle (on_enter, on_exit, on_update, on_fixed_update), time warp input
  gameplay_state_sim.cpp          # physics stepping, time warp transitions, rails warp, formation hold
  gameplay_state_scene.cpp        # scene bootstrap, orbitsim init, orbiter/celestial spawning, player switching
  gameplay_state_ui.cpp           # on_draw_ui, ImGui HUD panels, settings extract/apply, debug windows
  gameplay_settings.h / .cpp      # GameplaySettings struct and JSON load/save
  orbit_helpers.h                 # CelestialBodyInfo, OrbiterInfo, OrbitalScenario, N-body math helpers
  time_warp_state.h               # TimeWarpState (mode, level, warp factors)
  frame_monitor.h / .cpp          # FrameMonitor (FPS history + ImGui overlay)
  maneuver/                       # maneuver-node editor (panel UI, gizmo, execution)
  prediction/                     # orbit prediction display pipeline (runtime, derived cache, drawing)
  scenario/                       # scenario config data model and JSON serialization
```

## What Lives Here

### Headers

- `gameplay_state.h`
  The `GameplayState` class, derived from `IGameState`. Declares all private methods and member state for the gameplay mode: simulation stepping, orbit prediction, maneuver nodes, time warp, scene setup, orbiter management, and UI drawing. Owns `GameWorld`, `PhysicsWorld`, `OrbitalScenario`, prediction services, and maneuver plan state.

- `gameplay_settings.h`
  `GameplaySettings` struct with prediction draw toggles, timing parameters, `PredictionSamplingPolicy`, `ManeuverPlanHorizonSettings`, `OrbitPlotBudgetSettings`, and debug flags. Also declares `load_gameplay_settings()` / `save_gameplay_settings()` for JSON persistence.

- `orbit_helpers.h`
  Runtime data types for the orbital scenario:
  - `CelestialBodyInfo` -- runtime state for a massive body (sim ID, render entity, name, radius, mass)
  - `OrbiterInfo` -- runtime state for an orbiting entity (entity, physics settings, rails state, formation hold)
  - `OrbitalScenario` -- owns the `GameSimulation` and celestial body registry with lookup helpers
  - `detail` namespace: `finite_vec3`, `contact_event_type_name`, `circular_orbit_relative_state_xz`, `two_body_circular_barycentric_xz`, `point_mass_accel`, `nbody_accel_body_centered`

- `time_warp_state.h`
  `TimeWarpState` struct: `Mode` enum (`Realtime`, `PhysicsWarp`, `RailsWarp`), warp level (0-6), factor lookup table (x1 through x1000), and `mode_for_level()` helper.

- `frame_monitor.h`
  `FrameMonitor` class: circular buffer of frame times (300 samples), min/max/avg FPS computation, and `draw_ui()` for the FPS overlay.

### Implementation Files

- `gameplay_state.cpp`
  Core lifecycle: constructor, destructor, `on_enter()` (scenario/settings loading, scene setup), `on_exit()` (teardown), `on_update()` (frame interpolation, component update, prediction polling, maneuver cache refresh, debug overlays), `on_fixed_update()` (time warp dispatch, physics stepping, maneuver execution, prediction updates), `reset_time_warp_state()`, `handle_time_warp_input()`, `build_component_context()`.

- `gameplay_state_sim.cpp`
  Simulation implementation: `step_physics()` (N-body gravity + physics world step), `set_time_warp_level()`, `enter_rails_warp()` / `exit_rails_warp()` / `rails_warp_step()` (on-rails time warp with thrust propagation), `update_runtime_orbiter_rails()` / `sync_runtime_orbiter_rails()` (automatic on/off-rails transitions by distance), `update_formation_hold()` (critically-damped spring LVLH station-keeping), `sync_celestial_render_entities()`.

- `gameplay_state_scene.cpp`
  Scene bootstrap: `default_earth_moon_config()` (compiled fallback scenario), `setup_scene()` (physics world creation, celestial body spawning, orbiter instantiation from `ScenarioConfig`), `setup_environment()` (lighting, skybox, camera), `init_orbitsim()` (N-body simulation initialization with barycentric two-body states), orbiter lookup helpers (`find_player_orbiter`, `find_orbiter` by entity/name), `player_entity()`, `select_rebase_anchor_entity()`, `update_rebase_anchor()`, `sync_player_camera_target()`, `sync_player_collision_callbacks()`, `set_active_player_orbiter()`, `cycle_player_orbiter()`.

- `gameplay_state_ui.cpp`
  UI and settings: `extract_settings()` / `apply_settings()` for `GameplaySettings` round-trip, `on_draw_ui()` (the main HUD: simulation info panel, orbiter state, time warp controls, prediction config, maneuver panels, orbit plot stats, contact log, scenario/settings save/load, frame monitor), `draw_orbit_drag_debug_window()`.

- `gameplay_settings.cpp`
  JSON load/save for `GameplaySettings` using `nlohmann::json`. Supports schema versioning and legacy field aliases for backward compatibility.

- `frame_monitor.cpp`
  `FrameMonitor::update()` (circular buffer write, min/max/avg computation) and `draw_ui()` (ImGui overlay with FPS text and frame-time plot).

### `maneuver/` Subfolder

Maneuver-node editor: panel UI, in-world gizmo, node management, runtime execution. See `maneuver/README.md`.

### `prediction/` Subfolder

Orbit prediction display pipeline: derived cache service, frame transforms, drawing, picking, runtime orchestration. See `prediction/README.md`.

### `scenario/` Subfolder

Scenario configuration data model and JSON serialization. See `scenario/README.md`.

## How It Is Called

`GameplayState` is an `IGameState` managed by the `GameStateManager` stack. It is pushed as the active state when the player enters gameplay from the title screen.

- `on_enter()` / `on_exit()`
  Called by `GameStateManager` on state transitions. `on_enter()` loads the scenario JSON, applies settings, and bootstraps the scene. `on_exit()` tears down all owned resources.

- `on_update(dt)`
  Called every frame by `GameStateManager`. Drives component interpolation, prediction result polling, maneuver cache refresh, and debug overlay emission.

- `on_fixed_update(fixed_dt)`
  Called at fixed timestep. Drives physics stepping (with warp), orbitsim advancement, maneuver execution, formation hold, and prediction request submission.

- `on_draw_ui()`
  Called every frame after rendering. Draws the gameplay HUD, prediction config, maneuver panels, orbit plots, and debug windows via ImGui.

Relevant call sites are in `src/game/state/game_state_manager.h` and `src/game/space_combat_game.cpp`.

## If You Want To Change...

- The gameplay lifecycle (enter, exit, reset):
  Start in `gameplay_state.cpp`.

- Physics stepping, gravity, or time warp behavior:
  Start in `gameplay_state_sim.cpp`.

- Scene setup, entity spawning, or orbitsim initialization:
  Start in `gameplay_state_scene.cpp`.

- The HUD, ImGui panels, or debug windows:
  Start in `gameplay_state_ui.cpp`.

- Gameplay settings fields or JSON keys:
  Start in `gameplay_settings.h` (struct) and `gameplay_settings.cpp` (serialization).

- Runtime data for celestials, orbiters, or the orbital scenario:
  Start in `orbit_helpers.h`.

- Time warp levels, modes, or factor tables:
  Start in `time_warp_state.h`.

- The FPS overlay or frame-time tracking:
  Start in `frame_monitor.h` / `frame_monitor.cpp`.

- Maneuver-node editing, gizmo, or execution:
  Start in `maneuver/` (see `maneuver/README.md`).

- Orbit prediction display, drawing, or runtime:
  Start in `prediction/` (see `prediction/README.md`).

- Scenario data model or JSON format:
  Start in `scenario/` (see `scenario/README.md`).

## Notes About Structure

- Each `.cpp` file is an independent translation unit implementing `GameplayState` methods. They share state via the class declaration in `gameplay_state.h`.
- The split across `_sim`, `_scene`, and `_ui` files is by responsibility: simulation logic, scene bootstrap/entity management, and UI/settings respectively. `gameplay_state.cpp` holds lifecycle and top-level dispatch.
- Physics and orbitsim are optional: the physics world is conditionally compiled behind `VULKAN_ENGINE_USE_JOLT`.
- Time warp has three modes: `Realtime` (x1), `PhysicsWarp` (x2-x10, runs multiple physics sub-steps per tick), and `RailsWarp` (x50-x1000, bypasses physics and advances the orbit simulation directly).
- `OrbiterInfo` tracks both physics-body state (off-rails) and `RailsState` (on-rails), with automatic promotion/demotion based on distance from the player.
- The gameplay settings JSON supports schema versioning and legacy field aliases, so older settings files remain loadable.
- N-body gravity helpers in `orbit_helpers.h` compute body-centered relative accelerations (tidal forces) for the physics step.
- Subfolders (`maneuver/`, `prediction/`, `scenario/`) are self-contained modules with their own READMEs.
