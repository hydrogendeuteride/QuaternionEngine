# Prediction Folder Guide

This folder contains the orbit prediction display pipeline used by `GameplayState`: the derived cache service, frame transforms, drawing, picking, and runtime orchestration.

## Folder Structure

```
prediction/
  gameplay_state_prediction_types.h              # shared types, caches, draw config, telemetry
  gameplay_prediction_cache_internal.h / .cpp    # low-level cache helper functions shared by builders
  prediction_trajectory_sampler.h / .cpp         # public sampling, player lookup, and segment-slice helpers
  prediction_frame_cache_builder.h / .cpp        # display-frame cache rebuild entry point
  prediction_metrics_builder.h / .cpp            # orbital metrics/analysis cache rebuild entry point
  streamed_chunk_assembly_builder.h / .cpp       # streamed/published chunk assembly entry point
  gameplay_prediction_derived_service.h / .cpp   # background-threaded derived cache builder
  prediction_system.h / .cpp                     # facade over runtime, invalidation, solver, and derived services
  gameplay_state_prediction.cpp                  # world-state resolution for prediction subjects
  gameplay_state_prediction_frames.cpp           # display-frame selection, frame rebuild, analysis spec
  draw/
    gameplay_state_prediction_draw_internal.h     # shared draw/pick context structs
    gameplay_state_prediction_draw.cpp            # top-level draw entry point
    gameplay_state_prediction_draw_helpers.cpp    # draw utility functions
    gameplay_state_prediction_draw_render_helpers.cpp  # render-specific draw helpers
    gameplay_state_prediction_draw_shared.cpp     # shared draw logic (base + planned)
    gameplay_state_prediction_draw_track.cpp      # per-track orbit drawing
    gameplay_state_prediction_draw_pick.cpp       # orbit line picking (hit-test)
    gameplay_state_prediction_draw_prepare.cpp    # pre-draw preparation (LOD, frustum, pick cache)
  runtime/
    gameplay_state_prediction_runtime_internal.h  # inline runtime helpers (drag state, generation checks)
    gameplay_state_prediction_runtime.cpp         # top-level runtime update entry point
    gameplay_state_prediction_runtime_requests.cpp  # prediction request submission
    gameplay_state_prediction_runtime_results.cpp   # solver/derived result polling and cache application
    gameplay_state_prediction_runtime_solver.cpp    # solver request building and dispatch
```

## What Lives Here

### Headers

- `gameplay_state_prediction_types.h`
  All shared data types for the prediction subsystem: `PredictionSubjectKey`, `OrbitPredictionCache` (inertial + display-frame trajectory data, render curves, orbital metrics), `PredictionLinePickCache`, `PredictionTrackState`, `PredictionFrameSelectionState`, `PredictionAnalysisSelectionState`, `PredictionDragDebugTelemetry`, `OrbitPredictionDrawConfig`, `OrbitPredictionDrawPalette`, `OrbitPlotPerfStats`, and `PredictionGroup`/`PredictionSelectionState` for multi-track overlays.

- `gameplay_prediction_cache_internal.h`
  Low-level helpers used by the derived cache builders and frame rebuild code:
  - Frame transform options: `build_frame_segment_transform_options`
  - Diagnostics: `update_derived_diagnostics`
  - Utilities: `collect_maneuver_node_times`, `sample_prediction_segments`

- `prediction_trajectory_sampler.h`
  `PredictionTrajectorySampler` -- public entry point for trajectory sampling, player lookup lambda creation, and cursor-based segment slicing used by gameplay runtime code.

- `prediction_frame_cache_builder.h`
  `PredictionFrameCacheBuilder` -- named entry point for transforming solver-frame trajectory data into the selected display frame. Wraps full-frame and planned-only rebuild paths.

- `prediction_metrics_builder.h`
  `PredictionMetricsBuilder` -- named entry point for rebuilding analysis samples and orbital HUD metrics from solver/display cache data.

- `streamed_chunk_assembly_builder.h`
  `StreamedChunkAssemblyBuilder` -- named entry point for turning solver published/streamed chunk metadata into display-frame chunk overlays and flattened planned display caches.

- `gameplay_prediction_derived_service.h`
  `OrbitPredictionDerivedService` -- a background-threaded worker that takes solver results from `OrbitPredictionService` and builds display-frame caches (frame transforms, resampling, render curves, orbital metrics). Supports per-track generation-based staleness detection and request coalescing, similar to the solver service.

- `prediction_system.h`
  `PredictionSystem` -- a thin facade owned by `GameplayState` that keeps prediction runtime orchestration, invalidation, and solver/derived service access behind one boundary while `GameplayPredictionState` remains the compatibility state container.

### Implementation Files

- `gameplay_state_prediction.cpp`
  World-state resolution: `get_player_world_state`, `get_orbiter_world_state`, `get_prediction_subject_world_state`. Resolves live entity positions/velocities from either the orbit sim (on-rails) or the physics body (off-rails).

- `gameplay_state_prediction_frames.cpp`
  Display-frame management: frame-spec selection, display-frame option building, frame rebuild triggers, analysis body resolution, and helper functions like `find_celestial_body_info` and `find_massive_body`. Drives `PredictionFrameCacheBuilder` when the user switches display frames.

- `gameplay_prediction_derived_service.cpp`
  `OrbitPredictionDerivedService` lifecycle (constructor spawns 1-2 worker threads, destructor joins), `request()` with per-track coalescing, `poll_completed()`, `reset()`, `build_cache()` orchestration, and `worker_loop`. Actual frame, metrics, and chunk cache work is delegated to the named builder entry points.

### `draw/` Subfolder

Drawing and picking implementation split files. Not included outside the prediction module.

- `gameplay_state_prediction_draw_internal.h`
  Shared context structs: `OrbitDrawWindowContext` (LOD camera, frustum, frame-to-world transform), `PickWindow`, `PredictionGlobalDrawContext`.

- `gameplay_state_prediction_draw.cpp`
  Top-level draw entry point called from `GameplayState::on_draw_ui()`.

- `gameplay_state_prediction_draw_helpers.cpp`
  General-purpose draw utility functions.

- `gameplay_state_prediction_draw_render_helpers.cpp`
  Render-specific helpers (line submission, dash patterns, alpha fading).

- `gameplay_state_prediction_draw_shared.cpp`
  Shared draw logic for both base and planned trajectories.

- `gameplay_state_prediction_draw_track.cpp`
  Per-track orbit line drawing (base orbit, planned orbit, overlay tracks).

- `gameplay_state_prediction_draw_pick.cpp`
  Orbit line picking: pick cache construction, frustum culling, and hit-test against rendered orbit lines.

- `gameplay_state_prediction_draw_prepare.cpp`
  Pre-draw preparation: LOD context setup, frustum computation, pick cache rebuild, and render curve resolution.

### `runtime/` Subfolder

Runtime orchestration split files. Not included outside the prediction module.

- `gameplay_state_prediction_runtime_internal.h`
  Inline helpers: `maneuver_drag_active`, `elapsed_ms`, `update_last_and_peak`, `visible_generation_id`, `latest_solver_generation_published`, and lifecycle classification helpers layered over the current track flags/overlays.

- `gameplay_state_prediction_runtime.cpp`
  Top-level runtime update entry point called from `GameplayState::on_update()` / `on_fixed_update()`.

- `gameplay_state_prediction_runtime_requests.cpp`
  Prediction request submission: builds solver requests from current world state and submits them to `OrbitPredictionService`, then chains derived requests to `OrbitPredictionDerivedService`.

- `gameplay_state_prediction_runtime_results.cpp`
  Result polling: polls `OrbitPredictionService` and `OrbitPredictionDerivedService` for completed results, applies them to `PredictionTrackState` caches, and updates drag debug telemetry.

- `gameplay_state_prediction_runtime_solver.cpp`
  Solver request building: assembles `OrbitPredictionService::Request` structs with integrator config, maneuver nodes, and ephemeris state.

## Pipeline Overview

The gameplay prediction display pipeline has three layers:

1. **Solver** (background thread, in `game/orbit/`)
   `OrbitPredictionService` propagates inertial trajectories and produces Hermite-based segments + maneuver previews.

2. **Derived Service** (background thread, this folder)
   `OrbitPredictionDerivedService` takes solver results, transforms them into the selected display frame (BCI, Synodic, LVLH, etc.), rebuilds render curves and orbital metrics.

3. **Draw** (main thread, per-frame)
   Prepares LOD/frustum context, resolves adaptive render curves, submits orbit lines to the plot system, and builds pick caches for orbit line hit-testing.

## How It Is Called

The entry points are declared on `GameplayState` in:

- `src/game/states/gameplay/gameplay_state.h`

The feature is driven from `GameplayState` like this:

- `on_update()` / `on_fixed_update()`
  Submits prediction requests (runtime), polls solver and derived results, applies caches to track state.

- `on_draw_ui()`
  Prepares draw context, draws orbit lines per-track, and processes picking.

## If You Want To Change...

- What data a prediction track stores, draw palette, or telemetry fields:
  Start in `gameplay_state_prediction_types.h`.

- Hermite sampling, frame cache rebuild logic, or orbital metrics computation:
  Start in `prediction_frame_cache_builder.h/.cpp`, `prediction_metrics_builder.h/.cpp`, or `streamed_chunk_assembly_builder.h/.cpp`. Drop to `gameplay_prediction_cache_internal.h/.cpp` for the low-level sampling and transform helpers.

- Derived service threading, request coalescing, or cache build workflow:
  Start in `gameplay_prediction_derived_service.h/.cpp`.

- How world-state is resolved for prediction subjects (orbiters, celestials):
  Start in `gameplay_state_prediction.cpp`.

- Display-frame selection, frame option menus, or analysis body resolution:
  Start in `gameplay_state_prediction_frames.cpp`.

- Top-level draw entry point or draw orchestration:
  Start in `draw/gameplay_state_prediction_draw.cpp`.

- Per-track orbit line rendering or overlay drawing:
  Start in `draw/gameplay_state_prediction_draw_track.cpp`.

- Orbit line picking or pick cache:
  Start in `draw/gameplay_state_prediction_draw_pick.cpp`.

- Pre-draw LOD setup, frustum context, or render curve resolution:
  Start in `draw/gameplay_state_prediction_draw_prepare.cpp`.

- Draw utility functions or dash patterns:
  Start in `draw/gameplay_state_prediction_draw_helpers.cpp` or `draw/gameplay_state_prediction_draw_render_helpers.cpp`.

- Runtime update orchestration:
  Start in `runtime/gameplay_state_prediction_runtime.cpp`.

- Prediction request submission or derived request chaining:
  Start in `runtime/gameplay_state_prediction_runtime_requests.cpp`.

- Result polling or cache application:
  Start in `runtime/gameplay_state_prediction_runtime_results.cpp`.

- Solver request assembly (integrator config, maneuver nodes):
  Start in `runtime/gameplay_state_prediction_runtime_solver.cpp`.

- Runtime inline helpers (drag state, generation checks):
  Start in `runtime/gameplay_state_prediction_runtime_internal.h`.

## Notes About Structure

- Each `.cpp` file is an independent translation unit.
- Files in `draw/` and `runtime/` are internal implementation details -- they are not included outside the prediction module.
- The derived service runs on background threads (1-2 workers); all draw and frame files are main-thread only.
- Prediction data flows as: solver inertial segments -> derived frame transform -> display-frame segments + samples -> render curves -> orbit line submission.
- The derived service uses generation-based staleness detection: stale results are discarded before publishing, and newer requests supersede older ones for the same track.
- Display-frame transforms support Inertial (passthrough), BCI, Synodic, and LVLH frame types with adaptive re-segmentation for non-inertial frames.
- `OrbitPredictionCache` holds both base (no-maneuver) and planned (with-maneuver) trajectory data side by side.
