# Orbit Folder Guide

This folder contains the orbit prediction, LOD tessellation, and rendering pipeline used by `GameplayState`.

## Folder Structure

```
orbit/
  orbit_prediction_tuning.h          # tuning constants
  orbit_prediction_math.h / .cpp     # orbital mechanics math
  orbit_prediction_service.h / .cpp  # prediction service public API, lifecycle, cache, threading
  trajectory/
    trajectory_utils.h / .cpp         # shared trajectory segment utilities
  orbit_render_curve.h               # LOD tree public API
  orbit_plot_util.h / .cpp           # shared plot helpers
  prediction/
    prediction_diagnostics_util.h                   # shared diagnostics builders
    orbit_prediction_service_internal.h              # shared types, context, declarations
    orbit_prediction_service_compute.cpp             # compute_prediction entry, job setup, route dispatch
    orbit_prediction_service_route_solvers.cpp       # ephemeris/celestial/baseline route solvers
    orbit_prediction_service_spacecraft_route.cpp    # spacecraft route orchestration
    orbit_prediction_service_planned_route.cpp       # planned route orchestration + staged publishing
    orbit_prediction_service_planned_stage_publisher.cpp # planned publish chunk/result assembly
    orbit_prediction_service_planned.cpp             # planned trajectory solving (maneuvers)
    orbit_prediction_service_trajectory.cpp          # segment math (Hermite eval, boundary split)
    orbit_prediction_service_sampling.cpp            # uniform resampling
    orbit_prediction_service_policy_integrator.cpp   # integrator profile configuration
    orbit_prediction_service_policy_chunking.cpp     # time-band chunk planning
    orbit_prediction_service_policy_profile.cpp      # activity classification + profile resolution
    orbit_prediction_service_policy_adaptive.cpp     # adaptive segment/ephemeris options
    orbit_prediction_service_policy_ephemeris.cpp    # ephemeris compatibility + build
  render_curve/
    orbit_render_curve_internal.h     # inline helpers (frustum, eval)
    orbit_render_curve.cpp            # tree construction + LOD select
    orbit_render_curve_render.cpp     # adaptive render subdivision
    orbit_render_curve_pick.cpp       # frustum cull + pick downsample
```

## What Lives Here

### Prediction (public headers in `orbit/`)

- `orbit_prediction_tuning.h`
  Compile-time tuning constants for every stage of the orbit prediction pipeline.
  Horizon limits, sample budgets, multi-band density settings, integrator step caps, ephemeris cadence, thrust-mode overrides, and maneuver-gizmo rebuild rate all live here.

- `orbit_prediction_math.h / .cpp`
  Pure math helpers for orbital mechanics.
  `OrbitalElementsEstimate` struct (SMA, eccentricity, period, periapsis, apoapsis), `estimate_orbital_period_s()`, `compute_orbital_elements()`, `select_prediction_horizon_and_dt()`, and sample-pair trajectory evaluation helpers.

- `orbit_prediction_service.h / .cpp`
  Background-threaded prediction worker.
  `OrbitPredictionService` owns a thread pool that consumes `Request` jobs (spacecraft or celestial), propagates trajectories via `orbitsim`, applies maneuver impulses, builds/caches celestial ephemerides, and publishes `Result` structs with inertial trajectory samples and segments. Supports generation-based staleness detection and per-track request coalescing.
  The `.cpp` contains lifecycle (constructor, destructor, `request`, `poll_completed`, `reset`), threading (`worker_loop`), and caching (`get_or_build_ephemeris`, baseline cache). The compute and planned trajectory logic live in `prediction/`.

- `trajectory/trajectory_utils.h / .cpp`
  Public orbit-module trajectory helpers shared by prediction service and gameplay-derived caches.
  Contains finite-state checks, trajectory continuity validation, segment end/span helpers, Hermite segment evaluation, boundary-aware segment sampling, segment slicing, and uniform segment resampling.

### `prediction/` subfolder

Internal helpers split out from `orbit_prediction_service.cpp`.

- `orbit_prediction_service_internal.h`
  Shared service-only types (`PlannedSegmentBoundaryState`, `CelestialPredictionSamplingSpec`, `PlannedTrajectoryContext`, `PlannedChunkPacket`, `PlannedSolveOutput`, etc.), constants, and template functions (`select_primary_index_with_hysteresis`). Declares service implementation functions used across the `.cpp` files below.

- `prediction_diagnostics_util.h`
  Diagnostics builders shared by orbit prediction and gameplay-derived cache code.

- `orbit_prediction_service_compute.cpp`
  `compute_prediction()` entry point and job orchestration helpers. Owns worker-job setup, cancellation/publish handling, route service adapter wiring, route dispatch, and final result publishing.

- `orbit_prediction_service_route_solvers.cpp`
  Route-level solver helpers for ephemeris resolution, celestial body prediction, and spacecraft baseline trajectory prediction.

- `orbit_prediction_service_spacecraft_route.cpp`
  Spacecraft route orchestration. Builds the transient spacecraft, resolves ephemeris data, applies reusable baseline cache data, dispatches planned maneuver solving, and stores reusable baseline results.

- `orbit_prediction_service_planned_route.cpp`
  Planned route orchestration for maneuver-bearing requests. Handles staged preview, full solve, and planned suffix refine decisions around `solve_planned_chunk_range()`.

- `orbit_prediction_service_planned_stage_publisher.cpp`
  Planned publish assembly helpers. Builds `PublishedChunk` metadata, staged `Result` payloads, cached prefix stream chunks, and full-stream publish batches.

- `orbit_prediction_service_planned.cpp`
  Planned trajectory solving for maneuver-bearing predictions. Contains `solve_planned_chunk_range()` (chunk-by-chunk adaptive solving with seam validation), chunk cache operations, and maneuver impulse application.

- `orbit_prediction_service_trajectory.cpp`
  Service-specific trajectory helpers: maneuver preview building, celestial ephemeris-to-segment conversion, and planned boundary splitting. Shared segment math lives in `trajectory/trajectory_utils.*`.

- `orbit_prediction_service_sampling.cpp`
  Uniform resampling of trajectory segments and ephemeris data.

- `orbit_prediction_service_policy_integrator.cpp`
  Integrator profile configuration: Lagrange and standard prediction integrator parameter application.

- `orbit_prediction_service_policy_chunking.cpp`
  Time-band chunk planning: boundary generation, merging, and `build_prediction_solve_plan()` that partitions the prediction window into prioritized chunks.

- `orbit_prediction_service_policy_profile.cpp`
  Activity classification (`classify_chunk_activity()`) and profile resolution (`resolve_prediction_profile_definition()`). Determines numerical accuracy level per chunk based on heading changes, jerk, gravity dominance, and SOI transitions.

- `orbit_prediction_service_policy_adaptive.cpp`
  Adaptive segment and ephemeris option builders: tolerance ramps, timestep limits, and per-chunk option scaling based on profile definitions.

- `orbit_prediction_service_policy_ephemeris.cpp`
  Ephemeris compatibility checking, build request assembly, ephemeris construction from simulation, and celestial prediction sampling spec derivation.

### Rendering (public header in `orbit/`)

- `orbit_plot_util.h / .cpp`
  Low-level plot utilities shared by LOD builders and render curves.
  `eval_segment_local_position()` evaluates a Hermite-interpolated position within a `TrajectorySegment`, and `meters_per_px_at_world()` computes the world-space size of a screen pixel at a given point (used for pixel-error LOD decisions).

- `orbit_render_curve.h`
  Public interface for orbit curve LOD and final line generation. `build()` owns the persistent binary-tree LOD hierarchy; `select_segments()` / `resolve_curve_segments()` select camera-appropriate Hermite segments from that tree; the span-based `build_render_lod()` and `build_pick_lod()` overloads convert already-selected flat segments into final render or pick line segments.

### `render_curve/` subfolder

Internal implementation split files for `OrbitRenderCurve`. Not included outside the orbit module.

- `orbit_render_curve_internal.h`
  Shared inline helpers: `segment_end_time`, `eval_segment_world_position`, and frustum containment tests (`frustum_contains_point_margin`, `frustum_accept_segment_margin`).

- `orbit_render_curve.cpp`
  Tree construction and stage-1 LOD selection. `build()` constructs a balanced binary tree where each internal node merges a range of source Hermite segments and records worst-case approximation error. `select_segments()` walks the tree with pixel-error-driven descent. `resolve_curve_segments()` is the shared select+transform entry point for both render and pick paths; it does not create final draw-line tessellation.

- `orbit_render_curve_render.cpp`
  Stage-2 adaptive subdivision for rendering. The span-based `build_render_lod()` bisects each already-selected segment until chord-vs-curve midpoint error falls below a screen-pixel threshold, using an explicit stack. Segments outside the frustum are accepted without further subdivision. The curve-based overload is a convenience pipeline that runs tree selection first and then delegates here.

- `orbit_render_curve_pick.cpp`
  Final line generation for picking. The span-based `build_pick_lod()` converts already-selected segments to world-space lines, frustum-culls, then uniformly downsamples if over budget -- preserving endpoints and anchor-time segments. The curve-based overload runs tree selection first and then delegates here.

## Pipeline Overview

The orbit visualization pipeline has four layers:

1. **Prediction** (background thread)
   `OrbitPredictionService` propagates spacecraft and celestial trajectories on a worker thread. Results include inertial trajectory samples, Hermite-based trajectory segments, and maneuver-node previews.

2. **Tree LOD Selection** (main thread, per-frame)
   `OrbitRenderCurve::select_segments()` walks the binary tree and chooses a camera-appropriate set of Hermite curve segments. This stage decides which source ranges can be represented by coarser merged curves; it does not directly define the final line segments sent to drawing or picking.

3. **Final Line Generation** (main thread, per-frame)
   Render paths run an additional screen-space midpoint-error subdivision pass over the selected curves. Pick paths convert selected curves to lines, apply frustum culling, and downsample if needed while preserving endpoints and anchor-time boundaries.

4. **Rendering**
   The final world-space line segments are submitted to the debug-draw / line rendering system for display.

## How It Is Called

The entry point is `GameplayState`, which:

- Submits prediction requests via `OrbitPredictionService::request()` each frame.
- Polls completed results via `OrbitPredictionService::poll_completed()`.
- Feeds the resulting trajectory segments into `OrbitRenderCurve` to produce adaptive render and pick line data.
- Draws the output segments as orbit lines.

## If You Want To Change...

- Prediction horizon, sample budgets, integrator precision, or any numeric tuning:
  Start in `orbit_prediction_tuning.h`.

- Orbital element computation, period estimation, or Hermite interpolation:
  Start in `orbit_prediction_math.h/.cpp`.

- Worker threading, request queuing, ephemeris caching, or baseline cache:
  Start in `orbit_prediction_service.h/.cpp`.

- Compute entry point, celestial prediction, or spacecraft baseline:
  Start in `prediction/orbit_prediction_service_compute.cpp`.

- Maneuver impulse application, planned chunk solving, seam validation, or chunk streaming:
  Start in `prediction/orbit_prediction_service_planned.cpp`.

- Trajectory segment Hermite evaluation or segment slicing:
  Start in `trajectory/trajectory_utils.h/.cpp`.

- Planned boundary splitting:
  Start in `prediction/orbit_prediction_service_trajectory.cpp`.

- Trajectory or ephemeris resampling:
  Start in `prediction/orbit_prediction_service_sampling.cpp`.

- Integrator profiles:
  Start in `prediction/orbit_prediction_service_policy_integrator.cpp`.

- Chunk planning or time-band boundaries:
  Start in `prediction/orbit_prediction_service_policy_chunking.cpp`.

- Activity classification or profile resolution:
  Start in `prediction/orbit_prediction_service_policy_profile.cpp`.

- Adaptive segment/ephemeris options or tolerance ramps:
  Start in `prediction/orbit_prediction_service_policy_adaptive.cpp`.

- Ephemeris compatibility, build requests, or celestial sampling spec:
  Start in `prediction/orbit_prediction_service_policy_ephemeris.cpp`.

- Per-segment Hermite evaluation or pixel-size computation:
  Start in `orbit_plot_util.h/.cpp`.

- Binary-tree LOD structure, segment merging, or stage-1 tree traversal:
  Start in `render_curve/orbit_render_curve.cpp`.

- Stage-2 adaptive subdivision for rendering (pixel-error threshold, max depth):
  Start in `render_curve/orbit_render_curve_render.cpp`.

- Final pick line generation, frustum culling, picking decimation, or anchor-time preservation:
  Start in `render_curve/orbit_render_curve_pick.cpp`.

## Notes About Structure

- `orbit_prediction_service` runs on background threads; all other files are main-thread only.
- Trajectory data flows as `orbitsim::TrajectorySegment` (Hermite spline segments with start/end position+velocity and time span).
- Both tree selection and render subdivision use pixel-error decisions, but at different levels: tree selection chooses coarser or finer Hermite curve segments from the persistent hierarchy, while render subdivision tessellates those selected curves into final draw lines.
- `OrbitRenderCurve` owns both the persistent hierarchy and the final line-generation logic used by the gameplay orbit overlay; keep those responsibilities distinct when profiling or changing budgets.
- Files in `prediction/` and `render_curve/` are internal implementation details -- they are not included outside the orbit module and should not expose any public API.
