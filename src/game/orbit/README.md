# Orbit Folder Guide

This folder contains the orbit prediction, LOD tessellation, and rendering pipeline used by `GameplayState`.

## Folder Structure

```
orbit/
  orbit_prediction_tuning.h          # tuning constants
  orbit_prediction_math.h / .cpp     # orbital mechanics math
  orbit_prediction_service.h / .cpp  # prediction service public API + class methods
  orbit_render_curve.h               # LOD tree public API
  orbit_plot_util.h / .cpp           # shared plot helpers
  prediction/
    orbit_prediction_service_internal.h     # shared types + declarations
    orbit_prediction_service_trajectory.cpp # segment math
    orbit_prediction_service_sampling.cpp   # multi-band sampling
    orbit_prediction_service_policy.cpp     # policy resolvers + ephemeris
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
  `OrbitalElementsEstimate` struct (SMA, eccentricity, period, periapsis, apoapsis), `estimate_orbital_period_s()`, `compute_orbital_elements()`, `select_prediction_horizon_and_dt()`, and `hermite_position_world()` (cubic Hermite interpolation on trajectory samples).

- `orbit_prediction_service.h / .cpp`
  Background-threaded prediction worker.
  `OrbitPredictionService` owns a thread pool that consumes `Request` jobs (spacecraft or celestial), propagates trajectories via `orbitsim`, applies maneuver impulses, builds/caches celestial ephemerides, and publishes `Result` structs with inertial trajectory samples and segments. Supports generation-based staleness detection and per-track request coalescing.
  The `.cpp` contains only the class method implementations; all internal helpers live in `prediction/`.

### `prediction/` subfolder

Internal helpers split out from `orbit_prediction_service.cpp`. Not included by anything outside the orbit module.

- `orbit_prediction_service_internal.h`
  Shared types (`PlannedSegmentBoundaryState`, `MultiBandSampleWindow`, `SpacecraftSamplingBudget`, `CelestialPredictionSamplingSpec`), constants, inline micro-helpers (`finite_vec3`, `finite_state`, `prediction_segment_end_time`), and template functions (`sample_multi_band`, `select_primary_index_with_hysteresis`). Also declares all functions implemented in the three `.cpp` files below.

- `orbit_prediction_service_trajectory.cpp`
  Trajectory segment math: Hermite interpolation (`eval_segment_state`), binary-search lookup (`sample_trajectory_segment_state`), segment construction from samples (`trajectory_segments_from_samples`), boundary splitting (`split_trajectory_segments_at_known_boundaries`), maneuver preview building, and planned boundary state merging.

- `orbit_prediction_service_sampling.cpp`
  Multi-band sampling system: density-tagged window construction from band templates (`build_base_band_windows`, `build_multi_band_sample_windows`), per-maneuver-node high-density window insertion, budget distribution across windows (`distribute_sample_budget`), and multi-band trajectory/ephemeris sampling.

- `orbit_prediction_service_policy.cpp`
  Prediction policy resolvers: long-range detection, horizon/dt/step caps, integrator profile application (Lagrange and standard), spacecraft sampling budgets, ephemeris compatibility checking, ephemeris construction from request, celestial prediction sampling spec, and ephemeris build request assembly.

### Rendering (public header in `orbit/`)

- `orbit_plot_util.h / .cpp`
  Low-level plot utilities shared by LOD builders and render curves.
  `eval_segment_local_position()` evaluates a Hermite-interpolated position within a `TrajectorySegment`, and `meters_per_px_at_world()` computes the world-space size of a screen pixel at a given point (used for pixel-error LOD decisions).

- `orbit_render_curve.h`
  Binary-tree LOD structure over trajectory segments. Public interface, types (`LineSegment`, `RenderResult`, `PickResult`, `Node`, `SelectionContext`), and entry points (`build`, `build_render_lod`, `build_pick_lod`).

### `render_curve/` subfolder

Internal implementation split files for `OrbitRenderCurve`. Not included outside the orbit module.

- `orbit_render_curve_internal.h`
  Shared inline helpers: `segment_end_time`, `eval_segment_world_position`, and frustum containment tests (`frustum_contains_point_margin`, `frustum_accept_segment_margin`).

- `orbit_render_curve.cpp`
  Tree construction and LOD selection. `build()` constructs a balanced binary tree where each internal node merges a range of source Hermite segments and records worst-case approximation error. `select_segments()` walks the tree with pixel-error-driven descent. `resolve_curve_segments()` is the shared select+transform entry point for both render and pick paths.

- `orbit_render_curve_render.cpp`
  Adaptive subdivision for rendering. `build_render_lod()` bisects each segment until chord-vs-curve midpoint error falls below a screen-pixel threshold, using an explicit stack. Segments outside the frustum are accepted without further subdivision.

- `orbit_render_curve_pick.cpp`
  Frustum culling and downsampling for picking. `build_pick_lod()` converts segments to world-space lines, frustum-culls, then uniformly downsamples if over budget -- preserving endpoints and anchor-time segments.

## Pipeline Overview

The orbit visualization pipeline has three layers:

1. **Prediction** (background thread)
   `OrbitPredictionService` propagates spacecraft and celestial trajectories on a worker thread. Results include inertial trajectory samples, Hermite-based trajectory segments, and maneuver-node previews.

2. **LOD Selection** (main thread, per-frame)
   `OrbitRenderCurve` selects the coarse trajectory representation for the current camera/window, then produces the final render or pick line data.

3. **Rendering**
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

- Background propagation, maneuver impulse application, ephemeris caching, or worker threading:
  Start in `orbit_prediction_service.h/.cpp`.

- Trajectory segment Hermite evaluation, boundary splitting, or sample-to-segment conversion:
  Start in `prediction/orbit_prediction_service_trajectory.cpp`.

- Multi-band sampling windows, density distribution, or per-maneuver-node sample density:
  Start in `prediction/orbit_prediction_service_sampling.cpp`.

- Long-range policy, integrator profiles, ephemeris compatibility, or celestial sampling spec:
  Start in `prediction/orbit_prediction_service_policy.cpp`.

- Per-segment Hermite evaluation or pixel-size computation:
  Start in `orbit_plot_util.h/.cpp`.

- Binary-tree LOD structure, segment merging, or error-driven tree traversal:
  Start in `render_curve/orbit_render_curve.cpp`.

- Adaptive subdivision for rendering (pixel-error threshold, max depth):
  Start in `render_curve/orbit_render_curve_render.cpp`.

- Frustum culling, picking decimation, or anchor-time preservation:
  Start in `render_curve/orbit_render_curve_pick.cpp`.

## Notes About Structure

- `orbit_prediction_service` runs on background threads; all other files are main-thread only.
- Trajectory data flows as `orbitsim::TrajectorySegment` (Hermite spline segments with start/end position+velocity and time span).
- LOD decisions are pixel-error-driven: the chord-vs-curve midpoint deviation is measured in screen pixels, so near geometry is automatically refined more than distant geometry.
- `OrbitRenderCurve` owns both the persistent hierarchy and the final line-generation logic used by the gameplay orbit overlay.
- Files in `prediction/` and `render_curve/` are internal implementation details -- they are not included outside the orbit module and should not expose any public API.
