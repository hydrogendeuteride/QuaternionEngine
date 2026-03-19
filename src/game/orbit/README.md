# Orbit Folder Guide

This folder contains the orbit prediction, LOD tessellation, and rendering pipeline used by `GameplayState`.

## What Lives Here

- `orbit_prediction_tuning.h`
  Compile-time tuning constants for every stage of the orbit prediction pipeline.
  Horizon limits, sample budgets, multi-band density settings, integrator step caps, ephemeris cadence, thrust-mode overrides, and maneuver-gizmo rebuild rate all live here.

- `orbit_prediction_math.h / .cpp`
  Pure math helpers for orbital mechanics.
  `OrbitalElementsEstimate` struct (SMA, eccentricity, period, periapsis, apoapsis), `estimate_orbital_period_s()`, `compute_orbital_elements()`, `select_prediction_horizon_and_dt()`, and `hermite_position_world()` (cubic Hermite interpolation on trajectory samples).

- `orbit_prediction_service.h / .cpp`
  Background-threaded prediction worker.
  `OrbitPredictionService` owns a thread pool that consumes `Request` jobs (spacecraft or celestial), propagates trajectories via `orbitsim`, applies maneuver impulses, builds/caches celestial ephemerides, and publishes `Result` structs with inertial trajectory samples and segments. Supports generation-based staleness detection and per-track request coalescing.

- `orbit_plot_util.h / .cpp`
  Low-level plot utilities shared by LOD builders and render curves.
  `eval_segment_local_position()` evaluates a Hermite-interpolated position within a `TrajectorySegment`, and `meters_per_px_at_world()` computes the world-space size of a screen pixel at a given point (used for pixel-error LOD decisions).

- `orbit_render_curve.h / .cpp`
  Binary-tree LOD structure over trajectory segments.
  `OrbitRenderCurve::build()` constructs a balanced binary tree where each internal node merges a range of source segments into a single coarser Hermite segment and records its worst-case approximation error. The runtime path uses `build_render_lod()` / `build_pick_lod()` as the main entry points: they descend the tree, honor anchor times, and produce final render or pick line data, including final subdivision and frustum-aware filtering.

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

- Per-segment Hermite evaluation or pixel-size computation:
  Start in `orbit_plot_util.h/.cpp`.

- Adaptive subdivision for rendering, frustum culling, or picking decimation:
  Start in `orbit_render_curve.h/.cpp`.

- Binary-tree LOD structure, segment merging, or error-driven tree traversal:
  Start in `orbit_render_curve.h/.cpp`.

## Notes About Structure

- `orbit_prediction_service` runs on background threads; all other files are main-thread only.
- Trajectory data flows as `orbitsim::TrajectorySegment` (Hermite spline segments with start/end position+velocity and time span).
- LOD decisions are pixel-error-driven: the chord-vs-curve midpoint deviation is measured in screen pixels, so near geometry is automatically refined more than distant geometry.
- `OrbitRenderCurve` owns both the persistent hierarchy and the final line-generation logic used by the gameplay orbit overlay.
