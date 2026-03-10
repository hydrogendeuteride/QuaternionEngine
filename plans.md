# Prediction Frame Follow-Up Plan

## Goal

Extend gameplay prediction so rotating frames such as synodic and LVLH can be selected without breaking orbit rendering, maneuver gizmos, or derived analysis values.

## Current State

- Prediction results are stored canonically in the orbitsim inertial frame.
- Gameplay rebuilds display-frame trajectories locally from inertial samples.
- World-space orbit lines currently assume the selected frame can be rendered as:
  `world_point = frame_origin_world + frame_position`
- That assumption is valid for inertial and body-centered inertial, but not for rotating frames such as synodic, body-fixed, or LVLH.

## Phase 1: Separate Display And Analysis Frames

1. Add distinct selection state for:
   - display frame: how orbit lines and node markers are shown
   - analysis frame: how altitude, speed, Pe/Ap, and period are computed
2. Keep analysis defaults conservative:
   - spacecraft: nearest useful primary body in BCI
   - celestial subjects: parent-body BCI when available
3. Update HUD labels so users can see which frame is driving the plotted path versus orbital metrics.

## Phase 2: Introduce Frame-To-World Rendering Support

1. Add a helper that converts a frame-space sample back into gameplay world coordinates.
2. For rotating frames, use the full frame basis, not just frame origin translation.
3. Refactor orbit plot point generation, picking data, and maneuver gizmo placement to go through the same helper.
4. Keep inertial and BCI on the fast path so existing behavior stays cheap.

## Phase 3: Expand Frame Options

1. Add selectable `Synodic` frame options for meaningful body pairs:
   - Earth-Moon
   - parent-child celestial pairs discovered from scenario data
2. Add `LVLH` frame options for:
   - active player spacecraft
   - tracked prediction subject when it is an orbiter
3. Only surface options when required data exists:
   - synodic needs two valid bodies
   - LVLH needs a target spacecraft lookup

## Phase 4: Maneuver Integration

1. Ensure node previews can be transformed into any selected display frame.
2. Keep maneuver authoring semantics explicit:
   - display frame controls visualization
   - node RTN primary body controls burn basis
3. Consider optional node UI actions:
   - "bind node primary to current analysis body"
   - "retarget all nodes to current analysis body"

## Phase 5: Caching And Performance

1. Cache frame-converted trajectories by frame spec to avoid rebuilding them every UI change and gizmo refresh.
2. Reuse cached transformed trajectories for:
   - orbit line emission
   - point picking
   - maneuver node preview positioning
3. Invalidate only when source inertial prediction or relevant ephemeris changes.

## Validation

1. Manual:
   - switch between inertial, BCI, synodic, and LVLH while observing path continuity
   - verify node markers stay on the displayed curve
   - verify HUD orbital metrics remain stable when only display frame changes
2. Automated in `third_party/orbitsim` where possible:
   - frame transform round-trip tests
   - synodic transform regression tests
   - LVLH trajectory transform tests
3. Gameplay-side tests if/when a test target exists:
   - cache invalidation for frame changes
   - frame option generation from scenario data

## Risks

- Rotating frames may expose mismatches between frame-space rendering and gameplay world anchoring.
- LVLH options need a robust spacecraft-state lookup path for prediction-only subjects.
- Derived orbital elements can become misleading if analysis silently follows a rotating display frame.
