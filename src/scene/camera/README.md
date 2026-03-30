# Camera Folder Guide

This folder contains the camera rig and its pluggable camera modes, used by `SceneManager` to control viewpoint behavior.

## Folder Structure

```
camera/
  icamera_mode.h       # abstract interface for all camera modes
  camera_rig.h / .cpp  # mode manager, settings storage, target resolution, terrain clamping
  mode_free.h / .cpp   # 6DOF free-flight camera (WASD + mouse look)
  mode_orbit.h / .cpp  # orbit camera around a target (yaw/pitch/distance)
  mode_follow.h / .cpp # rigid follow: locks to target with fixed local offset
  mode_chase.h / .cpp  # smoothed chase: follows target with exponential lag
  mode_fixed.h / .cpp  # fixed camera: no input, no movement
```

## What Lives Here

### Headers

- `icamera_mode.h`
  Abstract `ICameraMode` interface. Declares the three virtual methods every mode must implement: `on_activate`, `process_input`, and `update`. Forward-declares `Camera`, `InputSystem`, `SceneManager`.

- `camera_rig.h`
  `CameraRig` class -- the central mode manager. Owns the active `ICameraMode` instance and settings structs for all five modes: `FreeCameraSettings`, `OrbitCameraSettings`, `FollowCameraSettings`, `ChaseCameraSettings`, `FixedCameraSettings`. Also defines `CameraMode` enum, `CameraTargetType` enum, and `CameraTarget` struct for target-based modes. Exposes `resolve_target` (resolves `CameraTarget` to world position/rotation via `SceneManager` or `PlanetSystem`), terrain surface clamping controls, and orbit reference-up helpers.

- `mode_free.h`
  `FreeCameraMode` -- 6DOF free-flight. Tracks RMB state, velocity vector, and roll direction.

- `mode_orbit.h`
  `OrbitCameraMode` -- orbits around a `CameraTarget` at a configurable distance. Tracks RMB state for mouse-drag rotation.

- `mode_follow.h`
  `FollowCameraMode` -- rigidly attaches to a target with a fixed local-space position and rotation offset.

- `mode_chase.h`
  `ChaseCameraMode` -- smoothed version of follow. Uses exponential decay (`position_lag`, `rotation_lag`) to interpolate toward the target pose.

- `mode_fixed.h`
  `FixedCameraMode` -- no-op mode. Camera stays wherever it was placed.

### Implementation Files

- `camera_rig.cpp`
  Mode lifecycle: `init`, `set_mode`, `recreate_mode` (factory via `make_mode`). Input/update dispatch to the active mode. `resolve_target` resolves `WorldPoint`, `MeshInstance`, and `GLTFInstance` target types via `SceneManager` lookups (falls back to `PlanetSystem` for mesh targets). Terrain surface clamping via `clamp_camera_above_terrain` (iterates planet bodies, samples terrain displacement, pushes camera above surface). Orbit reference-up helpers: `align_orbit_up_to_target`, `set_orbit_reference_up`.

- `mode_free.cpp`
  Input: WASD movement, Q/E roll, RMB+mouse for yaw/pitch (6DOF quaternion-based), scroll wheel for move speed (exponential scaling) or FOV (Ctrl+scroll). Update: applies roll and translates camera in local space.

- `mode_orbit.cpp`
  `on_activate` derives yaw/pitch/distance from current camera-to-target vector to avoid snap. Input: RMB+drag for yaw/pitch, scroll for distance (exponential) or FOV (Ctrl+scroll). Update: builds orbit frame from `reference_up`, applies yaw/pitch rotations, positions camera at `target + backward * distance`. Uses `orientation_from_backward_and_up` to compute camera quaternion.

- `mode_follow.cpp`
  `on_activate` captures current camera-to-target offset in target-local space. Update: transforms offset back to world space each frame -- rigid attachment with no smoothing.

- `mode_chase.cpp`
  `on_activate` captures offset like follow mode. Update: computes desired pose, then applies exponential smoothing (`1 - exp(-lag * dt)`) for both position (lerp) and rotation (slerp).

- `mode_fixed.cpp`
  All methods are no-ops. Camera retains its current pose indefinitely.

## How It Is Called

`CameraRig` is owned by `SceneManager` (declared in `src/scene/vk_scene.h`). The call sites are:

- `SceneManager` initialization calls `CameraRig::init()` to set up the default mode.
- Per-frame, the engine calls `CameraRig::process_input()` with the current `InputSystem` state, then `CameraRig::update()` with the frame delta time.
- Game states (e.g., `GameplayState`) switch modes via `CameraRig::set_mode()` and configure target-based modes by writing to the settings structs (`orbit_settings()`, `chase_settings()`, etc.).
- `CameraRig::resolve_target()` is also called internally by mode implementations (Orbit, Follow, Chase) during their `update` to resolve the current target world position.

## If You Want To Change...

- The set of available camera modes or the mode enum:
  Start in `camera_rig.h` (`CameraMode` enum) and the `make_mode` factory in `camera_rig.cpp`.

- Free camera movement speed, sensitivity, or input bindings:
  Start in `mode_free.cpp`.

- Orbit distance limits, orbit plane computation, or reference-up behavior:
  Start in `mode_orbit.cpp` and `OrbitCameraSettings` in `camera_rig.h`.

- Follow/chase offset capture or chase smoothing rates:
  Start in `mode_follow.cpp` or `mode_chase.cpp`.

- Target resolution (how `CameraTarget` maps to a world position):
  Start in `CameraRig::resolve_target()` in `camera_rig.cpp`.

- Terrain surface clamping behavior:
  Start in `clamp_camera_above_terrain` in `camera_rig.cpp`.

- Adding a new camera mode:
  Start in `icamera_mode.h` (implement the interface), add a settings struct and enum value in `camera_rig.h`, and wire it into `make_mode` in `camera_rig.cpp`.

## Notes About Structure

- Each `.cpp` file is an independent translation unit.
- All camera modes share the same `ICameraMode` interface; `CameraRig` owns the active mode as a `std::unique_ptr<ICameraMode>` and delegates input/update through it.
- Settings structs are owned by `CameraRig`, not the mode instances. Mode instances hold references to their settings, so settings persist across mode switches.
- Target-based modes (Orbit, Follow, Chase) call `CameraRig::resolve_target()` each frame to track moving targets (scene instances, planet bodies, or static world points).
- Orbit mode uses a custom orbit-plane basis built from `reference_up` rather than assuming Y-up, allowing it to work correctly in planetary contexts with arbitrary gravity directions.
- Terrain clamping runs after the active mode's `update`, so it applies uniformly regardless of which mode is active.
