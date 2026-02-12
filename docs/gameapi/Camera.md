# Camera System

Control camera position, rotation, FOV, and camera modes (free, orbit, follow, chase, fixed).

## Camera Modes

```cpp
enum class CameraMode : uint8_t
{
    Free = 0,     // Manual control (FPS-style)
    Orbit = 1,    // Orbit around target
    Follow = 2,   // Follow target with fixed offset
    Chase = 3,    // Follow target with smoothing
    Fixed = 4     // Fixed position and rotation
};
```

### Setting Camera Mode

```cpp
api.set_camera_mode(GameAPI::CameraMode::Free);
GameAPI::CameraMode mode = api.get_camera_mode();
```

## Position and Rotation

### Single Precision

```cpp
// Set position
api.set_camera_position(glm::vec3(0.0f, 2.0f, 10.0f));
glm::vec3 pos = api.get_camera_position();

// Set rotation (pitch and yaw in radians)
float pitch = glm::radians(-15.0f);  // Look down 15 degrees
float yaw = glm::radians(45.0f);     // Face northeast
api.set_camera_rotation(pitch, yaw);

// Query rotation
float pitch, yaw;
api.get_camera_rotation(pitch, yaw);
```

### Double Precision (Large Worlds)

```cpp
// Set position with double precision
api.set_camera_position(glm::dvec3(1000000.0, 100.0, 500000.0));
glm::dvec3 posD = api.get_camera_position_d();
```

Use double precision for large world coordinates (space, planetary, etc.).

### Look At

```cpp
// Point camera at target (single precision)
api.camera_look_at(glm::vec3(0.0f, 0.0f, 0.0f));

// Double precision
api.camera_look_at(glm::dvec3(1000000.0, 0.0, 0.0));
```

## Field of View

```cpp
// Set FOV in degrees
api.set_camera_fov(75.0f);
float fov = api.get_camera_fov();
```

## Free Camera Mode

Manual FPS-style camera control.

### Settings

```cpp
struct FreeCameraSettings
{
    float moveSpeed{1.8f};        // World units / second
    float lookSensitivity{0.0020f};
    float rollSpeed{1.0f};        // Radians / second
};

GameAPI::FreeCameraSettings settings;
settings.moveSpeed = 5.0f;
settings.lookSensitivity = 0.0015f;
api.set_free_camera_settings(settings);

GameAPI::FreeCameraSettings current = api.get_free_camera_settings();
```

### Typical Usage

```cpp
api.set_camera_mode(GameAPI::CameraMode::Free);

// In game loop (handled by engine InputSystem)
// WASD moves camera, mouse rotates view
```

## Orbit Camera Mode

Orbit around a target with mouse-driven pitch/yaw.

### Settings

```cpp
struct CameraTarget
{
    CameraTargetType type{CameraTargetType::None};
    std::string name{};
    glm::dvec3 worldPoint{0.0, 0.0, 0.0};
};

enum class CameraTargetType : uint8_t
{
    None = 0,
    WorldPoint = 1,
    MeshInstance = 2,
    GLTFInstance = 3
};

struct OrbitCameraSettings
{
    CameraTarget target{};
    double distance{10.0};
    float yaw{0.0f};              // Radians
    float pitch{0.0f};            // Radians
    float lookSensitivity{0.0020f};
};
```

### Example

```cpp
// Orbit around world point
GameAPI::OrbitCameraSettings orbit;
orbit.target.type = GameAPI::CameraTargetType::WorldPoint;
orbit.target.worldPoint = glm::dvec3(0.0, 0.0, 0.0);
orbit.distance = 15.0;
orbit.yaw = glm::radians(45.0f);
orbit.pitch = glm::radians(-20.0f);

api.set_orbit_camera_settings(orbit);
api.set_camera_mode(GameAPI::CameraMode::Orbit);
```

### Target from Last Pick

```cpp
// Orbit around last selected object
if (api.set_camera_target_from_last_pick()) {
    api.set_camera_mode(GameAPI::CameraMode::Orbit);
}
```

### Reference Up Vector

```cpp
// Align orbit up vector to target's local Y axis
api.align_orbit_camera_up_to_target();

// Set explicit up vector
api.set_orbit_camera_reference_up(glm::vec3(0.0f, 1.0f, 0.0f));
```

## Follow Camera Mode

Follow target with fixed offset (no smoothing, rigid attachment).

### Settings

```cpp
struct FollowCameraSettings
{
    CameraTarget target{};
    glm::vec3 positionOffsetLocal{0.0f, 2.0f, 6.0f};  // Offset in target's local space
    glm::quat rotationOffset{1.0f, 0.0f, 0.0f, 0.0f}; // Rotation offset
};
```

### Example

```cpp
// Follow player from behind and above
GameAPI::FollowCameraSettings follow;
follow.target.type = GameAPI::CameraTargetType::GLTFInstance;
follow.target.name = "player";
follow.positionOffsetLocal = glm::vec3(0.0f, 3.0f, 8.0f);  // Behind and up

api.set_follow_camera_settings(follow);
api.set_camera_mode(GameAPI::CameraMode::Follow);
```

## Chase Camera Mode

Follow target with smoothing (typical third-person camera).

### Settings

```cpp
struct ChaseCameraSettings
{
    CameraTarget target{};
    glm::vec3 positionOffsetLocal{0.0f, 2.0f, 6.0f};
    glm::quat rotationOffset{1.0f, 0.0f, 0.0f, 0.0f};
    float positionLag{8.0f};      // Smoothing rate (1/sec), higher = snappier
    float rotationLag{10.0f};     // Rotation smoothing rate (1/sec)
};
```

### Example

```cpp
// Smooth third-person chase camera
GameAPI::ChaseCameraSettings chase;
chase.target.type = GameAPI::CameraTargetType::GLTFInstance;
chase.target.name = "vehicle";
chase.positionOffsetLocal = glm::vec3(0.0f, 2.5f, 7.0f);
chase.positionLag = 6.0f;   // Smooth position
chase.rotationLag = 8.0f;   // Smooth rotation

api.set_chase_camera_settings(chase);
api.set_camera_mode(GameAPI::CameraMode::Chase);
```

## Fixed Camera Mode

Camera stays at fixed position and rotation (cutscenes, security cameras, etc.).

```cpp
// Set position and rotation
api.set_camera_position(glm::vec3(10.0f, 5.0f, 0.0f));
api.camera_look_at(glm::vec3(0.0f, 0.0f, 0.0f));

// Switch to fixed mode (camera won't respond to input)
api.set_camera_mode(GameAPI::CameraMode::Fixed);
```

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Start with free camera
api.set_camera_mode(GameAPI::CameraMode::Free);
api.set_camera_position(glm::vec3(0.0f, 2.0f, 10.0f));
api.set_camera_fov(75.0f);

GameAPI::FreeCameraSettings freeSettings;
freeSettings.moveSpeed = 10.0f;
api.set_free_camera_settings(freeSettings);

// On player spawn, switch to chase camera
GameAPI::ChaseCameraSettings chase;
chase.target.type = GameAPI::CameraTargetType::GLTFInstance;
chase.target.name = "player";
chase.positionOffsetLocal = glm::vec3(0.0f, 2.0f, 6.0f);
chase.positionLag = 8.0f;
chase.rotationLag = 10.0f;

api.set_chase_camera_settings(chase);
api.set_camera_mode(GameAPI::CameraMode::Chase);

// On right-click, orbit selected object
if (right_click && pick.valid) {
    api.set_camera_target_from_last_pick();
    api.set_camera_mode(GameAPI::CameraMode::Orbit);
}

// On cutscene start, switch to fixed
api.set_camera_mode(GameAPI::CameraMode::Fixed);
api.set_camera_position(cutscene_position);
api.camera_look_at(cutscene_target);
```

## Use Cases

### Racing Game Camera

```cpp
GameAPI::ChaseCameraSettings racing;
racing.target.type = GameAPI::CameraTargetType::GLTFInstance;
racing.target.name = "car";
racing.positionOffsetLocal = glm::vec3(0.0f, 1.2f, 4.0f);
racing.positionLag = 10.0f;  // Snappy for fast movement
racing.rotationLag = 15.0f;

api.set_chase_camera_settings(racing);
api.set_camera_mode(GameAPI::CameraMode::Chase);
```

### Cinematic Camera

```cpp
// Sequence of fixed camera shots
std::vector<glm::vec3> shotPositions = { /* ... */ };
std::vector<glm::vec3> shotTargets = { /* ... */ };

for (size_t i = 0; i < shotPositions.size(); ++i) {
    api.set_camera_mode(GameAPI::CameraMode::Fixed);
    api.set_camera_position(shotPositions[i]);
    api.camera_look_at(shotTargets[i]);
    wait(3.0f);  // Hold for 3 seconds
}
```

### RTS Camera

```cpp
// Free camera with adjusted settings
GameAPI::FreeCameraSettings rts;
rts.moveSpeed = 20.0f;  // Fast panning
rts.lookSensitivity = 0.0010f;  // Slower rotation

api.set_free_camera_settings(rts);
api.set_camera_mode(GameAPI::CameraMode::Free);

// Lock pitch to prevent looking straight down
float pitch, yaw;
api.get_camera_rotation(pitch, yaw);
pitch = glm::clamp(pitch, glm::radians(-80.0f), glm::radians(-10.0f));
api.set_camera_rotation(pitch, yaw);
```

## See Also

- [Input System](../InputSystem.md) — Keyboard/mouse input for camera control
- [Floating Origin](../FloatingOrigin.md) — Large-world camera positioning
- [Picking](Picking.md) — Select objects to orbit/follow
