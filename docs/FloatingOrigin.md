## Floating Origin & Double-Precision Coordinates

Precision-safe coordinate system that stores authoritative world positions in double precision and dynamically shifts the rendering origin to keep local float coordinates near zero.

### Problem

Single-precision floats (32-bit) have ~7 significant digits. At large world positions (e.g., 100km from origin), sub-meter precision is lost, causing:

- Vertex jitter / z-fighting
- Camera stutter during movement
- Picking inaccuracy
- Physics instability

### Solution

1. **Double-precision world coordinates**: Store authoritative positions as `glm::dvec3` (`WorldVec3`).
2. **Floating origin**: Maintain a world-space origin point that follows the camera.
3. **Local-space rendering**: Convert world positions to local float coordinates relative to the origin.

This keeps all render-local coordinates within a few hundred meters of (0,0,0), preserving full float precision.

### Core Types (`src/core/world.h`)

```c++
// Authoritative world-space coordinates (double precision)
using WorldVec3 = glm::dvec3;

// Convert world position to local float (for rendering)
glm::vec3 world_to_local(const WorldVec3 &world, const WorldVec3 &origin_world);

// Convert local float back to world position
WorldVec3 local_to_world(const glm::vec3 &local, const WorldVec3 &origin_world);

// Snap a world position to a grid (for stable origin placement)
WorldVec3 snap_world(const WorldVec3 &p, double grid_size);
```

### Origin Management (`SceneManager`)

The `SceneManager` automatically recenters the **render origin** when the camera drifts too far.

The authoritative **render origin** lives in `EngineContext`. The authoritative **physics origin** lives in `Physics::PhysicsContext` (game-owned):

```c++
// In EngineContext (src/core/context.h)
WorldVec3 origin_world{0.0, 0.0, 0.0};
uint64_t origin_revision{0};

// Space sim: a separate origin for physics (independent from render origin).
// This state lives in Physics::PhysicsContext (game-owned) rather than EngineContext.
//
// Position: world = origin_world + local
// Velocity: world = velocity_origin_world + local
```

```c++
// Private members in SceneManager
glm::vec3 _camera_position_local{0.0f, 0.0f, 0.0f};
double _floating_origin_recenter_threshold = 1000.0;  // meters
double _floating_origin_snap_size = 100.0;            // grid snap size
```

**Recentering Logic** (in `update_scene()`):

1. Compute distance from camera to current origin.
2. If distance exceeds threshold, snap camera position to grid and use as new origin.
3. Recompute all local positions relative to new origin.

```c++
if (_floating_origin_recenter_threshold > 0.0) {
    const WorldVec3 origin_before = get_world_origin();
    const WorldVec3 d = mainCamera.position_world - origin_before;
    if (glm::length2(d) > threshold2) {
        const WorldVec3 new_origin = snap_world(mainCamera.position_world, _floating_origin_snap_size);
        _context->set_origin_world(new_origin); // increments origin_revision
    }
}
_camera_position_local = world_to_local(mainCamera.position_world, get_world_origin());
```

### Coordinate Storage

| Component | Type | Description |
|-----------|------|-------------|
| `Camera::position_world` | `WorldVec3` | Camera world position (double) |
| `MeshInstance::translation_world` | `WorldVec3` | Instance world position |
| `GLTFInstance::translation_world` | `WorldVec3` | glTF instance world position |
| `PointLight::position_world` | `WorldVec3` | Light world position |
| `EngineContext::origin_world` | `WorldVec3` | Current render floating origin |
| `EngineContext::physics_context` | `Physics::PhysicsContext*` | Optional physics coordinate context (game-owned) |
| `SceneManager::_camera_position_local` | `glm::vec3` | Camera in local/render space |

### Rendering Pipeline

During `update_scene()`, all world-space objects are converted to local coordinates:

1. **Dynamic instances**: Convert `translation_world` to local position, build TRS matrix.
2. **Point lights**: Convert `position_world` to local for GPU upload.
3. **Camera**: Store both world and local positions; view matrix uses local.

```c++
// Dynamic instance positioning
glm::vec3 tLocal = world_to_local(inst.translation_world, get_world_origin());
glm::mat4 instanceTransform = make_trs_matrix(tLocal, inst.rotation, inst.scale);
```

### GameAPI Double-Precision Variants

The `GameAPI` exposes both float and double-precision transform types:

```c++
// Float-precision (relative/local usage)
struct Transform {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

// Double-precision (world-space usage)
struct TransformD {
    glm::dvec3 position{0.0};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

// API accepts both variants
bool add_gltf_instance(const std::string& name, const std::string& modelPath,
                       const Transform& transform, bool preloadTextures = true);
bool add_gltf_instance(const std::string& name, const std::string& modelPath,
                       const TransformD& transform, bool preloadTextures = true);
```

Similarly for `PointLight` / `PointLightD` and `IBLVolume` / `IBLVolumeD`.

### Picking Integration

Picking operates in local coordinates but returns world positions:

```c++
// Ray origin in local space
glm::vec3 rayOrigin = world_to_local(mainCamera.position_world, get_world_origin());

// ... perform ray-object intersection in local space ...

// Convert hit position back to world
outWorldPos = local_to_world(bestHitPos, get_world_origin());
```

### Query Functions

```c++
// Get current floating origin (world coordinates)
WorldVec3 SceneManager::get_world_origin() const;

// Get camera position in local/render coordinates
glm::vec3 SceneManager::get_camera_local_position() const;
```

### Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `_floating_origin_recenter_threshold` | 1000.0 | Distance (m) before recentering |
| `_floating_origin_snap_size` | 100.0 | Grid snap size (m) for new origin |

Setting `_floating_origin_recenter_threshold` to 0 or negative disables floating origin.

### Best Practices

1. **Store world positions in double**: Use `WorldVec3` / `TransformD` for anything that persists or needs absolute positioning.

2. **Work in local space for rendering**: All GPU-visible transforms should be in local coordinates.

3. **Convert at boundaries**: Convert world↔local at the scene update boundary, not per-frame in shaders.

4. **Use snap size**: Grid-aligned origins prevent micro-shifts that could cause subtle jitter.

5. **Consider threshold tuning**:
   - Smaller threshold (500m): More frequent recenters, tighter precision
   - Larger threshold (2000m): Fewer recenters, slightly reduced precision at edges

### Debugging

The engine UI displays origin and camera positions:

```
Origin (world): (1200.000, 0.000, 3400.000)
Camera (local): (23.456, 1.234, -12.789)
```

If you observe jitter at large world coordinates, verify:
- Positions are stored as `WorldVec3`, not `glm::vec3`
- `world_to_local()` is applied before GPU upload
- Origin recentering is enabled (threshold > 0)

### Related Files

- `src/core/world.h` — `WorldVec3` type and conversion functions
- `src/scene/vk_scene.h` — `SceneManager` origin management
- `src/scene/vk_scene.cpp` — Recentering logic in `update_scene()`
- `src/scene/camera.h` — `Camera::position_world` double-precision position
- `src/core/game_api.h` — `Transform` / `TransformD` API types
