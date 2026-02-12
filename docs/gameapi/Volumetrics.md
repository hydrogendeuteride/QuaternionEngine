# Volumetric Effects

GPU-based voxel volumetric rendering for clouds, smoke, and flame. Supports up to 4 independent volumes.

## Enable/Disable

```cpp
api.set_volumetrics_enabled(true);
bool enabled = api.get_volumetrics_enabled();
```

## Volume Capacity

```cpp
size_t maxVolumes = api.get_max_voxel_volumes();  // Returns 4
```

## Volume Settings

```cpp
struct VoxelVolumeSettings
{
    bool enabled{false};
    VoxelVolumeType type{VoxelVolumeType::Clouds};

    // Positioning
    bool followCameraXZ{false};     // Follow camera in XZ, offset in Y
    bool animateVoxels{true};       // Run voxel advection/update compute
    glm::vec3 volumeCenterLocal{0.0f, 2.0f, 0.0f};
    glm::vec3 volumeHalfExtents{8.0f, 8.0f, 8.0f};
    glm::vec3 volumeVelocityLocal{0.0f, 0.0f, 0.0f};  // Drift when not following camera

    // Raymarch quality
    float densityScale{1.0f};
    float coverage{0.0f};        // 0..1 threshold (higher = emptier)
    float extinction{1.0f};      // Absorption/extinction scale
    int stepCount{48};           // Raymarch steps

    // Voxel grid resolution (cubic)
    uint32_t gridResolution{48};

    // Animation (advection + injection)
    glm::vec3 windVelocityLocal{0.0f, 2.0f, 0.0f};  // Local units/sec
    float dissipation{1.25f};    // Density decay rate (1/sec)
    float noiseStrength{1.0f};   // Injection rate
    float noiseScale{8.0f};      // Noise frequency in UVW space
    float noiseSpeed{1.0f};      // Time scale for injection noise

    // Source (smoke/flame emitter)
    glm::vec3 emitterUVW{0.5f, 0.05f, 0.5f};  // Normalized (0-1)
    float emitterRadius{0.18f};  // Normalized

    // Shading
    glm::vec3 albedo{1.0f, 1.0f, 1.0f};  // Scattering tint
    float scatterStrength{1.0f};
    glm::vec3 emissionColor{1.0f, 0.6f, 0.25f};  // Flame emissive tint
    float emissionStrength{0.0f};
};

enum class VoxelVolumeType : uint32_t
{
    Clouds = 0,
    Smoke = 1,
    Flame = 2
};
```

## Managing Volumes

### Get Volume Settings

```cpp
GameAPI::VoxelVolumeSettings volume;
if (api.get_voxel_volume(0, volume)) {
    fmt::println("Volume 0 enabled: {}", volume.enabled);
}
```

Volume indices: 0-3.

### Set Volume Settings

```cpp
GameAPI::VoxelVolumeSettings volume;
volume.enabled = true;
volume.type = GameAPI::VoxelVolumeType::Clouds;
volume.volumeCenterLocal = glm::vec3(0.0f, 50.0f, 0.0f);
volume.volumeHalfExtents = glm::vec3(100.0f, 20.0f, 100.0f);
// ... configure other settings

bool success = api.set_voxel_volume(0, volume);
```

## Complete Examples

### Cloud Layer (Follow Camera)

```cpp
GameAPI::Engine api(&engine);
api.set_volumetrics_enabled(true);

GameAPI::VoxelVolumeSettings clouds;
clouds.enabled = true;
clouds.type = GameAPI::VoxelVolumeType::Clouds;
clouds.followCameraXZ = true;  // Follow camera horizontally
clouds.animateVoxels = true;

// Volume shape (sky layer)
clouds.volumeCenterLocal = glm::vec3(0.0f, 50.0f, 0.0f);  // Y offset only
clouds.volumeHalfExtents = glm::vec3(100.0f, 20.0f, 100.0f);

// Raymarch quality
clouds.densityScale = 0.8f;
clouds.coverage = 0.5f;  // 50% coverage
clouds.extinction = 1.0f;
clouds.stepCount = 64;

// Voxel grid
clouds.gridResolution = 128;

// Animation (gentle wind)
clouds.windVelocityLocal = glm::vec3(2.0f, 0.0f, 1.0f);
clouds.dissipation = 0.5f;
clouds.noiseStrength = 0.5f;
clouds.noiseScale = 4.0f;
clouds.noiseSpeed = 0.5f;

// Shading (bluish white)
clouds.albedo = glm::vec3(0.9f, 0.95f, 1.0f);
clouds.scatterStrength = 1.0f;
clouds.emissionStrength = 0.0f;

api.set_voxel_volume(0, clouds);
```

### Campfire Flame

```cpp
GameAPI::VoxelVolumeSettings flame;
flame.enabled = true;
flame.type = GameAPI::VoxelVolumeType::Flame;
flame.followCameraXZ = false;  // Fixed position
flame.animateVoxels = true;

// Volume shape (small fire)
flame.volumeCenterLocal = glm::vec3(0.0f, 0.5f, -5.0f);
flame.volumeHalfExtents = glm::vec3(0.5f, 1.0f, 0.5f);

// Raymarch quality
flame.densityScale = 2.0f;
flame.coverage = 0.3f;
flame.extinction = 2.0f;
flame.stepCount = 48;

// Voxel grid
flame.gridResolution = 48;

// Animation (strong upward flow)
flame.windVelocityLocal = glm::vec3(0.0f, 5.0f, 0.0f);  // Buoyancy
flame.dissipation = 2.0f;
flame.noiseStrength = 2.0f;
flame.noiseScale = 10.0f;
flame.noiseSpeed = 2.0f;

// Emitter (bottom center)
flame.emitterUVW = glm::vec3(0.5f, 0.1f, 0.5f);
flame.emitterRadius = 0.3f;

// Shading (orange flame)
flame.albedo = glm::vec3(1.0f, 0.8f, 0.6f);
flame.scatterStrength = 0.5f;
flame.emissionColor = glm::vec3(1.0f, 0.5f, 0.1f);
flame.emissionStrength = 3.0f;

api.set_voxel_volume(1, flame);
```

### Smoke Plume

```cpp
GameAPI::VoxelVolumeSettings smoke;
smoke.enabled = true;
smoke.type = GameAPI::VoxelVolumeType::Smoke;
smoke.followCameraXZ = false;
smoke.animateVoxels = true;

// Volume shape (tall chimney smoke)
smoke.volumeCenterLocal = glm::vec3(10.0f, 5.0f, -8.0f);
smoke.volumeHalfExtents = glm::vec3(2.0f, 8.0f, 2.0f);

// Raymarch quality
smoke.densityScale = 1.5f;
smoke.coverage = 0.4f;
smoke.extinction = 1.5f;
smoke.stepCount = 56;

// Voxel grid
smoke.gridResolution = 64;

// Animation (upward with wind)
smoke.windVelocityLocal = glm::vec3(1.0f, 3.0f, 0.5f);
smoke.dissipation = 1.0f;
smoke.noiseStrength = 1.0f;
smoke.noiseScale = 6.0f;
smoke.noiseSpeed = 1.0f;

// Emitter (bottom)
smoke.emitterUVW = glm::vec3(0.5f, 0.05f, 0.5f);
smoke.emitterRadius = 0.2f;

// Shading (dark gray)
smoke.albedo = glm::vec3(0.4f, 0.4f, 0.4f);
smoke.scatterStrength = 0.8f;
smoke.emissionStrength = 0.0f;

api.set_voxel_volume(2, smoke);
```

## Dynamic Control

### Moving Volume

```cpp
// Update volume position each frame
GameAPI::VoxelVolumeSettings volume;
api.get_voxel_volume(volumeIdx, volume);
volume.volumeCenterLocal = moving_source_position;
api.set_voxel_volume(volumeIdx, volume);
```

### Wind Changes

```cpp
GameAPI::VoxelVolumeSettings clouds;
api.get_voxel_volume(0, clouds);
clouds.windVelocityLocal = current_wind_direction * wind_speed;
api.set_voxel_volume(0, clouds);
```

### Enable/Disable

```cpp
GameAPI::VoxelVolumeSettings volume;
api.get_voxel_volume(volumeIdx, volume);
volume.enabled = should_render;
api.set_voxel_volume(volumeIdx, volume);
```

### Adjust Density/Coverage

```cpp
// Make clouds thinner over time
GameAPI::VoxelVolumeSettings clouds;
api.get_voxel_volume(0, clouds);
clouds.densityScale = lerp(1.0f, 0.3f, dissolve_progress);
clouds.coverage = lerp(0.4f, 0.9f, dissolve_progress);  // Higher coverage = emptier
api.set_voxel_volume(0, clouds);
```

## Performance Considerations

- Each volume runs a compute shader for advection/injection (if `animateVoxels` is true)
- Raymarch cost scales with `stepCount` and screen coverage
- Higher `gridResolution` = more VRAM and compute cost
- Use lower resolutions for background/distant volumes
- Disable `animateVoxels` for static volumes (clouds that don't need to evolve)

**Recommended Settings:**
- Clouds (distant): 64-128 resolution, 32-48 steps
- Smoke/Flame (close): 48-64 resolution, 48-64 steps
- If targeting 60 FPS, prefer lower step counts (32-48)

## Combining Volumes

You can layer multiple volumes:

```cpp
// Volume 0: High cloud layer (follows camera)
api.set_voxel_volume(0, high_clouds);

// Volume 1: Low fog layer (follows camera)
api.set_voxel_volume(1, ground_fog);

// Volume 2: Campfire smoke (fixed position)
api.set_voxel_volume(2, campfire_smoke);

// Volume 3: Torch flame (fixed position)
api.set_voxel_volume(3, torch_flame);
```

Volumes are rendered in order (0-3), with alpha blending.

## Use Cases

### Weather System

```cpp
// Clear sky -> Cloudy -> Rainy

// Start: No clouds
GameAPI::VoxelVolumeSettings clouds;
clouds.enabled = false;
api.set_voxel_volume(0, clouds);

// Transition: Enable clouds, low coverage
clouds.enabled = true;
clouds.coverage = 0.7f;  // Sparse
api.set_voxel_volume(0, clouds);

// Rain: Dense clouds
clouds.coverage = 0.3f;  // Dense
clouds.densityScale = 1.5f;
api.set_voxel_volume(0, clouds);
```

### Explosion Smoke

```cpp
// Spawn smoke volume at explosion position
GameAPI::VoxelVolumeSettings explosion;
explosion.enabled = true;
explosion.type = GameAPI::VoxelVolumeType::Smoke;
explosion.volumeCenterLocal = explosion_position;
explosion.volumeHalfExtents = glm::vec3(5.0f, 8.0f, 5.0f);
explosion.windVelocityLocal = glm::vec3(0.0f, 2.0f, 0.0f);
explosion.dissipation = 3.0f;  // Quickly fade
api.set_voxel_volume(3, explosion);

// Disable after a few seconds
wait(5.0f);
explosion.enabled = false;
api.set_voxel_volume(3, explosion);
```

## See Also

- [Volumetrics System](../Volumetrics.md) — Low-level implementation details
- [Particles](Particles.md) — Particle-based effects
- [Compute](../Compute.md) — GPU compute pipeline
