# Volumetric Cloud System

The volumetric system provides GPU-accelerated voxel-based rendering for clouds, smoke, and flame effects using raymarching and procedural density simulation.

## Architecture Overview

The system is implemented across multiple components:

- **CloudPass** (`src/render/passes/clouds.h/.cpp`) — Render pass managing voxel volumes, compute simulation, and raymarching
- **GameAPI** (`src/core/game_api.h`) — High-level API for configuring volumetric effects
- **Shaders**
  - `shaders/cloud_voxel_advect.comp` — Voxel density simulation (advection + injection)
  - `shaders/clouds.frag` — Raymarching fragment shader

## Key Features

- **Voxel-based density**: Cubic grids (4-256³ resolution) storing per-voxel density values
- **Three volume types**: Clouds (infinite XZ wrap), Smoke (localized), Flame (emissive)
- **GPU simulation**: Semi-Lagrangian advection with procedural noise injection
- **Raymarching composite**: Beer-Lambert absorption + single-scattering approximation
- **Ping-pong buffers**: Double-buffered voxel grids for temporal stability
- **Camera following**: Volumes can anchor to camera XZ (infinite clouds) or drift in world-space
- **Floating-origin stable**: Automatically adjusts volume positions when world origin shifts
- **Multi-volume support**: Up to 4 independent volumes (`MAX_VOXEL_VOLUMES = 4`)

## Volume Types

### Clouds (Type 0)

- **Behavior**: Continuous XZ wrapping for infinite cloud layers
- **Injection**: Broad slab with height-based shaping (upper/lower bounds)
- **Advection**: Wind wraps in XZ, clamped in Y
- **Typical use**: Sky clouds, atmospheric layers

### Smoke (Type 1)

- **Behavior**: Localized emission with soft dissipation
- **Injection**: Spherical emitter in UVW space with softer noise threshold
- **Advection**: Fully clamped (no wrapping)
- **Typical use**: Smoke columns, steam, fog banks

### Flame (Type 2)

- **Behavior**: Flickering emissive source with strong noise
- **Injection**: Spiky procedural noise, blends toward injected field (avoids fog accumulation)
- **Advection**: Fully clamped (no wrapping)
- **Rendering**: Adds emission term (`emissionColor × emissionStrength`)
- **Typical use**: Fires, torches, explosions

## Creating Volumetric Effects

### Via GameAPI

```cpp
#include "core/game_api.h"

GameAPI::Engine api(&engine);

// Enable volumetrics globally
api.set_volumetrics_enabled(true);

// Configure a cloud volume (index 0)
GameAPI::VoxelVolumeSettings cloud;
cloud.enabled = true;
cloud.type = GameAPI::VoxelVolumeType::Clouds;

// Position: follow camera XZ, offset in Y
cloud.followCameraXZ = true;
cloud.volumeCenterLocal = glm::vec3(0.0f, 50.0f, 0.0f); // 50 units above camera
cloud.volumeHalfExtents = glm::vec3(100.0f, 20.0f, 100.0f); // 200×40×200 box

// Animation: enable voxel advection
cloud.animateVoxels = true;
cloud.windVelocityLocal = glm::vec3(5.0f, 2.0f, 0.0f); // Drift +X, rise +Y
cloud.dissipation = 0.5f; // Slow decay
cloud.noiseStrength = 0.8f;
cloud.noiseScale = 8.0f;
cloud.noiseSpeed = 0.3f;

// Rendering
cloud.densityScale = 1.5f;
cloud.coverage = 0.3f; // Higher = less dense (threshold)
cloud.extinction = 1.0f;
cloud.stepCount = 64; // Raymarch steps (quality vs performance)
cloud.gridResolution = 64; // 64³ voxel grid

// Shading
cloud.albedo = glm::vec3(1.0f, 1.0f, 1.0f); // White clouds
cloud.scatterStrength = 1.2f;
cloud.emissionColor = glm::vec3(0.0f); // No emission
cloud.emissionStrength = 0.0f;

api.set_voxel_volume(0, cloud);
```

### Flame Effect

```cpp
GameAPI::VoxelVolumeSettings flame;
flame.enabled = true;
flame.type = GameAPI::VoxelVolumeType::Flame;

// Position: absolute world location
flame.followCameraXZ = false;
flame.volumeCenterLocal = glm::vec3(0.0f, 1.0f, 0.0f);
flame.volumeHalfExtents = glm::vec3(1.0f, 2.0f, 1.0f); // 2×4×2 box

// Animation
flame.animateVoxels = true;
flame.windVelocityLocal = glm::vec3(0.0f, 8.0f, 0.0f); // Rise upward
flame.dissipation = 2.0f; // Fast decay
flame.noiseStrength = 1.5f;
flame.noiseScale = 10.0f;
flame.noiseSpeed = 2.0f;

// Emitter in UVW space (bottom center)
flame.emitterUVW = glm::vec3(0.5f, 0.05f, 0.5f);
flame.emitterRadius = 0.2f; // 20% of volume size

// Shading
flame.densityScale = 2.0f;
flame.coverage = 0.0f;
flame.extinction = 0.8f;
flame.stepCount = 48;
flame.gridResolution = 48;

flame.albedo = glm::vec3(1.0f, 0.6f, 0.2f); // Orange scatter
flame.scatterStrength = 0.5f;
flame.emissionColor = glm::vec3(1.0f, 0.5f, 0.1f); // Orange-red glow
flame.emissionStrength = 3.0f; // Strong emission

api.set_voxel_volume(1, flame);
```

## Simulation Details

### Voxel Advection (Compute Shader)

The `cloud_voxel_advect.comp` shader updates voxel density each frame:

1. **Semi-Lagrangian advection**: Backtrace along wind velocity
   ```glsl
   vec3 back = uvw - (windVelocityLocal / volumeSize) * dt;
   ```
   - Clouds: Wrap XZ (`fract(back.xz)`), clamp Y
   - Smoke/Flame: Clamp all axes

2. **Trilinear sampling**: Sample input density at backtraced position
   ```glsl
   float advected = sample_density_trilinear(back, gridResolution);
   ```

3. **Dissipation**: Exponential decay
   ```glsl
   advected *= exp(-dissipation * dt);
   ```

4. **Noise injection**: Procedural density injection using 4-octave FBM
   - **Clouds**: Broad slab with height shaping
     ```glsl
     injected = smoothstep(0.55, 0.80, fbm3(uvw * noiseScale + time * noiseSpeed));
     low = smoothstep(0.0, 0.18, uvw.y);
     high = 1.0 - smoothstep(0.78, 1.0, uvw.y);
     injected *= low * high;
     ```
   - **Smoke**: Spherical emitter with softer threshold
     ```glsl
     shape = 1.0 - smoothstep(emitterRadius, emitterRadius * 1.25, distance(uvw, emitterUVW));
     injected = smoothstep(0.45, 0.75, fbm3(...)) * shape;
     ```
   - **Flame**: Spiky noise with flickering
     ```glsl
     injected = (fbm3(...) ^ 2) * shape;
     out_density = mix(advected, injected, noiseStrength * dt); // Blend toward injected
     ```

5. **Write output**: Write to ping-pong buffer
   ```glsl
   vox_out.density[idx3(c, gridResolution)] = clamp(out_density, 0.0, 1.0);
   ```

### Raymarching (Fragment Shader)

The `clouds.frag` shader composites volumes onto the HDR buffer:

1. **Ray setup**:
   - Reconstruct world-space ray from screen UV
   - Define AABB from `volumeCenterLocal ± volumeHalfExtents`
   - Compute ray-AABB intersection (`t0`, `t1`)

2. **Geometry clipping**:
   - Sample G-buffer position (`posTex`)
   - If opaque geometry exists, clamp `t1` to surface distance
   - Prevents clouds rendering behind solid objects

3. **Raymarching loop**:
   ```glsl
   float transmittance = 1.0;
   vec3 scattering = vec3(0.0);

   for (int i = 0; i < stepCount; ++i) {
       vec3 p = camPos + rd * t;
       float density = sample_voxel_density(p, bmin, bmax);

       // Apply coverage threshold
       density = max(density - coverage, 0.0) * densityScale;

       // Beer-Lambert absorption
       float extinction_coeff = density * extinction;
       float step_transmittance = exp(-extinction_coeff * dt);

       // In-scattering (single-scattering approximation)
       vec3 light_contrib = albedo * scatterStrength * density;

       // Flame emission
       if (volumeType == 2) {
           light_contrib += emissionColor * emissionStrength * density;
       }

       scattering += transmittance * (1.0 - step_transmittance) * light_contrib;
       transmittance *= step_transmittance;

       t += dt;
   }
   ```

4. **Composite**:
   ```glsl
   vec3 finalColor = baseColor * transmittance + scattering;
   outColor = vec4(finalColor, 1.0);
   ```

### Floating-Origin Stability

When the world origin shifts (`CloudPass::update_time_and_origin_delta()`):
- Volumes with `followCameraXZ = false` are adjusted: `volumeCenterLocal -= origin_delta`
- Ensures volumes stay in the same world-space location despite coordinate changes

### Volume Drift

For non-camera-following volumes:
```cpp
volumeCenterLocal += volumeVelocityLocal * dt;
```
Allows volumes to drift independently (e.g., moving storm clouds).

## Memory Management

### Voxel Buffers

Each volume maintains two ping-pong buffers (`voxelDensity[2]`):
- **Read buffer**: Input to advection compute shader and raymarch fragment shader
- **Write buffer**: Output of advection compute shader
- Buffers swap each frame (`voxelReadIndex = 1 - voxelReadIndex`)

Buffer size: `gridResolution³ × sizeof(float)` bytes
- Example: 64³ grid = 1 MB per buffer (2 MB total per volume)
- Maximum 4 volumes = 8 MB total (at 64³ resolution)

### Lazy Allocation

Voxel buffers are allocated only when:
- `enabled = true`
- `gridResolution` changes
- Called via `rebuild_voxel_density()`

Initial density is procedurally generated using the same FBM noise as injection.

## Render Graph Integration

The cloud pass registers after lighting/SSR:

```cpp
RGImageHandle CloudPass::register_graph(RenderGraph* graph,
                                         RGImageHandle hdrInput,
                                         RGImageHandle gbufPos)
{
    // For each enabled volume:
    //   1. Optional: Add compute pass for voxel advection (if animateVoxels == true)
    //   2. Add graphics pass for raymarching composite

    // Passes read/write ping-pong buffers and sample G-buffer depth
    // Returns final HDR image with clouds composited
}
```

**Pass structure** (per volume):
1. **VoxelUpdate** (compute, optional): Read voxel buffer → advect → write voxel buffer
2. **Volumetrics** (graphics): Read HDR input + G-buffer + voxel buffer → raymarch → write HDR output

Volumes are rendered sequentially (volume 0 → 1 → 2 → 3) to allow layered effects.

## Performance Considerations

- **Voxel resolution**: Higher resolution = better detail but 8× memory per doubling (64³ = 1 MB, 128³ = 8 MB)
- **Raymarch steps**: More steps = smoother results but linear fragment cost (48-128 typical)
- **Fill rate**: Volumetrics are fragment-shader intensive; reduce `stepCount` on low-end hardware
- **Advection cost**: Compute cost is `O(resolution³)` but typically <1ms for 64³
- **Multi-volume overhead**: Each active volume adds a full raymarch pass; budget 2-3 volumes max

### Recommended Settings

**High quality (desktop)**:
```cpp
gridResolution = 128;
stepCount = 128;
```

**Medium quality (mid-range)**:
```cpp
gridResolution = 64;
stepCount = 64;
```

**Low quality (mobile/low-end)**:
```cpp
gridResolution = 32;
stepCount = 32;
```

## Parameter Reference

### VoxelVolumeSettings

```cpp
struct VoxelVolumeSettings
{
    // Enable/type
    bool enabled{false};
    VoxelVolumeType type{Clouds}; // Clouds, Smoke, Flame

    // Positioning
    bool followCameraXZ{false};        // Anchor to camera XZ
    bool animateVoxels{true};          // Enable voxel simulation
    glm::vec3 volumeCenterLocal{0,2,0};
    glm::vec3 volumeHalfExtents{8,8,8};
    glm::vec3 volumeVelocityLocal{0};  // Drift velocity (if !followCameraXZ)

    // Rendering
    float densityScale{1.0};           // Density multiplier
    float coverage{0.0};               // 0..1 threshold (higher = less dense)
    float extinction{1.0};             // Absorption coefficient
    int stepCount{48};                 // Raymarch steps (8-256)
    uint32_t gridResolution{48};       // Voxel grid resolution (4-256)

    // Simulation (advection)
    glm::vec3 windVelocityLocal{0,2,0}; // Wind velocity (units/sec)
    float dissipation{1.25};            // Density decay (1/sec)
    float noiseStrength{1.0};           // Injection rate
    float noiseScale{8.0};              // Noise frequency
    float noiseSpeed{1.0};              // Time scale

    // Emitter (smoke/flame only)
    glm::vec3 emitterUVW{0.5,0.05,0.5}; // Normalized (0..1)
    float emitterRadius{0.18};          // Normalized (0..1)

    // Shading
    glm::vec3 albedo{1,1,1};            // Scattering tint
    float scatterStrength{1.0};
    glm::vec3 emissionColor{1,0.6,0.25};// Flame emission tint
    float emissionStrength{0.0};        // Flame emission strength
};
```

## Common Presets

### Stratocumulus Clouds

```cpp
cloud.type = Clouds;
cloud.followCameraXZ = true;
cloud.volumeCenterLocal = glm::vec3(0, 80, 0);
cloud.volumeHalfExtents = glm::vec3(200, 30, 200);
cloud.windVelocityLocal = glm::vec3(3, 1, 0);
cloud.dissipation = 0.3f;
cloud.densityScale = 1.2f;
cloud.coverage = 0.4f;
cloud.gridResolution = 64;
cloud.stepCount = 64;
```

### Torch Flame

```cpp
flame.type = Flame;
flame.followCameraXZ = false;
flame.volumeCenterLocal = glm::vec3(0, 1.5, 0);
flame.volumeHalfExtents = glm::vec3(0.3, 0.8, 0.3);
flame.windVelocityLocal = glm::vec3(0, 6, 0);
flame.dissipation = 2.5f;
flame.noiseStrength = 2.0f;
flame.emitterUVW = glm::vec3(0.5, 0.1, 0.5);
flame.emitterRadius = 0.25f;
flame.emissionColor = glm::vec3(1.0, 0.4, 0.1);
flame.emissionStrength = 4.0f;
flame.gridResolution = 32;
flame.stepCount = 32;
```

### Smoke Plume

```cpp
smoke.type = Smoke;
smoke.followCameraXZ = false;
smoke.volumeCenterLocal = glm::vec3(0, 2, 0);
smoke.volumeHalfExtents = glm::vec3(2, 5, 2);
smoke.windVelocityLocal = glm::vec3(1, 4, 0);
smoke.dissipation = 1.0f;
smoke.noiseStrength = 1.2f;
smoke.emitterUVW = glm::vec3(0.5, 0.05, 0.5);
smoke.emitterRadius = 0.15f;
smoke.albedo = glm::vec3(0.4, 0.4, 0.4);
smoke.scatterStrength = 0.8f;
smoke.gridResolution = 48;
smoke.stepCount = 48;
```

## Troubleshooting

**Volumes not visible**:
- Ensure `enabled = true` and `volumetrics_enabled = true` globally
- Check AABB intersects camera frustum
- Reduce `coverage` (lower = denser)
- Increase `densityScale`

**Blocky/noisy appearance**:
- Increase `gridResolution` (64 → 128)
- Increase `stepCount` (48 → 96)
- Adjust `noiseScale` for finer detail

**Performance issues**:
- Reduce `gridResolution` (64 → 32)
- Reduce `stepCount` (64 → 32)
- Disable `animateVoxels` for static volumes
- Reduce number of active volumes

**Volumes don't animate**:
- Ensure `animateVoxels = true`
- Check `windVelocityLocal` is non-zero
- Verify `noiseStrength > 0` and `noiseSpeed > 0`

**Volumes flicker/pop**:
- Increase `dissipation` to smooth density changes
- Lower `noiseStrength` for subtler injection
- Use higher `gridResolution` for temporal stability

## API Reference

### GameAPI::Engine Volumetric Methods

```cpp
// Global enable/disable
void set_volumetrics_enabled(bool enabled);
bool get_volumetrics_enabled() const;

// Volume configuration (index 0-3)
void set_voxel_volume(int index, const VoxelVolumeSettings& settings);
VoxelVolumeSettings get_voxel_volume(int index) const;

// Retrieve all volumes
std::vector<VoxelVolumeSettings> get_voxel_volumes() const;
```

### CloudPass

```cpp
class CloudPass : public IRenderPass
{
    // Render graph registration
    RGImageHandle register_graph(RenderGraph* graph,
                                  RGImageHandle hdrInput,
                                  RGImageHandle gbufPos);

    // Internal voxel management
    void rebuild_voxel_density(uint32_t volume_index,
                               uint32_t resolution,
                               const VoxelVolumeSettings& settings);
};
```

## See Also

- `docs/ParticleSystem.md` — GPU particle system documentation
- `docs/RenderGraph.md` — Render graph integration details
- `docs/RenderPasses.md` — Pass execution and pipeline management
- `docs/GameAPI.md` — High-level game API
- `docs/Compute.md` — Compute pipeline details
