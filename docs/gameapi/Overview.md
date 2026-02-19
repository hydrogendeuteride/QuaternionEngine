# Game API Overview

`GameAPI::Engine` is a high-level, game-friendly wrapper around the Vulkan engine. It provides a stable interface for common game tasks without requiring direct Vulkan knowledge.

## Architecture

The Game API sits between your game logic and the engine internals:

```
Your Game Code
      ↓
GameAPI::Engine (snake_case, stable)
      ↓
VulkanEngine (internal)
      ↓
Vulkan/GPU
```

## Creating the API

```cpp
#include "core/engine.h"
#include "core/game_api.h"

VulkanEngine engine;
engine.init();

GameAPI::Engine api(&engine);  // Non-owning wrapper
```

The `GameAPI::Engine` instance is lightweight and non-owning—it simply holds a pointer to the `VulkanEngine`.

## Design Principles

### 1. Stable Interface
- Methods are organized by responsibility (textures, lighting, particles, etc.)
- Uses `snake_case` naming for game-friendly API
- Return types are simple (`bool` for success, handles for resources)

### 2. Automatic Resource Management
- Textures are cached and streamed based on VRAM budget
- Assets are loaded asynchronously when possible
- Resources are reference-counted and auto-evicted by LRU

### 3. Double-Precision Support
Most spatial types have both single and double-precision variants:

```cpp
// Single precision (for typical games)
void set_camera_position(const glm::vec3& pos);
bool add_point_light(const PointLight& light);

// Double precision (for large worlds: space, flight sims)
void set_camera_position(const glm::dvec3& pos);
bool add_point_light(const PointLightD& light);
```

Use double-precision types when working with large world coordinates to avoid floating-point precision loss.

## Coordinate System

The engine uses:
- **Right-handed Y-up coordinate system**
- **-Z forward** for cameras and directions
- **World origin** is double-precision (see [Floating Origin](../FloatingOrigin.md))

## API Categories

The `GameAPI::Engine` groups functionality into logical categories:

### Scene & Objects
- Add/remove glTF instances and primitive meshes
- Control transforms and visibility
- Manage instance lifecycle

See: [Scene Management](Scene.md)

### Animation
- Play/stop glTF animations
- Control animation speed and looping
- Apply per-node pose offsets

See: [Animation](Animation.md)

### Textures
- Load textures from file or memory
- Control VRAM budget and streaming
- Pin critical textures (UI, etc.)

See: [Textures](Textures.md)

### Lighting
- Set sunlight direction and color
- Add/remove point and spot lights
- Manage IBL volumes

See: [Lighting](Lighting.md), [IBL](IBL.md)

### Shadows
- Toggle shadow modes (raster, hybrid, ray-traced)
- Control shadow map resolution
- Adjust shadow quality settings

See: [Shadows](Shadows.md)

### Camera
- Set camera position and rotation
- Switch camera modes (free, orbit, follow, chase)
- Control FOV and look-at targets

See: [Camera](Camera.md)

### Effects
- Create particle systems with flipbook animation
- Manage voxel volumes (clouds, smoke, flame)
- Add procedural planets with atmosphere
- Blackbody emission materials for hot-metal effects (nozzles, barrels)
- Mesh VFX materials for animated procedural effects (exhaust, shields)

See: [Particles](Particles.md), [Volumetrics](Volumetrics.md), [Planets](Planets.md), [Blackbody](Blackbody.md), [Mesh VFX](MeshVFX.md)

### Post-Processing
- Tonemapping (Reinhard, ACES)
- Bloom threshold and intensity
- FXAA anti-aliasing
- Screen-space reflections (SSR)

See: [Post-Processing](PostProcessing.md)

### Interaction
- Query last click selection
- Navigate glTF node hierarchies
- Toggle picking modes (CPU raycast vs. ID-buffer)

See: [Picking](Picking.md)

### Debug Visualization
- Draw lines, spheres, boxes, capsules
- Control depth testing and duration
- Layer-based visibility

See: [Debug Drawing](Debug.md)

## Time and Statistics

```cpp
// Frame delta time (clamped to 0.0-0.1 seconds)
float dt = api.get_delta_time();

// Engine statistics
GameAPI::Stats stats = api.get_stats();
fmt::println("FPS: {:.1f} | Tris: {} | Draws: {}",
             1000.0f / stats.frametime,
             stats.triangleCount,
             stats.drawCallCount);
```

## Render Scale

Control internal rendering resolution for performance:

```cpp
api.set_render_scale(0.75f);  // 75% resolution (0.3 - 1.0)
```

This scales the draw extent relative to the swapchain, trading resolution for performance.

## Pass Toggles

Enable/disable specific render passes at runtime:

```cpp
api.set_pass_enabled("FXAA", true);
api.set_pass_enabled("SSR", false);
```

Pass names correspond to RenderGraph pass registration names (see [Render Graph](../RenderGraph.md)).

## Hot Reload

Reload modified shaders without restarting:

```cpp
api.hot_reload_shaders();
```

This recompiles all changed GLSL files and rebuilds pipelines.

## Next Steps

- **[Scene Management](Scene.md)** — Learn how to spawn and control objects
- **[Textures](Textures.md)** — Understand texture loading and VRAM management
- **[Lighting](Lighting.md)** — Add lights to your scene
- **[Camera](Camera.md)** — Control camera movement and modes
