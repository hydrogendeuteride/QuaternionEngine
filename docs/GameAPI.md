# Game API

High-level, game-friendly API for the Vulkan engine. The `GameAPI::Engine` class provides a stable interface for building games without directly managing Vulkan resources.

## Quick Start

```cpp
#include "core/engine.h"
#include "core/game_api.h"

VulkanEngine engine;
engine.init();

GameAPI::Engine api(&engine);

// Load a model
GameAPI::Transform t;
t.position = glm::vec3(0.0f, 0.0f, -5.0f);
api.add_gltf_instance("player", "characters/player.gltf", t);

// Add a light
GameAPI::PointLight light;
light.position = glm::vec3(0.0f, 3.0f, 0.0f);
light.color = glm::vec3(1.0f, 0.9f, 0.8f);
light.intensity = 100.0f;
api.add_point_light(light);

// Game loop
while (engine.running()) {
    float dt = api.get_delta_time();
    // ... update game logic
    engine.draw();
}
```

## Documentation Index

### Core Systems
- **[Overview](gameapi/Overview.md)** — Architecture, `GameAPI::Engine` introduction, basic concepts
- **[Scene Management](gameapi/Scene.md)** — Instances, primitives, transforms, object lifecycle
- **[Animation](gameapi/Animation.md)** — glTF animations, playback control, node offsets

### Rendering
- **[Textures](gameapi/Textures.md)** — Texture loading, streaming, VRAM budget management
- **[Lighting](gameapi/Lighting.md)** — Directional (sun), point, and spot lights
- **[Shadows](gameapi/Shadows.md)** — Shadow modes (raster/RT), resolution, quality settings
- **[IBL](gameapi/IBL.md)** — Image-based lighting, global and local volumes
- **[Post-Processing](gameapi/PostProcessing.md)** — Tonemapping, bloom, FXAA, SSR

### Effects
- **[Particles](gameapi/Particles.md)** — GPU particle systems with flipbook animation
- **[Volumetrics](gameapi/Volumetrics.md)** — Voxel-based clouds, smoke, and flame
- **[Planets](gameapi/Planets.md)** — Procedural planets, terrain, atmosphere, clouds

### Camera & Interaction
- **[Camera](gameapi/Camera.md)** — Camera modes (free/orbit/follow/chase), controls
- **[Picking](gameapi/Picking.md)** — Object selection, hover detection, hierarchy navigation

### Debugging
- **[Debug Drawing](gameapi/Debug.md)** — Runtime visualization (lines, spheres, boxes, etc.)

## Related Systems

These systems are accessed separately from `GameAPI::Engine`:

- **[Input System](InputSystem.md)** — Keyboard, mouse, and cursor mode handling
- **[ImGui System](ImGuiSystem.md)** — Immediate-mode UI integration
- **[Scene Manager](Scene.md)** — Low-level scene graph and draw context
- **[Render Graph](RenderGraph.md)** — Custom render passes and resource management

## Header and Implementation

- **Header**: `src/core/game_api.h`
- **Implementation**: `src/core/game_api/*.cpp` (split by subsystem)
  - `game_api.cpp` — Core and transform utilities
  - `game_api_textures.cpp` — Texture streaming
  - `game_api_scene.cpp` — Instance management
  - `game_api_camera.cpp` — Camera control
  - `game_api_lighting.cpp` — Light management
  - `game_api_particles.cpp` — Particle systems
  - `game_api_volumetrics.cpp` — Voxel volumes
  - `game_api_planets.cpp` — Planet system
  - `game_api_postfx.cpp` — Post-processing
  - `game_api_debug.cpp` — Debug drawing

## Design Philosophy

`GameAPI::Engine` is designed to:
- Provide stable, game-friendly snake_case methods
- Abstract Vulkan/engine internals
- Support both single-precision (`vec3`) and double-precision (`dvec3`) coordinates for large worlds
- Minimize frame-to-frame state management (streaming, caching, resource lifetime handled automatically)
- Allow easy integration with ImGui for debug UI
