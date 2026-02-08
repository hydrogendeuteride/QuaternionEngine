# Core

> Engine foundation layer — device management, resource allocation, descriptors, pipelines, asset loading, input, debug drawing, and shared types.

## Purpose

Provides every subsystem that sits between the raw Vulkan API and the
higher-level rendering / scene modules. All core systems are accessible
through the central `EngineContext` dependency-injection container and follow
the same `init → per-frame pump → cleanup` lifecycle pattern.

## Directory Layout

```
core/
├── config.h              — compile-time constants (render resolution, shadow params, VRAM budget)
├── context.h / .cpp      — EngineContext: central DI container holding all manager pointers
├── engine.h / .cpp       — VulkanEngine: top-level engine class, main loop orchestration
├── engine_ui.cpp         — built-in debug UI (ImGui panels for scene, lighting, post-FX, etc.)
├── game_api.h            — GameAPI namespace: high-level stable API for gameplay code
├── types.h               — shared GPU types (AllocatedImage/Buffer, Vertex, GPUSceneData, Node, etc.)
├── world.h               — WorldVec3 (double-precision), floating-origin coordinate helpers
│
├── assets/               — asset loading, texture streaming, IBL environment maps
│   └── README.md
├── debug_draw/           — immediate-mode wireframe debug visualization
│   └── README.md
├── descriptor/           — descriptor set layout creation, writing, and pool management
│   └── README.md
├── device/               — Vulkan device init, VMA resource allocation, swapchain
│   └── README.md
├── frame/                — per-frame state (command buffer, sync primitives, deletion queue)
│   └── README.md
├── game_api/             — GameAPI implementation split by domain (camera, lighting, scene, etc.)
├── input/                — platform-agnostic input system (keyboard, mouse, window events)
│   └── README.md
├── picking/              — object selection (CPU ray-cast / GPU ID-buffer readback)
│   └── README.md
├── pipeline/             — graphics pipeline registry, hot-reload, sampler management
│   └── README.md
├── raytracing/           — Vulkan acceleration structures (BLAS/TLAS) for hybrid shadows
│   └── README.md
├── ui/                   — ImGui integration (Vulkan dynamic rendering, DPI-aware fonts)
│   └── README.md
└── util/                 — Vulkan struct initializers (vkinit) and debug labels (vkdebug)
    └── README.md
```

## Subsystem Overview

| Subfolder | Primary Class | Role |
|-----------|--------------|------|
| `assets/` | `AssetManager`, `TextureCache`, `IBLManager`, `AsyncAssetLoader` | glTF loading, texture streaming with LRU eviction, IBL cubemap pipeline |
| `debug_draw/` | `DebugDrawSystem` | Immediate-mode wireframe primitives (lines, boxes, spheres, etc.) with depth / overlay modes |
| `descriptor/` | `DescriptorManager`, `DescriptorAllocatorGrowable`, `DescriptorWriter` | Shared descriptor layouts, auto-growing pools, batch descriptor writes |
| `device/` | `DeviceManager`, `ResourceManager`, `SwapchainManager` | Vulkan instance/device, VMA allocation, buffer/image creation, swapchain lifecycle |
| `frame/` | `FrameResources` | Per-frame command buffer, sync primitives, transient descriptor pool, `DeletionQueue` |
| `game_api/` | `GameAPI` (namespace) | High-level stable API for cameras, lighting, shadows, particles, textures, scene, post-FX |
| `input/` | `InputSystem`, `InputState` | SDL2-backed keyboard/mouse polling and event system, cursor mode switching |
| `picking/` | `PickingSystem` | Click/hover/drag-box selection via CPU ray-cast or GPU ID-buffer readback |
| `pipeline/` | `PipelineManager`, `SamplerManager` | Named graphics pipeline registry with hot-reload, compute forwarding, preset samplers |
| `raytracing/` | `RayTracingManager` | BLAS caching, async build queue, per-frame TLAS rebuild for `rayQueryEXT` shadows |
| `ui/` | `ImGuiSystem` | Dear ImGui Vulkan backend, DPI-aware fonts, draw callback registration |
| `util/` | `vkinit`, `vkdebug` | Vulkan struct factory functions, command buffer debug labels |

## Root-Level Files

| File | Role |
|------|------|
| `config.h` | Compile-time constants: render resolution (`kRenderWidth/Height`), shadow cascade count/distances/bias, texture budget, validation layer toggle |
| `context.h` | `EngineContext` — central struct holding pointers to all managers, per-frame state, shadow/render settings, engine stats |
| `engine.h/.cpp` | `VulkanEngine` — owns all subsystems, runs init/main-loop/cleanup, orchestrates frame recording and RenderGraph execution |
| `engine_ui.cpp` | Built-in ImGui debug panels: scene hierarchy, lighting controls, shadow settings, post-FX tweaks, performance stats |
| `game_api.h` | `GameAPI` namespace — stable high-level API surface for gameplay code (texture streaming, shadows, cameras, animation, particles, etc.) |
| `types.h` | Shared GPU-visible types: `AllocatedImage`, `AllocatedBuffer`, `GPUMeshBuffers`, `GPUSceneData`, `GPUDrawPushConstants`, `Vertex`, `Node`, `DeletionQueue`, `VK_CHECK` |
| `world.h` | `WorldVec3` (`glm::dvec3`) typedef, `world_to_local` / `local_to_world` coordinate conversion for floating-origin support |

## Lifecycle

```
VulkanEngine::init()
  ├─ DeviceManager::init_vulkan()         ← Vulkan instance, device, VMA
  ├─ ResourceManager::init()              ← immediate-submit, staging
  ├─ SwapchainManager::init()             ← swapchain + render targets
  ├─ DescriptorManager::init()            ← shared descriptor layouts
  ├─ SamplerManager::init()               ← preset VkSamplers
  ├─ PipelineManager::init()              ← pipeline registry + hot-reload thread
  ├─ AssetManager::init()                 ← path resolution, mesh/material factory
  ├─ TextureCache::init()                 ← decode worker threads
  ├─ AsyncAssetLoader::init()             ← background glTF loading threads
  ├─ IBLManager::init()                   ← IBL descriptor layout
  ├─ RayTracingManager::init()            ← acceleration structure function pointers
  ├─ InputSystem()                        ← SDL2 input backend
  ├─ PickingSystem::init()                ← readback buffer
  ├─ DebugDrawSystem()                    ← debug command buffer
  ├─ ImGuiSystem::init()                  ← ImGui context + Vulkan backend
  └─ FrameResources[FRAME_OVERLAP]::init()← per-frame cmd pool, sync, descriptors

── per frame ──
  InputSystem::begin_frame() + pump_events()
  PickingSystem::begin_frame()
  DebugDrawSystem::begin_frame()
  TextureCache::pumpLoads()
  AsyncAssetLoader::pump_main_thread()
  IBLManager::pump_async()
  RayTracingManager::pump_blas_builds()
  RayTracingManager::buildTLASFromDrawContext()
  PipelineManager::hotReloadChanged() + pump_main_thread()
  ResourceManager::register_upload_pass()
  ImGuiSystem::begin_frame() / end_frame()

── shutdown ──
  vkDeviceWaitIdle()
  (reverse order cleanup of all subsystems)
```

## EngineContext (Dependency Injection)

All subsystems are wired together through `EngineContext`, which is passed by
pointer to every render pass, scene operation, and compute dispatch:

```cpp
struct EngineContext {
    DeviceManager*      device;
    ResourceManager*    resources;
    SwapchainManager*   swapchain;
    DescriptorManager*  descriptorLayouts;
    SamplerManager*     samplerManager;
    PipelineManager*    pipelineManager;
    AssetManager*       assets;
    TextureCache*       textures;
    IBLManager*         ibl;
    ComputeManager*     compute;
    RayTracingManager*  raytracing;
    InputSystem*        input;
    FrameResources*     currentFrame;
    // + per-frame state, shadow settings, engine stats, ...
};
```

## GameAPI Split

The `GameAPI` namespace is declared entirely in `game_api.h` and implemented
across multiple files in `game_api/`, split by domain:

| File | Domain |
|------|--------|
| `game_api.cpp` | Core initialization and utility |
| `game_api_camera.cpp` | Camera creation, movement, orbit, look-at |
| `game_api_debug.cpp` | Debug draw wrappers, stats queries |
| `game_api_lighting.cpp` | Sun, point lights, spot lights, shadow settings |
| `game_api_particles.cpp` | Particle emitter creation and control |
| `game_api_planets.cpp` | Procedural planet system API |
| `game_api_postfx.cpp` | Tonemap, bloom, FXAA, SSR settings |
| `game_api_scene.cpp` | Scene loading, node spawning, transforms, animation |
| `game_api_textures.cpp` | Texture streaming, material overrides |
| `game_api_volumetrics.cpp` | Volumetric fog/light settings |

## Integration

`core/` is the foundational layer of the engine. Every other top-level module
depends on it:

- **`src/render/`** — render passes and the RenderGraph consume `DeviceManager`,
  `ResourceManager`, `PipelineManager`, `DescriptorManager`, and `FrameResources`
- **`src/scene/`** — scene graph uses `AssetManager`, `TextureCache`, and
  `ResourceManager` for mesh/material GPU uploads
- **`src/compute/`** — `ComputeManager` uses `DeviceManager` and
  `DescriptorAllocatorGrowable` for compute pipeline management

No module outside `core/` should need to call Vulkan directly — all GPU
operations go through the core abstractions.

## Related Docs

- [docs/EngineContext.md](../../docs/EngineContext.md) — EngineContext detailed documentation
- [docs/ResourceManager.md](../../docs/ResourceManager.md) — VMA resource allocation
- [docs/FrameResources.md](../../docs/FrameResources.md) — per-frame state management
- [docs/Descriptors.md](../../docs/Descriptors.md) — descriptor system
- [docs/PipelineManager.md](../../docs/PipelineManager.md) — pipeline registry
- [docs/RenderGraph.md](../../docs/RenderGraph.md) — DAG-based pass scheduling
- [docs/GameAPI.md](../../docs/GameAPI.md) — high-level gameplay API
