# Vulkan Engine Documentation

## Quick Start

- **[BUILD.md](BUILD.md)** — Build instructions, dependencies, and platform-specific setup
- **[RUNTIME.md](RUNTIME.md)** — Runtime architecture and execution flow
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** — Common issues and solutions

## Core Architecture

### Engine Foundation

- **[EngineContext.md](EngineContext.md)** — Central dependency injection container and per-frame state
- **[FrameResources.md](FrameResources.md)** — Frame-in-flight synchronization and resource management
- **[ResourceManager.md](ResourceManager.md)** — VMA-based GPU memory allocation and resource lifecycle
- **[FloatingOrigin.md](FloatingOrigin.md)** — Large-world support with double-precision coordinates

### Rendering

- **[RenderGraph.md](RenderGraph.md)** — DAG-based render pass scheduling with automatic barriers
- **[RenderPasses.md](RenderPasses.md)** — Built-in passes: geometry, lighting, SSR, volumetrics, particles, tonemap, FXAA
- **[PipelineManager.md](PipelineManager.md)** — Graphics/compute pipeline creation and hot-reloading
- **[Descriptors.md](Descriptors.md)** — Descriptor set management and binding strategies
- **[SHADERS.md](SHADERS.md)** — Shader compilation, includes, and conventions

### Advanced Rendering Features

- **[MultiLighting.md](MultiLighting.md)** — Deferred lighting with point/spot lights and IBL
- **[IBL.md](IBL.md)** — Image-based lighting and local reflection probes
- **[RayTracing.md](RayTracing.md)** — Ray-traced shadows and reflections with hybrid modes
- **[ParticleSystem.md](ParticleSystem.md)** — GPU particle simulation (128K particles, flipbook, soft particles)
- **[Volumetrics.md](Volumetrics.md)** — Voxel-based clouds, smoke, and flame with raymarching
- **[materials.md](materials.md)** — PBR material system and texture bindings

### Scene Management

- **[Scene.md](Scene.md)** — Scene graph, node hierarchy, and draw context
- **[ASSETS.md](ASSETS.md)** — Asset management overview
- **[asset_manager.md](asset_manager.md)** — AssetManager API and async loading
- **[TextureLoading.md](TextureLoading.md)** — Texture streaming, VRAM budgeting, and KTX2 support
- **[Picking.md](Picking.md)** — BVH-based object picking and selection

### UI and Input

- **[ImGuiSystem.md](ImGuiSystem.md)** — ImGui integration and debug UI
- **[InputSystem.md](InputSystem.md)** — Keyboard, mouse, and cursor handling

### Compute and Effects

- **[Compute.md](Compute.md)** — Compute pipeline creation and dispatch

### Game Development API

- **[GameAPI.md](GameAPI.md)** — High-level game-facing API (textures, lighting, picking, particles, volumetrics)
- **[debug_draw_api_examples.md](debug_draw_api_examples.md)** — Debug drawing examples (lines, spheres, AABBs, etc.)

## Documentation Organization

### By System

**Core Systems:**
- Engine: [EngineContext.md](EngineContext.md), [FrameResources.md](FrameResources.md), [ResourceManager.md](ResourceManager.md)
- Rendering: [RenderGraph.md](RenderGraph.md), [RenderPasses.md](RenderPasses.md), [PipelineManager.md](PipelineManager.md)
- Scene: [Scene.md](Scene.md), [asset_manager.md](asset_manager.md), [TextureLoading.md](TextureLoading.md)

**Rendering Features:**
- Lighting: [MultiLighting.md](MultiLighting.md), [IBL.md](IBL.md)
- Effects: [ParticleSystem.md](ParticleSystem.md), [Volumetrics.md](Volumetrics.md)
- Post-processing: [RenderPasses.md](RenderPasses.md) (SSR, Tonemap, FXAA sections)
- Ray Tracing: [RayTracing.md](RayTracing.md)

**Developer Tools:**
- Debugging: [debug_draw_api_examples.md](debug_draw_api_examples.md), [ImGuiSystem.md](ImGuiSystem.md)
- Input: [InputSystem.md](InputSystem.md), [Picking.md](Picking.md)

### By Task

**Setting up the engine:**
1. [BUILD.md](BUILD.md) — Build and dependencies
2. [RUNTIME.md](RUNTIME.md) — Understanding the runtime loop
3. [EngineContext.md](EngineContext.md) — Core architecture
4. [GameAPI.md](GameAPI.md) — High-level API

**Creating content:**
1. [ASSETS.md](ASSETS.md) — Asset pipeline overview
2. [TextureLoading.md](TextureLoading.md) — Loading textures
3. [Scene.md](Scene.md) — Adding objects to the scene
4. [materials.md](materials.md) — Material setup

**Adding effects:**
1. [MultiLighting.md](MultiLighting.md) — Point/spot lights
2. [ParticleSystem.md](ParticleSystem.md) — Particles (fire, smoke, sparks)
3. [Volumetrics.md](Volumetrics.md) — Clouds and atmospheric effects
4. [IBL.md](IBL.md) — Environment lighting

**Debugging and visualization:**
1. [debug_draw_api_examples.md](debug_draw_api_examples.md) — Debug primitives
2. [ImGuiSystem.md](ImGuiSystem.md) — Debug UI
3. [Picking.md](Picking.md) — Object selection

**Optimizing performance:**
1. [TextureLoading.md](TextureLoading.md) — VRAM budgeting
2. [RenderGraph.md](RenderGraph.md) — Render pass optimization
3. [FrameResources.md](FrameResources.md) — Frame synchronization

**Writing shaders:**
1. [SHADERS.md](SHADERS.md) — Shader conventions
2. [Descriptors.md](Descriptors.md) — Descriptor bindings
3. [RenderPasses.md](RenderPasses.md) — Custom passes

**Advanced topics:**
1. [RayTracing.md](RayTracing.md) — Hardware ray tracing
2. [FloatingOrigin.md](FloatingOrigin.md) — Large worlds
3. [Compute.md](Compute.md) — GPU compute

## Rendering Pipeline Overview

The engine uses a deferred PBR pipeline with the following stages:

1. **Background** — Sky/gradient generation (compute)
2. **Geometry** — G-Buffer pass (position, normal, albedo, AO/emissive)
3. **Shadows** — Cascaded shadow maps (4 cascades, optional RT)
4. **Lighting** — Deferred PBR lighting (point/spot/directional, IBL)
5. **SSR** — Screen-space reflections (optional RT fallback)
6. **Volumetrics** — Voxel clouds/smoke/flame (up to 4 volumes)
7. **Particles** — GPU particle systems (up to 128K particles)
8. **Tonemap + Bloom** — HDR → LDR conversion
9. **FXAA** — Anti-aliasing
10. **Transparent** — Forward rendering for transparent objects
11. **DebugDraw** — Debug visualization
12. **ImGui** — UI overlay
13. **Present** — Swapchain presentation

See [RenderPasses.md](RenderPasses.md) for details.

## Key Features

- **Modern Vulkan API** — Dynamic rendering, synchronization2, ray query
- **Deferred PBR Pipeline** — Physically-based materials with IBL
- **GPU-Driven Systems** — Particles and volumetrics fully GPU-simulated
- **Render Graph** — Automatic barrier insertion and resource management
- **Ray Tracing** — Hybrid shadows and reflections (optional)
- **Texture Streaming** — VRAM budgeting with LRU eviction
- **Floating-Origin** — Double-precision world coordinates for large worlds
- **Hot-Reload** — Shader recompilation without restart
- **Debug Tools** — Immediate-mode debug drawing and ImGui integration

## Architecture Highlights

### Rendering

- **Render Graph** ([RenderGraph.md](RenderGraph.md)): DAG-based execution with automatic resource transitions
- **Pipeline Manager** ([PipelineManager.md](PipelineManager.md)): Hot-reloadable shaders and compute pipelines
- **Multi-Lighting** ([MultiLighting.md](MultiLighting.md)): Clustered forward+ deferred hybrid

### GPU-Driven Effects

- **Particles** ([ParticleSystem.md](ParticleSystem.md)): 128K particle global pool, compute-based simulation, block-level depth sorting
- **Volumetrics** ([Volumetrics.md](Volumetrics.md)): Semi-Lagrangian advection, procedural noise injection, raymarch composite

### Asset Pipeline

- **Async Loading** ([asset_manager.md](asset_manager.md)): Background thread pool with priority queuing
- **Texture Streaming** ([TextureLoading.md](TextureLoading.md)): Automatic VRAM management with upload budgeting
- **KTX2 Support**: Compressed texture formats (BC7, ASTC) with mipmaps

### Developer Experience

- **GameAPI** ([GameAPI.md](GameAPI.md)): Stable, high-level C++ API abstracting Vulkan details
- **Debug Drawing** ([debug_draw_api_examples.md](debug_draw_api_examples.md)): Immediate-mode primitives with depth testing
- **ImGui Integration** ([ImGuiSystem.md](ImGuiSystem.md)): Full engine UI with live parameter editing

## Contributing

When adding new features:

1. Update relevant documentation in `docs/`
2. Add examples to [GameAPI.md](GameAPI.md) if exposing new API
3. Include shader documentation in [SHADERS.md](SHADERS.md) for new shaders

## Getting Help

- **Build issues**: [BUILD.md](BUILD.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Runtime errors**: [RUNTIME.md](RUNTIME.md), [EngineContext.md](EngineContext.md)
- **Performance**: [TextureLoading.md](TextureLoading.md), [RenderGraph.md](RenderGraph.md)
- **Usage questions**: [GameAPI.md](GameAPI.md), [debug_draw_api_examples.md](debug_draw_api_examples.md)

