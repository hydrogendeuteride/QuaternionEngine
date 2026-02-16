# Render

> Rendering layer — DAG-based render graph, deferred PBR pipeline passes, material system, pipeline builder, and procedural primitives.

## Purpose

Contains everything between the core engine abstractions and the final
presented image. The `RenderGraph` schedules and synchronizes passes
automatically; individual render passes implement each pipeline stage
(shadows, G-Buffer, lighting, post-processing, overlays); and shared
utilities provide material definitions, pipeline construction helpers,
and procedural mesh generation.

## Directory Layout

```
render/
├── renderpass.h / .cpp   — IRenderPass interface + RenderPassManager (pass ownership and lifecycle)
├── materials.h / .cpp    — GLTFMetallic_Roughness: PBR material pipelines, descriptor writes
├── pipelines.h / .cpp    — PipelineBuilder: Vulkan graphics pipeline construction helper
├── primitives.h          — primitives namespace: CPU-side mesh generators (cube, sphere, plane, capsule)
│
├── graph/                — RenderGraph: DAG pass scheduling, automatic barriers, transient resources
│   └── README.md
└── passes/               — individual render passes (shadow, geometry, lighting, post-FX, overlays)
    └── README.md
```

## Subsystem Overview

| Subfolder / File | Primary Class | Role |
|------------------|--------------|------|
| `graph/` | `RenderGraph`, `RGResourceRegistry`, `RGPassBuilder` | DAG-based pass scheduling with automatic barrier insertion, layout transitions, and transient resource management |
| `passes/` | `IRenderPass` implementations | 16 render passes: shadow, background, sun disk, geometry, lighting, SSR, volumetrics, atmosphere, particles, mesh VFX, transparent, auto-exposure, tonemap, FXAA, debug draw, ImGui |
| `renderpass.h` | `IRenderPass`, `RenderPassManager` | Abstract pass interface (`init`/`cleanup`/`execute`/`getName`) and pass registry with typed lookup |
| `materials.h` | `GLTFMetallic_Roughness` | PBR metallic-roughness material: opaque/transparent/mesh-VFX/G-Buffer pipeline variants, descriptor set writing for 5 texture maps + constants |
| `pipelines.h` | `PipelineBuilder` | Fluent builder for `VkGraphicsPipeline`: shader stages, topology, rasterizer, blending, depth, MRT formats |
| `primitives.h` | `primitives` namespace | CPU mesh generators: `buildCube`, `buildSphere`, `buildPlane`, `buildCapsule` producing `Vertex`/index arrays |

## Root-Level Files

| File | Role |
|------|------|
| `renderpass.h/.cpp` | `IRenderPass` — pure virtual interface all passes implement. `RenderPassManager` — constructs, owns, and manages lifecycle of all pass instances + ImGui pass |
| `materials.h/.cpp` | `GLTFMetallic_Roughness` — builds opaque, transparent, mesh-VFX, and G-Buffer material pipelines; writes descriptor sets binding color/metallic-roughness/normal/occlusion/emissive textures + material constants buffer |
| `pipelines.h/.cpp` | `PipelineBuilder` — wraps `VkGraphicsPipelineCreateInfo` construction with fluent API. `vkutil::load_shader_module` loads SPIR-V binaries |
| `primitives.h` | Header-only procedural mesh generators in `primitives` namespace: unit cube, sphere (configurable sectors/stacks), XZ plane, Y-axis capsule |

## Rendering Pipeline

The full deferred PBR pipeline, executed via RenderGraph each frame:

```
┌─────────────────────────────────────────────────────────┐
│ Shadow (CSM)     — depth-only, N cascades               │
├─────────────────────────────────────────────────────────┤
│ Background       — sky gradient / IBL cubemap → HDR     │
│ SunDisk          — analytic sun overlay → HDR           │
├─────────────────────────────────────────────────────────┤
│ Geometry         — G-Buffer fill (5 MRT + depth)        │
│ Lighting         — fullscreen deferred PBR + IBL + CSM  │
├─────────────────────────────────────────────────────────┤
│ SSR              — screen-space reflections (HDR)       │
│ Volumetrics      — voxel cloud/smoke raymarch (HDR)     │
│ Atmosphere       — Rayleigh/Mie scattering (HDR)        │
│ Particles        — GPU particle render (HDR)            │
│ MeshVFX          — unlit mesh VFX (HDR, alpha/fresnel)  │
│ Transparent      — forward alpha-blended objects (HDR)  │
├─────────────────────────────────────────────────────────┤
│ AutoExposure     — compute luminance measurement        │
│ Tonemap + Bloom  — HDR → LDR (ACES/Reinhard)           │
│ FXAA             — post-process AA (LDR)                │
├─────────────────────────────────────────────────────────┤
│ DebugDraw        — wireframe overlays                   │
│ ImGui            — debug UI on swapchain                │
│ PresentChain     — letterbox blit → PRESENT_SRC         │
└─────────────────────────────────────────────────────────┘
```

## Lifecycle

```
VulkanEngine::init()
  ├─ RenderPassManager::init()
  │   ├─ BackgroundPass::init()
  │   ├─ SunDiskPass::init()
  │   ├─ ShadowPass::init()
  │   ├─ GeometryPass::init()
  │   ├─ LightingPass::init()
  │   ├─ SSRPass::init()
  │   ├─ CloudPass::init()
  │   ├─ AtmospherePass::init()
  │   ├─ ParticlePass::init()
  │   ├─ FxaaPass::init()
  │   ├─ DebugDrawPass::init()
  │   ├─ TransparentPass::init()
  │   ├─ AutoExposurePass::init()
  │   └─ TonemapPass::init()
  ├─ RenderPassManager::setImGuiPass()
  └─ RenderGraph::init()

── per frame ─────────────────────────────────────
  RenderGraph::clear()
  import images (draw, depth, g-buffers, swapchain)
  ShadowPass::register_graph()
  BackgroundPass::register_graph()
  SunDiskPass::register_graph()
  GeometryPass::register_graph()
  LightingPass::register_graph()
  SSRPass::register_graph()
  CloudPass::register_graph()
  AtmospherePass::register_graph()
  ParticlePass::register_graph()
  AutoExposurePass::register_graph()
  TonemapPass::register_graph()
  FxaaPass::register_graph()
  TransparentPass::register_graph()
  DebugDrawPass::register_graph()
  RenderGraph::add_present_chain()
  ImGuiPass::register_graph()
  RenderGraph::compile()
  RenderGraph::execute(cmd)
  RenderGraph::resolve_timings()
──────────────────────────────────────────────────

── shutdown ──
  RenderGraph::shutdown()
  RenderPassManager::cleanup()
```

## Integration

`render/` sits between `core/` (engine foundation) and `src/scene/` (scene graph):

- **Depends on `core/`** — `DeviceManager`, `ResourceManager`, `PipelineManager`,
  `DescriptorManager`, `SamplerManager`, `FrameResources`, `EngineContext`
- **Consumed by `VulkanEngine`** — engine orchestrates per-frame graph construction
  and execution in `draw()`
- **Reads from `scene/`** — `DrawContext` provides render object lists for geometry,
  shadow, and transparent passes

`GLTFMetallic_Roughness` material pipelines are built once at init and shared
by both `GeometryPass` (G-Buffer variant) and `TransparentPass` (forward variant).

## Related Docs

- [graph/README.md](graph/README.md) — RenderGraph internals
- [passes/README.md](passes/README.md) — individual pass documentation
- [docs/RenderGraph.md](../../docs/RenderGraph.md) — detailed render graph system documentation
- [docs/Shadows.md](../../docs/Shadows.md) — cascaded shadow maps
- [docs/Particles.md](../../docs/Particles.md) — GPU particle system
- [docs/SSR.md](../../docs/SSR.md) — screen-space reflections
