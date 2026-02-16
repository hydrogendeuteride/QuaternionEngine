# Render Passes

> Individual render passes that implement the deferred PBR pipeline stages, post-processing effects, and overlays.

## Purpose

Each pass encapsulates a single stage of the rendering pipeline. All passes
implement the `IRenderPass` interface and expose a `register_graph()` method to
declare their resource dependencies (image/buffer reads, writes, attachments)
on the `RenderGraph`. The graph then handles barrier insertion, layout
transitions, and dynamic rendering begin/end automatically.

## Directory Layout

```
passes/
├── shadow.h / .cpp          — cascaded shadow map (depth-only, per-cascade)
├── background.h / .cpp      — sky background (compute gradient or IBL cubemap)
├── sun_disk.h / .cpp        — analytic sun disk overlay on HDR background
├── geometry.h / .cpp        — deferred G-Buffer fill (position, normal, albedo, extra, ID)
├── lighting.h / .cpp        — deferred fullscreen lighting (PBR + IBL + CSM shadows)
├── ssr.h / .cpp             — screen-space reflections (ray-march G-Buffer)
├── clouds.h / .cpp          — volumetric voxel clouds/smoke (raymarch SSBO density)
├── atmosphere.h / .cpp      — single-scattering Rayleigh/Mie atmosphere
├── particles.h / .cpp       — GPU particle system (compute sim + billboard render)
├── mesh_vfx.h / .cpp        — mesh-based VFX pass (unlit emissive + alpha/fresnel)
├── auto_exposure.h / .cpp   — compute average luminance + CPU readback for exposure
├── tonemap.h / .cpp         — HDR→LDR tonemap (ACES/Reinhard) with bloom
├── fxaa.h / .cpp            — FXAA anti-aliasing on LDR output
├── transparent.h / .cpp     — forward-rendered transparent objects (alpha blend)
├── debug_draw.h / .cpp      — wireframe debug primitives (lines, boxes, spheres)
└── imgui_pass.h / .cpp      — Dear ImGui rendering onto swapchain
```

## Pipeline Order

Passes are registered on the `RenderGraph` each frame in this order
(the graph may reorder via topological sort based on declared dependencies):

```
1. Shadow          — depth-only CSM cascades (N passes, one per cascade)
2. Background      — sky fill (compute or IBL cubemap)
3. SunDisk         — analytic sun overlay on HDR buffer
4. Geometry        — deferred G-Buffer fill (5 MRT + depth)
5. Lighting        — fullscreen deferred PBR + IBL + shadow sampling
6. SSR             — screen-space reflections (HDR)
7. Volumetrics     — voxel cloud/smoke raymarch (HDR)
8. Atmosphere      — Rayleigh/Mie scattering (HDR)
9. Particles       — GPU particle render (HDR, depth-tested)
10. MeshVFX        — mesh-based half-transparent effects (HDR)
11. Transparent    — forward alpha-blended objects (HDR)
12. AutoExposure   — compute luminance measurement
13. Tonemap        — HDR → LDR with bloom
14. FXAA           — anti-aliasing (LDR)
15. DebugDraw      — wireframe overlays
16. ImGui          — debug UI on swapchain
17. PresentChain   — letterbox blit + layout transition to PRESENT_SRC
```

## Key Types

| Type | Role |
|------|------|
| `IRenderPass` | Abstract interface: `init`, `cleanup`, `execute`, `getName` |
| `RenderPassManager` | Owns all pass instances, manages lifecycle |
| `ShadowPass` | Depth-only CSM with N cascades; per-cascade render graph registration |
| `BackgroundPass` | Compute gradient effects or IBL environment cubemap sampling |
| `SunDiskPass` | Analytic sun disk rendered as fullscreen quad |
| `GeometryPass` | Deferred G-Buffer fill with 5 color attachments + depth |
| `LightingPass` | Fullscreen deferred PBR: G-Buffer sampling, CSM, IBL, point/spot lights |
| `SSRPass` | Screen-space reflections via ray-march on G-Buffer depth/normal |
| `CloudPass` | Volumetric clouds: raymarch bounded volume with SSBO voxel density grid |
| `AtmospherePass` | Single-scattering Rayleigh/Mie atmosphere (per-pixel ray integration) |
| `ParticlePass` | GPU particle system: compute simulation + billboard rendering with flipbook/noise |
| `MeshVfxPass` | Mesh-based unlit VFX pass with alpha blend and fresnel controls |
| `AutoExposurePass` | Compute average log-luminance, CPU readback, smooth exposure adaptation |
| `TonemapPass` | HDR→LDR tonemap (ACES default), bloom threshold/intensity controls |
| `FxaaPass` | FXAA post-process anti-aliasing with configurable edge thresholds |
| `TransparentPass` | Forward-rendered transparent objects with alpha blending |
| `DebugDrawPass` | Wireframe debug primitives from `DebugDrawSystem` |
| `ImGuiPass` | Dear ImGui draw data recording onto swapchain image |

## Pass Pattern

Every pass follows the same pattern:

```cpp
class MyPass : public IRenderPass {
public:
    void init(EngineContext* context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;  // legacy; unused when using RenderGraph
    const char* getName() const override { return "MyPass"; }

    // Declare dependencies and register record callback on the graph
    void register_graph(RenderGraph* graph, RGImageHandle input, RGImageHandle output);

private:
    // Actual rendering logic, invoked by the graph's record callback
    void draw_my_pass(VkCommandBuffer cmd, EngineContext* ctx,
                      const RGPassResources& res, RGImageHandle input);
};
```

- `init()` — create pipelines, descriptor layouts, fallback resources
- `register_graph()` — declare image/buffer reads/writes via `RGPassBuilder`, capture record lambda
- `draw_*()` — bind pipeline, descriptors, push constants, issue draw/dispatch calls
- `cleanup()` — destroy owned GPU resources via `DeletionQueue`

## Integration

All passes are instantiated and owned by `RenderPassManager` (constructed in
`renderpass.cpp`). The engine calls `register_graph()` on each pass during
frame recording in `VulkanEngine::draw()`. Passes access all engine subsystems
through `EngineContext*`.

Passes that produce intermediate images (e.g. `TonemapPass`, `FxaaPass`,
`AtmospherePass`, `CloudPass`) return new `RGImageHandle`s from `register_graph()`
so subsequent passes can chain outputs.

## Related Docs

- [graph/README.md](../graph/README.md) — RenderGraph system (barrier/layout automation)
- [docs/RenderGraph.md](../../../docs/RenderGraph.md) — detailed render graph documentation
- [docs/Shadows.md](../../../docs/Shadows.md) — cascaded shadow map system
- [docs/Particles.md](../../../docs/Particles.md) — GPU particle system
