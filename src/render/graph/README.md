# Render Graph

> DAG-based render pass scheduler with automatic barrier insertion, layout transitions, and dynamic rendering.

## Purpose

Provides a declarative, per-frame render graph that manages Vulkan synchronization
automatically. Passes declare which images and buffers they read/write; the graph
topologically sorts passes, computes precise `VkImageMemoryBarrier2` /
`VkBufferMemoryBarrier2` sequences, and records dynamic rendering begin/end around
graphics passes. Supports both imported (externally owned) and transient
(graph-allocated, frame-scoped) resources.

## Directory Layout

```
graph/
├── types.h        — enums (RGPassType, RGImageUsage, RGBufferUsage), handle types, descriptor structs
├── resources.h    — RGResourceRegistry: image/buffer record storage and lookup
├── resources.cpp  — implementation (import dedup, transient allocation, cleanup)
├── builder.h      — RGPassBuilder (pass declaration API), RGPassResources (record-time resource access)
├── builder.cpp    — implementation
├── graph.h        — RenderGraph class: pass registration, compile, execute, debug helpers
└── graph.cpp      — implementation (topo-sort, barrier gen, dynamic rendering, GPU timing)
```

## Key Types

| Type | Role |
|------|------|
| `RenderGraph` | Central entry point — pass registration, compile, execute, timing |
| `RGResourceRegistry` | Tracks all image/buffer records; deduplicates imports, allocates transients |
| `RGPassBuilder` | Builder passed to `add_pass` setup lambda — declare reads, writes, attachments |
| `RGPassResources` | Read-only accessor passed to record callback — resolve handles to `VkImage`/`VkBuffer` |
| `RGImageHandle` / `RGBufferHandle` | Lightweight opaque handles (uint32 ID) to graph-managed resources |
| `RGPassType` | `Graphics`, `Compute`, `Transfer` |
| `RGImageUsage` | `SampledFragment`, `SampledCompute`, `ColorAttachment`, `DepthAttachment`, `ComputeWrite`, `TransferSrc/Dst`, `Present` |
| `RGBufferUsage` | `VertexRead`, `IndexRead`, `UniformRead`, `StorageRead`, `StorageReadWrite`, `IndirectArgs`, `TransferSrc/Dst` |
| `RGAttachmentInfo` | Color/depth attachment descriptor with clear/load/store control |

## Lifecycle

```
init(EngineContext*)
  └─ initializes resource registry

── per frame ──────────────────────────────────────
clear()
  └─ resets passes and resource registry

import_image() / import_draw_image() / import_depth_image() / ...
  └─ register externally owned images (swapchain, g-buffers, depth)

import_buffer()
  └─ register externally owned buffers

create_image() / create_depth_image() / create_buffer()
  └─ allocate transient resources (freed via DeletionQueue at frame end)

add_pass(name, type, buildCallback, recordCallback)
  ├─ buildCallback(RGPassBuilder&)  ← declare reads/writes/attachments
  └─ recordCallback(cmd, RGPassResources&, ctx)  ← record Vulkan commands

compile()
  ├─ topological sort via Kahn's algorithm (RAW/WAR/WAW edges)
  ├─ compute per-pass VkImageMemoryBarrier2 / VkBufferMemoryBarrier2
  ├─ track image layouts across passes
  └─ record resource lifetime (firstUse/lastUse)

execute(cmd)
  ├─ create GPU timestamp query pool
  ├─ for each enabled pass:
  │   ├─ emit debug label
  │   ├─ pipeline barrier (pre-pass)
  │   ├─ begin dynamic rendering (if color/depth attachments)
  │   ├─ invoke record callback
  │   ├─ end dynamic rendering
  │   └─ write GPU timestamp
  └─ measure CPU time per pass

resolve_timings()
  └─ read back GPU timestamps → per-pass ms
───────────────────────────────────────────────────

shutdown()
  └─ destroy timestamp query pool
```

## Usage

### Declaring a graphics pass

```cpp
auto drawImg   = graph.import_draw_image();
auto depthImg  = graph.import_depth_image();
auto gbufPos   = graph.import_gbuffer_position();

graph.add_pass("GeometryPass", RGPassType::Graphics,
    [=](RGPassBuilder& b, EngineContext*) {
        b.write_color(drawImg, true, clearColor);
        b.write_color(gbufPos, true);
        b.write_depth(depthImg, true, depthClear);
    },
    [](VkCommandBuffer cmd, const RGPassResources& res, EngineContext* ctx) {
        // bind pipelines, draw meshes ...
    });
```

### Declaring a compute pass

```cpp
graph.add_pass("Tonemap", RGPassType::Compute,
    [=](RGPassBuilder& b, EngineContext*) {
        b.read(drawImg, RGImageUsage::SampledCompute);
        b.write(outputImg, RGImageUsage::ComputeWrite);
    },
    [](VkCommandBuffer cmd, const RGPassResources& res, EngineContext* ctx) {
        // bind compute pipeline, dispatch ...
    });
```

### Present chain helper

```cpp
auto swapImg = graph.import_swapchain_image(swapchainIndex);
graph.add_present_chain(drawImg, swapImg);
// Adds PresentLetterbox + PreparePresent passes automatically
```

### Buffer tracking

```cpp
graph.add_pass("ParticleSim", RGPassType::Compute,
    [=](RGPassBuilder& b, EngineContext*) {
        b.write_buffer(particleBuf, RGBufferUsage::StorageReadWrite);
    },
    [](VkCommandBuffer cmd, const RGPassResources& res, EngineContext* ctx) {
        // dispatch particle compute ...
    });
```

## Integration

`RenderGraph` is owned by `VulkanEngine` and rebuilt each frame (clear → import → add passes → compile → execute).
It is accessed via `EngineContext`. Each render pass in `src/render/passes/` registers itself
through `add_pass` with appropriate build and record callbacks.

Convenience import helpers (`import_draw_image`, `import_gbuffer_*`, etc.) read
from `EngineContext::swapchain` to avoid boilerplate.

Transient images/buffers are allocated via `ResourceManager` (VMA) and enqueued on
the per-frame `DeletionQueue` for automatic cleanup.

## Debug

`debug_get_passes()`, `debug_get_images()`, `debug_get_buffers()` expose per-frame
introspection data (pass names, resource lifetimes, attachment counts). `resolve_timings()`
provides per-pass GPU/CPU milliseconds via timestamp queries, viewable in the ImGui
render graph inspector.

## Related Docs

- [docs/RenderGraph.md](../../../docs/RenderGraph.md) — detailed render graph system documentation
