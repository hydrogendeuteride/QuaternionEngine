# Ray Tracing

> Manages Vulkan acceleration structures (BLAS/TLAS) for hybrid and full ray-query shadows.

## Purpose

Provides BLAS (Bottom-Level Acceleration Structure) caching per mesh and per-frame
TLAS (Top-Level Acceleration Structure) rebuilds. BLAS builds are spread across
multiple frames via an async queue to avoid large frame-time spikes. The resulting
TLAS is consumed by shadow render passes that use `VK_KHR_ray_query` for
hardware-accelerated shadow rays.

## Directory Layout

```
raytracing/
├── raytracing.h    — AccelStructureHandle, RayTracingManager class
└── raytracing.cpp  — implementation
```

## Key Types

| Type | Role |
|------|------|
| `RayTracingManager` | Central entry point — BLAS caching, async build queue, TLAS rebuild |
| `AccelStructureHandle` | RAII-style handle holding `VkAccelerationStructureKHR`, backing buffer, and device address |
| `PendingBlasBuild` | Internal queue entry for deferred BLAS builds |

## Lifecycle

```
init(DeviceManager*, ResourceManager*)
  ├─ resolves VK_KHR_acceleration_structure function pointers
  └─ queries minAccelerationStructureScratchOffsetAlignment

getOrBuildBLAS(mesh)
  ├─ returns cached BLAS if available
  ├─ returns empty handle if build is already queued
  └─ enqueues new PendingBlasBuild otherwise

pump_blas_builds(max_per_frame=1)          ← call once per frame
  └─ pops up to N jobs from queue, calls build_blas_for_mesh()
       └─ immediate-submits BLAS build command, caches result

buildTLASFromDrawContext(dc, frameDQ)      ← call once per frame
  ├─ collects instances from opaque surfaces with ready BLAS
  ├─ uploads instance buffer (CPU_TO_GPU)
  ├─ ensure_tlas_storage() — recreates TLAS if size changed (old deferred via DQ)
  └─ immediate-submits TLAS build command

flushPendingDeletes()                      ← call after GPU fence wait
  └─ destroys BLAS handles scheduled for deferred deletion

cleanup()
  └─ destroys all BLAS, TLAS, instance buffer, pending deletes
```

## Usage

### Queuing a BLAS build

```cpp
// Queues an async build; returns empty handle until ready
AccelStructureHandle blas = ctx->raytracing->getOrBuildBLAS(meshAsset);
```

### Per-frame pump and TLAS rebuild

```cpp
// After GPU fence wait — flush stale BLAS
ctx->raytracing->flushPendingDeletes();

// Build up to 1 BLAS per frame from the async queue
ctx->raytracing->pump_blas_builds(1);

// Rebuild TLAS from current draw context
VkAccelerationStructureKHR tlas = ctx->raytracing->buildTLASFromDrawContext(drawContext, frameDeletionQueue);
```

### Removing a cached BLAS

```cpp
// When a mesh is destroyed or its GPU buffers are freed:
ctx->raytracing->removeBLASForMesh(meshPtr);
// or by vertex buffer handle:
ctx->raytracing->removeBLASForBuffer(vertexBuffer);
// Actual destruction is deferred until flushPendingDeletes()
```

## Design Notes

- **Async BLAS builds:** `getOrBuildBLAS()` never blocks; it enqueues work that `pump_blas_builds()` processes incrementally. This spreads build cost across frames.
- **Deferred destruction:** Old TLAS storage is pushed to the frame `DeletionQueue`; removed BLAS handles are batched in `_pendingBlasDestroy` and flushed after fence wait. This prevents freeing resources still referenced by in-flight frames.
- **Extension loading:** `VK_KHR_acceleration_structure` functions are resolved via `vkGetDeviceProcAddr` at init time rather than relying on the loader.
- **Scratch alignment:** Scratch buffers are padded to `minAccelerationStructureScratchOffsetAlignment` (minimum 256 bytes) as required by the spec.

## Integration

`RayTracingManager` is owned by `VulkanEngine` and exposed via `EngineContext`.
Shadow passes query `tlas()` / `tlasAddress()` to bind the acceleration structure
for `rayQueryEXT` in lighting shaders.

## Related Docs

- [docs/RayTracing.md](../../../docs/RayTracing.md) — high-level ray tracing system documentation
