## Ray Tracing Manager: BLAS Cache and Per-Frame TLAS

Optional subsystem that enables hybrid or full ray traced shadows via Ray Query. It builds and caches BLAS per mesh and rebuilds a TLAS from the current `DrawContext` when enabled.

- Files: `src/core/vk_raytracing.h/.cpp`

### Device Feature & Extension Enablement
- Feature detection happens in `DeviceManager::init_vulkan()` and sets:
  - `VK_KHR_acceleration_structure`, `VK_KHR_ray_query`, and `VK_KHR_deferred_host_operations` (if supported).
  - Device features are appended via `DeviceBuilder.add_pNext(...)`.
- `DescriptorManager::gpuSceneDataLayout()` adds a TLAS binding at `(set=0, binding=1)` when AS is supported.

### BLAS Build & Cache
- `AccelStructureHandle getOrBuildBLAS(const std::shared_ptr<MeshAsset>& mesh)`:
  - One GAS per `MeshAsset`, keyed by vertex buffer `VkBuffer`.
  - Populated with one triangle geometry per `GeoSurface`.
  - Built with `VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR` and device-local storage + scratch.
  - Cached in `_blasByVB` for reuse across frames.
  - Called from `AssetManager::createMesh(...)` and from GLTF loader after mesh upload.

### TLAS Rebuild Per Frame
- `VkAccelerationStructureKHR buildTLASFromDrawContext(const DrawContext& dc)`:
  - Iterates `dc.OpaqueSurfaces` and creates one instance per render object.
  - Looks up BLAS by `RenderObject::vertexBuffer`; if missing, instance is skipped.
  - Uploads instances to a CPU→GPU buffer with device address.
  - Builds TLAS with `immediate_submit` and stores device address for Ray Query.

### Renderer Integration
- In `VulkanEngine::draw()` before building passes:
  - If RT mode is enabled (`shadowSettings.mode != 0`) and manager exists, TLAS is rebuilt from the latest draw context.
- Lighting pass binds the TLAS at `set=0,binding=1` when available.

### Modes & UI
- Mode 0: Shadow maps only (CSM).
- Mode 1: Hybrid — selected cascades assisted by Ray Query (configurable bitmask).
- Mode 2: Ray Query only (no shadow maps).

### Notes & Caveats
- BLAS cache key is the vertex buffer handle; if you rebuild meshes in-place, BLAS must be invalidated.
- CPU→GPU memory is used for the TLAS instance buffer to simplify updates. On some platforms, you may prefer staging + device-local.
- The RT path requires Vulkan 1.2+ with Ray Query and Acceleration Structure features available.

