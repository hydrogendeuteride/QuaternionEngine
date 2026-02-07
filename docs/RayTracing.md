## Ray Tracing Manager: BLAS Cache and Per-Frame TLAS

Optional subsystem that enables hybrid or full ray traced shadows via Ray Query. It builds and caches BLAS per mesh and rebuilds a TLAS from the current `DrawContext` when enabled.

- Files: `src/core/raytracing/raytracing.h/.cpp`

### Device Feature & Extension Enablement
- Feature detection happens in `DeviceManager::init_vulkan()` and sets:
  - `VK_KHR_acceleration_structure`, `VK_KHR_ray_query`, and `VK_KHR_deferred_host_operations` (if supported).
  - Device features are appended via `DeviceBuilder.add_pNext(...)`.
- `DescriptorManager::gpuSceneDataLayout()` adds a TLAS binding at `(set=0, binding=1)` when AS is supported.

### BLAS Build & Cache
- `AccelStructureHandle getOrBuildBLAS(const std::shared_ptr<MeshAsset>& mesh)`:
  - One GAS per `MeshAsset`, keyed by mesh pointer.
  - Populated with one triangle geometry per `GeoSurface`.
  - Built with `VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR` and device-local storage + scratch.
  - Cached in `_blasByMesh` for reuse across frames.
  - When a BLAS does not exist yet, the mesh is queued for an asynchronous build and an empty handle is returned; callers must treat this as "BLAS not ready" and skip the instance for the current frame (see TLAS section below).
  - `pump_blas_builds(max_builds_per_frame)` advances an internal BLAS build queue and is called once per frame from the engine main loop to spread work across multiple frames instead of doing all BLAS builds in a single spike.

### TLAS Rebuild Per Frame
- `VkAccelerationStructureKHR buildTLASFromDrawContext(const DrawContext& dc)`:
  - Iterates `dc.OpaqueSurfaces` and creates one instance per render object.
  - Looks up BLAS by `RenderObject::sourceMesh` (the `MeshAsset*`); if a BLAS is not cached yet, it calls `getOrBuildBLAS` with a non-owning `shared_ptr` to queue a build and then skips the instance for this frame.
  - Uploads instances to a CPU→GPU buffer with device address.
  - Builds TLAS with `immediate_submit` and stores device address for Ray Query.

### Renderer Integration
- In `VulkanEngine::draw()` before building passes:
  - If RT mode is enabled (`shadowSettings.mode != 0`) or ray-traced SSR is enabled (`enableSSR && reflectionMode != 0`), and the manager exists, TLAS is rebuilt from the latest draw context.
  - TLAS only references BLAS that are already built; instances whose meshes are still in the BLAS build queue are skipped until their BLAS completes.
- In `VulkanEngine::run()` at the start of each frame (after waiting for the previous frame fence):
  - Calls `RayTracingManager::flushPendingDeletes()` to safely destroy any BLAS scheduled for deferred deletion.
  - Calls `RayTracingManager::pump_blas_builds(1)` to build at most one queued BLAS per frame (tunable if you want more or fewer builds per frame).
- Lighting pass binds the TLAS at `set=0,binding=1` when available.

### Modes & UI
- Mode 0: Shadow maps only (CSM).
- Mode 1: Hybrid — selected cascades assisted by Ray Query (configurable bitmask).
- Mode 2: Ray Query only (no shadow maps).

### Notes & Caveats
- BLAS cache key is the `MeshAsset*`. If you destroy or rebuild meshes (or their GPU buffers) you must invalidate associated BLAS via `RayTracingManager::removeBLASForMesh(mesh)` or `removeBLASForBuffer(vertexBuffer)`.
- CPU→GPU memory is used for the TLAS instance buffer to simplify updates. On some platforms, you may prefer staging + device-local.
- Because BLAS builds are asynchronous and capped per frame, newly spawned meshes may take a few frames before they appear in the ray-traced path; this is a deliberate tradeoff to avoid large hitches when many meshes are introduced.
- The RT path requires Vulkan 1.2+ with Ray Query and Acceleration Structure features available.

