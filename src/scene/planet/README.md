# Planet Folder Guide

This folder contains the planetary body rendering system used by `SceneManager`: cube-sphere geometry, LOD quadtree selection, terrain patch building, height map sampling, and runtime planet management.

## Folder Structure

```
planet/
  planet_system.h / .cpp         # top-level PlanetSystem class: planet CRUD, update/emit, mesh planets
  planet_terrain.cpp             # terrain patch infrastructure: cache, materials, height maps, async build workers
  planet_quadtree.h / .cpp       # screen-space-error LOD quadtree with frustum/horizon culling
  planet_heightmap.h / .cpp      # BC4 KTX2 height map loader and bilinear sampler
  planet_patch_helpers.h / .cpp  # shared patch helpers: bounds, normals, edge stitching, LOD balancing
  cubesphere.h / .cpp            # cube-sphere math: direction mapping, tile UV bounds, patch mesh generation
```

## What Lives Here

### Headers

- `planet_system.h`
  `PlanetSystem` class -- owns a list of `PlanetBody` entries (mesh or terrain), per-body `TerrainState`, and async patch build workers. Exposes CRUD operations (`create_mesh_planet`, `create_terrain_planet`, `destroy_planet`, `clear_planets`), property setters (`set_planet_center`, `set_planet_radius`, `set_planet_visible`, `set_planet_terrain`), quadtree settings, patch cache tuning, debug stats, and `sample_terrain_displacement_m` for physics queries. Key types: `MeshPlanetCreateInfo`, `TerrainPlanetCreateInfo`, `PlanetBody`, `EarthDebugStats`, `TerrainPatch`, `TerrainState`, `TerrainBuildRequest`, `TerrainBuildResult`.

- `planet_quadtree.h`
  `planet::PlanetQuadtree` -- view-dependent LOD selection over six cube faces. Configured via `Settings` (max level, target SSE pixels, frustum/horizon culling, RT guardrail). `update()` runs a DFS that splits patches based on screen-space error, enforces 2:1 LOD balance, and produces a sorted `visible_leaves()` vector of `PatchKey` entries. Also defines `PatchKey` and `PatchKeyHash`.

- `planet_heightmap.h`
  `planet::HeightFace` struct (R8 texel grid) and two free functions: `load_heightmap_bc4` (loads a KTX2 BC4_UNORM file and decodes to R8) and `sample_height` (bilinear interpolation, returns normalized [0..1]).

- `planet_patch_helpers.h`
  Internal helpers in `planet_helpers` namespace shared across the `.cpp` files: `compute_patch_bounds`, `make_planet_constants` (PBR material constants with emission and force-clipmap flag), `debug_color_for_level`, `recompute_patch_normals`, `refine_patch_edge_normals_from_height`, `reinforce_patch_skirts`, `stitch_patch_edges_to_parent_grid`, `compute_patch_edge_stitch_mask`, `patch_needs_balance_split`.

- `cubesphere.h`
  `planet::CubeFace` enum (KTX/Vulkan face order), direction/UV mapping functions (`cubesphere_unit_direction`, `cubesphere_direction_to_face_uv`), tile math (`cubesphere_tile_uv_bounds`, `cubesphere_patch_center_direction`, `cubesphere_patch_center_world`, `cubesphere_patch_edge_m`, `cubesphere_skirt_depth_m`), and mesh builders (`build_cubesphere_patch_indices`, `build_cubesphere_patch_vertices`, `build_cubesphere_patch_mesh`). Also provides `cube_face_name` for file path construction.

### Implementation Files

- `planet_system.cpp`
  Core `PlanetSystem` lifecycle and runtime management: `init`, `cleanup`, `update_and_emit`, planet CRUD (`create_mesh_planet`, `create_terrain_planet`, `destroy_planet`, `clear_planets`), property setters, terrain displacement sampling, and the ready-render-cut algorithm that resolves which patches to draw when async builds are incomplete (falls back to coarser ready ancestors).

- `planet_terrain.cpp`
  Terrain patch infrastructure: `TerrainState` management (`get_or_create_terrain_state`, `find_terrain_state`), patch cache operations (`ensure_terrain_patch`, `clear_terrain_patch_cache`, `trim_terrain_patch_cache`), GPU resource setup (`ensure_earth_patch_index_buffer`, `ensure_earth_patch_material_layout`, `ensure_terrain_material_constants_buffer`, `ensure_terrain_face_materials`, `ensure_terrain_height_maps`), async build pipeline (`start_terrain_patch_workers`, `stop_terrain_patch_workers`, `terrain_patch_worker_loop`, `build_terrain_patch_cpu`, `request_terrain_patch_build`, `commit_terrain_patch_result`, `pump_completed_terrain_patch_builds`).

- `planet_quadtree.cpp`
  `PlanetQuadtree::update` implementation: DFS traversal over six cube faces, per-node visibility (horizon cull, frustum cull), screen-space error refinement, optional RT guardrail, visible-patch budget enforcement, and multi-pass 2:1 LOD balance.

- `planet_heightmap.cpp`
  `load_heightmap_bc4` (KTX2 file I/O via libktx, BC4 block decoding to R8 texels) and `sample_height` (bilinear interpolation with clamp).

- `planet_patch_helpers.cpp`
  Patch geometry helpers: AABB bounds computation, PBR material constant factory, LOD debug tint colors, normal recomputation with edge blending, height-map-based edge normal refinement for stitched boundaries, skirt reinforcement, edge stitching to parent grid (2:1 LOD), edge stitch mask computation, and balance-split detection.

- `cubesphere.cpp`
  Cube-sphere direction mapping (face-to-direction and direction-to-face-UV), tile UV bounds, patch center direction/world position, edge length and skirt depth heuristics, index buffer generation (base grid + skirt quads), vertex generation (positions relative to patch center, face-space UVs, skirts), and full patch mesh builder with optional MikkTSpace tangent generation.

## How It Is Called

`PlanetSystem` is owned by `SceneManager` (`src/scene/vk_scene.h`) and accessed via `SceneManager::get_planet_system()`.

- `SceneManager::update_scene()` calls `PlanetSystem::update_and_emit()`, which runs quadtree LOD selection, manages the async patch build pipeline, and emits terrain/mesh draw objects into the `DrawContext`.

- Game code (e.g., `GameplayState`) accesses `PlanetSystem` through `GameAPI` / `SceneManager` for planet creation, property changes, and terrain displacement queries (`sample_terrain_displacement_m`).

## If You Want To Change...

- Planet creation API, PBR defaults, or body properties:
  Start in `planet_system.h` (`MeshPlanetCreateInfo`, `TerrainPlanetCreateInfo`, `PlanetBody`).

- LOD selection behavior, max level, screen-space error threshold, or culling:
  Start in `planet_quadtree.h` (`Settings`) and `planet_quadtree.cpp`.

- Terrain patch resolution, cache limits, or per-frame build budget:
  Start in `planet_system.h` (tuning getters/setters) and `planet_terrain.cpp`.

- Async terrain build pipeline or worker threading:
  Start in `planet_terrain.cpp` (`start_terrain_patch_workers`, `terrain_patch_worker_loop`, `build_terrain_patch_cpu`).

- Height map loading, format support, or sampling:
  Start in `planet_heightmap.h/.cpp`.

- Patch normal computation, edge stitching, or skirt geometry:
  Start in `planet_patch_helpers.h/.cpp`.

- Cube-sphere direction mapping, UV conventions, or mesh generation:
  Start in `cubesphere.h/.cpp`.

- Per-face terrain textures (albedo, emission) or material setup:
  Start in `planet_terrain.cpp` (`ensure_terrain_face_materials`, `ensure_terrain_height_maps`).

- The ready-render-cut fallback (drawing coarser patches while finer ones build):
  Start in `planet_system.cpp` (`build_ready_render_cut`, `update_and_emit`).

- Debug LOD tinting:
  Start in `planet_patch_helpers.cpp` (`debug_color_for_level`) and `planet_system.h` (`set_earth_debug_tint_patches_by_lod`).

## Notes About Structure

- Each `.cpp` file is an independent translation unit. `planet_system.cpp` and `planet_terrain.cpp` both implement `PlanetSystem` methods but are split by responsibility (runtime management vs. terrain infrastructure).
- `planet_patch_helpers.h` is internal to this folder -- it is not included outside the planet module.
- Terrain patch building runs on background worker threads (thread pool started in `init`, joined in `cleanup`). All quadtree update, draw emission, and GPU resource management runs on the main thread.
- Data flow: `PlanetQuadtree::update()` produces `visible_leaves` -> `update_and_emit` computes edge stitch masks and submits async build requests -> worker threads run `build_terrain_patch_cpu` -> main thread polls completed results via `pump_completed_terrain_patch_builds` and uploads vertex buffers -> `build_ready_render_cut` resolves drawable patches (falling back to ready ancestors) -> draw objects emitted into `DrawContext`.
- Patch vertex positions are stored relative to the patch center direction on the sphere surface (not world-space). The draw emission step applies a per-patch model matrix to place them in the scene.
- The system supports both mesh-based planets (simple UV sphere) and terrain planets (cube-sphere quadtree with height maps, per-face albedo/emission textures).
- Height maps are loaded as CPU-side R8 texel grids (decoded from BC4 KTX2) for terrain vertex displacement and physics queries. GPU texturing uses the original KTX2 files loaded through the asset manager.
- Edge stitching uses a 2:1 LOD constraint: neighboring patches may differ by at most one level. Stitched edges snap odd vertices to midpoints of the coarser grid to prevent T-junction cracks.
