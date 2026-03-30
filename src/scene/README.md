# Scene Folder Guide

This folder contains the scene management layer used by the engine core: node hierarchy, glTF loading, dynamic instances, physics synchronization, picking, cameras, and planetary rendering.

## Folder Structure

```
scene/
  vk_scene.h / .cpp               # SceneManager class: init, cleanup, lighting
  vk_scene_internal.h             # internal helpers (node transform with overrides)
  vk_scene_instances.cpp          # mesh/GLTF/decal instance CRUD, transforms, animation
  vk_scene_physics.cpp            # physics collider sync: kinematic bodies, dynamic root bodies
  vk_scene_update.cpp             # per-frame update pipeline: timing, emission, view/projection, lights
  vk_scene_picking.cpp            # ray-cast picking, rectangle selection, object ID resolution
  vk_loader.h / .cpp              # glTF scene types, animation update, collider building, cleanup
  vk_loader_load.cpp              # glTF file parsing and GPU resource allocation
  mesh_bvh.h / .cpp               # CPU-side BVH for precise ray-mesh intersection during picking
  camera.h / .cpp                 # basic Camera struct: position, orientation, view matrix
  tangent_space.h / .cpp          # per-vertex tangent generation (MikkTSpace or fallback)
  camera/                         # pluggable camera rig with five modes (see camera/README.md)
  planet/                         # planetary body rendering with LOD quadtree (see planet/README.md)
```

## What Lives Here

### Headers

- `vk_scene.h`
  `SceneManager` class -- the central scene coordinator. Owns the camera, `CameraRig`, `PlanetSystem`, dynamic instance lists (mesh, glTF, decal), punctual lights (point, spot), and the per-frame `DrawContext`. Defines key types: `RenderObject` (drawable mesh with bounds, transform, material), `DrawContext` (render object lists partitioned by Opaque/Transparent/MeshVFX/Decals), `MeshInstance`, `GLTFInstance`, `DecalInstance`, `PointLight`, `SpotLight`, `BoundsType` enum, and `Bounds`. Exposes instance CRUD, picking, physics collider sync, lighting, and camera access.

- `vk_scene_internal.h`
  Internal helper: `SceneInternal::build_node_world_with_overrides` -- recursively builds a world transform from a glTF node chain, applying per-node local overrides (used for animation pose queries).

- `vk_loader.h`
  glTF asset types: `MeshAsset` (mesh with surfaces, GPU buffers, optional CPU BVH), `GeoSurface` (start index, count, bounds, material), `GLTFMaterial` (material wrapper), `LoadedGLTF` (complete scene with nodes, meshes, materials, animations, colliders). Animation types: `Animation` (clip with channels), `AnimationChannel` (per-property keyframes for translation/rotation/scale), `AnimationState` (per-instance playback with cross-fade blending). Also defines `GLTFLoadCallbacks` for progress/cancellation and the `loadGltf` free function.

- `mesh_bvh.h`
  `MeshBVH` -- CPU-side triangle BVH for precise picking. `MeshBVHPrimitiveRef` maps BVH primitives to surface and triangle indices. `MeshBVHPickHit` carries intersection result (world position, triangle, surface). Free functions: `build_mesh_bvh` (constructs BVH, optional multi-threading) and `intersect_ray_mesh_bvh` (ray query returning closest hit).

- `camera.h`
  `Camera` struct -- position (world-space), orientation (quaternion), FOV. Methods: `getViewMatrix` and `getRotationMatrix`.

- `tangent_space.h`
  Free functions: `generate_tangents` (all vertices) and `generate_tangents_range` (submesh range). Uses MikkTSpace when `ENABLE_MIKKTS` is on, otherwise falls back to Gram-Schmidt orthonormalization.

### Implementation Files

- `vk_scene.cpp`
  `SceneManager` lifecycle: `init` (creates camera, camera rig, planet system, default scene data), `cleanup` (tears down planet system and all dynamic instances). Punctual light management: `addPointLight`, `removePointLight`, `clearPointLights`, `addSpotLight`, `removeSpotLight`, `clearSpotLights`. Sunlight accessors: `setSunlightDirection`, `setSunlightColor`, `getSunlightIntensity`.

- `vk_scene_instances.cpp`
  Instance CRUD for all three instance types. Mesh instances: `addMeshInstance`, `getMeshInstanceTransform`, `setMeshInstanceTransform`, `setMeshInstanceMaterial`, `removeMeshInstance`, `clearMeshInstances`. Decals: `setDecal`, `getDecal`, `removeDecal`, `clearDecals`. glTF instances: `addGLTFInstance`, `removeGLTFInstance`, `getGLTFInstanceScene`, transform getters/setters (world and local variants), node offset overrides (`setGLTFInstanceNodeOffset`, `clearGLTFInstanceNodeOffset`, `clearGLTFInstanceNodeOffsets`), animation control (`setGLTFInstanceAnimation`, `setGLTFInstanceAnimationLoop`, `setGLTFInstanceAnimationSpeed`, `transitionGLTFInstanceAnimation`).

- `vk_scene_physics.cpp`
  Physics collider synchronization. Kinematic path: `enableColliderSync` (creates per-node kinematic bodies from glTF collider compounds and mesh instances), `disableColliderSync`, `syncColliders` (per-frame transform update). Dynamic root path: `enableDynamicRootColliderBody` (aggregates all colliders into a single dynamic body driven by physics), `disableDynamicRootColliderBody`, mass configuration (`setDynamicRootColliderMass`, `setDynamicRootColliderChildMassOverride`), `rebuildDynamicRootColliderBody` (rebuilds body after config changes, preserving velocity). Internal helpers: `aggregate_compounds_for_dynamic_root`, `scale_primitive_with_vec3`.

- `vk_scene_update.cpp`
  Main per-frame update orchestrator: `update_scene` calls `updateFrameTiming` (delta time, stats), updates camera rig, runs floating origin recentering (`recenterFloatingOriginIfNeeded`), syncs colliders, emits all instance types to `DrawContext` (`emitDynamicGLTFInstances`, `emitDynamicMeshInstances`, `emitDynamicDecals`), computes view/projection data (`updateViewProjectionData`), shadow data (`updateDirectionalShadowData`), ray-tracing acceleration structures (`updateRayTracingSceneData`), punctual light data (`updatePunctualLightData`), and planet occlusion (`updatePlanetOccluders`).

- `vk_scene_picking.cpp`
  Picking and selection: `pick` (ray-casts from camera through mouse position against all surfaces using shape-specific intersection -- sphere, capsule, box, mesh BVH -- returns closest hit and world position), `resolveObjectID` (looks up `RenderObject` by unique ID from the ID buffer), `selectRect` (screen-space rectangle selection via frustum AABB test). Internal intersection helpers: `intersect_ray_sphere`, `intersect_ray_box`, `intersect_ray_capsule`, `intersect_ray_bounds`, `box_overlaps_ndc_rect`, `intersect_terrain_planet_sphere`.

- `vk_loader.cpp`
  glTF runtime operations: `updateAnimation` (updates node TRS from animation clips with channel interpolation and cross-fade blending), `setActiveAnimation`, `transitionAnimation`, collider building (`build_colliders_from_markers`, `build_colliders_from_sidecar`, `build_mesh_colliders_from_markers`), mass override application (`apply_collider_child_mass_overrides`), bind pose management (`ensureRestTransformsCached`, `restoreNodeToRest`), resource cleanup (`clearAll`).

- `vk_loader_load.cpp`
  `loadGltf` -- glTF file parsing via fastgltf: loads buffers, images, samplers, materials (PBR metallic-roughness with normal/emissive maps), meshes (vertex/index buffers, surface extraction, BVH building), nodes (hierarchy, transforms), and animations (channel keyframe extraction). Uploads all GPU resources.

- `mesh_bvh.cpp`
  `build_mesh_bvh` -- constructs a two-level BVH from mesh surface triangles using the bvh2 library (multi-threading enabled above 4096 triangles). `intersect_ray_mesh_bvh` -- transforms ray to mesh-local space, queries BVH, returns world-space hit with surface and triangle reference.

- `camera.cpp`
  `Camera::getViewMatrix` (inverse of translation * rotation) and `Camera::getRotationMatrix` (quaternion to mat4 via `glm::toMat4`).

- `tangent_space.cpp`
  `generate_tangents_range` -- when `MIKKTS_ENABLE` is defined, uses MikkTSpace context with per-face/vertex callbacks; otherwise falls back to per-triangle tangent/bitangent accumulation with Gram-Schmidt orthonormalization. Internal helpers: `orthonormal_tangent`, `generate_fallback`.

### camera/

Pluggable camera rig with five switchable modes (Free, Orbit, Follow, Chase, Fixed), target resolution, and terrain surface clamping. See `camera/README.md`.

### planet/

Planetary body rendering system: cube-sphere geometry, LOD quadtree selection, terrain patch building, height map sampling, and mesh/terrain planet management. See `planet/README.md`.

## How It Is Called

`SceneManager` is owned by the engine core (`src/core/engine.h`) and exposed to game code through `GameAPI` (`src/core/game_api.h`).

- The engine calls `SceneManager::init()` at startup and `SceneManager::cleanup()` at shutdown.
- Per-frame, the engine calls `SceneManager::update_scene()`, which drives the entire scene update pipeline: timing, camera, instances, physics sync, draw emission, view/projection/shadow data, and planet rendering.
- Render passes read from `SceneManager::main_draw_context` to obtain the lists of `RenderObject` entries for each pass (opaque, transparent, VFX, decals).
- Game states (e.g., `GameplayState`) call instance management methods through `GameAPI` to spawn/destroy meshes, glTF scenes, decals, and lights, configure cameras, and perform picking.
- The asset loading system calls `loadGltf` to parse and upload glTF files, producing `LoadedGLTF` scenes that are later instantiated via `addGLTFInstance`.

## If You Want To Change...

- Instance creation, transform management, or animation control:
  Start in `vk_scene_instances.cpp`.

- Physics collider synchronization or dynamic root bodies:
  Start in `vk_scene_physics.cpp`.

- The per-frame update pipeline, draw emission, or floating origin:
  Start in `vk_scene_update.cpp`.

- Ray-cast picking, rectangle selection, or bounds intersection:
  Start in `vk_scene_picking.cpp`.

- Lighting (point, spot, sun) or ambient color:
  Start in `vk_scene.cpp`.

- The `SceneManager` public API, instance types, or `DrawContext`:
  Start in `vk_scene.h`.

- glTF animation playback or cross-fade blending:
  Start in `vk_loader.cpp` (`updateAnimation`).

- glTF file parsing, material loading, or mesh upload:
  Start in `vk_loader_load.cpp`.

- Precise mesh picking (BVH construction or ray query):
  Start in `mesh_bvh.h/.cpp`.

- Tangent generation algorithm or MikkTSpace integration:
  Start in `tangent_space.cpp`.

- Camera behavior, modes, or terrain clamping:
  Start in `camera/` (see `camera/README.md`).

- Planet rendering, LOD selection, or terrain patches:
  Start in `planet/` (see `planet/README.md`).

## Notes About Structure

- `SceneManager` is split across five translation units by responsibility: `vk_scene.cpp` (lifecycle, lighting), `vk_scene_instances.cpp` (instance CRUD), `vk_scene_physics.cpp` (collider sync), `vk_scene_update.cpp` (frame update), and `vk_scene_picking.cpp` (ray-casting). All implement methods of the same `SceneManager` class.
- `vk_scene_internal.h` is internal to this folder -- it is not included outside the scene module.
- The loader files (`vk_loader.cpp`, `vk_loader_load.cpp`) implement methods of `LoadedGLTF` and free functions declared in `vk_loader.h`. The split separates runtime operations (animation, colliders, cleanup) from one-time loading (file parsing, GPU upload).
- Physics synchronization supports two modes: per-node kinematic bodies (scene drives physics transforms each frame) and a single dynamic root body (physics drives the instance transform). These are mutually exclusive per instance.
- The floating origin system (`recenterFloatingOriginIfNeeded`) periodically recenters the world around the camera to prevent floating-point precision loss at large distances.
- Picking supports multiple bounds types (`BoundsType::Sphere`, `Capsule`, `Box`, `Mesh`) with shape-specific ray intersection. Mesh-level picking uses a CPU-side BVH built at load time.
- Draw emission converts dynamic instances into `RenderObject` entries in the `DrawContext`, which render passes then consume. glTF instances are emitted recursively through the node hierarchy.
