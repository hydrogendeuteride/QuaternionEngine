## Scene System: Cameras, DrawContext, and Instances

Thin scene layer that produces `RenderObject`s for the renderer. It gathers opaque/transparent/mesh‑VFX surfaces, maintains the main camera and camera rig, manages dynamic instances with double‑precision world coordinates and a floating origin, and exposes runtime APIs for lights, animation, picking, and physics collider synchronization.

### Components

- `SceneManager` (src/scene/vk_scene.h/.cpp)
  - Owns the main `Camera`, `CameraRig`, `GPUSceneData`, and `DrawContext`.
  - Creates and manages runtime mesh/GLTF instances (glTF assets are loaded via the engine asset pipeline).
  - Updates per‑frame transforms, camera rig, floating origin, cascaded shadow matrices, and `GPUSceneData` (`view/proj/viewproj`, sun/ambient, lights, RT/shadow options).
  - Integrates `PlanetSystem` for planetary rendering and analytic planet shadow occluders.

- `DrawContext`
  - Three lists: `OpaqueSurfaces`, `TransparentSurfaces`, and `MeshVfxSurfaces` of `RenderObject`.
  - Monotonic `nextID` counter assigns stable per‑frame object IDs for ID‑buffer picking.
  - Optional `gltfNodeLocalOverrides` pointer enables per‑instance node pose overrides during draw.
  - Populated by drawing all active instances each frame.

- `RenderObject`
  - Geometry: `indexBuffer`, `vertexBuffer` (for RG tracking), `vertexBufferAddress` (device address used by shaders).
  - Material: `MaterialInstance* material` with bound set and pipeline.
  - Transform and bounds for optional culling.
  - Source info: `sourceMesh`, `surfaceIndex`, `sourceScene`, `sourceNode` for debug/picking.
  - Owner tracking: `ownerType` (`None`, `GLTFInstance`, `MeshInstance`) and `ownerName`.
  - `objectID` for ID‑buffer picking (0 = none).

### World Coordinates & Floating Origin

All positions (camera, instances, lights) are stored in **double‑precision world space** (`WorldVec3 = glm::dvec3`, defined in `src/core/world.h`). Render‑local `float` coordinates are derived each frame via `world_to_local(pos, origin)`.

- **Floating origin recentering:** When the camera drifts beyond `_floating_origin_recenter_threshold` (default 1000 m) from the current origin, the origin is snapped to a grid (`_floating_origin_snap_size` = 100 m) near the camera. This keeps render‑local coordinates small, avoiding float‑precision artifacts.
- Conversion helpers: `world_to_local()`, `local_to_world()`, `snap_world()` (all in `src/core/world.h`).

### Camera & CameraRig

`SceneManager` owns a `Camera` (position + orientation in world space) and a `CameraRig` (`src/scene/camera/camera_rig.h`).

- **Camera modes:** `Free`, `Orbit`, `Follow`, `Chase`, `Fixed`.
- Each mode has its own settings struct (`FreeCameraSettings`, `OrbitCameraSettings`, etc.).
- `CameraTarget` can track a world point, a mesh instance, or a glTF instance by name.
- The rig is updated every frame in `update_scene()` before matrix computation.
- Terrain surface clamping can prevent the camera from going below planet surfaces.

### Frame Flow

1. `SceneManager::update_scene()`:
   - Releases any GLTF assets pending safe destruction (deferred via `DeletionQueue`).
   - Clears draw lists and resets `nextID`.
   - Computes per‑frame delta time (`dt`, clamped to 0–0.1 s).
   - Updates `CameraRig`.
   - Performs floating origin recentering if needed.
   - Advances animation and draws each dynamic GLTF instance (with per‑instance node overrides).
   - Synchronizes physics colliders (`syncColliders()`).
   - Draws each dynamic mesh instance.
   - Computes view/projection matrices (reversed infinite‑Z perspective).
   - Computes cascaded shadow map matrices (clipmap‑based).
   - Publishes RT/shadow settings and planet occluders to `GPUSceneData`.
   - Updates `PlanetSystem`.
   - Packs point lights and spot lights into `GPUSceneData` arrays.
2. Renderer consumes the lists:
   - Geometry pass sorts opaque by material and index buffer to improve locality.
   - Transparent pass sorts back‑to‑front against camera and blends to the HDR target.
3. Uniforms: Passes allocate a small per‑frame UBO (`GPUSceneData`) and bind it via a shared layout.

### Projection

The scene uses a **reversed infinite‑Z perspective** projection (right‑handed, -Z forward):
- Near plane maps to depth 1.0, infinity maps to depth 0.0.
- Vulkan clip space 0..1 (`GLM_FORCE_DEPTH_ZERO_TO_ONE`) with Y flip.
- Aspect ratio is derived from the fixed logical render size (`kRenderWidth` x `kRenderHeight`) rather than the window/swapchain size.
- Camera FOV is taken from `mainCamera.fovDegrees`.

### Sorting / Culling

- Opaque (geometry): stable sort by `material` then `indexBuffer` (see `src/render/passes/geometry.cpp`).
- Transparent: sort by camera‑space depth far→near (see `src/render/passes/transparent.cpp`).
- Mesh VFX: sort by camera‑space depth far→near in its dedicated pass (see `src/render/passes/mesh_vfx.cpp`).
- An example frustum test exists in `passes/geometry.cpp` (`is_visible`) and can be enabled to cull meshes.

### Dynamic Instances

All instances store position as `WorldVec3` (double‑precision) with separate `rotation` (quat) and `scale` (vec3) internally. The `glm::mat4 transform` parameter in add/set helpers is decomposed into TRS on input and recomposed (with floating‑origin shift) for rendering.

#### Mesh Instances

- `MeshInstance` struct: `mesh`, `translation_world` (WorldVec3), `rotation`, `scale`, optional `boundsTypeOverride`.
- API:
  - `addMeshInstance(name, mesh, transform = identity, boundsType = {})` — optional `BoundsType` override.
  - `removeMeshInstance(name)`, `clearMeshInstances()`.
  - `getMeshInstanceTransform(name, outTransform)` / `setMeshInstanceTransform(name, transform)` — world‑space matrix (float, uses WorldVec3 translation directly).
  - `getMeshInstanceTransformLocal(name, outTransformLocal)` / `setMeshInstanceTransformLocal(name, transformLocal)` — render‑local (floating‑origin shifted).
  - `getMeshInstanceTRSWorld(name, outTranslationWorld, outRotation, outScale)` / `setMeshInstanceTRSWorld(name, translationWorld, rotation, scale)` — direct TRS access in double‑precision world space.

#### GLTF Instances

- `GLTFInstance` struct: `scene`, `translation_world` (WorldVec3), `rotation`, `scale`, `AnimationState`, `nodeLocalOverrides`.
- API:
  - `addGLTFInstance(name, LoadedGLTF, transform = identity)`, `removeGLTFInstance(name)`, `clearGLTFInstances()`.
  - `getGLTFInstanceTransform` / `setGLTFInstanceTransform` — world‑space matrix.
  - `getGLTFInstanceTransformLocal` / `setGLTFInstanceTransformLocal` — render‑local matrix.
  - `getGLTFInstanceTRSWorld` / `setGLTFInstanceTRSWorld` — direct TRS in world space.
  - `getGLTFInstanceScene(instanceName)` — returns the `shared_ptr<LoadedGLTF>` for a named instance.
- Deferred destruction: when a GLTF instance is removed, the `shared_ptr<LoadedGLTF>` is deferred via `DeletionQueue` to ensure GPU resources aren't destroyed while still in‑flight.

### GLTF Animation / "Actions"

GLTF files can contain one or more animation clips (e.g. `Idle`, `Walk`, `Run`). The loader (`LoadedGLTF`) parses these into `LoadedGLTF::Animation` objects. Animation *state* (which clip, time, loop flag, playback speed, blend state) is stored outside the glTF asset:

- One `AnimationState` per runtime glTF instance (`SceneManager::GLTFInstance`).

This means that **animation is independent per instance**, even if they share the same underlying `LoadedGLTF` asset and meshes.

**Runtime GLTF instances**

GLTF instances are created via:

- `scene->addGLTFInstance("player", playerGltf, playerTransform);`

You can treat each instance as an "actor" and drive its current action from your game state. Each instance has its own `AnimationState`, even if multiple instances share the same `LoadedGLTF`.

- By index (per‑instance state):
  - `scene->setGLTFInstanceAnimation("player", 0);`
  - `scene->setGLTFInstanceAnimation("player", -1);   // disable animation for this actor`
  - Optional `resetTime` parameter (default `true`): pass `false` to keep current playback time.
- By name (per‑instance state):
  - `scene->setGLTFInstanceAnimation("player", "Idle");`
  - `scene->setGLTFInstanceAnimation("player", "Run");`
- Looping (per‑instance state):
  - `scene->setGLTFInstanceAnimationLoop("player", true);`
- Playback speed:
  - `scene->setGLTFInstanceAnimationSpeed("player", 2.0f);  // 2x speed`
- Animation transitions (cross‑fade blending):
  - `scene->transitionGLTFInstanceAnimation("player", "Run", 0.3f);   // blend over 0.3 seconds`
  - `scene->transitionGLTFInstanceAnimation("player", 2, 0.5f);       // by index`
  - Optional `resetTime` parameter (default `true`).

These helpers update the instance's `AnimationState`. `SceneManager::update_scene()` advances each instance's state every frame using a per‑frame `dt` before drawing, so once you select an action, it will keep playing automatically until you change it or disable looping for that instance.

### Per‑Instance Node / Joint Overrides

For non‑skinned models (rigid parts), you can apply local‑space pose offsets to specific glTF nodes on a **per‑instance** basis. This is useful for things like control surfaces, doors, or turrets layered on top of an existing animation.

- API (on `SceneManager`):
  - `bool setGLTFInstanceNodeOffset(const std::string &instanceName, const std::string &nodeName, const glm::mat4 &offset);`
  - `bool clearGLTFInstanceNodeOffset(const std::string &instanceName, const std::string &nodeName);`
  - `void clearGLTFInstanceNodeOffsets(const std::string &instanceName);`

Notes:

- Offsets are **local‑space** post‑multipliers:
  - Effective local transform = `node.localTransform * offset`.
- Offsets are *per instance*:
  - Different instances of the same glTF can have different joint poses at the same animation time.
- Overrides are applied during draw via `DrawContext::gltfNodeLocalOverrides` and `MeshNode::Draw`, without modifying the shared glTF asset.

### Node World Transform Queries

You can query the world transform of a specific glTF node within an instance. This is useful for attaching objects, spawning effects at bone positions, or reading turret orientations.

- World‑space (uses instance's stored `WorldVec3` translation, result as float matrix):
  - `bool getGLTFInstanceNodeWorldTransform(instanceName, nodeName, outWorldTransform)`
  - `glm::mat4 getGLTFInstanceNodeWorldTransform(instanceName, nodeName)` — convenience, returns identity on failure.
- Render‑local (floating‑origin shifted, better precision near camera):
  - `bool getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, outWorldTransformLocal)`
  - `bool getGLTFInstanceNodeWorldTransformLocal(instanceName, nodeName, origin_world, outWorldTransformLocal)` — custom origin.
  - `glm::mat4` convenience overloads also available.
- Per‑instance node overrides are automatically incorporated into the returned transforms.

### Sunlight (Directional Light)

- `setSunlightDirection(const glm::vec3 &dir)` — auto‑normalizes; falls back to (0, -1, 0) for zero input.
- `getSunlightDirection()` — returns the current normalized direction.
- `setSunlightColor(const glm::vec3 &color, float intensity)` — sets RGB color and intensity.
- `getSunlightColor()` / `getSunlightIntensity()` — read back current values.

### GPU Scene Data

- `GPUSceneData` carries camera matrices, lighting constants, shadow cascades, RT options, and planet occluders for the frame.
- Key fields: `view`, `proj`, `viewproj`, `sunlightDirection`, `sunlightColor`, `ambientColor`, `lightViewProj`, `lightViewProjCascades[4]`, `cascadeSplitsView`, `punctualLights[]`, `spotLights[]`, `planetOccluders[4]`, `lightCounts`, `rtOptions`, `rtParams`, `shadowTuning`.
  - `shadowTuning.x` = minimum sun shadow visibility (0..1). Controls how dark fully-shadowed areas can be; populated from `ShadowSettings::shadowMinVisibility`.
- Passes map and fill it into a per-frame UBO, bindable with `DescriptorManager::gpuSceneDataLayout()`.

### Cascaded Shadow Maps

`update_scene()` computes clipmap‑based cascaded shadow matrices for the directional (sun) light:

- `kShadowCascadeCount` cascade levels (default 4), each covering a square region around the camera with radius `R_i = kShadowClipBaseRadius * 2^i`.
- The clipmap center is biased slightly in the camera's forward direction (per‑cascade blend factor) to spend more resolution in the visible area.
- Centers are snapped to the light‑space texel grid for stable, shimmer‑free shadows.
- Shadow resolution is queried from `EngineContext::getShadowMapResolution()`.
- Depth uses forward 0..1 mapping (`glm::orthoRH_ZO`) paired with a GREATER depth test.

### Point Lights

- `SceneManager::PointLight`
  - `position_world` – `WorldVec3` (double‑precision world‑space position).
  - `radius` – approximate influence radius (used for falloff).
  - `color` – RGB color.
  - `intensity` – scalar brightness.
- API
  - `addPointLight(const PointLight &light)`
  - `clearPointLights()`
  - `getPointLightCount()`, `getPointLight(index, outLight)`, `setPointLight(index, light)`, `removePointLight(index)`
  - `getPointLights()` — returns const reference to the full vector.
- Usage pattern
  - On level load: call `addPointLight` for each baked/runtime point light.
  - At runtime (e.g. gameplay): read/modify lights via the indexed helpers.
- Packed into `GPUSceneData::punctualLights[]` each frame (up to `kMaxPunctualLights`), with positions converted to render‑local coordinates.

### Spot Lights

- `SceneManager::SpotLight`
  - `position_world` – `WorldVec3` (double‑precision world‑space position).
  - `direction` – world‑space unit direction (cone axis, default `(0, -1, 0)`).
  - `radius` – approximate influence radius (default 10).
  - `inner_angle_deg`, `outer_angle_deg` – cone half‑angles in degrees (inner ≤ outer).
  - `color` – RGB color (default white).
  - `intensity` – scalar brightness (default 1).
- API
  - `addSpotLight(const SpotLight &light)`
  - `clearSpotLights()`
  - `getSpotLightCount()`, `getSpotLight(index, outLight)`, `setSpotLight(index, light)`, `removeSpotLight(index)`
  - `getSpotLights()` — returns const reference to the full vector.
- Usage pattern
  - On level load: call `addSpotLight` for each flashlight/beam/cone light.
  - At runtime: read/modify lights via the indexed helpers.
- Packed into `GPUSceneData::spotLights[]` each frame (up to `kMaxSpotLights`), with direction auto‑normalized and angles clamped.

### Planet System

`SceneManager` owns a `PlanetSystem` (`src/scene/planet/planet_system.h`) for planetary‑scale rendering.

- Initialized in `SceneManager::init()`, updated and emitted every frame in `update_scene()`.
- Up to 4 planet bodies are packed as analytic shadow occluders into `GPUSceneData::planetOccluders[]`, sorted by largest angular size from the camera.
- In RT‑only shadow mode, the scene checks whether any planet surface point falls inside a shadow cascade and enables planet receiver shadow maps accordingly (`rtParams.z`).

### Picking & Selection (Game‑Facing)

The scene system exposes CPU ray‑based picking and rectangle selection that the engine uses for editor tools, but you can also call them directly from game code.

- Single‑object ray pick
  - Function: `bool SceneManager::pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, WorldVec3 &outWorldPos)`
  - Input: `mousePosPixels` in window coordinates (SDL style), origin at top‑left.
  - Output:
    - `outObject` – the closest hit `RenderObject` (opaque, transparent, or mesh VFX) along the camera ray.
    - `outWorldPos` – world‑space hit position as `WorldVec3` (mesh BVH‑refined when available).
  - Returns `true` when something was hit, `false` otherwise.
  - `PickingDebug` struct records BVH usage stats for the last pick (accessible via `getPickingDebug()`).

- ID‑buffer picking
  - In addition to CPU ray picks, the engine can render an object‑ID buffer and read back a single pixel.
  - Core API:
    - `SceneManager::resolveObjectID(uint32_t id, RenderObject &outObject) const`
      - Takes an ID from the ID buffer and resolves it back to the `RenderObject` in the latest `DrawContext`.
  - Engine integration:
    - `RenderObject::objectID` is assigned in `MeshNode::Draw` and consumed by the geometry pass to write the ID buffer.
    - `VulkanEngine` wires a small `PickReadback` render‑graph pass that copies one pixel from the ID buffer into a CPU readback buffer when ID‑buffer picking is enabled.

- Rectangle selection
  - Function: `void SceneManager::selectRect(const glm::vec2 &p0, const glm::vec2 &p1, std::vector<RenderObject> &outObjects) const`
  - Input: `p0`, `p1` – opposite corners of a screen‑space rectangle in window coordinates (top‑left origin).
  - Output: `outObjects` – all `RenderObject`s whose projected bounds intersect the rectangle.
  - Internals:
    - Uses `sceneData.viewproj` and `box_overlaps_ndc_rect` to test each object's bounds in clip space.

### Engine‑Level Picking Helpers

`VulkanEngine` wraps the scene picking functions and keeps a small amount of per‑frame state that game/editor code can query:

- Structures on `VulkanEngine`
  - `struct PickInfo`
    - `MeshAsset *mesh`, `LoadedGLTF *scene`, `Node *node`
    - `RenderObject::OwnerType ownerType`, `std::string ownerName`
    - `std::string nodeName`, `std::string nodeParentName`
    - `std::vector<std::string> nodeChildren`, `std::vector<std::string> nodePath`
    - `glm::vec3 worldPos`, `glm::mat4 worldTransform`
    - `uint32_t indexCount`, `firstIndex`, `surfaceIndex`
    - `bool valid`
  - Fields:
    - `_lastPick` – result of the last click selection (via CPU ray or ID‑buffer).
    - `_hoverPick` – result of the last per‑frame hover raycast (under the mouse cursor).
    - `_dragSelection` – list of `PickInfo` filled by `SceneManager::selectRect` when a drag‑select completes.
    - `_useIdBufferPicking` – toggles between CPU ray picking and ID‑buffer picking.

- How the engine fills them (for reference)
  - Mouse hover:
    - The frame loop tracks `_mousePosPixels` from SDL mouse motion events.
    - Each frame, `VulkanEngine::draw()` calls `SceneManager::pick(_mousePosPixels, ...)` and stores the result in `_hoverPick`.
  - Click selection:
    - On mouse button release, the engine either:
      - Queues a pick request for the ID buffer (if `_useIdBufferPicking` is true), or
      - Calls `SceneManager::pick(...)` directly (CPU ray).
    - The resolved result is stored in `_lastPick`.
  - Drag selection:
    - The engine tracks a drag rectangle in screen space and, on release, calls `SceneManager::selectRect(...)` and converts each `RenderObject` into a `PickInfo` stored in `_dragSelection`.

- Typical game‑side usage
  - Hover tooltips:
    - Read `engine->_hoverPick` each frame; if `valid`, use `ownerName`, `worldPos`, or `ownerType` to drive UI.
  - Click selection:
    - After input handling, inspect `engine->_lastPick` for the most recent clicked object.
  - Multi‑select:
    - After a drag, iterate `engine->_dragSelection` for group actions.

### Physics Collider Synchronization

`SceneManager` can automatically create and synchronize physics bodies for glTF instances that have collider data (from `COL_*` markers or sidecar files).

- **Enable sync:**
  - `size_t enableColliderSync(instanceName, PhysicsWorld*, layer = 0, user_data = 0)`
  - Creates one **kinematic** body per collider‑owning node (compound shapes, scaled uniformly by instance scale).
  - Creates **static** bodies for mesh colliders (`COL_MESH` from sidecars).
  - Returns the number of bodies created.
- **Disable sync:**
  - `bool disableColliderSync(instanceName)` — destroys all associated physics bodies.
- **Query:**
  - `bool isColliderSyncEnabled(instanceName)` — check if sync is active.
  - `std::vector<Physics::BodyId> getColliderSyncBodies(instanceName)` — get all body IDs for debug.
- **Per‑frame update:**
  - `syncColliders()` is called automatically in `update_scene()` after animation updates.
  - Each kinematic body's transform is set to match the node's current world transform (including animation and node overrides).
  - Transform is computed relative to the physics origin (`physics_context->origin_world()`).
- Collider sync is automatically cleaned up when `removeGLTFInstance()` or `clearGLTFInstances()` is called.

### Scene Stats

- `SceneManager::SceneStats` tracks performance timing:
  - `scene_update_time` — wall‑clock time of `update_scene()` in milliseconds.
- Accessible via `stats` member.

### Tips

- Treat `DrawContext` as immutable during rendering; build it fully in `update_scene()`.
- Keep `RenderObject` small; use device addresses for vertex data to avoid per-draw vertex buffer binds.
- For custom sorting/culling, modify only the scene layer; render passes stay simple.
- Use `WorldVec3` for all authoritative positions; convert to float only at render time via `world_to_local()`.
- When working at planetary scales, rely on the floating origin — never store large float positions directly.
