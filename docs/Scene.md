## Scene System: Cameras, DrawContext, and Instances

Thin scene layer that produces `RenderObject`s for the renderer. It gathers opaque/transparent surfaces, maintains the main camera, and exposes simple runtime instance APIs.

### Components

- `SceneManager` (src/scene/vk_scene.h/.cpp)
  - Owns the main `Camera`, `GPUSceneData`, and `DrawContext`.
  - Loads GLTF scenes via `AssetManager`/`LoadedGLTF` and creates dynamic mesh/GLTF instances.
  - Updates per‑frame transforms, camera, and `GPUSceneData` (`view/proj/viewproj`, sun/ambient).

- `DrawContext`
  - Two lists: `OpaqueSurfaces` and `TransparentSurfaces` of `RenderObject`.
  - Populated by scene graph traversal and dynamic instances each frame.

- `RenderObject`
  - Geometry: `indexBuffer`, `vertexBuffer` (for RG tracking), `vertexBufferAddress` (device address used by shaders).
  - Material: `MaterialInstance* material` with bound set and pipeline.
  - Transform and bounds for optional culling.

### Frame Flow

1. `SceneManager::update_scene()` clears the draw lists and rebuilds them by drawing all active scene/instance nodes.
2. Renderer consumes the lists:
   - Geometry pass sorts opaque by material and index buffer to improve locality.
   - Transparent pass sorts back‑to‑front against camera and blends to the HDR target.
3. Uniforms: Passes allocate a small per‑frame UBO (`GPUSceneData`) and bind it via a shared layout.

### Sorting / Culling

- Opaque (geometry): stable sort by `material` then `indexBuffer` (see `src/render/passes/geometry.cpp`).
- Transparent: sort by camera‑space depth far→near (see `src/render/passes/transparent.cpp`).
- An example frustum test exists in `passes/geometry.cpp` (`is_visible`) and can be enabled to cull meshes.

### Dynamic Instances

- Mesh instances
  - `addMeshInstance(name, mesh, transform)`, `removeMeshInstance(name)`, `clearMeshInstances()`.
  - Useful for spawning primitives or asset meshes at runtime.

- GLTF instances
  - `addGLTFInstance(name, LoadedGLTF, transform)`, `removeGLTFInstance(name)`, `clearGLTFInstances()`.

### GLTF Animation / “Actions”

GLTF files can contain one or more animation clips (e.g. `Idle`, `Walk`, `Run`). The loader (`LoadedGLTF`) parses these into `LoadedGLTF::Animation` objects, and `SceneManager` exposes a thin API to pick which clip is currently playing.

> Note: a `LoadedGLTF` is typically shared by multiple instances. Changing the active animation on a shared `LoadedGLTF` will affect all instances that point to it. If you want per‑character independent actions, load separate `LoadedGLTF` objects (one per character) or duplicate the asset in your game layer.

**Static scenes (loaded via `loadScene`)**

Example: engine default scene in `VulkanEngine::init()`:

- `structure` is loaded and registered via:
  - `sceneManager->loadScene("structure", structureFile);`

To control its animation:

- By index:
  - `scene->setSceneAnimation("structure", 0);        // first clip`
  - `scene->setSceneAnimation("structure", 1, true);  // second clip, reset time`
- By name (matches glTF animation name):
  - `scene->setSceneAnimation("structure", "Idle");`
  - `scene->setSceneAnimation("structure", "Run");`
- Looping:
  - `scene->setSceneAnimationLoop("structure", true);   // enable loop`
  - `scene->setSceneAnimationLoop("structure", false);  // play once and stop at end`

All functions return `bool` to indicate whether the scene name was found.

**Runtime GLTF instances**

GLTF instances are created via:

- `scene->addGLTFInstance("player", playerGltf, playerTransform);`

You can treat each instance as an “actor” and drive its current action from your game state:

- By index:
  - `scene->setGLTFInstanceAnimation("player", 0);`
- By name:
  - `scene->setGLTFInstanceAnimation("player", "Idle");`
  - `scene->setGLTFInstanceAnimation("player", "Run");`
- Looping:
  - `scene->setGLTFInstanceAnimationLoop("player", true);`

These helpers forward to the underlying `LoadedGLTF`’s `setActiveAnimation(...)` and `animationLoop` fields. `SceneManager::update_scene()` advances animations every frame using a per‑frame `dt`, so once you select an action, it will keep playing automatically until you change it or disable looping.

### GPU Scene Data

- `GPUSceneData` carries camera matrices and lighting constants for the frame.
- Passes map and fill it into a per-frame UBO, bindable with `DescriptorManager::gpuSceneDataLayout()`.

### Point Lights

- `SceneManager::PointLight`
  - `position` – world‑space position.
  - `radius` – approximate influence radius (used for falloff).
  - `color` – RGB color.
  - `intensity` – scalar brightness.
- API
  - `addPointLight(const PointLight &light)`
  - `clearPointLights()`
  - `getPointLightCount()`, `getPointLight(index, outLight)`, `setPointLight(index, light)`, `removePointLight(index)`
- Usage pattern
  - On level load: call `addPointLight` for each baked/runtime point light.
  - At runtime (e.g. gameplay): read/modify lights via the indexed helpers.

### Picking & Selection (Game‑Facing)

The scene system exposes CPU ray‑based picking and rectangle selection that the engine uses for editor tools, but you can also call them directly from game code.

- Single‑object ray pick
  - Function: `bool SceneManager::pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, glm::vec3 &outWorldPos)`
  - Input: `mousePosPixels` in window coordinates (SDL style), origin at top‑left.
  - Output:
    - `outObject` – the closest hit `RenderObject` (opaque or transparent) along the camera ray.
    - `outWorldPos` – world‑space hit position (mesh BVH‑refined when available).
  - Returns `true` when something was hit, `false` otherwise.

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
    - Uses `sceneData.viewproj` and `box_overlaps_ndc_rect` to test each object’s bounds in clip space.

### Engine‑Level Picking Helpers

`VulkanEngine` wraps the scene picking functions and keeps a small amount of per‑frame state that game/editor code can query:

- Structures on `VulkanEngine`
  - `struct PickInfo`
    - `MeshAsset *mesh`, `LoadedGLTF *scene`, `Node *node`
    - `RenderObject::OwnerType ownerType`, `std::string ownerName`
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

### Tips

- Treat `DrawContext` as immutable during rendering; build it fully in `update_scene()`.
- Keep `RenderObject` small; use device addresses for vertex data to avoid per-draw vertex buffer binds.
- For custom sorting/culling, modify only the scene layer; render passes stay simple.
