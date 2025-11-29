## Game‑Facing API Overview

This document summarizes the main engine APIs that are directly useful when building a game (spawn actors, control lights/animations, and interact via picking).

For details on the underlying systems, see also:

- `docs/Scene.md` – cameras, draw context, instances, picking.
- `docs/EngineContext.md` – access to managers and per‑frame state.
- `docs/RenderGraph.md` – render‑graph API for custom passes.

---

## VulkanEngine Helpers

Header: `src/core/engine.h`

- Lifecycle
  - `void init()` – initialize SDL, device, managers, scene, render graph.
  - `void run()` – main loop: handles events, updates scene, builds Render Graph, submits frames.
  - `void cleanup()` – destroy managers and GPU resources.

- GLTF spawn helper
  - `bool addGLTFInstance(const std::string &instanceName, const std::string &modelRelativePath, const glm::mat4 &transform = glm::mat4(1.f));`
    - Loads a glTF from `assets/models/...` (via `AssetManager`) and registers it as a runtime scene instance.
    - `instanceName` becomes the logical name used by `SceneManager` (e.g. for picking and animation control).

- Picking access
  - `struct PickInfo` (nested in `VulkanEngine`)
    - `MeshAsset *mesh`, `LoadedGLTF *scene`, `Node *node`
    - `RenderObject::OwnerType ownerType` (e.g. `StaticGLTF`, `GLTFInstance`, `MeshInstance`)
    - `std::string ownerName`
    - `glm::vec3 worldPos`
    - `glm::mat4 worldTransform`
    - `uint32_t indexCount`, `firstIndex`, `surfaceIndex`
    - `bool valid`
  - `const PickInfo &get_last_pick() const`
    - Returns the last click selection result.
    - Filled by the engine from either CPU ray picking or ID‑buffer picking depending on `_useIdBufferPicking`.
    - Typical usage:
      - On mouse‑up in your game layer, read `engine->get_last_pick()` and, if `valid`, use `ownerName`/`worldPos` to drive selection logic.

> Note: hover picks and drag selections are also available as internal fields on `VulkanEngine` (`_hoverPick`, `_dragSelection`) and are documented in `docs/Scene.md`. You can expose additional getters if you want to rely on them from game code.

---

## Scene & Instances (Actors, Lights, Animations)

Header: `src/scene/vk_scene.h`  
Docs: `docs/Scene.md`

### Dynamic Mesh/GLTF Instances

- Mesh instances
  - `void addMeshInstance(const std::string &name, std::shared_ptr<MeshAsset> mesh, const glm::mat4 &transform = glm::mat4(1.f), std::optional<BoundsType> boundsType = {});`
  - `bool getMeshInstanceTransform(const std::string &name, glm::mat4 &outTransform);`
  - `bool setMeshInstanceTransform(const std::string &name, const glm::mat4 &transform);`
  - `bool removeMeshInstance(const std::string &name);`
  - `void clearMeshInstances();`
  - Typical usage:
    - Spawn primitives or dynamic meshes at runtime (e.g. projectiles, props).
    - Use `setMeshInstanceTransform` every frame to move them based on game logic.

- GLTF instances (actors)
  - `void addGLTFInstance(const std::string &name, std::shared_ptr<LoadedGLTF> scene, const glm::mat4 &transform = glm::mat4(1.f));`
  - `bool getGLTFInstanceTransform(const std::string &name, glm::mat4 &outTransform);`
  - `bool setGLTFInstanceTransform(const std::string &name, const glm::mat4 &transform);`
  - `bool removeGLTFInstance(const std::string &name);`
  - `void clearGLTFInstances();`
  - Usage pattern:
    - Treat each GLTF instance as an “actor” with a name; use transforms to place characters, doors, props, etc.

### Animations (GLTF)

- Scene‑level
  - `bool setSceneAnimation(const std::string &sceneName, int animationIndex, bool resetTime = true);`
  - `bool setSceneAnimation(const std::string &sceneName, const std::string &animationName, bool resetTime = true);`
  - `bool setSceneAnimationLoop(const std::string &sceneName, bool loop);`
- Instance‑level
  - `bool setGLTFInstanceAnimation(const std::string &instanceName, int animationIndex, bool resetTime = true);`
  - `bool setGLTFInstanceAnimation(const std::string &instanceName, const std::string &animationName, bool resetTime = true);`
  - `bool setGLTFInstanceAnimationLoop(const std::string &instanceName, bool loop);`
- Notes:
  - All functions return `bool` indicating whether the named scene/instance exists.
  - Animation state is **independent per scene and per instance**:
    - Each named scene has its own `AnimationState`.
    - Each glTF instance has its own `AnimationState`, even when sharing the same `LoadedGLTF`.
  - An index `< 0` (e.g. `-1`) disables animation for that scene/instance (pose is frozen at the last evaluated state).
  - `SceneManager::update_scene()` advances each active animation state every frame using engine delta time.

### Per‑Instance Node / Joint Control (Non‑Skinned)

For rigid models and simple “joints” (e.g. flaps, doors, turrets), you can apply local‑space pose offsets to individual glTF nodes per instance:

- `bool setGLTFInstanceNodeOffset(const std::string &instanceName, const std::string &nodeName, const glm::mat4 &offset);`
- `bool clearGLTFInstanceNodeOffset(const std::string &instanceName, const std::string &nodeName);`
- `void clearGLTFInstanceNodeOffsets(const std::string &instanceName);`

Typical usage:

- Use glTF animation for the base motion (e.g. gear deployment).
- Layer game‑driven offsets on top for per‑instance control:

  ```cpp
  // Rotate a control surface on one aircraft instance
  glm::mat4 offset =
      glm::rotate(glm::mat4(1.f),
                  glm::radians(aileronDegrees),
                  glm::vec3(1.f, 0.f, 0.f));
  sceneMgr->setGLTFInstanceNodeOffset("plane01", "LeftAileron", offset);
  ```

### Point Lights

- Struct:
  - `struct PointLight { glm::vec3 position; float radius; glm::vec3 color; float intensity; };`
- API:
  - `void addPointLight(const PointLight &light);`
  - `void clearPointLights();`
  - `size_t getPointLightCount() const;`
  - `bool getPointLight(size_t index, PointLight &outLight) const;`
  - `bool setPointLight(size_t index, const PointLight &light);`
  - `bool removePointLight(size_t index);`
- Typical usage:
  - On level load, add all static lights.
  - At runtime, animate or toggle lights based on gameplay events (e.g. explosions, flickering lamps).

---

## Picking & Selection (Interaction)

Picking lives in `SceneManager` and is wired into `VulkanEngine`’s frame loop.

Header: `src/scene/vk_scene.h`  
Implementation: `src/scene/vk_scene_picking.cpp`

### Single‑Object Ray Picking

- `bool pick(const glm::vec2 &mousePosPixels, RenderObject &outObject, glm::vec3 &outWorldPos);`
  - Input:
    - `mousePosPixels` – window coordinates (SDL style), origin at top‑left.
  - Output on success:
    - `outObject` – closest `RenderObject` hit by the camera ray.
    - `outWorldPos` – precise world‑space hit position (uses mesh BVH when available).
  - Returns:
    - `true` if any object was hit, otherwise `false`.

### ID‑Buffer Picking

- `bool resolveObjectID(uint32_t id, RenderObject &outObject) const;`
  - Takes an ID read back from the ID buffer and resolves it to the corresponding `RenderObject` in the latest `DrawContext`.
  - Used by the engine when `_useIdBufferPicking` is enabled to implement asynchronous picking.

### Rectangle Selection (Drag Box)

- `void selectRect(const glm::vec2 &p0, const glm::vec2 &p1, std::vector<RenderObject> &outObjects) const;`
  - Inputs:
    - `p0`, `p1` – opposite corners of a window‑space rectangle (top‑left origin).
  - Output:
    - `outObjects` – appended with all `RenderObject`s whose projected bounds intersect the rectangle.
  - Internals:
    - Uses `sceneData.viewproj` and an NDC‑space bounds test to determine overlap.

### Typical Game Usage

- Hover tooltips:
  - Track mouse position in window coordinates.
  - Use `SceneManager::pick` directly, or read `VulkanEngine::get_last_pick()` and/or `_hoverPick` as documented in `Scene.md`.
- Object selection / interaction:
  - On mouse click release, inspect `engine->get_last_pick()`:
    - If `valid`, dispatch interaction based on `ownerType` / `ownerName` (e.g. select unit, open door).
- Multi‑select:
  - When implementing drag selection, call `SceneManager::selectRect` with the drag rectangle to get all hits, or reuse the engine’s `_dragSelection` mechanism.

---

## ImGui / ImGuizmo Editor Utilities

File: `src/core/engine_ui.cpp`

These are primarily debug/editor features but can be kept in a game build to provide in‑game tools.

- Main entry:
  - `void vk_engine_draw_debug_ui(VulkanEngine *eng);`
    - Called once per frame by the engine to build the “Debug” window tabs.
- Useful tools for games:
  - Scene tab:
    - Spawn glTF instances at runtime using `VulkanEngine::addGLTFInstance(...)`.
    - Spawn primitive mesh instances (cube/sphere) using `SceneManager::addMeshInstance(...)`.
    - Point‑light editor UI built on `SceneManager` light APIs.
  - Object gizmo (ImGuizmo):
    - Uses last pick / hover pick as the current target and manipulates transforms via `setMeshInstanceTransform` / `setGLTFInstanceTransform`.
