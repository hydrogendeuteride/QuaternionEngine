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

- Opaque (geometry): stable sort by `material` then `indexBuffer` (see `src/render/vk_renderpass_geometry.cpp`).
- Transparent: sort by camera‑space depth far→near (see `src/render/vk_renderpass_transparent.cpp`).
- An example frustum test exists in `vk_renderpass_geometry.cpp` (`is_visible`) and can be enabled to cull meshes.

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
- Passes map and fill it into a per‑frame UBO, bindable with `DescriptorManager::gpuSceneDataLayout()`.

### Tips

- Treat `DrawContext` as immutable during rendering; build it fully in `update_scene()`.
- Keep `RenderObject` small; use device addresses for vertex data to avoid per‑draw vertex buffer binds.
- For custom sorting/culling, modify only the scene layer; render passes stay simple.

