## Asset Manager

Centralized asset path resolution, glTF loading, and runtime mesh creation (including simple materials and primitives). Avoids scattered relative paths and duplicates by resolving roots at runtime and caching results.

### Path Resolution

- Environment root: Honors `VKG_ASSET_ROOT` (expected to contain `assets/` and/or `shaders/`).
- Upward search: If unset, searches upward from the current directory for folders named `assets` and `shaders`.
- Fallbacks: Tries `./assets`, `../assets` and `./shaders`, `../shaders`.
- Methods: `shaderPath(name)`, `assetPath(name)`, and `modelPath(name)` (alias of `assetPath`). Relative or absolute input is returned if already valid; otherwise resolution is attempted as above.

Access the manager anywhere via `EngineContext`:

```c++
auto *assets = context->getAssets();
auto spv = assets->shaderPath("mesh.vert.spv");
auto chairPath = assets->modelPath("models/chair.glb");
```

### API Summary

- Paths
  - `std::string shaderPath(std::string_view)`
  - `std::string assetPath(std::string_view)` / `modelPath(std::string_view)`
  - `const AssetPaths& paths() const` / `void setPaths(const AssetPaths &p)` — get/set asset paths
- glTF
  - `std::optional<std::shared_ptr<LoadedGLTF>> loadGLTF(std::string_view nameOrPath)` — cached by canonical absolute path
  - `std::optional<std::shared_ptr<LoadedGLTF>> loadGLTF(std::string_view nameOrPath, const GLTFLoadCallbacks *cb)` — with custom callbacks
  - `size_t prefetchGLTFTextures(std::string_view nameOrPath)` — schedule texture loads ahead of time
  - `GLTFTexturePrefetchResult prefetchGLTFTexturesWithHandles(std::string_view nameOrPath)` — returns handles for tracking
- Meshes
  - `std::shared_ptr<MeshAsset> createMesh(const MeshCreateInfo &info)`
  - `std::shared_ptr<MeshAsset> createMesh(const std::string &name, std::span<Vertex> v, std::span<uint32_t> i, std::shared_ptr<GLTFMaterial> material = {}, bool build_bvh = true)`
  - `std::shared_ptr<MeshAsset> getMesh(const std::string &name) const`
  - `std::shared_ptr<MeshAsset> getPrimitive(std::string_view name) const` — returns existing default primitives if created
  - `bool removeMesh(const std::string &name)`
  - `bool removeMeshDeferred(const std::string &name, DeletionQueue &dq)` — deferred cleanup via deletion queue
  - `void cleanup()` — releases meshes, material buffers, and any images owned by the manager
- Materials
  - `std::shared_ptr<GLTFMaterial> createMaterialFromConstants(const std::string &name, const GLTFMetallic_Roughness::MaterialConstants &constants, MaterialPass pass = MaterialPass::MainColor)` — create PBR material from constants using engine default textures
- Mesh VFX Materials
  - `bool createOrUpdateMeshVfxMaterial(const std::string &name, const MeshVfxSettings &settings)` — create or hot-update a VFX material with noise scrolling, gradient, and fresnel parameters. Textures are streamed via `TextureCache` watch bindings (albedo → slot 1, noise1 → slot 2, noise2 → slot 3).
  - See [Mesh VFX documentation](gameapi/MeshVFX.md) for full parameter reference and usage examples.

### Mesh Creation Model

Use either the convenience descriptor (`MeshCreateInfo`) or the direct overload with vertex/index spans.

```c++
struct AssetManager::MaterialOptions {
  std::string albedoPath;        // resolved through AssetManager
  std::string metalRoughPath;    // resolved through AssetManager
  std::string normalPath;        // resolved through AssetManager (tangent-space normal)
  std::string occlusionPath;     // resolved through AssetManager (ambient occlusion)
  std::string emissivePath;      // resolved through AssetManager (emissive/glow)
  bool albedoSRGB      = true;   // VK_FORMAT_R8G8B8A8_SRGB when true
  bool metalRoughSRGB  = false;  // VK_FORMAT_R8G8B8A8_UNORM when false
  bool normalSRGB      = false;  // normal maps should be UNORM
  bool occlusionSRGB   = false;  // occlusion should be UNORM
  bool emissiveSRGB    = true;   // emissive is typically sRGB
  GLTFMetallic_Roughness::MaterialConstants constants{}; // extra[0].x as normalScale
  MaterialPass pass    = MaterialPass::MainColor; // or Transparent
};

struct AssetManager::MeshGeometryDesc {
  enum class Type { Provided, Cube, Sphere, Plane, Capsule };
  Type type = Type::Provided;
  std::span<Vertex> vertices{};  // when Provided
  std::span<uint32_t> indices{}; // when Provided
  int sectors = 16;              // for Sphere
  int stacks  = 16;              // for Sphere
};

struct AssetManager::MeshMaterialDesc {
  enum class Kind { Default, Textured };
  Kind kind = Kind::Default;
  MaterialOptions options{};     // used when Textured
};

struct AssetManager::MeshCreateInfo {
  std::string name;              // cache key; reused if already created
  MeshGeometryDesc geometry;     // Provided / Cube / Sphere / Plane / Capsule
  MeshMaterialDesc material;     // Default or Textured
  std::optional<BoundsType> boundsType; // optional override for collision/picking bounds
};
```

Behavior and lifetime:
- Default material: If no material is given, a white material is created (2× white textures, per-mesh UBO with sane defaults).
- Textured material: When `MeshMaterialDesc::Textured`, images are loaded via `stb_image` and uploaded; per-mesh UBO is allocated and filled from `constants`.
- Ownership: Material buffers and any images created by the AssetManager are tracked and destroyed on `removeMesh(name)` or `cleanup()`.
- Caching: Meshes are cached by `name`. Re-creating with the same name returns the existing mesh (no new uploads).

### Examples

Create a simple plane and render it (default material):

```c++
std::vector<Vertex> v = {
  {{-0.5f, 0.0f, -0.5f}, 0.0f, {0,1,0}, 0.0f, {1,1,1,1}},
  {{ 0.5f, 0.0f, -0.5f}, 1.0f, {0,1,0}, 0.0f, {1,1,1,1}},
  {{-0.5f, 0.0f,  0.5f}, 0.0f, {0,1,0}, 1.0f, {1,1,1,1}},
  {{ 0.5f, 0.0f,  0.5f}, 1.0f, {0,1,0}, 1.0f, {1,1,1,1}},
};
std::vector<uint32_t> i = { 0,1,2, 2,1,3 };

auto plane = ctx->getAssets()->createMesh("plane", v, i); // default white material
glm::mat4 xform = glm::scale(glm::mat4(1.f), glm::vec3(10.f, 1.f, 10.f));
ctx->scene->addMeshInstance("ground", plane, xform);
```

Generate primitives via `MeshCreateInfo`:

```c++
AssetManager::MeshCreateInfo ci{};
ci.name = "cubeA";
ci.geometry.type = AssetManager::MeshGeometryDesc::Type::Cube;
ci.material.kind = AssetManager::MeshMaterialDesc::Kind::Default;
auto cube = ctx->getAssets()->createMesh(ci);
ctx->scene->addMeshInstance("cube.instance", cube,
    glm::translate(glm::mat4(1.f), glm::vec3(-2.f, 0.f, -2.f)));

AssetManager::MeshCreateInfo si{};
si.name = "sphere48x24";
si.geometry.type = AssetManager::MeshGeometryDesc::Type::Sphere;
si.geometry.sectors = 48; si.geometry.stacks = 24;
si.material.kind = AssetManager::MeshMaterialDesc::Kind::Default;
auto sphere = ctx->getAssets()->createMesh(si);
ctx->scene->addMeshInstance("sphere.instance", sphere,
    glm::translate(glm::mat4(1.f), glm::vec3(2.f, 0.f, -2.f)));

// Plane primitive
AssetManager::MeshCreateInfo pi{};
pi.name = "groundPlane";
pi.geometry.type = AssetManager::MeshGeometryDesc::Type::Plane;
pi.material.kind = AssetManager::MeshMaterialDesc::Kind::Default;
auto plane = ctx->getAssets()->createMesh(pi);

// Capsule primitive
AssetManager::MeshCreateInfo capi{};
capi.name = "capsuleA";
capi.geometry.type = AssetManager::MeshGeometryDesc::Type::Capsule;
capi.material.kind = AssetManager::MeshMaterialDesc::Kind::Default;
auto capsule = ctx->getAssets()->createMesh(capi);
```

Textured primitive (albedo + metal-rough + normal + occlusion + emissive):

```c++
AssetManager::MeshCreateInfo ti{};
ti.name = "ground.textured";
// provide vertices/indices for a plane (see first example)
ti.geometry.type = AssetManager::MeshGeometryDesc::Type::Provided;
ti.geometry.vertices = std::span<Vertex>(v.data(), v.size());
ti.geometry.indices  = std::span<uint32_t>(i.data(), i.size());
ti.material.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
ti.material.options.albedoPath = "textures/ground_albedo.png";     // sRGB
ti.material.options.metalRoughPath = "textures/ground_mr.png";     // UNORM, G=roughness, B=metallic
ti.material.options.normalPath     = "textures/ground_n.png";      // UNORM
ti.material.options.occlusionPath  = "textures/ground_ao.png";     // UNORM (optional)
ti.material.options.emissivePath   = "textures/ground_emit.png";   // sRGB (optional)
ti.material.options.constants.extra[0].x = 1.0f;                    // normalScale
// ti.material.options.pass = MaterialPass::Transparent; // optional

auto texturedPlane = ctx->getAssets()->createMesh(ti);
glm::mat4 tx = glm::scale(glm::mat4(1.f), glm::vec3(10.f, 1.f, 10.f));
ctx->scene->addMeshInstance("ground.textured", texturedPlane, tx);
```

Textured cube/sphere/plane/capsule via options is analogous — set `geometry.type` to `Cube`, `Sphere`, `Plane`, or `Capsule` and fill `material.options`.

Using custom material from constants:

```c++
// Create a material with custom PBR values (using engine default textures)
GLTFMetallic_Roughness::MaterialConstants constants{};
constants.colorFactors = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f); // red
constants.metal_rough_factors = glm::vec4(0.0f, 0.8f, 0.0f, 0.0f); // non-metallic, rough

auto redMaterial = ctx->getAssets()->createMaterialFromConstants(
    "red_rough_material",
    constants,
    MaterialPass::MainColor
);

// Use with custom mesh
auto mesh = ctx->getAssets()->createMesh("custom_mesh", vertices, indices, redMaterial);
```

Runtime glTF spawning:

```c++
auto chair = ctx->getAssets()->loadGLTF("models/chair.glb");
if (chair)
{
  glm::mat4 t = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -3.f));
  ctx->scene->addGLTFInstance("chair01", *chair, t);
}
// Move / overwrite
ctx->scene->addGLTFInstance("chair01", *chair,
  glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.5f, -3.f)));
// Remove
ctx->scene->removeGLTFInstance("chair01");
```

### Texture Prefetching

Queue texture loads for a glTF file ahead of time. This parses the glTF, builds TextureCache keys for referenced images (both external URIs and embedded images in buffers), and issues `TextureCache::request()` calls. Actual uploads happen via the normal per-frame pump.

```c++
// Simple version: returns number of textures scheduled
size_t count = ctx->getAssets()->prefetchGLTFTextures("models/heavy_asset.glb");

// Advanced version: returns handles for tracking progress
auto result = ctx->getAssets()->prefetchGLTFTexturesWithHandles("models/heavy_asset.glb");
fmt::println("Scheduled {} textures", result.scheduled);
// Use result.handles with TextureCache to monitor loading state
```

Texture prefetching is particularly useful when combined with `AsyncAssetLoader` for loading large models in the background.

### Async Asset Loading

The `AsyncAssetLoader` class provides asynchronous glTF loading with worker threads for CPU-bound tasks (file I/O, parsing, mesh/BVH building). GPU uploads are still deferred through ResourceManager and the Render Graph.

```c++
// Access via EngineContext
auto *loader = ctx->async_loader;

// Queue a model to load in the background
auto jobID = loader->loadGltfAsync(
    "spaceship_01",                      // scene instance name
    "models/spaceship.glb",              // model path (resolved via AssetManager)
    glm::translate(glm::mat4(1.f), glm::vec3(0, 5, -10)), // transform
    true                                  // preload textures
);

// Check progress in your update loop
JobState state;
float progress;
std::string error;
if (loader->getJobStatus(jobID, state, progress, &error)) {
    if (state == JobState::Completed) {
        fmt::println("Model loaded successfully!");
    } else if (state == JobState::Failed) {
        fmt::println("Failed to load: {}", error);
    }
}

// Commit completed jobs to the scene (call once per frame)
loader->pumpMainThread(*ctx->scene);

// Alternative: use WorldVec3 for large-world coordinates
auto jobID2 = loader->loadGltfAsync(
    "distant_building",
    "models/building.glb",
    WorldVec3{1000000.0, 0.0, 500000.0}, // world position
    glm::quat(1.0f, 0.0f, 0.0f, 0.0f),   // rotation
    glm::vec3(1.0f),                      // scale
    false                                  // don't preload textures
);
```

The `AsyncAssetLoader` integrates with `TextureCache` to track texture streaming progress. When `preload_textures` is true, the loader will schedule all model textures for loading and track their residency state.

### Notes

- Default primitives: The engine creates default Cube/Sphere/Plane/Capsule meshes via `AssetManager` and registers them as dynamic scene instances.
- Reuse by name: `createMesh("name", ...)` returns the cached mesh if it already exists. Use a unique name or call `removeMesh(name)` to replace.
- sRGB/UNORM: Albedo and emissive are sRGB by default, metal-rough/normal/occlusion are UNORM by default. Adjust via `MaterialOptions`.
- Hot reload: Shaders are resolved via `shaderPath()`; pipeline hot reload is handled by the pipeline manager, not the AssetManager.
- Normal maps: Supported. If `normalPath` is empty, a flat normal is used.
- Occlusion & Emissive: Supported via `occlusionPath` and `emissivePath` in `MaterialOptions`.
- Tangents: Loaded from glTF when present; otherwise generated. Enable MikkTSpace at configure time with `-DENABLE_MIKKTS=ON`.
- BVH building: Enabled by default for meshes (`build_bvh = true`). Required for picking and ray-tracing.
- Deferred cleanup: Use `removeMeshDeferred()` when destroying meshes during rendering to avoid destroying resources that are in-flight on the GPU.
