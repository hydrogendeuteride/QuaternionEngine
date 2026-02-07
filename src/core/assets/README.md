# Assets

> Asset loading, caching, path resolution, and texture streaming for the engine.

## Purpose

Provides the full asset pipeline — from locating files on disk, through
asynchronous glTF loading and texture streaming, to IBL environment map
management. All subsystems are designed around deferred GPU uploads and
frame-paced resource creation to avoid stalls.

## Directory Layout

```
core/assets/
├── manager.h / .cpp        — AssetManager: central facade for meshes, materials, glTF
├── async_loader.h / .cpp   — AsyncAssetLoader: background glTF loading with worker threads
├── texture_cache.h / .cpp  — TextureCache: deduplicated texture streaming with LRU eviction
├── ktx_loader.h / .cpp     — ktxutil: KTX2 cubemap / 2D helpers (libktx wrapper)
├── ibl_manager.h / .cpp    — IBLManager: specular + diffuse + BRDF LUT environment maps
└── locator.h / .cpp        — AssetLocator / AssetPaths: filesystem path resolution
```

## Key Types

| Type | Role |
|------|------|
| `AssetManager` | Central entry point — glTF loading, mesh/material creation, path helpers, fallback textures |
| `AssetLocator` | Resolves shader / model / asset paths from an auto-detected or user-set root |
| `AssetPaths` | POD holding root, assets, and shaders directory paths |
| `AsyncAssetLoader` | Multi-threaded glTF loading orchestrator with job tracking and progress reporting |
| `TextureCache` | Streaming cache with dedup, LRU eviction, descriptor patching (UPDATE_AFTER_BIND), and async decode |
| `TextureKey` | Hash-based key for texture deduplication (file path or raw bytes, sRGB, mip options) |
| `IBLManager` | KTX2-based IBL pipeline — specular cubemap, diffuse cubemap, BRDF LUT, SH coefficients |
| `IBLPaths` | POD specifying the .ktx2 paths for specular, diffuse, BRDF, and optional background maps |
| `ktxutil::KtxCubemap` | Loaded cubemap data with Vulkan copy regions ready for staging upload |
| `ktxutil::Ktx2D` | Loaded 2D texture data (e.g. BRDF LUT) with Vulkan copy regions |

## Lifecycle

```
AssetLocator::init()
  └─ AssetPaths::detect() — walks up from CWD to find assets/ and shaders/ dirs

AssetManager::init(engine)
  └─ initializes locator, ready to load glTF / create meshes

TextureCache::init(ctx)
  └─ spawns decode worker threads

AsyncAssetLoader::init(engine, assets, textures)
  └─ spawns worker threads for background glTF parsing

IBLManager::init(ctx)
  └─ creates descriptor set layout (set=3) for IBL shaders

── per frame ──
TextureCache::pumpLoads(rm, frame)
  ├─ drain_ready_uploads()  — upload decoded images to GPU (budget-gated)
  ├─ patch_ready_entry()    — rewrite watched descriptors to new image views
  └─ evictCpuToBudget()     — trim retained CPU source bytes

AsyncAssetLoader::pump_main_thread(scene)
  └─ commit completed jobs into SceneManager

IBLManager::pump_async()
  └─ finalize background IBL load (GPU upload + SH buffer)

── shutdown ──
AsyncAssetLoader::shutdown()
TextureCache::cleanup()
IBLManager::unload()
AssetManager::cleanup()
  └─ destroys mesh buffers, material buffers, owned images, clears glTF cache
```

## Usage

### Loading a glTF model

```cpp
auto gltf = ctx->assets->loadGLTF("models/helmet.glb");
if (gltf)
    scene.loadScene(*gltf, transform);
```

### Asynchronous glTF loading

```cpp
auto jobId = engine.load_gltf_async("main_scene", "models/city.glb",
                                     transform, /*preloadTextures=*/true);

// Each frame:
asyncLoader->pump_main_thread(scene);

// Query progress:
AsyncAssetLoader::JobState state;
float progress;
asyncLoader->get_job_status(jobId, state, progress);
```

### Texture streaming

```cpp
TextureCache::TextureKey key;
key.kind = TextureCache::TextureKey::SourceKind::FilePath;
key.path = "/textures/brick_albedo.png";
key.srgb = true;
auto handle = ctx->textures->request(key, sampler);

// Patch descriptor when ready:
ctx->textures->watchBinding(handle, descriptorSet, binding, sampler, fallbackView);

// Each frame:
ctx->textures->pumpLoads(resourceManager, frame);
ctx->textures->evictToBudget(budgetBytes);
```

### Creating a procedural mesh with PBR material

```cpp
AssetManager::MeshCreateInfo ci;
ci.name = "my_box";
ci.geometry.type = AssetManager::MeshGeometryDesc::Type::Cube;
ci.material.kind = AssetManager::MeshMaterialDesc::Kind::Textured;
ci.material.options.albedoPath = "textures/wood_albedo.png";
ci.material.options.metalRoughPath = "textures/wood_mr.png";
auto mesh = ctx->assets->createMesh(ci);
```

### IBL environment loading

```cpp
IBLPaths paths;
paths.specularCube = "env/sky_specular.ktx2";
paths.diffuseCube  = "env/sky_diffuse.ktx2";
paths.brdfLut2D    = "env/brdf_lut.ktx2";
ctx->ibl->load_async(paths);

// Each frame:
auto result = ctx->ibl->pump_async();
if (result.completed && result.success)
    rebuild_ibl_descriptors();
```

## Integration

`AssetManager` is owned by `VulkanEngine` and exposed via `EngineContext::assets`.
`TextureCache` is exposed via `EngineContext::textures`.
`IBLManager` is exposed via `EngineContext::ibl`.
`AsyncAssetLoader` is owned by `VulkanEngine` (`_asyncLoader`) and accessed through
the engine's `load_gltf_async()` convenience wrappers.

`TextureCache` integrates with `ResourceManager` for GPU uploads and leverages
`UPDATE_AFTER_BIND` descriptors to patch bindings in-place without rebuilding
descriptor sets.

## Related Docs

- [docs/AssetManager.md](../../../docs/AssetManager.md) — asset management system documentation
- [docs/TextureCache.md](../../../docs/TextureCache.md) — texture streaming documentation
