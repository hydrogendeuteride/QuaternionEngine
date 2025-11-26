Texture Loading & Streaming

Overview
- Streaming cache: `src/core/texture_cache.{h,cpp}` asynchronously decodes images (stb_image) on a small worker pool (1–4 threads, clamped by hardware concurrency) and uploads them via `ResourceManager` with optional mipmaps. For FilePath keys, a sibling `<stem>.ktx2` (or direct `.ktx2`) is preferred over PNG/JPEG. Descriptors registered up‑front are patched in‑place once the texture becomes resident. Large decodes can be downscaled on workers before upload to cap peak memory.
- Uploads: `src/core/vk_resource.{h,cpp}` stages pixel data and either submits immediately or registers a Render Graph transfer pass. Mipmaps use `vkutil::generate_mipmaps(...)` and finish in `SHADER_READ_ONLY_OPTIMAL`.
- Integration points:
  - Materials: layouts use `UPDATE_AFTER_BIND`; descriptors can be rewritten after bind.
  - glTF loader: `src/scene/vk_loader.cpp` builds keys, requests handles, and registers descriptor patches with the cache.
  - Primitives/adhoc: `src/core/asset_manager.cpp` builds materials and registers texture watches.
  - Visibility: `src/render/passes/geometry.cpp` and `src/render/passes/transparent.cpp` call `TextureCache::markSetUsed(...)` for sets that are actually drawn.
- IBL: high‑dynamic‑range environment textures are typically loaded directly as `.ktx2` via `IBLManager` instead of the generic streaming cache. See “Image‑Based Lighting (IBL)” below.

Data Flow
- Request
  - Build a `TextureCache::TextureKey` (FilePath or Bytes), set `srgb` and `mipmapped`.
  - Call `request(key, sampler)` → returns a stable `TextureHandle`, deduplicated by a 64‑bit FNV‑1a hash. For FilePath keys the path plus the sRGB bit are hashed; for Bytes keys the payload hash is XOR’d with a constant when `srgb=true`.
  - Register target descriptors via `watchBinding(handle, set, binding, sampler, fallbackView)`.
- Visibility‑gated scheduling
  - `pumpLoads(...)` looks for entries in `Unloaded` or `Evicted` state that were seen recently (`now == 0` or `now - lastUsed <= 1`) and starts at most `max_loads_per_pump` decodes per call, while enforcing a byte budget for uploads per frame.
  - Render passes mark used sets each frame with `markSetUsed(...)` (or specific handles via `markUsed(...)`).
- Decode
  - FilePath: if the path ends with `.ktx2` or a sibling exists, we load via libktx (and transcode to BCn if needed). Otherwise, decode to RGBA8 via stb_image.
  - Bytes: always decode via stb_image (no sibling discovery possible).
- Admission & Upload
  - Before upload, an expected resident size is computed (exact for KTX2 by summing level byte lengths; estimated for raster by format×area×mip‑factor). A per‑frame byte budget (`max_bytes_per_pump`) throttles uploads.
  - If a GPU texture budget is set, the cache evicts least‑recently‑used textures not used this frame. If it still cannot fit, the decode is deferred or dropped with backoff.
  - Raster: `ResourceManager::create_image(...)` stages a single region, then optionally generates mips on GPU.
  - KTX2: `ResourceManager::create_image_compressed(...)` allocates an image with the file’s `VkFormat` (from libktx) and records one `VkBufferImageCopy` per mip level (no GPU mip gen). Immediate path transitions to `SHADER_READ_ONLY_OPTIMAL`; the RenderGraph path transitions after copy when no mip gen.
  - If the device cannot sample the KTX2 format, the cache falls back to raster decode.
  - After upload: state → `Resident`, descriptors recorded via `watchBinding` are rewritten to the new image view with the chosen sampler and `SHADER_READ_ONLY_OPTIMAL` layout. For Bytes‑backed keys, compressed source bytes are dropped unless `keep_source_bytes` is enabled.
- Eviction & Reload
  - `evictToBudget(bytes)` rewrites watchers to fallbacks, destroys images, and marks entries `Evicted`. Evicted entries can reload automatically when seen again and a short cooldown has passed (default ~2 frames), avoiding immediate thrash.

Runtime UI
- ImGui → Debug → Textures (see `src/core/vk_engine.cpp`)
  - Shows: device‑local budget/usage (from VMA), texture streaming budget (~35% of device‑local by default), resident MiB, CPU source MiB, counts per state, and a Top‑N table of consumers.
  - Controls: `Loads/Frame`, `Upload Budget (MiB)` (byte‑based throttle), `Keep Source Bytes`, `CPU Source Budget (MiB)`, `Max Upload Dimension` (progressive downscale cap), and `Trim To Budget Now`.

Key APIs (src/core/texture_cache.h)
- `TextureHandle request(const TextureKey&, VkSampler)`
- `void watchBinding(TextureHandle, VkDescriptorSet, uint32_t binding, VkSampler, VkImageView fallback)`
- `void unwatchSet(VkDescriptorSet)` — call before destroying descriptor pools/sets
- `void markSetUsed(VkDescriptorSet, uint32_t frameIndex)` and `void markUsed(TextureHandle, uint32_t frameIndex)`
- `void pumpLoads(ResourceManager&, FrameResources&)`
- `void evictToBudget(size_t bytes)`
- Controls: `set_max_loads_per_pump`, `set_keep_source_bytes`, `set_cpu_source_budget`, `set_gpu_budget_bytes`

Defaults & Budgets
- Worker threads: 1–4 decode threads depending on hardware.
- Loads per pump: default 4.
- Upload byte budget: default 128 MiB per frame.
- GPU budget: unlimited until the engine sets one each frame. The engine queries ~35% of device‑local memory (via VMA) and calls `set_gpu_budget_bytes(...)`, then runs `evictToBudget(...)` and `pumpLoads(...)` during the frame loop (`src/core/vk_engine.cpp`).
- CPU source bytes: default budget 64 MiB; `keep_source_bytes` defaults to false. Retention only applies to entries created from Bytes keys.

Examples
- Asset materials (`src/core/asset_manager.cpp`)
  - Create materials with visible fallbacks (checkerboard/white/flat‑normal), then:
    - Build a key from an asset path, `request(key, sampler)`, and `watchBinding(handle, materialSet, binding, sampler, fallbackView)` for albedo (1), metal‑rough (2), normal (3).
- glTF loader (`src/scene/vk_loader.cpp`)
  - Builds keys from URI/Vector/BufferView sources, requests handles, and registers watches for material textures. On teardown, calls `unwatchSet(materialSet)` before resetting descriptor pools to avoid patching dead sets. The geometry/transparent passes mark used sets each frame.

Implementation Notes
- Uploads and layouts
  - Deferred uploads: the RG transfer pass transitions `UNDEFINED → TRANSFER_DST_OPTIMAL`, copies, and either generates mipmaps (finishing in `SHADER_READ_ONLY_OPTIMAL`) or transitions directly there. No extra transition is needed after mip gen.
- Descriptor rewrites
  - Material descriptor sets and pools are created with `UPDATE_AFTER_BIND` flags; patches are applied safely across frames using a `DescriptorWriter`.
- Key hashing
  - 64‑bit FNV‑1a for dedup. FilePath keys hash `PATH:<path>#(sRGB|UNORM)`. Bytes keys hash the payload and XOR an sRGB tag when requested.
- Format selection and channel packing
  - `TextureKey::channels` can be `Auto` (default), `R`, `RG`, or `RGBA`. The cache chooses `VK_FORMAT_R8/R8G8/RGBA8` (sRGB variants when requested) and packs channels on CPU for `R`/`RG` to reduce staging + VRAM.
- Progressive downscale
  - The decode thread downsizes large images by powers of 2 until within `Max Upload Dimension`, reducing both staging and VRAM. You can increase the cap or disable it (set to 0) from the UI.

KTX2 specifics
- Supported: 2D, single‑face, single‑layer KTX2. If BasisLZ/UASTC, libktx transcodes to BCn. sRGB/UNORM is honored from the file’s DFD and can be nudged by request (albedo sRGB, MR/normal UNORM).
- Not supported: Cube/array/multilayer KTX2 in the generic cache path (it assumes single‑layer, 2D). Cubemap KTX2 for IBL is loaded via `IBLManager` (see below).

Limitations / Future Work
- Linear‑blit capability check
  - `vkutil::generate_mipmaps` / `generate_mipmaps_levels` always use `VK_FILTER_LINEAR` for blits without checking `VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT`. Add a per‑format capability check and a fallback path (nearest or compute downsample) for formats that do not support linear filtering (especially some compressed formats).
- Texture formats
  - Raster path: limited to 8‑bit R/RG/RGBA via `stbi_load`. KTX2 path in `TextureCache::worker_loop` currently accepts only BCn/BC6H formats and rejects other VkFormats returned by libktx (e.g., uncompressed `R16G16B16A16_SFLOAT`). Future work: ASTC/ETC2, specialized R8/RG8 parsing, and float HDR support (`stbi_loadf` → `R16G16B16A16_SFLOAT`) so HDR albedo/lighting data can stream through the generic cache (today HDR IBL uses the separate `IBLManager` path).
- Normal‑map mip quality
  - Normal maps share the same linear blit pipeline as color textures; no renormalization pass runs after mip generation. Consider a compute or fragment pass to renormalize normal map mips (or a dedicated normal‑aware downsample) to improve shading at grazing angles and distant LODs.
- Samplers
  - Anisotropy is currently disabled in `SamplerManager` (`anisotropyEnable = VK_FALSE`). Enable it when the feature is present, expose a knob in the Debug UI, and consider per‑material/per‑texture anisotropy settings.
- Minor robustness
  - `enqueue_decode()` computes the handle from the entry pointer (`&e - _entries.data()`) and passes it to worker threads. This is safe as long as `_entries` is not resized during enqueue, but storing the index explicitly when the entry is created (in `request()`) would make the relationship clearer and robust against future refactors.

Operational Tips
- Keep deferred uploads enabled (`ResourceManager::set_deferred_uploads(true)`) to coalesce copies per frame (engine does this during init).
- To debug VMA allocations and name images, set `VE_VMA_DEBUG=1`.

Image‑Based Lighting (IBL) Textures
- Manager: `src/core/ibl_manager.{h,cpp}` owns IBL GPU resources and the shared descriptor set layout for set=3.
- Inputs (`IBLPaths`):
  - `specularCube`: preferred is a GPU‑ready `.ktx2` (BC6H or `R16G16B16A16_SFLOAT`) containing either a cubemap or an equirectangular 2D env with prefiltered mips.
  - `diffuseCube`: optional `.ktx2` cubemap for diffuse irradiance. If missing, diffuse IBL falls back to SH only.
  - `brdfLut2D`: `.ktx2` 2D RG LUT (e.g., `VK_FORMAT_R8G8_UNORM` or BC5).
- Loading:
  - Specular:
    - If `specularCube` is a cubemap `.ktx2`, `IBLManager` uses `ktxutil::load_ktx2_cubemap` and uploads via `ResourceManager::create_image_compressed_layers`, preserving the file’s format and mip chain.
    - If cubemap load fails, it falls back to 2D `.ktx2` via `ktxutil::load_ktx2_2d` + `ResourceManager::create_image_compressed`. The image is treated as equirectangular with prefiltered mips and sampled with explicit LOD in shaders.
    - If the format is float HDR (`R16G16B16A16_SFLOAT` or `R32G32B32A32_SFLOAT`) and the aspect ratio is 2:1, `IBLManager` additionally computes 2nd‑order SH coefficients (9×`vec3`) on the CPU for diffuse irradiance and uploads them to a UBO (`_shBuffer`).
  - Diffuse (optional):
    - If `diffuseCube` is provided and valid, it is uploaded as a cubemap using `create_image_compressed_layers`. Current shaders use the SH buffer for diffuse; this cubemap can be wired into a future path if you want to sample it directly.
  - BRDF LUT:
    - `brdfLut2D` is loaded as 2D `.ktx2` via `ktxutil::load_ktx2_2d` and uploaded with `create_image_compressed`.
  - Fallbacks:
    - `LightingPass` and `TransparentPass` create tiny 1×1 UNORM textures (grey 2D for env, RG for BRDF LUT) so shaders can safely sample IBL bindings even when IBL assets are not loaded.
- Descriptor layout & bindings:
  - `IBLManager::ensureLayout()` creates a descriptor set layout for set=3 with:
    - binding 0: `COMBINED_IMAGE_SAMPLER` — specular env (2D equirect with mips or cubemap sampled via 2D path).
    - binding 1: `COMBINED_IMAGE_SAMPLER` — BRDF LUT 2D.
    - binding 2: `UNIFORM_BUFFER` — SH coefficients (`vec4 sh[9]`, RGB in `.xyz`).
  - Render passes that use IBL fetch this layout from `EngineContext::ibl` and allocate per‑frame sets:
    - `passes/lighting.cpp`: deferred lighting (set=3).
    - `passes/transparent.cpp`: forward/transparent PBR materials (set=3).
    - `passes/background.cpp`: environment background (set=3; only binding 0 is used in the shader).
