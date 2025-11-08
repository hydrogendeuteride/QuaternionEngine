Texture Loading & Streaming

Overview
- Streaming cache: `src/core/texture_cache.{h,cpp}` asynchronously decodes images (stb_image) on a small worker pool (1–4 threads, clamped by hardware concurrency) and uploads them via `ResourceManager` with optional mipmaps. Descriptors registered up‑front are patched in‑place once the texture becomes resident. Large decodes can be downscaled on workers before upload to cap peak memory.
- Uploads: `src/core/vk_resource.{h,cpp}` stages pixel data and either submits immediately or registers a Render Graph transfer pass. Mipmaps use `vkutil::generate_mipmaps(...)` and finish in `SHADER_READ_ONLY_OPTIMAL`.
- Integration points:
  - Materials: layouts use `UPDATE_AFTER_BIND`; descriptors can be rewritten after bind.
  - glTF loader: `src/scene/vk_loader.cpp` builds keys, requests handles, and registers descriptor patches with the cache.
  - Primitives/adhoc: `src/core/asset_manager.cpp` builds materials and registers texture watches.
  - Visibility: `src/render/vk_renderpass_geometry.cpp` and `src/render/vk_renderpass_transparent.cpp` call `TextureCache::markSetUsed(...)` for sets that are actually drawn.

Data Flow
- Request
  - Build a `TextureCache::TextureKey` (FilePath or Bytes), set `srgb` and `mipmapped`.
  - Call `request(key, sampler)` → returns a stable `TextureHandle`, deduplicated by a 64‑bit FNV‑1a hash. For FilePath keys the path plus the sRGB bit are hashed; for Bytes keys the payload hash is XOR’d with a constant when `srgb=true`.
  - Register target descriptors via `watchBinding(handle, set, binding, sampler, fallbackView)`.
- Visibility‑gated scheduling
  - `pumpLoads(...)` looks for entries in `Unloaded` or `Evicted` state that were seen recently (`now == 0` or `now - lastUsed <= 1`) and starts at most `max_loads_per_pump` decodes per call, while enforcing a byte budget for uploads per frame.
  - Render passes mark used sets each frame with `markSetUsed(...)` (or specific handles via `markUsed(...)`).
- Decode
  - Worker threads decode to RGBA8 with stb_image (`stbi_load` / `stbi_load_from_memory`). Results are queued for the main thread.
-- Admission & Upload
  - Before upload, an expected resident size is computed from chosen format (R/RG/RGBA) and mip count (full chain or clamped). A per‑frame byte budget (`max_bytes_per_pump`) throttles the total amount uploaded each pump.
  - If a GPU texture budget is set, the cache tries to free space by evicting least‑recently‑used textures not used this frame. If it still cannot fit, the decode is deferred (kept in the ready queue) or dropped with backoff if VRAM is tight.
  - Uploads are created via `ResourceManager::create_image(...)`, which now supports an explicit mip count. Deferred upload paths generate exactly the requested number of mips.
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

Limitations / Future Work
- Linear‑blit capability check
  - `generate_mipmaps` always uses `VK_FILTER_LINEAR`. Add a format/feature check and a fallback path (nearest or compute downsample).
- Texture formats
  - Only 8‑bit RGBA uploads via stb_image today. Consider KTX2/BasisU for ASTC/BCn, specialized R8/RG8 paths, and float HDR support (`stbi_loadf` → `R16G16B16A16_SFLOAT`).
- Normal‑map mip quality
  - Linear blits reduce normal length; consider a compute renormalization pass.
- Samplers
  - Anisotropy is currently disabled in `SamplerManager`; enable when supported and expose a knob.
- Minor robustness
  - `enqueue_decode()` derives the handle via pointer arithmetic on `_entries`. Passing the precomputed index would avoid any future reallocation hazards.

Operational Tips
- Keep deferred uploads enabled (`ResourceManager::set_deferred_uploads(true)`) to coalesce copies per frame (engine does this during init).
- To debug VMA allocations and name images, set `VE_VMA_DEBUG=1`.
