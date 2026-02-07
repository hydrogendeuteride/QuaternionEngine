Image-Based Lighting (IBL)

Overview
- IBL assets (environment maps + BRDF LUT + SH coefficients) are managed by `IBLManager` (`src/core/assets/ibl_manager.{h,cpp}`) and exposed to passes via `EngineContext::ibl`.
- Shaders share a common include, `shaders/ibl_common.glsl`, which defines the IBL bindings for descriptor set 3 and helper functions used by deferred, forward, and background passes.
- The engine currently supports:
  - Specular environment from an equirectangular 2D texture with prefiltered mips (`sampler2D iblSpec2D`).
  - Diffuse irradiance from 2nd-order SH (9 coefficients baked on the CPU).
  - A 2D BRDF integration LUT used for the split-sum approximation.
  - An optional separate background environment texture (`sampler2D iblBackground2D`); when not provided, the system falls back to using the specular environment for background rendering.

Data Flow
- Init:
  - `VulkanEngine::init_vulkan()` creates an `IBLManager`, calls `init(context)`, and publishes it via `EngineContext::ibl`.
  - The engine optionally loads default IBL assets (`IBLPaths` in `src/core/engine.cpp`), typically a BRDF LUT plus a specular environment `.ktx2`.
- Loading (IBLManager):
  - `IBLManager::load(const IBLPaths&)` (synchronous, mostly used in tools/tests):
    - Specular:
      - Tries `ktxutil::load_ktx2_cubemap` first. If successful, uploads via `ResourceManager::create_image_compressed_layers` with `VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT`.
      - If cubemap loading fails, falls back to 2D `.ktx2` via `ktxutil::load_ktx2_2d` and `create_image_compressed`. The image is treated as equirectangular with prefiltered mips.
      - When the specular `.ktx2` is HDR (`R16G16B16A16_SFLOAT` or `R32G32B32A32_SFLOAT`) and 2:1 aspect, `IBLManager` computes 9 SH coefficients on the CPU:
        - Integrates the environment over the sphere using real SH basis functions (L2) with solid-angle weighting.
        - Applies Lambert band scaling (A0 = pi, A1 = 2pi/3, A2 = pi/4).
        - Uploads the result as `vec4 sh[9]` in a uniform buffer (`_shBuffer`).
    - Diffuse:
      - If `IBLPaths::diffuseCube` is provided and valid, loads it as a cubemap via `load_ktx2_cubemap` + `create_image_compressed_layers`.
      - Current shaders only use the SH buffer for diffuse; the diffuse cubemap is reserved for future variants.
    - Background:
      - If `IBLPaths::background2D` is provided and valid, loads it as a 2D equirectangular `.ktx2` via `load_ktx2_2d` + `create_image_compressed`.
      - This allows using a separate, potentially higher-resolution or unfiltered environment for the sky background while using a prefiltered version for specular IBL.
    - BRDF LUT:
      - Loaded as 2D `.ktx2` via `ktxutil::load_ktx2_2d` and uploaded with `create_image_compressed`.
    - Fallbacks:
      - If `diffuseCube` is missing but a specular env exists, `_diff` is aliased to `_spec`.
      - If `background2D` is missing but a specular env exists, `_background` is aliased to `_spec`.
  - `IBLManager::unload()` releases GPU images, the SH buffer, and the descriptor set layout.
- Descriptor layout:
  - `IBLManager::ensureLayout()` builds a descriptor set layout (set=3) with:
    - binding 0: `COMBINED_IMAGE_SAMPLER` - specular environment (2D equirect).
    - binding 1: `COMBINED_IMAGE_SAMPLER` - BRDF LUT 2D.
    - binding 2: `UNIFORM_BUFFER` - SH coefficients (`vec4 sh[9]`).
    - binding 3: `COMBINED_IMAGE_SAMPLER` - background environment (2D equirect, optional).
  - Passes request this layout from `EngineContext::ibl` and plug it into their pipeline set layouts:
    - Background: `vk_renderpass_background.cpp` (set 3 used for env background).
    - Lighting: `vk_renderpass_lighting.cpp` (deferred lighting pass, set 3).
    - Transparent: `vk_renderpass_transparent.cpp` (forward/transparent materials, set 3).

Asynchronous Loading
- Overview:
  - `IBLManager` provides an asynchronous loading path via `load_async()` + `pump_async()` to avoid blocking the main/game loop during IBL environment switches or initial loading.
  - Heavy CPU work (KTX2 file I/O, decompression, SH coefficient baking) runs on a dedicated worker thread.
  - GPU resource creation (image uploads, buffer allocation) is deferred to the main thread via `pump_async()`.
- API:
  - `bool load_async(const IBLPaths &paths)`:
    - Queues an asynchronous IBL load job.
    - Returns `false` if the job could not be queued (e.g., context not initialized).
    - If called while a previous job is still pending, the new request supersedes the old one (the old result is discarded when ready).
  - `struct AsyncResult { bool completed; bool success; }`:
    - `completed`: `true` when an async job finished since the last `pump_async()` call.
    - `success`: `true` when the finished job successfully produced new GPU IBL resources.
  - `AsyncResult pump_async()`:
    - Must be called on the main thread, typically once per frame after the previous frame's GPU work is idle.
    - If a completed async job is pending:
      - Destroys old IBL images and SH buffer via `destroy_images_and_sh()`.
      - Creates new GPU images with `create_image_compressed(_layers)` and uploads the SH buffer.
    - Returns `AsyncResult` indicating whether a job completed and its success status.
- Internal Architecture:
  - `IBLManager::init()` spawns a persistent worker thread that waits on a condition variable.
  - When `load_async()` is called:
    - The request paths and a unique job ID are stored in `AsyncStateData`.
    - The worker thread is signaled via condition variable.
    - Any previous pending result is invalidated (superseded by the new job ID).
  - Worker thread execution:
    - Calls `prepare_ibl_cpu()` to load KTX2 files and bake SH coefficients.
    - Stores the prepared data (`PreparedIBLData`) in `AsyncStateData`.
    - Marks the result as ready with the corresponding job ID.
    - If the job ID no longer matches (superseded), the result is discarded.
  - Main thread integration (`pump_async()`):
    - Checks if a result is ready.
    - If ready, calls `commit_prepared()` to create GPU resources from the prepared CPU data.
    - Clears the ready flag and returns the result status.
- Thread Safety:
  - All shared state in `AsyncStateData` is protected by a mutex.
  - The worker thread only reads request data and writes result data.
  - The main thread only reads result data and writes request data.
  - GPU resource creation is strictly on the main thread.
- Usage Example:
  ```cpp
  // Queue async IBL load (non-blocking)
  iblManager->load_async(IBLPaths{
      .specularCube = "assets/ibl/studio_spec.ktx2",
      .brdfLut2D = "assets/ibl/brdf_lut.ktx2",
      .background2D = "assets/ibl/studio_bg.ktx2"
  });

  // In main loop, after waiting for previous frame:
  auto result = iblManager->pump_async();
  if (result.completed) {
      if (result.success) {
          // New IBL environment is now active
      } else {
          // Loading failed, handle error (e.g., keep previous IBL)
      }
  }
  ```
- Benefits:
  - No frame stalls when loading large HDR environment maps.
  - Seamless IBL volume transitions (e.g., entering a building with different lighting).
  - SH baking (CPU-intensive) happens off the main thread.
- Cleanup:
  - `IBLManager::unload()` shuts down the async worker thread (joins) and releases all GPU resources.
  - The destructor also calls `shutdown_async()` to ensure clean termination.

Shader Side (`shaders/ibl_common.glsl`)
- Bindings:
  - `layout(set=3, binding=0) uniform sampler2D iblSpec2D;`
  - `layout(set=3, binding=1) uniform sampler2D iblBRDF;`
  - `layout(std140, set=3, binding=2) uniform IBL_SH { vec4 sh[9]; } iblSH;`
  - `layout(set=3, binding=3) uniform sampler2D iblBackground2D;`
- Helpers:
  - `vec3 sh_eval_irradiance(vec3 n)`:
    - Evaluates the 9 SH basis functions (L2) at direction `n` using the same real SH basis as the CPU bake.
    - Multiplies each basis value by the corresponding `iblSH.sh[i].rgb` coefficient and sums the result.
    - Coefficients are already convolved with the Lambert kernel on the CPU; the function returns diffuse irradiance directly.
  - `vec2 dir_to_equirect(vec3 d)`:
    - Normalizes `d`, computes `(phi, theta)` and returns equirectangular UV in `[0,1]^2`.
    - Used consistently by background, deferred, and forward pipelines.
  - `float ibl_lod_from_roughness(float roughness, float levels)`:
    - Computes the mip LOD for specular IBL using `roughness^2 * (levels - 1)`.
    - This biases mid-roughness reflections towards blurrier mips and avoids overly sharp reflections.

Usage in Passes
- Deferred lighting (`shaders/deferred_lighting.frag` and `shaders/deferred_lighting_nort.frag`):
  - Include:
    - `#include "input_structures.glsl"`
    - `#include "ibl_common.glsl"`
  - IBL contribution (per pixel):
    - Specular:
      - `vec3 R = reflect(-V, N);`
      - `float levels = float(textureQueryLevels(iblSpec2D));`
      - `float lod = ibl_lod_from_roughness(roughness, levels);`
      - `vec2 uv = dir_to_equirect(R);`
      - `vec3 prefiltered = textureLod(iblSpec2D, uv, lod).rgb;`
      - `vec2 brdf = texture(iblBRDF, vec2(max(dot(N,V),0.0), roughness)).rg;`
      - `vec3 specIBL = prefiltered * (F0 * brdf.x + brdf.y);`
    - Diffuse:
      - `vec3 diffIBL = (1.0 - metallic) * albedo * sh_eval_irradiance(N);`
    - Combined:
      - `color += diffIBL + specIBL;`
- Forward/transparent (`shaders/mesh.frag`):
  - Same include and IBL logic as deferred, applied after direct lighting.
  - Uses the same `ibl_lod_from_roughness` helper for LOD selection.
- Background (`shaders/background_env.frag`):
  - Includes `ibl_common.glsl` and uses `dir_to_equirect(worldDir)` + `textureLod(iblBackground2D, uv, 0.0)` to render the environment at LOD 0.
  - When a dedicated background texture is provided via `IBLPaths::background2D`, the background pass renders from `iblBackground2D` which may differ from `iblSpec2D`.

Authoring IBL Assets
- Specular environment:
  - Preferred: prefiltered HDR cubemap in `.ktx2` (BC6H or `R16G16B16A16_SFLOAT`) with multiple mips.
  - Alternative: prefiltered equirectangular 2D `.ktx2` with width = 2 x height and full mip chain.
  - Make sure the mip chain is generated with a GGX importance sampling tool so the BRDF LUT + mip chain match.
- BRDF LUT:
  - A standard 2D preintegrated GGX LUT (RG), usually stored as `R8G8_UNORM` or BC5.
  - The LUT is sampled with `(NoV, roughness)` coordinates.
- Diffuse:
  - The engine currently uses SH coefficients baked from the specular equirectangular map. If you provide a separate diffuse cubemap, the CPU SH bake still uses the specular HDR; you can adjust this in `IBLManager` if you want SH to come from a different source.
- Background:
  - Optional: equirectangular 2D `.ktx2` used exclusively for the sky background pass.
  - Useful when you want a sharper or unfiltered environment for the visible sky while using a prefiltered version for specular reflections.
  - If not provided, the system falls back to using `specularCube` for background rendering.

Implementation Notes
- CPU SH bake:
  - Implemented in `IBLManager::load` using libktx to access raw HDR pixel data from `.ktx2`.
  - Uses a simple nested loop over pixels with solid-angle weighting and the same SH basis as `sh_eval_irradiance`.
- Fallbacks:
  - Lighting and transparent passes create small fallback textures so that the IBL descriptor set is always valid, even when no IBL assets are loaded.
  - Background pass builds a 1x1x6 black cube as a fallback env.
  - When `background2D` is not provided, `IBLManager::background()` returns the same image as `specular()`.

