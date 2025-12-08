**Shaders & Hot Reload**

- Locations
  - Sources live under `shaders/` and are compiled to `.spv` next to the sources.

- Build integration
  - CMake invokes `glslangValidator -V` for `*.vert`, `*.frag`, `*.comp` targeting Vulkan 1.2.
  - Windows PowerShell helper `compile_shaders.ps1` uses `glslc` targeting Vulkan 1.3 and supports additional stages:
    - `.mesh` (`-fshader-stage=mesh`), `.task`, and ray tracing stages (`.rgen`, `.rmiss`, `.rchit`, `.rahit`, `.rint`, `.rcall`).
  - Keep `glslangValidator`/`glslc` on `PATH` and ensure your Vulkan SDK is installed.

- Hot reload
  - `PipelineManager::hotReloadChanged()` watches `.spv` modification times.
  - Pipelines rebind the next frame when a shader file timestamp changes.
  - ImGui → Pipelines window provides a manual “Reload Changed” button and lists currently registered pipelines.

- Conventions
  - Name SPIR‑V files with full extension, e.g. `fullscreen.vert.spv`, `deferred_lighting.frag.spv`.
  - Use `EngineContext::getAssets()->shaderPath("<name>.spv")` when registering pipelines.
  - Use sRGB formats for albedo textures and UNORM for PBR control textures (see `docs/asset_manager.md`).
  - Material UBO layout (`GLTFMaterialData`):
    - `vec4 colorFactors;`
    - `vec4 metal_rough_factors; // x = metallic, y = roughness`
    - `vec4 extra[14];            // extra[0].x = normalScale`
  - Material texture bindings (set=1):
    - binding=1 `colorTex`, binding=2 `metalRoughTex`, binding=3 `normalMap`, binding=4 `occlusionTex`, binding=5 `emissiveTex`.

- Adding a pipeline (graphics)
  - Fill `GraphicsPipelineCreateInfo` with shader paths, descriptor set layouts, optional push constants, and a `configure(PipelineBuilder&)` callback to set topology, raster, depth/blend, and attachment formats.
  - Register with `PipelineManager::createGraphicsPipeline(name, info)`. Retrieve via `getGraphics` or `getMaterialPipeline`.

Shader File Reference

| File | Stage | Description |
|------|-------|-------------|
| `fullscreen.vert` | Vertex | Fullscreen triangle for post-process passes |
| `mesh.vert` | Vertex | Standard mesh vertex transform |
| `mesh.frag` | Fragment | Forward shading (deprecated, use gbuffer) |
| `gbuffer.frag` | Fragment | G-Buffer output (position, normal, albedo, AO+emissive) |
| `deferred_lighting.frag` | Fragment | Deferred lighting with RT shadows + IBL |
| `deferred_lighting_nort.frag` | Fragment | Deferred lighting without RT |
| `shadow.vert/.frag` | Vertex/Fragment | Shadow map generation |
| `ssr.frag` | Fragment | Screen-space reflections (ray march) |
| `ssr_rt.frag` | Fragment | SSR + RT fallback (ray query) |
| `tonemap.frag` | Fragment | HDR→LDR tonemapping + bloom |
| `fxaa.frag` | Fragment | Fast approximate anti-aliasing |
| `sky.comp` | Compute | Procedural sky background |
| `gradient_color.comp` | Compute | Gradient background |
| `background_env.frag` | Fragment | Environment map background |

GLSL Includes

| File | Purpose |
|------|---------|
| `input_structures.glsl` | SceneData UBO, material bindings, light structs |
| `lighting_common.glsl` | BRDF evaluation, point light helpers |
| `ibl_common.glsl` | IBL split-sum, SH irradiance |

