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

- Adding a pipeline (graphics)
  - Fill `GraphicsPipelineCreateInfo` with shader paths, descriptor set layouts, optional push constants, and a `configure(PipelineBuilder&)` callback to set topology, raster, depth/blend, and attachment formats.
  - Register with `PipelineManager::createGraphicsPipeline(name, info)`. Retrieve via `getGraphics` or `getMaterialPipeline`.

