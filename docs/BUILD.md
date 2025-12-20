**Build & Environment**

- Prerequisites
  - Vulkan SDK installed and `VULKAN_SDK` set.
  - A C++20 compiler and CMake ≥ 3.8.
  - GPU drivers with Vulkan 1.2+.
  - KTX software with libktx

- Configure
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`
  - Re-run configure after toolchain or dependency changes.

- Build (single-config)
  - `cmake --build build --target vulkan_engine`

- Build (multi-config, e.g., MSVC)
  - `cmake --build build --config Release`

- Run
  - `./bin/vulkan_engine` (Linux/macOS)
  - `bin/vulkan_engine.exe` (Windows)

- Shaders
  - CMake compiles GLSL via `glslangValidator` to SPIR‑V targeting Vulkan 1.2:
    - Files under `shaders/*.vert|*.frag|*.comp` are rebuilt on `cmake --build`.
  - Helper: `./compile_shaders.py --config Debug|Release` uses `glslc` with `--target-env=vulkan1.3` and supports additional stages (mesh/task/ray tracing).
    - Windows shim: `./compile_shaders.ps1 -Config Debug|Release` (calls the Python script).
  - Ensure `glslangValidator`/`glslc` is on `PATH`. See `docs/SHADERS.md`.

- Windows SDK note
  - `CMakeLists.txt` includes a default SDK path for Windows `1.3.296.0`:
    - Update the path or set `VULKAN_SDK` accordingly if your version differs.

- Third‑party deps
  - Vendored under `third_party/` and brought in via CMake. Do not edit headers directly; update through targets.

- Optional: MikkTSpace tangents
  - Enable at configure time: `-DENABLE_MIKKTS=ON` (default ON if found).
  - Requires `third_party/MikkTSpace/mikktspace.c` and `mikktspace.h` (provided).
  - Disable to use the built‑in Gram–Schmidt generator: `-DENABLE_MIKKTS=OFF`.

- Validation Layers
  - Enabled in Debug (`kUseValidationLayers = true` in `src/core/config.h`).
  - Disable by building Release or toggling the flag during local experimentation.
