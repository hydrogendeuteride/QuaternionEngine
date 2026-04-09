**Build & Environment**

- Prerequisites
  - Vulkan SDK installed and `VULKAN_SDK` set.
  - A C++20 compiler and CMake ≥ 3.16.
  - GPU drivers with Vulkan 1.2+.
  - KTX software with libktx
  - Ninja build system on PATH.

- Configure & Build (default, system compiler)
  ```bash
  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
  cmake --build build --target vulkan_engine -j 12
  ```

- Windows clang-cl
  - Requires LLVM (clang-cl, lld-link, llvm-rc) on PATH.
  - Configure:
    ```bash
    cmake -S . -B build -G Ninja \
      -DCMAKE_C_COMPILER=clang-cl \
      -DCMAKE_CXX_COMPILER=clang-cl \
      -DCMAKE_LINKER=lld-link \
      -DCMAKE_RC_COMPILER=llvm-rc \
      -DCMAKE_BUILD_TYPE=Debug
    ```
  - Build:
    ```bash
    cmake --build build --target vulkan_engine -j 12
    ```

- Linux clang
  - Configure:
    ```bash
    cmake -S . -B build -G Ninja \
      -DCMAKE_C_COMPILER=clang \
      -DCMAKE_CXX_COMPILER=clang++ \
      -DCMAKE_BUILD_TYPE=Debug
    ```
  - Build:
    ```bash
    cmake --build build --target vulkan_engine -j 12
    ```

- CLion on Windows (clang-cl)
  - Settings → Build, Execution, Deployment → CMake → CMake options:
    ```
    -G Ninja -DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl -DCMAKE_LINKER=lld-link -DCMAKE_RC_COMPILER=llvm-rc
    ```
  - Build type is selected separately in the CLion profile.

- Run
  - Linux/macOS: `./build/bin/vulkan_engine`
  - Windows: `build\bin\vulkan_engine.exe`

- Shaders
  - CMake compiles GLSL via `glslangValidator` to SPIR-V targeting Vulkan 1.2:
    - Files under `shaders/*.vert|*.frag|*.comp` are rebuilt on `cmake --build`.
  - Helper: `./compile_shaders.py --config Debug|Release` uses `glslc` with `--target-env=vulkan1.3` and supports additional stages (mesh/task/ray tracing).
    - Windows shim: `./compile_shaders.ps1 -Config Debug|Release` (calls the Python script).
  - Ensure `glslangValidator`/`glslc` is on `PATH`. See `docs/SHADERS.md`.

- Windows SDK note
  - `CMakeLists.txt` includes a default SDK path for Windows `1.3.296.0`:
    - Update the path or set `VULKAN_SDK` accordingly if your version differs.

- Third-party deps
  - Vendored under `third_party/` and brought in via CMake. Do not edit headers directly; update through targets.

- Optional: MikkTSpace tangents
  - Enable at configure time: `-DENABLE_MIKKTS=ON` (default ON if found).
  - Requires `third_party/MikkTSpace/mikktspace.c` and `mikktspace.h` (provided).
  - Disable to use the built-in Gram-Schmidt generator: `-DENABLE_MIKKTS=OFF`.

- Validation Layers
  - Enabled in Debug (`kUseValidationLayers = true` in `src/core/config.h`).
  - Disable by building Release or toggling the flag during local experimentation.
