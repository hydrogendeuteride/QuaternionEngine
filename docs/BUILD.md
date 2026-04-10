**Build & Environment**

- Prerequisites
  - Vulkan SDK installed and `VULKAN_SDK` set.
  - CMake installed.
  - Linux/WSL: `clang`, `clang++`, `make`.
  - Windows clang-cl: `clang-cl`, `lld-link`, `llvm-rc`, `ninja`.

- Quick build
  ```bash
  python3 ./build.py
  python3 ./build.py debug
  python3 ./build.py release linux ./out/release
  py .\build.py debug windows .\cmake-build-debug-win-clangcl
  ```
  - Format: `build.py [debug|release|reldeb] [linux|windows] [optional_build_dir]`

- Output
  - Runtime output defaults to repo-root `bin/`.
  - Linux/WSL: `./bin/vulkan_engine`
  - Windows single-config: `bin\vulkan_engine.exe`
  - Windows multi-config: `bin\Debug\vulkan_engine.exe` or `bin\Release\vulkan_engine.exe`
  - To keep runtime artifacts inside the build tree instead: `-DVULKAN_ENGINE_OUTPUT_TO_SOURCE_ROOT=OFF`

- compile_commands.json
  - If you need it, enable it explicitly in your build dir with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

- Manual CMake
  - Linux/WSL clang:
    ```bash
    cmake -S . -B cmake-build-release-linux-clang -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
    cmake --build cmake-build-release-linux-clang --target vulkan_engine --parallel 16
    ```
  - Windows clang-cl:
    ```bash
    cmake -S . -B cmake-build-release-win-clangcl -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl -DCMAKE_LINKER=lld-link -DCMAKE_RC_COMPILER=llvm-rc
    cmake --build cmake-build-release-win-clangcl --target vulkan_engine --parallel 16
    ```

- Shaders
  - CMake compiles `shaders/*.vert|*.frag|*.comp` to `.spv` on build.
  - Manual helper: `./compile_shaders.py --config Debug|Release`

- Validation Layers
  - Enabled in Debug in `src/core/config.h`.
  - Use Release to turn them off for normal runtime testing.
