**Assets & Paths**

- Default locations
  - `assets/` for models/textures and `shaders/` for GLSL live at the repo root.
  - The engine auto‑detects these folders by walking up from the working directory.

- Override root via environment
  - Set `VKG_ASSET_ROOT` to a directory containing `assets/` and/or `shaders/`.
  - Example: `VKG_ASSET_ROOT=/home/user/vulkan-engine ./bin/vulkan_engine`

- API
  - Use `AssetLocator` through `EngineContext::getAssets()` helpers:
    - `shaderPath("name.spv")` resolves a shader.
    - `assetPath("relative/or/absolute")` resolves runtime assets.
    - `modelPath("some.glb")` is an alias for `assetPath`.

- Sample content
  - The engine loads `assets/police_office.glb` by default in `VulkanEngine::init()`.
  - Ensure this file (and any textures it references) exists under your asset root, or adjust the path used by the sample scene.

- Materials & sRGB
  - See `docs/asset_manager.md` for mesh/material creation and sRGB/UNORM handling.

