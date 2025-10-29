**Troubleshooting**

- Shader compiler not found
  - Ensure `glslangValidator` (and/or `glslc` on Windows) is on `PATH`.
  - Re‑open your terminal after installing the Vulkan SDK.

- Windows SDK version mismatch
  - `CMakeLists.txt` references `C:/VulkanSDK/1.3.296.0` by default.
  - Update the path or set `VULKAN_SDK` to your installed version.

- Validation errors on startup
  - Update GPU drivers and Vulkan SDK.
  - Try running a Release build to confirm if the issue is validation‑only.

- Black screen or out‑of‑date swapchain
  - Resize the window once to force a swapchain rebuild.
  - Check the ImGui “Targets” window for swapchain/draw formats and extent.

- No models rendering
  - Verify `assets/police_office.glb` exists (see `docs/ASSETS.md`).
  - Open the “Scene” window to confirm draw counts > 0.

- Shader changes not visible
  - Confirm the `.spv` file changed (timestamp) and click “Reload Changed” in the Pipelines window.
  - Ensure you are editing the correct files referenced by `shaderPath()`.

