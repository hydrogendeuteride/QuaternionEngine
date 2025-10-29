**Runtime Controls & Debug UI**

- Camera
  - Move: `W/A/S/D`
  - Look: hold Right Mouse Button
  - Mouse wheel: adjust movement speed
  - Ctrl + wheel: adjust FOV (30°..110°)

- Windows (ImGui)
  - Background: choose compute background effect and `Render Scale`.
  - Stats: frame/draw/update timings, triangle and draw counts.
  - GPU/Resources: per‑frame allocations and render‑graph resource view.
  - Pipelines: list graphics pipelines and hot‑reload changed shaders.
  - Targets: swapchain/draw extent/format info.
  - PostFX: tonemap operator (Reinhard/ACES) and exposure.
  - Scene: counts of opaque/transparent draws.

- Shadow Modes
  - Cascaded shadow mapping (CSM) is the default; cascades are created per‑frame via the render graph.
  - If ray tracing extensions and device support are present, ray‑traced shadows can be enabled via `_context->shadowSettings.mode` (see `src/core/config.h` and usage in `LightingPass`).

