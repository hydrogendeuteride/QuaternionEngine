## Render Passes: Background → Geometry → Lighting → Transparent → ImGui

Pass classes (`IRenderPass`) define initialization and recording logic, but execution is now driven by the Render Graph. Each pass exposes a `register_graph(...)` method to declare dependencies and render targets; the graph handles barriers, layouts, and dynamic rendering.

### Overview

- Interface: Each pass implements `IRenderPass { init(context); execute(cmd); cleanup(); getName(); }`. Today, `execute()` is unused for built-in passes; work is recorded via the Render Graph record callback.
- Manager: `RenderPassManager::init()` creates and stores built-in passes: `BackgroundPass` (compute), `GeometryPass` (G-Buffer), `LightingPass` (deferred), `TransparentPass`, plus optional `ImGuiPass`.
- Render graph: Passes call `register_graph(graph, ...)` to declare image/buffer access and attachments. The graph inserts barriers and begins/ends dynamic rendering.
- Shared targets: Passes coordinate through `SwapchainManager` images: `drawImage`, `gBufferPosition/Normal/Albedo`, `depthImage` (imported into the graph each frame).
- Hot reload: Fetch graphics pipeline/layout by key each frame through `PipelineManager` in the record callback.

### Quick Start — Add a New Pass (Render Graph)

```c++
class MyPass : public IRenderPass {
public:
  void init(EngineContext* ctx) override {
    _ctx = ctx;
    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath   = _ctx->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _ctx->getAssets()->shaderPath("my_pass.frag.spv");
    info.setLayouts = { _ctx->getDescriptorLayouts()->gpuSceneDataLayout() };
    info.configure = [this](PipelineBuilder& b){
      b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
      b.set_polygon_mode(VK_POLYGON_MODE_FILL);
      b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
      b.set_multisampling_none(); b.disable_depthtest();
      b.set_color_attachment_format(_ctx->getSwapchain()->drawImage().imageFormat);
    };
    _ctx->pipelines->createGraphicsPipeline("my_pass", info);
  }

  void register_graph(RenderGraph* graph, RGImageHandle draw, RGImageHandle depth) {
    graph->add_pass(
      "MyPass",
      RGPassType::Graphics,
      [draw, depth](RGPassBuilder& b, EngineContext*) {
        b.write_color(draw);
        b.write_depth(depth, false);
      },
      [this](VkCommandBuffer cmd, const RGPassResources&, EngineContext* ctx){
        VkPipeline p{}; VkPipelineLayout l{};
        ctx->pipelines->getGraphics("my_pass", p, l);
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, p);
        VkViewport vp{0,0,(float)ctx->getDrawExtent().width,(float)ctx->getDrawExtent().height,0,1};
        vkCmdSetViewport(cmd, 0, 1, &vp);
        VkRect2D sc{{0,0}, ctx->getDrawExtent()};
        vkCmdSetScissor(cmd, 0, 1, &sc);
        vkCmdDraw(cmd, 3, 1, 0, 0);
      }
    );
  }

  void execute(VkCommandBuffer) override {} // unused with Render Graph
  void cleanup() override {}
  const char* getName() const override { return "MyPass"; }
private:
  EngineContext* _ctx{};
};

// Register in RenderPassManager::init()
auto myPass = std::make_unique<MyPass>();
myPass->init(context);
addPass(std::move(myPass));
```

### Built-in Passes

- Background (compute): Declares `ComputeWrite(drawImage)` and dispatches a selected effect instance.
- Geometry (G-Buffer): Declares 4 color attachments (position, normal+roughness, albedo+metallic, AO+emissive) and `DepthAttachment`, plus buffer reads for shared index/vertex buffers.
- Lighting (deferred): Reads G‑Buffer as sampled images and writes to `drawImage`. Applies AO to indirect lighting and adds emissive contribution.
- Shadows: Cascaded shadow maps render to per-frame transient depth images (four cascades). If Ray Query is enabled,
  the lighting pass additionally samples TLAS to evaluate shadow visibility according to the selected mode.
- SSR (Screen Space Reflections): Reads HDR lighting result + G-Buffer and outputs reflections blended with the scene.
  Two variants: `ssr.nort` (screen-space only) and `ssr.rt` (SSR + RT fallback using TLAS ray queries).
- Tonemap + Bloom: Converts HDR to LDR with exposure control and optional bloom. Supports Reinhard and ACES tonemapping.
- FXAA: Post-process anti-aliasing on the LDR tonemapped image. Simple 5-tap edge-detection blur.
- Transparent (forward): Writes to `drawImage` with depth test against `depthImage` after lighting.
- ImGui: Inserted just before present to draw on the swapchain image.

### API Summary

- `RenderPassManager::addPass(unique_ptr<IRenderPass>)`: Register a new pass (storage/ownership only).
- `RenderPassManager::setImGuiPass(...)`: Configure the optional ImGui pass.
- `IRenderPass::register_graph(...)` (per pass class): Declare resources and recording callbacks for the Render Graph.

### Tips

- Don’t call `vkCmdBeginRendering` or add manual transitions for declared attachments; the Render Graph handles it.
- Re-fetch pipeline and layout by key each frame to pick up hot-reloaded shaders.
- Allocate transient descriptor sets from `currentFrame->_frameDescriptors`; free pass-owned layouts in `cleanup()`.
- Use `EngineContext::getDrawExtent()` for viewport/scissor.

See also: `docs/RenderGraph.md` for the builder API and synchronization details.

---

## Post-Processing Pipeline

After deferred lighting, the engine runs a post-processing chain: SSR → Tonemap (with Bloom) → FXAA → Present.

### SSR (Screen Space Reflections)

Located in `src/render/passes/ssr.cpp` and `shaders/ssr.frag` / `shaders/ssr_rt.frag`.

**Algorithm:**
- World-space ray marching along the reflection vector `R = reflect(-V, N)`.
- Depth comparison against G-Buffer position to find intersection.
- Fresnel (Schlick) and glossiness-based blending with the base HDR color.

**Parameters (shader constants):**
- `MAX_STEPS = 64` – maximum ray march iterations (reduced for rough surfaces).
- `STEP_LENGTH = 0.5` – world units per step.
- `MAX_DISTANCE = 50.0` – maximum ray travel distance.
- `THICKNESS = 3.0` – depth tolerance for hit detection.

**Variants:**
- `ssr.nort` – Pure screen-space reflections.
- `ssr.rt` – SSR + RT fallback using TLAS ray queries when SSR misses (requires `GL_EXT_ray_query`).
  Reflection mode controlled via `sceneData.rtOptions.w`: 0 = SSR only, 1 = SSR + RT fallback, 2 = RT only.

**Inputs (set=1):**
- binding 0: `hdrColor` – HDR lighting result.
- binding 1: `posTex` – G-Buffer world position (RGBA32F).
- binding 2: `normalTex` – G-Buffer normal + roughness.
- binding 3: `albedoTex` – G-Buffer albedo + metallic.

---

### Tonemap + Bloom

Located in `src/render/passes/tonemap.cpp` and `shaders/tonemap.frag`.

**Tonemapping modes:**
- `mode = 0` – Reinhard: `x / (1 + x)`.
- `mode = 1` – ACES (Narkowicz approximation, default).

**Bloom:**
- Simple gather-based bloom computed in HDR space before tonemapping.
- 5×5 kernel (radius=2) samples neighbors; pixels exceeding `bloomThreshold` contribute weighted by their brightness.
- Accumulated bloom is multiplied by `bloomIntensity` and added to the HDR color.

**Runtime parameters:**
- `exposure` (default 1.0) – exposure multiplier.
- `bloomEnabled` (default true) – toggle bloom.
- `bloomThreshold` (default 1.0) – brightness threshold for bloom contribution.
- `bloomIntensity` (default 0.7) – bloom blend strength.

**Output:** LDR image (`VK_FORMAT_R8G8B8A8_UNORM`) with gamma correction (γ = 2.2).

---

### FXAA (Fast Approximate Anti-Aliasing)

Located in `src/render/passes/fxaa.cpp` and `shaders/fxaa.frag`.

**Algorithm:**
- Luma-based edge detection using a 5-tap cross pattern (N, S, E, W, center).
- If luma range exceeds threshold, apply a simple box blur; otherwise pass through.

**Runtime parameters:**
- `enabled` (default true) – toggle FXAA.
- `edge_threshold` (default 0.125) – relative contrast threshold.
- `edge_threshold_min` (default 0.0312) – absolute minimum threshold.

**Push constants:**
```glsl
layout(push_constant) uniform Push {
    float inverse_width;
    float inverse_height;
    float edge_threshold;
    float edge_threshold_min;
} pc;
```

