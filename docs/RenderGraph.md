## Render Graph: Per‑Frame Scheduling, Barriers, and Dynamic Rendering

Lightweight render graph that builds a per‑frame DAG from pass declarations, computes the necessary resource barriers/layout transitions, and records passes with dynamic rendering when attachments are declared.

### Why

- Centralize synchronization and image layout transitions across passes.
- Make passes declarative: author declares reads/writes; the graph inserts barriers and begins/ends rendering.
- Keep existing pass classes (`IRenderPass`) while migrating execution to the graph.

### High‑Level Flow

- Engine creates the graph each frame and imports swapchain/G‑Buffer images: `src/core/vk_engine.cpp:303`.
- Each pass registers its work by calling `register_graph(graph, ...)` and declaring resources via a builder.
- The graph appends a present chain (copy HDR `drawImage` → swapchain, then transition to `PRESENT`), optionally inserting ImGui before present.
- `compile()` topologically sorts passes by data dependencies (read/write) and computes per‑pass barriers.
- `execute(cmd)` emits barriers, begins dynamic rendering if attachments were declared, calls the pass record lambda, and ends rendering.

### Core API

- `RenderGraph::add_pass(name, RGPassType type, BuildCallback build, RecordCallback record)`
  - Declare image/buffer accesses and attachments inside `build` using `RGPassBuilder`.
  - Do your actual rendering/copies in `record` using resolved Vulkan objects from `RGPassResources`.
  - See: `src/render/graph/graph.h:36`, `src/render/graph/graph.cpp:51`.

- `RenderGraph::compile()` → builds ordering and per‑pass `Vk*MemoryBarrier2` lists. See `src/render/graph/graph.cpp:83`.

- `RenderGraph::execute(cmd)` → emits barriers and dynamic rendering begin/end. See `src/render/graph/graph.cpp:592`.

- Import helpers for engine images: `import_draw_image()`, `import_depth_image()`, `import_gbuffer_*()`, `import_swapchain_image(index)`. See `src/render/graph/graph.cpp:740`.

- Present chain: `add_present_chain(draw, swapchain, appendExtra)` inserts Copy→Present passes and lets you inject extra passes (e.g., ImGui) in between. See `src/render/graph/graph.cpp:705`.

### Declaring a Pass

Use `register_graph(...)` on your pass to declare resources and record work. The graph handles transitions and dynamic rendering.

```c++
void MyPass::register_graph(RenderGraph* graph,
                            RGImageHandle draw,
                            RGImageHandle depth) {
  graph->add_pass(
    "MyPass",
    RGPassType::Graphics,
    // Build: declare resources + attachments
    [draw, depth](RGPassBuilder& b, EngineContext*) {
      b.read(draw,  RGImageUsage::SampledFragment); // example read
      b.write_color(draw);                          // render target
      b.write_depth(depth, /*clear*/ false);        // depth test
    },
    // Record: issue Vulkan commands (no begin/end rendering needed)
    [this, draw](VkCommandBuffer cmd, const RGPassResources& res, EngineContext* ctx) {
      VkPipeline p{}; VkPipelineLayout l{};
      ctx->pipelines->getGraphics("my_pass", p, l); // hot‑reload safe
      vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, p);
      VkViewport vp{0,0,(float)ctx->getDrawExtent().width,(float)ctx->getDrawExtent().height,0,1};
      vkCmdSetViewport(cmd, 0, 1, &vp);
      VkRect2D sc{{0,0}, ctx->getDrawExtent()};
      vkCmdSetScissor(cmd, 0, 1, &sc);
      vkCmdDraw(cmd, 3, 1, 0, 0);
    }
  );
}
```

### Builder Reference (`RGPassBuilder`)

- Images
  - `read(RGImageHandle, RGImageUsage)` → sample/read usage for this pass.
  - `write(RGImageHandle, RGImageUsage)` → write usage (compute/storage/transfer).
  - `write_color(RGImageHandle, bool clearOnLoad=false, VkClearValue clear={})` → declares a color attachment.
  - `write_depth(RGImageHandle, bool clearOnLoad=false, VkClearValue clear={})` → declares a depth attachment.

- Buffers
  - `read_buffer(RGBufferHandle, RGBufferUsage)` / `write_buffer(RGBufferHandle, RGBufferUsage)`.
  - Convenience import: `read_buffer(VkBuffer, RGBufferUsage, size, name)` and `write_buffer(VkBuffer, ...)` dedup by raw handle.

See `src/render/graph/builder.h:39` and impl in `src/render/graph/builder.cpp:20`.

### Resource Model (`RGResourceRegistry`)

- Imported vs transient resources are tracked uniformly with lifetime indices (`firstUse/lastUse`).
- Imports are deduplicated by `VkImage`/`VkBuffer` and keep initial layout/stage/access as the starting state.
- Transients are created via `ResourceManager` and auto‑destroyed at end of frame using the frame deletion queue.
- See `src/render/graph/resources.h:11` and `src/render/graph/resources.cpp:1`.

### Synchronization and Layouts

- For each pass, `compile()` compares previous state with desired usage and, if needed, adds a pre‑pass barrier:
  - Images: `VkImageMemoryBarrier2` with stage/access/layout derived from `RGImageUsage`.
  - Buffers: `VkBufferMemoryBarrier2` with stage/access derived from `RGBufferUsage`.
- Initial state comes from the imported descriptor; if unknown, buffers default to `TOP_OF_PIPE`.
- Format/usage checks:
  - Warns if binding a depth format as color (and vice‑versa).
  - Warns if a transient resource is used with flags it wasn’t created with.

Image usage → layout/stage examples (subset):

- `SampledFragment` → `SHADER_READ_ONLY_OPTIMAL`, `FRAGMENT_SHADER`.
- `ColorAttachment` → `COLOR_ATTACHMENT_OPTIMAL`, `COLOR_ATTACHMENT_OUTPUT` (read|write).
- `DepthAttachment` → `DEPTH_ATTACHMENT_OPTIMAL`, `EARLY|LATE_FRAGMENT_TESTS`.
- `TransferDst` → `TRANSFER_DST_OPTIMAL`, `TRANSFER`.
- `Present` → `PRESENT_SRC_KHR`, `BOTTOM_OF_PIPE`.

Buffer usage → stage/access examples:

- `IndexRead` → `INDEX_INPUT`, `INDEX_READ`.
- `VertexRead` → `VERTEX_INPUT`, `VERTEX_ATTRIBUTE_READ`.
- `UniformRead` → `ALL_GRAPHICS|COMPUTE`, `UNIFORM_READ`.
- `StorageReadWrite` → `COMPUTE|FRAGMENT`, `SHADER_STORAGE_READ|WRITE`.

### Built‑In Pass Wiring (Current)

- Resource uploads (if any) → Background (compute) → Geometry (G‑Buffer) → Lighting (deferred) → Transparent → CopyToSwapchain → ImGui → PreparePresent.
- See registrations: `src/core/vk_engine.cpp:321`–`src/core/vk_engine.cpp:352`.

### Notes & Limits

- No aliasing or transient pooling yet; images created via `create_*` are released end‑of‑frame.
- Graph scheduling uses a topological order by data dependency; it does not parallelize across queues.
- Load/store control for attachments is minimal (`clearOnLoad`, `store` on `RGAttachmentInfo`).
- Render area is the min of all declared attachment extents and `EngineContext::drawExtent`.

### Debugging

- Each pass is wrapped with a debug label (`RG: <name>`).
- Compile prints warnings for suspicious usages or format mismatches.
