## Render Graph: Per‑Frame Scheduling, Barriers, and Dynamic Rendering

Lightweight render graph that builds a per‑frame DAG from pass declarations, computes the necessary resource barriers/layout transitions, and records passes with dynamic rendering when attachments are declared.

### Why

- Centralize synchronization and image layout transitions across passes.
- Make passes declarative: author declares reads/writes; the graph inserts barriers and begins/ends rendering.
- Keep existing pass classes (`IRenderPass`) while migrating execution to the graph.
- Provide runtime profiling and debugging capabilities for pass execution.

### High‑Level Flow

- Engine creates the graph each frame and imports swapchain/G‑Buffer images: `src/core/engine.cpp`.
- Each pass registers its work by calling `register_graph(graph, ...)` and declaring resources via a builder.
- The graph appends a present chain (copy HDR `drawImage` → swapchain, then transition to `PRESENT`), optionally inserting ImGui before present.
- `compile()` topologically sorts passes by data dependencies (read/write hazards: RAW/WAW/WAR) and computes per‑pass barriers using `VkDependencyInfo` with `Vk*MemoryBarrier2`.
- `execute(cmd)` creates timestamp query pools, emits barriers, begins dynamic rendering if attachments were declared, calls the pass record lambda, ends rendering, and records GPU/CPU timings.
- `resolve_timings()` retrieves GPU timestamp results after the fence is signaled, converting them to milliseconds.

### Core API

**Lifecycle:**
- `RenderGraph::init(ctx)` — Initialize with engine context. See `src/render/graph/graph.cpp:28`.
- `RenderGraph::clear()` — Clear all passes and reset resources. See `src/render/graph/graph.cpp:34`.
- `RenderGraph::shutdown()` — Destroy GPU resources (query pools) before device shutdown. See `src/render/graph/graph.cpp:40`.

**Pass Registration:**
- `RenderGraph::add_pass(name, RGPassType type, BuildCallback build, RecordCallback record)`
  - Declare image/buffer accesses and attachments inside `build` using `RGPassBuilder`.
  - Do your actual rendering/copies in `record` using resolved Vulkan objects from `RGPassResources`.
  - See: `src/render/graph/graph.h:42`, `src/render/graph/graph.cpp:91`.
- Legacy form: `add_pass(name, type, record)` for passes with no resource declarations. See `src/render/graph/graph.cpp:117`.

**Resource Creation:**
- `import_image(desc)` / `import_buffer(desc)` — Import externally owned resources (deduplicated by VkImage/VkBuffer handle).
- `create_image(desc)` / `create_buffer(desc)` — Create transient resources (destroyed at end of frame via deletion queue).
- `create_depth_image(name, extent, format=D32_SFLOAT)` — Convenience helper for depth-only images with depth attachment + sampled usage. See `src/render/graph/graph.cpp:67`.

**Compilation and Execution:**
- `RenderGraph::compile()` — Build topological ordering (Kahn's algorithm) and per‑pass `VkImageMemoryBarrier2` / `VkBufferMemoryBarrier2` lists. Returns false on error. See `src/render/graph/graph.cpp:123`.
- `RenderGraph::execute(cmd)` — Creates timestamp query pool, emits barriers via `vkCmdPipelineBarrier2`, begins dynamic rendering if attachments exist, invokes record callbacks, ends rendering, and writes GPU timestamps. See `src/render/graph/graph.cpp:874`.
- `RenderGraph::resolve_timings()` — Fetch GPU timestamp results after fence wait and convert to milliseconds. Must be called before next `execute()`. See `src/render/graph/graph.cpp:1314`.

**Import Helpers:**
- `import_draw_image()`, `import_depth_image()`, `import_gbuffer_position()`, `import_gbuffer_normal()`, `import_gbuffer_albedo()`, `import_gbuffer_extra()`, `import_id_buffer()`, `import_swapchain_image(index)` — Convenience wrappers for engine-owned images. See `src/render/graph/graph.cpp:1147–1312`.

**Present Chain:**
- `add_present_chain(draw, swapchain, appendExtra)` — Inserts `PresentLetterbox` pass (blit draw→swapchain with letterboxing) and `PreparePresent` pass (layout transition to `PRESENT_SRC_KHR`). Optional `appendExtra` callback injects passes (e.g., ImGui) in between. See `src/render/graph/graph.cpp:1043`.

**Debug and Profiling:**
- `pass_count()`, `pass_name(i)`, `pass_enabled(i)`, `set_pass_enabled(i, enabled)` — Runtime pass enable/disable. See `src/render/graph/graph.h:105–108`.
- `debug_get_passes(out)` — Retrieve pass metadata including GPU/CPU timings, resource access counts, attachment info. See `src/render/graph/graph.cpp:1163`.
- `debug_get_images(out)` — Retrieve image metadata (imported/transient, format, extent, usage, lifetime). See `src/render/graph/graph.cpp:1186`.
- `debug_get_buffers(out)` — Retrieve buffer metadata. See `src/render/graph/graph.cpp:1207`.

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

Passed to the `BuildCallback` to declare resource accesses and attachments. See `src/render/graph/builder.h:40`.

**Image Access:**
- `read(RGImageHandle, RGImageUsage)` — Declare sampled/read usage (e.g., `SampledFragment`, `TransferSrc`). See `src/render/graph/builder.cpp:20`.
- `write(RGImageHandle, RGImageUsage)` — Declare write usage (e.g., `ComputeWrite`, `TransferDst`). See `src/render/graph/builder.cpp:25`.
- `write_color(RGImageHandle, bool clearOnLoad=false, VkClearValue clear={})` — Declare color attachment with optional clear. Sets usage to `ColorAttachment` and `store=true` by default. See `src/render/graph/builder.cpp:30`.
- `write_depth(RGImageHandle, bool clearOnLoad=false, VkClearValue clear={})` — Declare depth attachment with optional clear. See `src/render/graph/builder.cpp:40`.

**Buffer Access:**
- `read_buffer(RGBufferHandle, RGBufferUsage)` — Declare buffer read (e.g., `VertexRead`, `IndexRead`, `UniformRead`, `StorageRead`). See `src/render/graph/builder.cpp:50`.
- `write_buffer(RGBufferHandle, RGBufferUsage)` — Declare buffer write (e.g., `StorageReadWrite`, `TransferDst`). See `src/render/graph/builder.cpp:55`.
- Convenience overloads: `read_buffer(VkBuffer, RGBufferUsage, size, name)` and `write_buffer(VkBuffer, ...)` automatically import and deduplicate by raw `VkBuffer` handle. See `src/render/graph/builder.cpp:60,70`.

**Resource Resolution (`RGPassResources`):**
Used inside the `RecordCallback` to fetch resolved Vulkan objects. See `src/render/graph/builder.h:22`.
- `image(RGImageHandle)` → `VkImage`
- `image_view(RGImageHandle)` → `VkImageView`
- `buffer(RGBufferHandle)` → `VkBuffer`

### Resource Model (`RGResourceRegistry`)

Manages both imported (externally owned) and transient (graph-owned) resources. See `src/render/graph/resources.h:52`.

**Imported Resources:**
- Deduplicated by raw Vulkan handle (`VkImage`/`VkBuffer`) using hash maps (`_imageLookup`/`_bufferLookup`). See `src/render/graph/resources.cpp`.
- Initial layout/stage/access preserved from `RGImportedImageDesc`/`RGImportedBufferDesc`.
- Ownership remains external; graph does not destroy these resources.

**Transient Resources:**
- Created via `ResourceManager` (`AllocatedImage`/`AllocatedBuffer`) with VMA allocations. See `src/render/graph/resources.cpp`.
- Automatically destroyed at end of frame via frame deletion queue.
- Usage flags must cover all declared usages (validated during `compile()`).

**Lifetime Tracking:**
- `firstUse` and `lastUse` indices computed during `compile()` (see `src/render/graph/graph.cpp:854–869`).
- Used for debug visualization and future aliasing/pooling optimizations.

**Records (`RGImageRecord`/`RGBufferRecord`):**
Unified representation storing `VkImage`/`VkBuffer`, `VkImageView`, format, extent, initial state, and allocation info. See `src/render/graph/resources.h:11,34`.

### Synchronization and Layouts

**Barrier Generation (see `src/render/graph/graph.cpp:232–851`):**

For each enabled pass, `compile()` tracks per-resource state (`ImageState`/`BufferState`) and inserts barriers when hazards are detected:

**Image Barriers (`VkImageMemoryBarrier2`):**
- Triggered by: layout change, prior write before read/write (RAW/WAW), prior reads before write (WAR).
- Stage/access/layout derived from `RGImageUsage` via `usage_info_image()` (see `src/render/graph/graph.cpp:313–365`).
- Aspect determined by usage and format (depth formats get `DEPTH_BIT`, others `COLOR_BIT`).
- Initial state from `RGImportedImageDesc::currentLayout/currentStage/currentAccess`; if unknown (layout ≠ UNDEFINED but stage=NONE), conservatively assumes `ALL_COMMANDS + MEMORY_READ|WRITE`.

**Buffer Barriers (`VkBufferMemoryBarrier2`):**
- Triggered by: prior write before read/write, prior reads before write.
- Stage/access derived from `RGBufferUsage` via `usage_info_buffer()` (see `src/render/graph/graph.cpp:367–411`).
- Size: exact size for transients, `VK_WHOLE_SIZE` for imports (to avoid validation errors).

**Usage Priority and Conflict Resolution:**
When a pass declares multiple conflicting usages for the same resource (e.g., both `SampledFragment` and `ColorAttachment`), the graph selects the highest-priority usage for layout determination (see `image_usage_priority()` at `src/render/graph/graph.cpp:499`). Stages and access masks are unioned. Warns if layout mismatch detected.

**Image Usage → Layout/Stage/Access Mapping:**
See `usage_info_image()` at `src/render/graph/graph.cpp:313`.

| RGImageUsage | Layout | Stage | Access |
|---|---|---|---|
| `SampledFragment` | `SHADER_READ_ONLY_OPTIMAL` | `FRAGMENT_SHADER` | `SHADER_SAMPLED_READ` |
| `SampledCompute` | `SHADER_READ_ONLY_OPTIMAL` | `COMPUTE_SHADER` | `SHADER_SAMPLED_READ` |
| `TransferSrc` | `TRANSFER_SRC_OPTIMAL` | `TRANSFER` | `TRANSFER_READ` |
| `TransferDst` | `TRANSFER_DST_OPTIMAL` | `TRANSFER` | `TRANSFER_WRITE` |
| `ColorAttachment` | `COLOR_ATTACHMENT_OPTIMAL` | `COLOR_ATTACHMENT_OUTPUT` | `COLOR_ATTACHMENT_READ\|WRITE` |
| `DepthAttachment` | `DEPTH_ATTACHMENT_OPTIMAL` | `EARLY_FRAGMENT_TESTS\|LATE_FRAGMENT_TESTS` | `DEPTH_STENCIL_ATTACHMENT_READ\|WRITE` |
| `ComputeWrite` | `GENERAL` | `COMPUTE_SHADER` | `SHADER_STORAGE_READ\|WRITE` |
| `Present` | `PRESENT_SRC_KHR` | `BOTTOM_OF_PIPE` | `MEMORY_READ` |

**Buffer Usage → Stage/Access Mapping:**
See `usage_info_buffer()` at `src/render/graph/graph.cpp:367`.

| RGBufferUsage | Stage | Access |
|---|---|---|
| `TransferSrc` | `TRANSFER` | `TRANSFER_READ` |
| `TransferDst` | `TRANSFER` | `TRANSFER_WRITE` |
| `VertexRead` | `VERTEX_INPUT` | `VERTEX_ATTRIBUTE_READ` |
| `IndexRead` | `INDEX_INPUT` | `INDEX_READ` |
| `UniformRead` | `ALL_GRAPHICS\|COMPUTE_SHADER` | `UNIFORM_READ` |
| `StorageRead` | `COMPUTE_SHADER\|ALL_GRAPHICS` | `SHADER_STORAGE_READ` |
| `StorageReadWrite` | `COMPUTE_SHADER\|ALL_GRAPHICS` | `SHADER_STORAGE_READ\|WRITE` |
| `IndirectArgs` | `DRAW_INDIRECT` | `INDIRECT_COMMAND_READ` |

**Validation Warnings:**
- Depth-format image declared as color attachment (or vice versa). See `src/render/graph/graph.cpp:645–657`.
- Transient resource used without required usage flags. See `src/render/graph/graph.cpp:659–667` (images), `818–826` (buffers).
- Multiple conflicting layouts in single pass. See `src/render/graph/graph.cpp:536–543`.

### Built‑In Pass Wiring (Current)

- Resource uploads (if any) → Background (compute) → Geometry (G‑Buffer) → Lighting (deferred) → SSR → Tonemap+Bloom → FXAA → Transparent → CopyToSwapchain → ImGui → PreparePresent.
- See registrations in `src/core/engine.cpp`.

### Topological Sorting and Scheduling

**Dependency Graph Construction (see `src/render/graph/graph.cpp:127–231`):**
- Reads/writes create directed edges: `writer → reader` (RAW), `writer → writer` (WAW), `reader → writer` (WAR).
- Disabled passes are skipped during edge construction but remain in the pass list.
- Kahn's algorithm produces a linear execution order respecting all dependencies.
- If cycle detected (topological sort fails), falls back to insertion order but still computes barriers.

**Execution Order:**
Passes execute in sorted order (or insertion order if cycle). Only enabled passes run; disabled passes are skipped during `execute()`. See `src/render/graph/graph.cpp:895`.

### Dynamic Rendering Setup

**Render Area Calculation (see `src/render/graph/graph.cpp:936–1000`):**
- Chooses min extent across all color/depth attachments.
- Falls back to `EngineContext::drawExtent` if no attachments.
- Warns if color attachments have mismatched extents.

**Attachment Construction:**
- Color attachments: `VkRenderingAttachmentInfo` with `clearOnLoad` → `LOAD_OP_CLEAR` / `LOAD_OP_LOAD`, `store` → `STORE_OP_STORE` / `STORE_OP_DONT_CARE`.
- Depth attachment: similar logic; `clearValue.depthStencil` used if `clearOnLoad=true`.
- Layout forced to `COLOR_ATTACHMENT_OPTIMAL` or `DEPTH_ATTACHMENT_OPTIMAL`.

See `src/render/graph/graph.cpp:927–1012`.

### Profiling and Timing

**GPU Timing (Timestamps):**
- Per-frame `VkQueryPool` with 2 queries per pass (begin/end). Created in `execute()`, destroyed in `resolve_timings()` or next `execute()`.
- `vkCmdWriteTimestamp2()` at `ALL_COMMANDS_BIT` stage before/after pass recording (see `src/render/graph/graph.cpp:919–923`, `1028–1032`).
- `resolve_timings()` fetches results with `VK_QUERY_RESULT_WAIT_BIT`, converts ticks to milliseconds using `timestampPeriod`. See `src/render/graph/graph.cpp:1314–1355`.

**CPU Timing:**
- `std::chrono::high_resolution_clock` measures command recording duration (`cpuStart`/`cpuEnd`). See `src/render/graph/graph.cpp:924`, `1026`.
- Stored in `_lastCpuMillis` vector; accessible via `debug_get_passes()`.

**Debug Structures:**
- `RGDebugPassInfo`: name, type, enabled, resource counts, attachment info, `gpuMillis`, `cpuMillis`. See `src/render/graph/graph.h:66`.
- `RGDebugImageInfo`: id, name, imported, format, extent, usage, lifetime. See `src/render/graph/graph.h:83`.
- `RGDebugBufferInfo`: id, name, imported, size, usage, lifetime. See `src/render/graph/graph.h:94`.

### Notes & Limits

- **No aliasing or transient pooling**: Transient images/buffers created via `create_*` are released end‑of‑frame via frame deletion queue.
- **Single-queue execution**: Topological order is linear; no multi-queue parallelization.
- **Minimal load/store control**: Only `clearOnLoad` and `store` flags on `RGAttachmentInfo`; no resolve or stencil control.
- **No mid-pass barriers**: Conflicting usages within a single pass cannot be synchronized (warns but proceeds with unioned stages/access).
- **No automatic resource aliasing**: Future work could reuse transient allocations based on lifetime non-overlap.

### Debugging

- **Per-pass debug labels**: `vkdebug::cmd_begin_label(cmd, "RG: <name>")` wraps each pass (see `src/render/graph/graph.cpp:903–906`, `1035–1038`).
- **Compile-time validation warnings**: Printed via `fmt::println` for format mismatches, missing usage flags, layout conflicts.
- **Runtime introspection**: Use `debug_get_*` APIs to export pass/image/buffer metadata for visualization/debugging tools.
