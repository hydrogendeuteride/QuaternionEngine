## Resource Manager: Buffers, Images, Uploads, and Lifetime

Central allocator and uploader built on VMA. Provides creation helpers, an immediate submit path, and a deferred upload queue that is converted into a Render Graph transfer pass each frame.

### Responsibilities

- Create/destroy GPU buffers and images (with mapped memory for CPU‑to‑GPU when requested).
- Stage and upload mesh/texture data either immediately or via a per‑frame deferred path.
- Integrate with `FrameResources` deletion queues to match lifetimes to the frame.
- Expose a Render Graph pass that batches all pending uploads.

### Key APIs (src/core/frame/resource.h)

- Creation
  - `AllocatedBuffer create_buffer(size, usage, memUsage)`
  - `AllocatedImage create_image(extent3D, format, usage[, mipmapped])`
  - `AllocatedImage create_image(data, extent3D, format, usage[, mipmapped])`
  - Destroy with `destroy_buffer`, `destroy_image`.

- Uploads
  - Deferred mode toggle: `set_deferred_uploads(bool)`; when true, mesh/texture uploads enqueue staging work.
  - Query queues: `pending_buffer_uploads()`, `pending_image_uploads()`; clear via `clear_pending_uploads()`.
  - Immediate path: `process_queued_uploads_immediate()` or `immediate_submit(lambda)` for custom commands.
  - Render Graph integration: `register_upload_pass(RenderGraph&, FrameResources&)` builds a single `Transfer` pass that:
    - Imports staging buffers and destination resources into the graph.
    - Inserts the appropriate `TransferSrc/Dst` declarations.
    - Records `vkCmdCopyBuffer` / `vkCmdCopyBufferToImage` and optional mip generation.
    - Schedules deletion of staging buffers at end of frame.

- Mesh upload convenience
  - `GPUMeshBuffers uploadMesh(span<uint32_t> indices, span<Vertex> vertices)` — returns device buffers and device address.

### Per‑Frame Lifetime

- `FrameResources::_deletionQueue` owns per‑frame cleanups for transient buffers/images created during rendering passes.
- The upload pass registers cleanups for staging buffers on the frame queue.

### Render Graph Interaction

- `register_upload_pass` is called during frame build before other passes (see `src/core/engine.cpp:315`).
- It uses graph `import_buffer` / `import_image` to deduplicate external resources and attach initial stage/layout.
- Barriers and final layouts for uploaded images are handled in the pass recording (`generate_mipmaps` path transitions to `SHADER_READ_ONLY_OPTIMAL`).

### Guidelines

- Prefer deferred uploads (`set_deferred_uploads(true)`) for frame‑coherent synchronization under the Render Graph.
- For tooling and one‑off setup, use `immediate_submit(lambda)` to avoid per‑frame queuing.
- When creating transient images/buffers used only inside a pass, prefer the Render Graph’s `create_*` so destruction is automatic at frame end.

### Known Issue (transition after mip generation)

- In the Render Graph upload pass, the mipmap path should leave the image in `VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL` after `vkutil::generate_mipmaps(...)`. If you see an extra transition back to `TRANSFER_DST_OPTIMAL` after mip generation, remove it. The helper already transitions to the final sampled layout.

