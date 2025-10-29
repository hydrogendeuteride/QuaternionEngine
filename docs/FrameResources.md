## Frame Resources: Per-Frame Command, Sync, and Transient Descriptors

Per-frame struct that owns the command buffer, semaphores/fence, a transient descriptor allocator, and a small deletion queue. Frames are indexed by `FRAME_OVERLAP` (currently 2) and rotated by `_frameNumber` in `VulkanEngine`.

- File: `src/core/frame_resources.h/.cpp`

### Responsibilities
- Command recording: `_mainCommandBuffer` allocated from a per-frame `_commandPool` with RESET flag.
- Synchronization: `_swapchainSemaphore` (image acquired), `_renderSemaphore` (render finished), `_renderFence` (CPU wait per frame).
- Transient descriptors: `_frameDescriptors` is a `DescriptorAllocatorGrowable` cleared every frame via `clear_pools()`.
- Lifetime: `_deletionQueue` holds lambdas for transient GPU objects created during the frame (buffers/images) and is flushed at the start of the next frame.

### Frame Flow (engine side)
- Start of frame:
  - Wait on `_renderFence` (previous GPU work for this frame index), flush `_deletionQueue`, and clear `_frameDescriptors` pools.
  - Acquire swapchain image signaling `_swapchainSemaphore`.
  - Reset `_renderFence` and `_mainCommandBuffer`; begin recording.
  - Publish `currentFrame` pointer and `drawExtent` on `EngineContext`.
- Graph build and execute:
  - Render Graph is cleared and rebuilt; `ResourceManager::register_upload_pass(...)` is added first if there are pending uploads.
  - Passes record using the published `currentFrame` to allocate transient descriptor sets and to enqueue per-frame cleanups.
- Submit and present:
  - Submit `cmd` with wait on `_swapchainSemaphore` and signal `_renderSemaphore`, fence `_renderFence`.
  - Present waits on `_renderSemaphore`.

### Do/Don’t
- Do use `currentFrame->_frameDescriptors` for descriptor sets that live only for this frame.
- Do push transient resource destruction into `currentFrame->_deletionQueue`.
- Don’t stash per-frame descriptor sets across frames — they are reset on `clear_pools()`.

### Extending
- If a pass needs additional short-lived command buffers, allocate them from `_commandPool` and reset per frame.
- If you add frames-in-flight, update `FRAME_OVERLAP` and verify fences/semaphores and swapchain image acquisition logic.

