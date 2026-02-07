# Frame Resources

> Per-frame state bundle owning the command buffer, synchronization primitives, transient descriptor pool, and a deferred deletion queue.

## Purpose

Encapsulates everything the renderer needs on a per-frame basis so that
multiple frames can be in flight simultaneously. Each `FrameResources`
instance is self-contained — it holds its own command pool, fence,
semaphores, a growable descriptor allocator (cleared every frame), and a
`DeletionQueue` for transient GPU resources created during that frame.

## Directory Layout

```
frame/
├── resources.h    — FrameResources struct declaration
└── resources.cpp  — init / cleanup implementation
```

## Key Types

| Type | Role |
|------|------|
| `FrameResources` | Per-frame state: command buffer, sync primitives, transient descriptors, deletion queue |
| `DeletionQueue` | LIFO queue of `std::function<void()>` lambdas flushed in reverse order (defined in `core/types.h`) |
| `DescriptorAllocatorGrowable` | Auto-growing descriptor pool used for frame-local descriptor sets |

## Members

| Member | Vulkan Type | Role |
|--------|-------------|------|
| `_swapchainSemaphore` | `VkSemaphore` | Signaled when swapchain image is acquired |
| `_renderSemaphore` | `VkSemaphore` | Signaled when GPU rendering finishes |
| `_renderFence` | `VkFence` | CPU waits on this before reusing the frame slot |
| `_commandPool` | `VkCommandPool` | Per-frame pool with `RESET_COMMAND_BUFFER_BIT` |
| `_mainCommandBuffer` | `VkCommandBuffer` | Primary command buffer for the frame |
| `_deletionQueue` | `DeletionQueue` | Deferred destruction of transient buffers/images |
| `_frameDescriptors` | `DescriptorAllocatorGrowable` | Transient descriptor sets, cleared every frame |

## Lifecycle

```
init(DeviceManager*, framePoolSizes)
  ├─ create VkCommandPool (graphics queue family, RESET flag)
  ├─ allocate VkCommandBuffer from pool
  ├─ create VkFence (SIGNALED)
  ├─ create 2x VkSemaphore (swapchain + render)
  └─ init DescriptorAllocatorGrowable (1000 initial sets)

── per frame ──
  wait _renderFence            ← CPU blocks until previous GPU work finishes
  flush _deletionQueue         ← destroy transient resources from previous use
  clear _frameDescriptors      ← reset all descriptor pools
  reset _renderFence
  reset _mainCommandBuffer
  ... record passes ...
  submit (wait _swapchainSemaphore, signal _renderSemaphore, fence _renderFence)
  present (wait _renderSemaphore)

cleanup(DeviceManager*)
  ├─ destroy descriptor pools
  ├─ destroy VkCommandPool
  ├─ destroy VkFence
  └─ destroy 2x VkSemaphore
```

## Usage

### Accessing the current frame

```cpp
// Engine rotates frames via FRAME_OVERLAP (currently 2)
FrameResources& frame = _frames[_frameNumber % FRAME_OVERLAP];

// Also published on EngineContext each frame:
FrameResources* frame = ctx->currentFrame;
```

### Allocating transient descriptor sets

```cpp
VkDescriptorSet set = ctx->currentFrame->_frameDescriptors
    .allocate(ctx->device->device(), layout);
// No manual free needed — cleared at the start of next frame reuse
```

### Deferring resource destruction

```cpp
auto buf = resourceManager->create_buffer(...);
ctx->currentFrame->_deletionQueue.push_function([rm, buf]() {
    rm->destroy_buffer(buf);
});
// Lambda runs when this frame slot is recycled (FRAME_OVERLAP frames later)
```

## Integration

`VulkanEngine` owns a fixed-size array `_frames[FRAME_OVERLAP]` and exposes
the active slot via `get_current_frame()`. At the start of each frame the
engine sets `EngineContext::currentFrame` so that render passes, the scene
manager, and compute dispatches can access frame-local resources without
coupling to the engine directly.

Render passes use `_frameDescriptors` for per-draw descriptor sets and
`_deletionQueue` for any per-frame GPU allocations (staging buffers,
temporary images, etc.).

## Related Docs

- [docs/FrameResources.md](../../../docs/FrameResources.md) — high-level frame resource documentation
- [src/core/descriptor/README.md](../descriptor/README.md) — descriptor allocator details
- [src/core/device/README.md](../device/README.md) — resource manager and VMA allocation
