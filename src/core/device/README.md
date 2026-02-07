# Device

> Vulkan device initialization, VMA-backed resource allocation, swapchain management, and image utility functions.

## Purpose

Provides the lowest-level GPU abstractions for the engine: instance/device creation
(with optional ray tracing extensions), memory allocation via VMA, buffer/image
creation and upload (immediate or deferred), swapchain lifecycle, and common image
operations (layout transitions, blits, mipmap generation).

## Directory Layout

```
device/
├── device.h / device.cpp       — Vulkan instance, device, VMA allocator setup
├── resource.h / resource.cpp   — Buffer/image creation, staging uploads, RenderGraph upload pass
├── swapchain.h / swapchain.cpp — Swapchain + per-frame render targets (HDR draw, depth, GBuffer)
└── images.h / images.cpp       — Image utility functions (transitions, blits, mipmaps, letterboxing)
```

## Key Types

| Type | Role |
|------|------|
| `DeviceManager` | Vulkan instance/device/GPU selection, VMA allocator, graphics queue, ray tracing capability query |
| `ResourceManager` | VMA-backed buffer/image creation, staging uploads (immediate or deferred), RenderGraph upload pass |
| `SwapchainManager` | Swapchain create/resize/destroy, per-frame render targets (draw, depth, GBuffer, ID buffer) |
| `vkutil` (namespace) | Image layout transitions, image blits (with letterboxing), mipmap generation |
| `AllocatedBuffer` | VMA-backed buffer handle (defined in `core/types.h`) |
| `AllocatedImage` | VMA-backed image + view handle (defined in `core/types.h`) |
| `GPUMeshBuffers` | Vertex + index buffers with device addresses for BDA and acceleration structure builds |

## Lifecycle

```
DeviceManager::init_vulkan(window)
  ├─ creates VkInstance (Vulkan 1.3, validation layers in Debug)
  ├─ creates VkSurfaceKHR via SDL
  ├─ selects physical device (dynamic rendering, synchronization2, descriptor indexing)
  ├─ queries and optionally enables ray tracing extensions (VK_KHR_ray_query, VK_KHR_acceleration_structure)
  ├─ creates VkDevice
  └─ creates VmaAllocator (with buffer device address)

ResourceManager::init(DeviceManager*)
  ├─ creates immediate-submit command pool, command buffer, and fence
  └─ ready to create buffers/images and queue uploads

SwapchainManager::init(DeviceManager*, ResourceManager*)
  └─ init_swapchain()
       ├─ create_swapchain(width, height)  — VkSwapchainKHR via vkb::SwapchainBuilder
       └─ resize_render_targets(renderExtent)
            ├─ HDR draw image (R16G16B16A16_SFLOAT)
            ├─ depth image (D32_SFLOAT)
            ├─ GBuffer: position (R32G32B32A32_SFLOAT), normal (R16G16B16A16_SFLOAT),
            │           albedo (R8G8B8A8_UNORM), extra (R16G16B16A16_SFLOAT)
            └─ ID buffer (R32_UINT)

── per frame ──
ResourceManager::register_upload_pass(graph, frame)
  └─ batches all pending buffer/image uploads into a single RenderGraph Transfer pass

── shutdown ──
SwapchainManager::cleanup()    → destroys render targets and swapchain
ResourceManager::cleanup()     → clears pending uploads, destroys command pool/fence
DeviceManager::cleanup()       → destroys VMA allocator, surface, device, instance
```

## Usage

### Buffer / Image Creation

```cpp
// GPU-only buffer
AllocatedBuffer buf = ctx->resources->create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                     VMA_MEMORY_USAGE_GPU_ONLY);

// Image with auto mipmap generation
AllocatedImage img = ctx->resources->create_image(pixelData, extent, VK_FORMAT_R8G8B8A8_SRGB,
                                                   VK_IMAGE_USAGE_SAMPLED_BIT, /*mipmapped=*/true);

// Compressed (KTX2/BCn) image with pre-computed mip levels
AllocatedImage tex = ctx->resources->create_image_compressed(bytes, byteSize, fmt, mipLevels);
```

### Mesh Upload

```cpp
GPUMeshBuffers mesh = ctx->resources->uploadMesh(indices, vertices);
// mesh.vertexBufferAddress available for BDA / acceleration structure builds
```

### Immediate Submit

```cpp
ctx->resources->immediate_submit([&](VkCommandBuffer cmd) {
    vkutil::transition_image(cmd, image, VK_IMAGE_LAYOUT_UNDEFINED,
                             VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
});
```

### Deferred Uploads (Render Graph)

```cpp
ctx->resources->set_deferred_uploads(true);
// ... create_image / uploadMesh calls queue staging work ...
ctx->resources->register_upload_pass(graph, frame);  // batched Transfer pass
```

## Integration

`DeviceManager`, `ResourceManager`, and `SwapchainManager` are owned by `VulkanEngine`
and exposed via `EngineContext`. All other engine subsystems access device resources
through `EngineContext` pointers rather than directly.

The `vkutil` image helpers are used throughout the render pipeline — by render passes
for layout transitions, by `SwapchainManager` for final present blits, and by
`ResourceManager` for mipmap generation during uploads.

## Related Docs

- [docs/ResourceManager.md](../../../docs/ResourceManager.md) — detailed upload system documentation
- [docs/FrameResources.md](../../../docs/FrameResources.md) — per-frame lifetime and deletion queues
- [docs/RenderGraph.md](../../../docs/RenderGraph.md) — DAG-based pass scheduling and barrier insertion
- [docs/EngineContext.md](../../../docs/EngineContext.md) — central dependency injection container
