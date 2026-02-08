# Util

> Vulkan boilerplate helpers — struct initializers and debug labeling utilities.

## Purpose

Eliminates repetitive Vulkan struct setup by providing factory functions that
return pre-filled `Vk*CreateInfo` / `Vk*Info` structs with sensible defaults.
Also provides lightweight command buffer debug labeling via `VK_EXT_debug_utils`.

## Directory Layout

```
util/
├── initializers.h   — factory function declarations (namespace vkinit)
├── initializers.cpp — implementation
├── debug.h          — command buffer debug label helpers (namespace vkdebug)
└── debug.cpp        — implementation with lazy function pointer lookup
```

## Key Types

### `vkinit` — Struct Initializers

| Function | Returns |
|----------|---------|
| `command_pool_create_info` | `VkCommandPoolCreateInfo` |
| `command_buffer_allocate_info` | `VkCommandBufferAllocateInfo` |
| `command_buffer_begin_info` | `VkCommandBufferBeginInfo` |
| `command_buffer_submit_info` | `VkCommandBufferSubmitInfo` |
| `fence_create_info` | `VkFenceCreateInfo` |
| `semaphore_create_info` | `VkSemaphoreCreateInfo` |
| `semaphore_submit_info` | `VkSemaphoreSubmitInfo` |
| `submit_info` | `VkSubmitInfo2` |
| `present_info` | `VkPresentInfoKHR` |
| `attachment_info` | `VkRenderingAttachmentInfo` (color) |
| `depth_attachment_info` | `VkRenderingAttachmentInfo` (depth, reverse-Z clear) |
| `rendering_info` | `VkRenderingInfo` (single color attachment) |
| `rendering_info_multi` | `VkRenderingInfo` (multiple color attachments) |
| `image_subresource_range` | `VkImageSubresourceRange` (all mips/layers) |
| `descriptorset_layout_binding` | `VkDescriptorSetLayoutBinding` |
| `descriptorset_layout_create_info` | `VkDescriptorSetLayoutCreateInfo` |
| `write_descriptor_image` | `VkWriteDescriptorSet` (image) |
| `write_descriptor_buffer` | `VkWriteDescriptorSet` (buffer) |
| `buffer_info` | `VkDescriptorBufferInfo` |
| `image_create_info` | `VkImageCreateInfo` (basic / extended with mips, layers, flags) |
| `imageview_create_info` | `VkImageViewCreateInfo` (basic / extended with view type, subresource) |
| `pipeline_layout_create_info` | `VkPipelineLayoutCreateInfo` |
| `pipeline_shader_stage_create_info` | `VkPipelineShaderStageCreateInfo` |

### `vkdebug` — Debug Labels

| Function | Role |
|----------|------|
| `cmd_begin_label` | Insert a named debug region on a command buffer (with RGBA color) |
| `cmd_end_label` | Close the current debug region |

Function pointers for `VK_EXT_debug_utils` are resolved lazily via
`vkGetDeviceProcAddr` and cached per device. Calls are silently skipped
when the extension is not available.

## Usage

### Initializers

```cpp
#include <core/util/initializers.h>

auto poolInfo = vkinit::command_pool_create_info(queueFamily,
    VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);

auto imgInfo = vkinit::image_create_info(
    VK_FORMAT_R8G8B8A8_SRGB,
    VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
    VkExtent3D{width, height, 1});

auto renderInfo = vkinit::rendering_info(extent, &colorAttachment, &depthAttachment);
```

### Debug labels

```cpp
#include <core/util/debug.h>

vkdebug::cmd_begin_label(device, cmd, "Shadow Pass", 1.0f, 0.5f, 0.0f);
// ... record shadow commands ...
vkdebug::cmd_end_label(device, cmd);
```

Labels appear in tools like RenderDoc and Nsight Graphics for easier debugging.

## Integration

`vkinit` functions are used throughout the engine wherever Vulkan structs need
to be constructed — engine initialization, pipeline creation, render graph
execution, image/buffer creation, and compute dispatch.

`vkdebug` labels are inserted by the render graph (`graph.cpp`) to annotate
each pass in GPU capture tools.
