## Descriptors: Builders, Allocators, and Layouts

Utilities to define descriptor layouts, write descriptor sets, and efficiently allocate them per-frame or globally.

### Overview

- Layouts: `DescriptorLayoutBuilder` assembles `VkDescriptorSetLayout` with staged bindings.
- Writing: `DescriptorWriter` collects buffer/image writes and updates a set in one call.
- Pools: `DescriptorAllocatorGrowable` manages a growable pool-of-pools for resilient allocations; `FrameResources` keeps one per overlapping frame.
- Common layouts: `DescriptorManager` pre-creates reusable layouts such as `gpuSceneDataLayout()` and `singleImageLayout()`.

### Quick Start — Transient Per-Frame Set

```c++
// 1) Create/update a small uniform buffer for the frame
AllocatedBuffer ubuf = context->getResources()->create_buffer(
    sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
context->currentFrame->_deletionQueue.push_function([=, &ctx=*context]{ ctx.getResources()->destroy_buffer(ubuf); });
VmaAllocationInfo ai{}; vmaGetAllocationInfo(context->getDevice()->allocator(), ubuf.allocation, &ai);
*static_cast<GPUSceneData*>(ai.pMappedData) = context->getSceneData();

// 2) Allocate a set from the frame allocator using a common layout
VkDescriptorSet set = context->currentFrame->_frameDescriptors.allocate(
    context->getDevice()->device(), context->getDescriptorLayouts()->gpuSceneDataLayout());

// 3) Write the buffer binding
DescriptorWriter writer;
writer.write_buffer(0, ubuf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
writer.update_set(context->getDevice()->device(), set);
```

### Defining a Custom Layout

```c++
DescriptorLayoutBuilder lb;
lb.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
lb.add_binding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
VkDescriptorSetLayout myLayout = lb.build(device, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT);
// ... remember to vkDestroyDescriptorSetLayout(device, myLayout, nullptr) in cleanup
```

### Global Growable Allocator

For long-lived sets (e.g., materials, compute instances), use `EngineContext::descriptors` which wraps a growable allocator shared across modules.

```c++
VkDescriptorSet persistent = context->getDescriptors()->allocate(device, myLayout);
DescriptorWriter writer;
writer.write_image(0, albedoView, sampler, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
writer.update_set(device, persistent);
// Freeing: call `context->getDescriptors()->destroy_pools(device)` at engine shutdown; sets die with their pools
```

### Compute Integration

`ComputeManager` uses an internal `DescriptorAllocatorGrowable` and offers higher-level bindings:

- Build pipeline: specify `descriptorTypes` and `pushConstantSize` in `ComputePipelineCreateInfo`.
- Ad-hoc dispatch: fill `ComputeDispatchInfo.bindings` with `ComputeBinding::{uniformBuffer, storageBuffer, sampledImage, storeImage}`.
- Persistent instances: create via `createInstance()` and set bindings with `setInstance*()`; descriptor sets are auto-updated and reused across dispatches.

See `PipelineManager.md` for a full compute quick start using the unified API.

### API Summary

- `DescriptorLayoutBuilder`: `add_binding(binding, type)`, `build(device, stages[, pNext, flags])`, `clear()`.
- `DescriptorWriter`: `write_buffer(binding, buffer, size, offset, type)`, `write_image(binding, view, sampler, layout, type)`, `update_set(device, set)`, `clear()`.
- `DescriptorAllocatorGrowable`: `init(device, initialSets, ratios)`, `allocate(device, layout[, pNext])`, `clear_pools(device)`, `destroy_pools(device)`.
- `DescriptorManager`: `gpuSceneDataLayout()`, `singleImageLayout()`.

### Best Practices

- Use per-frame allocator (`currentFrame->_frameDescriptors`) for transient sets to avoid lifetime pitfalls.
- Keep `DescriptorLayoutBuilder` small and local; free custom layouts in your pass/module `cleanup()`.
- Tune pool ratios to match workload; see how frames are initialized in `VulkanEngine::init_frame_resources()`.
- For persistent compute resources, prefer `ComputeManager` instances over manual descriptor lifecycle.
