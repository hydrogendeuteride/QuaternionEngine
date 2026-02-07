# Descriptor

> Low-level utilities for Vulkan descriptor set layout creation, descriptor writing, and pool management.

## Purpose

Provides reusable building blocks for the entire descriptor pipeline — from declaring
layouts, to allocating sets from pools, to writing resource bindings (buffers, images,
acceleration structures) into those sets. Also includes a central `DescriptorManager`
that owns engine-wide shared layouts.

## Directory Layout

```
descriptor/
├── descriptors.h    — DescriptorLayoutBuilder, DescriptorWriter, DescriptorAllocator, DescriptorAllocatorGrowable
├── descriptors.cpp  — implementation of all descriptor utilities
├── manager.h        — DescriptorManager class (shared layout ownership)
└── manager.cpp      — implementation
```

## Key Types

| Type | Role |
|------|------|
| `DescriptorLayoutBuilder` | Fluent builder for `VkDescriptorSetLayout` — accumulates bindings, then calls `build()` |
| `DescriptorWriter` | Batches `VkWriteDescriptorSet` entries (image, buffer, acceleration structure) and flushes them via `update_set()` |
| `DescriptorAllocator` | Simple fixed-size pool wrapper — init, allocate, reset, destroy |
| `DescriptorAllocatorGrowable` | Auto-growing pool allocator — creates new pools on demand (1.5x growth, capped at 4092 sets) |
| `DescriptorManager` | Owns engine-wide shared layouts (`gpuSceneDataLayout`, `singleImageLayout`) |

## Lifecycle

```
DescriptorManager::init(DeviceManager*)
  ├─ builds _singleImageDescriptorLayout   (COMBINED_IMAGE_SAMPLER, fragment stage)
  └─ builds _gpuSceneDataDescriptorLayout  (UNIFORM_BUFFER + optional ACCELERATION_STRUCTURE_KHR)

DescriptorLayoutBuilder
  add_binding(binding, type, count)  ← accumulate bindings
  build(device, shaderStages)        ← create VkDescriptorSetLayout

DescriptorAllocator
  init_pool(device, maxSets, poolRatios)  ← create single pool
  allocate(device, layout)                ← allocate one set
  clear_descriptors(device)               ← reset pool
  destroy_pool(device)                    ← destroy pool

DescriptorAllocatorGrowable
  init(device, initialSets, poolRatios)   ← create first pool, store ratios
  allocate(device, layout, pNext?)        ← allocate from ready pool; on OOM, retire full pool and create new
  clear_pools(device)                     ← reset all pools, move full → ready
  destroy_pools(device)                   ← destroy all pools

DescriptorWriter
  write_image(binding, imageView, sampler, layout, type)
  write_buffer(binding, buffer, size, offset, type)
  write_acceleration_structure(binding, as)
  update_set(device, set)  ← flush all pending writes
  clear()                  ← reset for reuse

DescriptorManager::cleanup()
  └─ destroys shared layouts
```

## Usage

### Building a layout

```cpp
DescriptorLayoutBuilder builder;
builder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
VkDescriptorSetLayout layout = builder.build(device, VK_SHADER_STAGE_FRAGMENT_BIT);
```

### Allocating and writing a descriptor set

```cpp
// Per-frame growable allocator (owned by FrameResources)
VkDescriptorSet set = frameDescriptors.allocate(device, layout);

DescriptorWriter writer;
writer.write_buffer(0, sceneBuffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
writer.write_image(1, texView, sampler, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                   VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
writer.update_set(device, set);
```

### Accessing shared layouts

```cpp
VkDescriptorSetLayout sceneLayout = ctx->descriptorLayouts->gpuSceneDataLayout();
VkDescriptorSetLayout imageLayout = ctx->descriptorLayouts->singleImageLayout();
```

## Integration

`DescriptorManager` is owned by `VulkanEngine` and exposed via `EngineContext::descriptorLayouts`.
Render passes use the shared layouts for pipeline layout creation and per-frame set allocation.

`DescriptorAllocatorGrowable` is embedded in `FrameResources` as `_frameDescriptors`,
providing transient per-frame descriptor set allocation that is reset each frame.
It is also used by `ComputeManager`, `MaterialSystem`, `GLTFLoader`, and `PlanetSystem`
for their own long-lived or per-frame descriptor needs.

`DescriptorWriter` is used throughout render passes, material setup, and compute dispatch
to batch-write resource bindings before flushing to the driver.

All pools are created with `VK_DESCRIPTOR_POOL_CREATE_UPDATE_AFTER_BIND_BIT` to support
safe cross-frame descriptor rewrites.

## Related Docs

- [docs/Descriptors.md](../../../docs/Descriptors.md) — high-level descriptor system documentation
- [docs/FrameResources.md](../../../docs/FrameResources.md) — per-frame resource management
