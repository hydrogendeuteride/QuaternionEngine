# Compute

> Lightweight manager for GPU compute pipelines, descriptor binding, and dispatch.

## Purpose

Provides a self-contained way to register compute shaders, bind resources
(storage buffers, storage images, sampled images, uniform buffers), and dispatch
work — either on an existing command buffer or via immediate submit. Also supports
long-lived **compute instances** with persistent bindings and owned resources.

## Directory Layout

```
compute/
├── vk_compute.h    — all types and ComputeManager / ComputePipeline classes
└── vk_compute.cpp  — implementation
```

## Key Types

| Type | Role |
|------|------|
| `ComputeManager` | Central entry point — pipeline registry, instance management, dispatch |
| `ComputePipeline` | RAII wrapper around `VkPipeline` + layout + descriptor layout |
| `ComputeBinding` | Union-based descriptor binding (buffer / sampled image / storage image) |
| `ComputePipelineCreateInfo` | Shader path, descriptor types, push constants, specialization |
| `ComputeDispatchInfo` | Group counts, bindings, push constants, optional barriers |
| `ComputeInstance` | Named long-lived binding set with optional owned images/buffers |

## Lifecycle

```
init(EngineContext*)
  └─ creates growable descriptor allocator

registerPipeline(name, createInfo)
  └─ loads SPIR-V, builds descriptor layout + pipeline layout + VkPipeline

dispatch(cmd, name, dispatchInfo)          ← one-shot, allocates transient descriptor set
dispatchImmediate(name, dispatchInfo)       ← immediate submit (blocking)

createInstance(instanceName, pipelineName)  ← persistent binding set
  ├─ setInstance{StorageImage,SampledImage,Buffer}(...)
  ├─ createAndBind{StorageImage,StorageBuffer}(...)  ← instance owns the resource
  └─ dispatchInstance(cmd, instanceName, dispatchInfo)

cleanup()
  └─ destroys all pipelines, instances, owned resources, descriptor pools
```

## Usage

### One-shot dispatch

```cpp
ComputePipelineCreateInfo ci;
ci.shaderPath         = "../shaders/my_effect.comp.spv";
ci.descriptorTypes    = { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE };
ci.pushConstantSize   = sizeof(MyPush);
ctx->compute->registerPipeline("my_effect", ci);

auto info = ComputeManager::createDispatch2D(width, height);
info.bindings.push_back(ComputeBinding::storeImage(0, imageView));
info.pushConstants     = &pushData;
info.pushConstantSize  = sizeof(pushData);
ctx->compute->dispatch(cmd, "my_effect", info);
```

### Persistent instance

```cpp
ctx->compute->createInstance("voxel_sim", "voxel_pipeline");
ctx->compute->createAndBindStorageImage("voxel_sim", 0, extent, VK_FORMAT_R16G16B16A16_SFLOAT);
ctx->compute->createAndBindStorageBuffer("voxel_sim", 1, bufferSize);
ctx->compute->dispatchInstance(cmd, "voxel_sim", dispatchInfo);
// owned resources are freed on destroyInstance / cleanup
```

## Integration

`ComputeManager` is owned by `VulkanEngine` and exposed via `EngineContext::compute`.
`PipelineManager` delegates its compute-related methods to `ComputeManager` internally,
so render passes typically access compute through `PipelineManager` rather than directly.

Built-in utility dispatches: `clearImage()`, `copyBuffer()` (lazily register their
own internal pipelines on first use).

## Related Docs

- [docs/Compute.md](../../docs/Compute.md) — detailed compute system documentation
