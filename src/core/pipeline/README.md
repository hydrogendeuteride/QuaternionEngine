# Pipeline

> Graphics pipeline registry with hot-reload support, compute pipeline forwarding, and centralized sampler management.

## Purpose

Provides a unified entry point for creating, retrieving, and hot-reloading
graphics pipelines by name. Also acts as a facade over `ComputeManager` so that
render passes can access both graphics and compute pipelines through a single
manager. A companion `SamplerManager` owns commonly-used `VkSampler` objects.

## Directory Layout

```
pipeline/
├── manager.h    — GraphicsPipelineCreateInfo, PipelineManager class
├── manager.cpp  — implementation (build, hot-reload worker, compute forwarding)
├── sampler.h    — SamplerManager class
└── sampler.cpp  — implementation (sampler creation / cleanup)
```

## Key Types

| Type | Role |
|------|------|
| `PipelineManager` | Central registry — register/get/unregister graphics pipelines, forward compute API, async hot-reload |
| `GraphicsPipelineCreateInfo` | Shader paths, descriptor set layouts, push constants, builder configure callback |
| `SamplerManager` | Owns a set of commonly-used `VkSampler` objects (linear, nearest, clamp, shadow) |

## Lifecycle

### PipelineManager

```
init(EngineContext*)
  └─ stores context, starts background hot-reload worker thread

registerGraphics(name, createInfo)
  └─ loads SPIR-V shaders, creates VkPipelineLayout + VkPipeline via PipelineBuilder

getGraphics(name) → pipeline, layout
getMaterialPipeline(name) → MaterialPipeline

hotReloadChanged()
  └─ checks shader file timestamps, enqueues async rebuild jobs for changed pipelines

pumpMainThread()
  └─ applies completed async rebuilds (swap old pipeline → new pipeline)

unregisterGraphics(name)
  └─ destroys pipeline + layout, removes from registry

cleanup()
  └─ stops worker thread, destroys all registered graphics pipelines
```

### SamplerManager

```
init(DeviceManager*)
  └─ creates five preset samplers

cleanup()
  └─ destroys all sampler handles
```

## Usage

### Registering a graphics pipeline

```cpp
GraphicsPipelineCreateInfo info;
info.vertexShaderPath   = "../shaders/deferred.vert.spv";
info.fragmentShaderPath = "../shaders/deferred.frag.spv";
info.setLayouts         = { sceneLayout, materialLayout };
info.pushConstants      = { pushRange };
info.configure          = [](PipelineBuilder& b) {
    b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
    b.set_polygon_mode(VK_POLYGON_MODE_FILL);
    b.set_cull_mode(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_COUNTER_CLOCKWISE);
    b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
    // ... color attachment formats, blend state, etc.
};
ctx->pipelineManager->registerGraphics("deferred_geo", info);
```

### Retrieving and binding

```cpp
VkPipeline pipeline; VkPipelineLayout layout;
if (ctx->pipelineManager->getGraphics("deferred_geo", pipeline, layout)) {
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
}
```

### Hot-reload (called each frame or on file-watch event)

```cpp
ctx->pipelineManager->hotReloadChanged();   // enqueue changed shaders
ctx->pipelineManager->pumpMainThread();   // apply completed rebuilds
```

### Compute forwarding

```cpp
// All compute calls are forwarded to ComputeManager under the hood
ctx->pipelineManager->createComputePipeline("blur", computeInfo);
ctx->pipelineManager->dispatchCompute(cmd, "blur", dispatchInfo);
```

### Preset samplers

```cpp
VkSampler linear = ctx->samplerManager->defaultLinear();
VkSampler shadow = ctx->samplerManager->shadowLinearClamp();
```

## Available Samplers

| Accessor | Filter | Address Mode | Notes |
|----------|--------|-------------|-------|
| `defaultLinear()` | Linear | Repeat | General-purpose textures |
| `defaultNearest()` | Nearest | Repeat | Pixel-perfect sampling |
| `linearClampEdge()` | Linear | Clamp-to-Edge | Screen-space effects, tiled textures |
| `nearestClampEdge()` | Nearest | Clamp-to-Edge | LUTs, non-filterable formats |
| `shadowLinearClamp()` | Linear | Clamp-to-Border (white) | Shadow map sampling, manual PCF |

## Integration

`PipelineManager` is owned by `VulkanEngine` and exposed via `EngineContext::pipelineManager`.
Render passes register their graphics pipelines at setup time and retrieve handles
during recording. The async hot-reload worker runs on a background thread; only
`pumpMainThread()` touches Vulkan state and must be called from the main thread.

`SamplerManager` is owned by `VulkanEngine` and exposed via `EngineContext::samplerManager`.
It is initialized early (requires only `DeviceManager`) and provides read-only
accessors safe to call from any thread after init.

## Related Docs

- [docs/PipelineManager.md](../../../docs/PipelineManager.md) — high-level pipeline management documentation
- [src/compute/README.md](../../compute/README.md) — compute pipeline system
- [src/core/descriptor/README.md](../descriptor/README.md) — descriptor set management
