## Pipeline Manager: Graphics + Compute

Centralizes pipeline creation and access with a clean, uniform API. Avoids duplication, enables hot reload (graphics), and makes passes/materials simpler.

### Overview

- Graphics pipelines: Owned by `PipelineManager` with per-name registry, hot-reloaded when shaders change.
- Compute pipelines: Created through `PipelineManager` but executed by `ComputeManager` under the hood.
- Access from anywhere via `EngineContext` (`context->pipelines`).

### Quick Start — Graphics

```c++
// In pass/material init
GraphicsPipelineCreateInfo info{};
info.vertexShaderPath = "../shaders/mesh.vert.spv";
info.fragmentShaderPath = "../shaders/mesh.frag.spv";
info.setLayouts = { context->getDescriptorLayouts()->gpuSceneDataLayout(), materialLayout };
info.pushConstants = { VkPushConstantRange{ VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(GPUDrawPushConstants) } };
info.configure = [context](PipelineBuilder& b){
  b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
  b.set_polygon_mode(VK_POLYGON_MODE_FILL);
  b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
  b.set_multisampling_none();
  b.disable_blending();
  b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
  b.set_color_attachment_format(context->getSwapchain()->drawImage().imageFormat);
  b.set_depth_format(context->getSwapchain()->depthImage().imageFormat);
};
context->pipelines->createGraphicsPipeline("mesh.opaque", info);

// Fetch for binding
MaterialPipeline mp{};
context->pipelines->getMaterialPipeline("mesh.opaque", mp);
vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mp.pipeline);
// ... bind sets using mp.layout
```

Notes:
- Graphics hot-reload runs each frame. If you cache pipeline handles, re-fetch with `getGraphics()` before use to pick up changes.

### Quick Start — Compute

Define and create a compute pipeline through the same manager:

```c++
ComputePipelineCreateInfo c{};
c.shaderPath = "../shaders/blur.comp.spv";
c.descriptorTypes = {
  VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,     // out image
  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER // in image
};
c.pushConstantSize = sizeof(MyBlurPC);

context->pipelines->createComputePipeline("blur", c);
```

Dispatch it when needed:

```c++
ComputeDispatchInfo di = ComputeManager::createDispatch2D(width, height, 16, 16);
di.bindings.push_back(ComputeBinding::storeImage(0, outView));
di.bindings.push_back(ComputeBinding::sampledImage(1, inView, context->getSamplers()->defaultLinear()));
di.pushConstants = &pc; // of type MyBlurPC
di.pushConstantSize = sizeof(MyBlurPC);

// Insert barriers as needed (optional)
// di.imageBarriers.push_back(...);

context->pipelines->dispatchCompute(cmd, "blur", di);
```

Tips:
- Use `dispatchComputeImmediate("name", di)` for one-off operations via an internal immediate command buffer.
- For complex synchronization, populate `memoryBarriers`, `bufferBarriers`, and `imageBarriers` in `ComputeDispatchInfo`.
- Compute pipelines are not hot-reloaded yet. If needed, re-create via `createComputePipeline(...)` and re-dispatch.

### When to Create vs. Use

- Create pipelines in pass/material `init()` and keep only the string keys around if you rely on hot reload.
- Re-fetch handles right before binding each frame to pick up changes:
  ```c++
  VkPipeline p; VkPipelineLayout l;
  if (context->pipelines->getGraphics("mesh.opaque", p, l)) { /* bind & draw */ }
  ```

### API Summary

- Graphics
  - `createGraphicsPipeline(name, GraphicsPipelineCreateInfo)`
  - `getGraphics(name, VkPipeline&, VkPipelineLayout&)`
  - `getMaterialPipeline(name, MaterialPipeline&)`
  - Hot reload: `hotReloadChanged()` is called by the engine each frame.

- Compute
  - `createComputePipeline(name, ComputePipelineCreateInfo)`
  - `destroyComputePipeline(name)` / `hasComputePipeline(name)`
  - `dispatchCompute(cmd, name, ComputeDispatchInfo)`
  - `dispatchComputeImmediate(name, ComputeDispatchInfo)`

### Persistent Compute Resources (Instances)

For long-lived compute workloads, create a compute instance that owns its descriptor set and (optionally) its resources.

```c++
// 1) Ensure the pipeline exists
ComputePipelineCreateInfo c{}; c.shaderPath = "../shaders/work.comp.spv"; c.descriptorTypes = { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE };
context->pipelines->createComputePipeline("work", c);

// 2) Create an instance bound to the pipeline
// You can go either via ComputeManager or PipelineManager
context->pipelines->createComputeInstance("work.main", "work");

// 3) Allocate persistent resources and bind to instance
auto img = context->pipelines->createAndBindComputeStorageImage("work.main", 0,
            VkExtent3D{width, height, 1}, VK_FORMAT_R8G8B8A8_UNORM);

// 4) Optionally add more bindings (buffers, sampled images, etc.)
auto buf = context->pipelines->createAndBindComputeStorageBuffer("work.main", 1, size);
// or reference external resources
context->pipelines->setComputeInstanceStorageImage("work.main", 2, someView);

// 5) Update and dispatch repeatedly (bindings persist)
ComputeDispatchInfo di = ComputeManager::createDispatch2D(width, height);
di.pushConstants = &myPC; di.pushConstantSize = sizeof(myPC);
context->pipelines->dispatchComputeInstance(cmd, "work.main", di);

// 6) Destroy when no longer needed
context->pipelines->destroyComputeInstance("work.main");
```

Notes:
- Instances keep their descriptor set and binding specification; you can modify bindings via `setInstance*` and call `dispatchInstance()` without respecifying them each frame.
- Owned images/buffers created via `createAndBind*` are automatically destroyed when the instance is destroyed or on engine cleanup.
- Descriptor sets are allocated from a growable pool and are freed when the compute manager is cleaned up.

### Best Practices

- Keep descriptor set layouts owned by the module that defines resource interfaces (e.g., material or pass). Pipelines/layouts created by the manager are managed by the manager.
- Prefer pipeline keys over cached handles to benefit from hot reload.
- Encapsulate fixed-function state in `GraphicsPipelineCreateInfo::configure` lambdas to keep pass code tidy.
