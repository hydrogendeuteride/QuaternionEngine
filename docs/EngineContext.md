## Engine Context: Access to Managers + Per‑Frame State

Central DI-style handle that modules use to access device/managers, per-frame state, and convenience data without depending directly on `VulkanEngine`.

### Overview

- Ownership: Holds shared owners for `DeviceManager`, `ResourceManager`, and a growable `DescriptorAllocatorGrowable` used across modules.
- Global managers: Non-owning pointers to `SwapchainManager`, `DescriptorManager` (prebuilt layouts), `SamplerManager`, and `SceneManager`.
- Per-frame state: `currentFrame` (command buffer, per-frame descriptor pool, deletion queue), `stats`, and `drawExtent`.
- Subsystems: `compute` (`ComputeManager`) and `pipelines` (`PipelineManager`) exposed for unified graphics/compute API.
- Window + content: `window` (SDL handle) and convenience meshes (`cubeMesh`, `sphereMesh`).

Context is wired in `VulkanEngine::init()` and refreshed each frame before passes execute.

### Render Graph Note

- Built‑in passes no longer call `vkCmdBeginRendering` or perform image layout transitions directly.
- Use your pass’ `register_graph(graph, ...)` to declare attachments and resource accesses; the Render Graph inserts barriers and begins/ends dynamic rendering.
- See `docs/RenderGraph.md` for the builder API and scheduling.

### Quick Start — In a Render Pass (essentials)

```c++
void MyPass::init(EngineContext* context) {
  _context = context;

  // Use common descriptor layouts provided by DescriptorManager
  VkDescriptorSetLayout sceneLayout = _context->getDescriptorLayouts()->gpuSceneDataLayout();

  // Build a pipeline via PipelineManager (re-fetch on draw for hot reload)
  GraphicsPipelineCreateInfo info{};
  info.vertexShaderPath   = "../shaders/fullscreen.vert.spv";
  info.fragmentShaderPath = "../shaders/my_pass.frag.spv";
  info.setLayouts         = { sceneLayout };
  info.configure = [this](PipelineBuilder& b){
    b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
    b.set_polygon_mode(VK_POLYGON_MODE_FILL);
    b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
    b.set_multisampling_none();
    b.disable_depthtest();
    b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
  };
  _context->pipelines->createGraphicsPipeline("my_pass", info);
}

void MyPass::execute(VkCommandBuffer cmd) {
  // Fetch latest pipeline in case of hot reload
  VkPipeline p{}; VkPipelineLayout l{};
  _context->pipelines->getGraphics("my_pass", p, l);

  // Per-frame uniform buffer via currentFrame allocator
  AllocatedBuffer ubuf = _context->getResources()->create_buffer(
      sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
  _context->currentFrame->_deletionQueue.push_function([=, this]{ _context->getResources()->destroy_buffer(ubuf); });

  VmaAllocationInfo ai{}; vmaGetAllocationInfo(_context->getDevice()->allocator(), ubuf.allocation, &ai);
  *static_cast<GPUSceneData*>(ai.pMappedData) = _context->getSceneData();

  VkDescriptorSet set = _context->currentFrame->_frameDescriptors.allocate(
      _context->getDevice()->device(), _context->getDescriptorLayouts()->gpuSceneDataLayout());
  DescriptorWriter writer; 
  writer.write_buffer(0, ubuf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
  writer.update_set(_context->getDevice()->device(), set);

  vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, p);
  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, l, 0, 1, &set, 0, nullptr);

  // Viewport/scissor from context draw extent
  VkViewport vp{0,0,(float)_context->getDrawExtent().width,(float)_context->getDrawExtent().height,0.f,1.f};
  vkCmdSetViewport(cmd, 0, 1, &vp);
  VkRect2D sc{{0,0},{_context->getDrawExtent().width,_context->getDrawExtent().height}};
  vkCmdSetScissor(cmd, 0, 1, &sc);

  vkCmdDraw(cmd, 3, 1, 0, 0);
}
```

### Life Cycle

- Init: `VulkanEngine` constructs managers, initializes `_context`, then assigns `pipelines`, `compute`, `scene`, etc.
- Per-frame: Engine sets `currentFrame` and `drawExtent`, optionally triggers `PipelineManager::hotReloadChanged()`.
- Cleanup: Managers own their resources; modules should free layouts/sets they create and push per-frame deletions to `currentFrame->_deletionQueue`.

### Best Practices

- Prefer `EngineContext` accessors (`getDevice()`, `getResources()`, `getSwapchain()`) for clarity and testability.
- Re-fetch pipeline/layout by key every frame if using hot reload.
- Use `currentFrame->_frameDescriptors` for transient sets; use `context->descriptors` for longer-lived sets.
- Push resource cleanup to the frame or pass deletion queues to match lifetime with usage.

