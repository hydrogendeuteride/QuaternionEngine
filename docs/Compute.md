## Compute System: Pipelines, Instances, and Dispatch

Standalone compute subsystem with a small, explicit API. Used by passes (e.g., Background) and tools. It lives under `src/compute` and is surfaced via `EngineContext::compute` and convenience wrappers on `PipelineManager`.

### Concepts

- **Pipelines**: Named compute pipelines created from a SPIR‑V module and a simple descriptor layout spec.
- **Instances**: Persistently bound descriptor sets keyed by instance name; useful for effects that rebind images/buffers across frames without re‑creating pipelines.
- **Dispatch**: Issue work with group counts, optional push constants, and ad‑hoc memory barriers.

### Key Types

#### `ComputeBinding` — Descriptor Binding Abstraction

Unified wrapper for all descriptor types. Use static factory methods:

```c++
struct ComputeBinding {
    uint32_t binding;
    VkDescriptorType type;
    // Union of buffer/image info

    static ComputeBinding uniformBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset = 0);
    static ComputeBinding storageBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset = 0);
    static ComputeBinding sampledImage(uint32_t binding, VkImageView view, VkSampler sampler,
                                       VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    static ComputeBinding storeImage(uint32_t binding, VkImageView view,
                                     VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL);
};
```

#### `ComputePipelineCreateInfo` — Pipeline Configuration

```c++
struct ComputePipelineCreateInfo {
    std::string shaderPath;                          // Path to .comp.spv file
    std::vector<VkDescriptorType> descriptorTypes;   // Descriptor layout (binding order)
    uint32_t pushConstantSize = 0;
    VkShaderStageFlags pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;

    // Optional: shader specialization constants
    std::vector<VkSpecializationMapEntry> specializationEntries;
    std::vector<uint32_t> specializationData;
};
```

#### `ComputeDispatchInfo` — Dispatch Configuration

```c++
struct ComputeDispatchInfo {
    uint32_t groupCountX = 1;                           // Workgroup counts
    uint32_t groupCountY = 1;
    uint32_t groupCountZ = 1;

    std::vector<ComputeBinding> bindings;               // Per-dispatch bindings (transient)

    const void *pushConstants = nullptr;                // Push constant data
    uint32_t pushConstantSize = 0;

    // Optional synchronization barriers (inserted after dispatch)
    std::vector<VkMemoryBarrier2> memoryBarriers;
    std::vector<VkBufferMemoryBarrier2> bufferBarriers;
    std::vector<VkImageMemoryBarrier2> imageBarriers;
};
```

### API Surface

#### Register/Destroy

| Method | Description |
|--------|-------------|
| `bool registerPipeline(name, ComputePipelineCreateInfo)` | Create and register a named compute pipeline |
| `void unregisterPipeline(name)` | Destroy and unregister a pipeline |
| `bool hasPipeline(name)` | Check if a pipeline exists |

#### Dispatch

| Method | Description |
|--------|-------------|
| `void dispatch(cmd, name, ComputeDispatchInfo)` | Record dispatch to command buffer |
| `void dispatchImmediate(name, ComputeDispatchInfo)` | Execute immediately (GPU-blocking) |

#### Dispatch Helpers

```c++
static uint32_t calculateGroupCount(uint32_t workItems, uint32_t localSize);
static ComputeDispatchInfo createDispatch2D(uint32_t w, uint32_t h, uint32_t lsX = 16, uint32_t lsY = 16);
static ComputeDispatchInfo createDispatch3D(uint32_t w, uint32_t h, uint32_t d,
                                            uint32_t lsX = 8, uint32_t lsY = 8, uint32_t lsZ = 8);
```

#### Instances

| Method | Description |
|--------|-------------|
| `bool createInstance(instanceName, pipelineName)` | Create persistent instance |
| `void destroyInstance(instanceName)` | Destroy instance and owned resources |
| `void setInstanceStorageImage(name, binding, imageView)` | Bind storage image |
| `void setInstanceSampledImage(name, binding, imageView, sampler)` | Bind sampled image |
| `void setInstanceBuffer(name, binding, buffer, size, type)` | Bind buffer |
| `AllocatedImage createAndBindStorageImage(...)` | Create and bind owned image |
| `AllocatedBuffer createAndBindStorageBuffer(...)` | Create and bind owned buffer |
| `void dispatchInstance(cmd, instanceName, info)` | Dispatch using instance bindings |

#### Utility Operations

```c++
void clearImage(VkCommandBuffer cmd, VkImageView imageView, const glm::vec4 &clearColor);
void copyBuffer(VkCommandBuffer cmd, VkBuffer src, VkBuffer dst, VkDeviceSize size,
                VkDeviceSize srcOffset = 0, VkDeviceSize dstOffset = 0);
```

### Quick Start — One‑Shot Dispatch

For ad-hoc compute work where bindings change each dispatch:

```c++
// 1. Register pipeline (once, e.g., in init)
ComputePipelineCreateInfo ci{};
ci.shaderPath = context->getAssets()->shaderPath("blur.comp.spv");
ci.descriptorTypes = { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER };
ci.pushConstantSize = sizeof(BlurPushConstants);
context->compute->registerPipeline("blur", ci);

// 2. Create dispatch info with bindings
ComputeDispatchInfo di = ComputeManager::createDispatch2D(width, height, 16, 16);
di.bindings.push_back(ComputeBinding::storeImage(0, outImageView));
di.bindings.push_back(ComputeBinding::sampledImage(1, inImageView, context->getSamplers()->defaultLinear()));

BlurPushConstants pc{ .radius = 5.0f };
di.pushConstants = &pc;
di.pushConstantSize = sizeof(pc);

// 3. Dispatch
context->compute->dispatch(cmd, "blur", di);
```

### Quick Start — Persistent Instance

For effects that reuse the same bindings across frames:

```c++
// 1. Create instance (once, e.g., in init)
context->compute->createInstance("background.sky", "sky");
context->compute->setInstanceStorageImage("background.sky", 0, ctx->getSwapchain()->drawImage().imageView);

// 2. Dispatch each frame (bindings persist)
ComputeDispatchInfo di = ComputeManager::createDispatch2D(ctx->getDrawExtent().width,
                                                          ctx->getDrawExtent().height);
di.pushConstants = &effect.data;
di.pushConstantSize = sizeof(ComputePushConstants);
context->compute->dispatchInstance(cmd, "background.sky", di);
```

### Quick Start — Immediate Execution

For GPU-blocking operations (uploads, preprocessing):

```c++
ComputeDispatchInfo di = ComputeManager::createDispatch2D(width, height);
di.bindings.push_back(ComputeBinding::storeImage(0, imageView));
di.pushConstants = &data;
di.pushConstantSize = sizeof(data);

// Blocks CPU until GPU completes
context->compute->dispatchImmediate("preprocess", di);
// Safe to read results here
```

### Integration With Render Graph

Use `RGPassType::Compute` for graph-managed compute passes:

```c++
graph->add_pass(
    "MyComputePass",
    RGPassType::Compute,
    // Build: declare resource usage
    [drawHandle](RGPassBuilder &builder, EngineContext *) {
        builder.write(drawHandle, RGImageUsage::ComputeWrite);  // Storage image write
    },
    // Record: issue dispatch
    [this, drawHandle](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
        VkImageView drawView = res.image_view(drawHandle);

        // Update binding to current frame's target
        ctx->pipelines->setComputeInstanceStorageImage("myeffect", 0, drawView);

        ComputeDispatchInfo di = ComputeManager::createDispatch2D(
            ctx->getDrawExtent().width, ctx->getDrawExtent().height, 16, 16);
        di.pushConstants = &params;
        di.pushConstantSize = sizeof(params);

        ctx->pipelines->dispatchComputeInstance(cmd, "myeffect", di);
    }
);
```

Benefits:
- Automatic layout transitions to `VK_IMAGE_LAYOUT_GENERAL` for storage images.
- Integrated barrier management with other passes.
- Per-frame synchronization tracking.

Reference: `src/render/passes/background.cpp`.

### Synchronization

#### Automatic (Render Graph)

When using the render graph, declare resource usage and let the graph handle barriers:
- `builder.write(image, RGImageUsage::ComputeWrite)` — storage image write
- `builder.read(image, RGImageUsage::SampledCompute)` — sampled image read in compute

#### Manual (Dispatch Barriers)

For standalone dispatches, add barriers to `ComputeDispatchInfo`:

```c++
ComputeDispatchInfo di{...};

// Image layout transition after compute write
VkImageMemoryBarrier2 barrier{
    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
    .srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
    .srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
    .dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
    .dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
    .oldLayout = VK_IMAGE_LAYOUT_GENERAL,
    .newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
    .image = image,
    .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }
};
di.imageBarriers.push_back(barrier);

context->compute->dispatch(cmd, "mycompute", di);
```

#### Frame-in-Flight Safety

- Descriptor sets use `VK_DESCRIPTOR_BINDING_UPDATE_AFTER_BIND_BIT` for safe concurrent updates.
- `dispatchInstance()` uses per-frame descriptor allocator when available, falling back to instance-owned sets.

### Shader Example

```glsl
// myeffect.comp
#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(set = 0, binding = 0, rgba16f) uniform image2D outImage;

layout(push_constant) uniform PushConstants {
    vec4 params;
} pc;

void main() {
    ivec2 texelCoord = ivec2(gl_GlobalInvocationID.xy);
    ivec2 size = imageSize(outImage);

    if (texelCoord.x >= size.x || texelCoord.y >= size.y) return;

    vec4 color = vec4(float(texelCoord.x) / size.x, float(texelCoord.y) / size.y, 0.0, 1.0);
    imageStore(outImage, texelCoord, color * pc.params);
}
```

### File Structure

```
src/compute/
├── vk_compute.h        # Core types and ComputeManager interface
└── vk_compute.cpp      # Implementation

src/core/
├── vk_pipeline_manager.h/cpp   # Unified graphics+compute wrapper
├── engine_context.h            # Contains ComputeManager* pointer
└── vk_resource.h/cpp           # immediate_submit() for GPU-blocking

src/render/
├── passes/background.cpp         # Example: gradient/sky compute passes
├── graph/types.h                 # RGPassType::Compute, RGImageUsage::ComputeWrite
└── graph/graph.h                 # Render graph integration

shaders/
├── gradient_color.comp     # Example: gradient background
└── sky.comp                # Example: procedural starfield
```

### Notes & Limits

- Compute pipelines are **not** hot-reloaded. Re-create via `registerPipeline()` if shader changes.
- `dispatchImmediate()` blocks the CPU; use sparingly for interactive workloads.
- Descriptor pool uses `UPDATE_AFTER_BIND` flag; ensure driver support (Vulkan 1.2+).
- Instance-owned resources are destroyed with the instance; external resources must be managed separately.

### Debugging

- Use Vulkan validation layers to catch descriptor/barrier issues.
- Each dispatch records debug labels when validation is enabled.
- Check `ComputeManager::hasPipeline()` before dispatch to avoid silent failures.

