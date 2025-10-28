#pragma once

#include <memory>
#include <core/vk_types.h>
#include <core/vk_descriptors.h>
// Avoid including vk_scene.h here to prevent cycles

struct EngineStats
{
    float frametime;
    int triangle_count;
    int drawcall_count;
    float scene_update_time;
    float mesh_draw_time;
};

class DeviceManager;
class ResourceManager;
class SwapchainManager;
class DescriptorManager;
class SamplerManager;
class SceneManager;
class MeshAsset;
struct DrawContext;
struct GPUSceneData;
class ComputeManager;
class PipelineManager;
struct FrameResources;
struct SDL_Window;
class AssetManager;
class RenderGraph;
class RayTracingManager;

struct ShadowSettings
{
    // 0 = Clipmap only, 1 = Clipmap + RT assist, 2 = RT only
    uint32_t mode = 0;
    bool hybridRayQueryEnabled = false;   // derived convenience: (mode != 0)
    uint32_t hybridRayCascadesMask = 0b1110; // bit i => cascade i uses ray query assist (default: 1..3)
    float hybridRayNoLThreshold = 0.25f;  // trigger when N·L below this (mode==1)
};

class EngineContext
{
public:
    // Owned shared resources
    std::shared_ptr<DeviceManager> device;
    std::shared_ptr<ResourceManager> resources;
    std::shared_ptr<DescriptorAllocatorGrowable> descriptors;

    // Non-owning pointers to global managers owned by VulkanEngine
    SwapchainManager* swapchain = nullptr;
    DescriptorManager* descriptorLayouts = nullptr;
    SamplerManager* samplers = nullptr;
    SceneManager* scene = nullptr;

    // Per-frame and subsystem pointers for modules to use without VulkanEngine
    FrameResources* currentFrame = nullptr;      // set by engine each frame
    EngineStats* stats = nullptr;                // points to engine stats
    ComputeManager* compute = nullptr;           // compute subsystem
    PipelineManager* pipelines = nullptr;        // graphics pipeline manager
    RenderGraph* renderGraph = nullptr;          // render graph (built per-frame)
    SDL_Window* window = nullptr;                // SDL window handle

    // Frequently used values
    VkExtent2D drawExtent{};

    // Optional convenience content pointers (moved to AssetManager for meshes)

    // Assets
    AssetManager* assets = nullptr;              // non-owning pointer to central AssetManager

    // Runtime settings visible to passes/shaders
    ShadowSettings shadowSettings{};

    // Ray tracing manager (optional, nullptr if unsupported)
    RayTracingManager* ray = nullptr;

    // Accessors
    DeviceManager *getDevice() const { return device.get(); }
    ResourceManager *getResources() const { return resources.get(); }
    DescriptorAllocatorGrowable *getDescriptors() const { return descriptors.get(); }
    SwapchainManager* getSwapchain() const { return swapchain; }
    DescriptorManager* getDescriptorLayouts() const { return descriptorLayouts; }
    SamplerManager* getSamplers() const { return samplers; }
    const GPUSceneData& getSceneData() const;
    const DrawContext& getMainDrawContext() const;
    VkExtent2D getDrawExtent() const { return drawExtent; }
    AssetManager* getAssets() const { return assets; }
    // Convenience alias (singular) requested
    AssetManager* getAsset() const { return assets; }
    RenderGraph* getRenderGraph() const { return renderGraph; }
};
