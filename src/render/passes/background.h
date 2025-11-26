#pragma once
#include "render/renderpass.h"
#include "compute/vk_compute.h"
#include "render/graph/types.h"

class RenderGraph;

class BackgroundPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;
    const char *getName() const override { return "Background"; }

    void register_graph(RenderGraph *graph, RGImageHandle drawHandle, RGImageHandle depthHandle);

    void setCurrentEffect(int index) { _currentEffect = index; }
    std::vector<ComputeEffect> &getEffects() { return _backgroundEffects; }

    std::vector<ComputeEffect> _backgroundEffects;
    int _currentEffect = 2;

private:
    EngineContext *_context = nullptr;

    void init_background_pipelines();

    // Graphics env background pipeline
    VkPipeline _envPipeline = VK_NULL_HANDLE;
    VkPipelineLayout _envPipelineLayout = VK_NULL_HANDLE;
    // Empty descriptor layout used as placeholder for sets 1 and 2
    VkDescriptorSetLayout _emptySetLayout = VK_NULL_HANDLE;
    // Fallback 1x1x6 black cube if IBL not loaded
    AllocatedImage _fallbackIblCube{};
};
