#pragma once

#include <core/types.h>
#include <render/renderpass.h>
#include <render/graph/types.h>

class EngineContext;
class RenderGraph;
class RGPassResources;

class TonemapPass final : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer) override; // Not used directly; executed via render graph
    const char *getName() const override { return "Tonemap"; }

    // Register pass in the render graph. Returns the LDR output image handle.
    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle hdrInput);

    // Runtime parameters
    void setExposure(float e) { _exposure = e; }
    float exposure() const { return _exposure; }
    void setMode(int m) { _mode = m; }
    int mode() const { return _mode; }

private:
    void draw_tonemap(VkCommandBuffer cmd, EngineContext *ctx, const RGPassResources &res,
                      RGImageHandle hdrInput);

    EngineContext *_context = nullptr;

    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _inputSetLayout = VK_NULL_HANDLE;

    float _exposure = 1.0f;
    int _mode = 1; // default to ACES

    DeletionQueue _deletionQueue;
};

