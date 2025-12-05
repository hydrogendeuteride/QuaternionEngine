#pragma once

#include <core/types.h>
#include <render/renderpass.h>
#include <render/graph/types.h>

class EngineContext;
class RenderGraph;
class RGPassResources;

// Simple post-process anti-aliasing pass (FXAA-like).
// Operates on the LDR tonemapped image and outputs a smoothed LDR image.
class FxaaPass final : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer) override; // Not used directly; executed via render graph
    const char *getName() const override { return "FXAA"; }

    // Register pass in the render graph. Returns the AA output image handle.
    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle ldrInput);

    // Runtime parameters
    void set_enabled(bool e) { _enabled = e; }
    bool enabled() const { return _enabled; }
    void set_edge_threshold(float v) { _edge_threshold = v; }
    float edge_threshold() const { return _edge_threshold; }
    void set_edge_threshold_min(float v) { _edge_threshold_min = v; }
    float edge_threshold_min() const { return _edge_threshold_min; }

private:
    struct FxaaPush
    {
        float inverse_width;
        float inverse_height;
        float edge_threshold;
        float edge_threshold_min;
    };

    void draw_fxaa(VkCommandBuffer cmd, EngineContext *ctx, const RGPassResources &res,
                   RGImageHandle ldrInput);

    EngineContext *_context = nullptr;

    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _inputSetLayout = VK_NULL_HANDLE;

    // Tunables for edge detection; chosen to be conservative by default.
    bool _enabled = true;
    float _edge_threshold = 0.125f;
    float _edge_threshold_min = 0.0312f;

    DeletionQueue _deletionQueue;
};
