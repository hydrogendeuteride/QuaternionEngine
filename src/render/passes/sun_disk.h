#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

class RenderGraph;
class RGPassResources;

// Analytic sun disk drawn as a fullscreen additive pass over the background.
// This is independent of atmosphere rendering and works in space.
class SunDiskPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "SunDisk"; }

    void register_graph(RenderGraph *graph, RGImageHandle hdrTarget);

private:
    void draw_sun_disk(VkCommandBuffer cmd,
                       EngineContext *context,
                       const RGPassResources &resources,
                       RGImageHandle hdrTarget);

    EngineContext *_context = nullptr;
    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
};

