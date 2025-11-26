#pragma once

#include "render/renderpass.h"
#include <render/graph/types.h>
#include <span>

class RenderGraph;
class EngineContext;
class RGPassResources;

// Depth-only directional shadow map pass (CSM-ready API)
class ShadowPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "ShadowMap"; }

    // Register N cascades; one graphics pass per cascade.
    void register_graph(RenderGraph *graph, std::span<RGImageHandle> cascades, VkExtent2D extent);

private:
    EngineContext *_context = nullptr;

    void draw_shadow(VkCommandBuffer cmd,
                     EngineContext *context,
                     const RGPassResources &resources,
                     RGImageHandle shadowDepth,
                     VkExtent2D extent,
                     uint32_t cascadeIndex) const;
};
