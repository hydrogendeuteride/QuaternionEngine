#pragma once

#include "render/renderpass.h"
#include <render/graph/types.h>
#include <span>

class RenderGraph;
class EngineContext;
class RGPassResources;

// Depth-only shadow-map pass for punctual (spot/point) lights.
class PunctualShadowPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "PunctualShadowMap"; }

    // Register one pass per spot light and per point-light cube face.
    void register_graph(RenderGraph *graph,
                        std::span<RGImageHandle> spotShadowMaps,
                        std::span<RGImageHandle> pointShadowFaces,
                        VkExtent2D spotExtent,
                        VkExtent2D pointExtent);

private:
    EngineContext *_context = nullptr;

    void draw_shadow(VkCommandBuffer cmd,
                     EngineContext *context,
                     VkExtent2D extent,
                     bool pointLight,
                     uint32_t lightIndex,
                     uint32_t faceIndex) const;
};
