#pragma once
#include "render/renderpass.h"
#include <render/graph/types.h>

class SwapchainManager;
class RenderGraph;

class GeometryPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "Geometry"; }

    void register_graph(RenderGraph *graph,
                        RGImageHandle gbufferPosition,
                        RGImageHandle gbufferNormal,
                        RGImageHandle gbufferAlbedo,
                        RGImageHandle gbufferExtra,
                        RGImageHandle idHandle,
                        RGImageHandle depthHandle);

private:
    EngineContext *_context = nullptr;

    void draw_geometry(VkCommandBuffer cmd,
                       EngineContext *context,
                       const class RGPassResources &resources,
                       RGImageHandle gbufferPosition,
                       RGImageHandle gbufferNormal,
                       RGImageHandle gbufferAlbedo,
                       RGImageHandle gbufferExtra,
                       RGImageHandle idHandle,
                       RGImageHandle depthHandle) const;
};
