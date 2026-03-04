#pragma once

#include <render/renderpass.h>
#include <render/graph/types.h>

class EngineContext;
class RenderGraph;
class RGPassResources;

// Orbit plot line rendering pass (dedicated gameplay trajectory overlay).
class OrbitPlotPass final : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer) override;
    const char *getName() const override { return "OrbitPlot"; }

    void register_graph(RenderGraph *graph,
                        RGImageHandle target_color,
                        RGImageHandle depth,
                        bool is_ldr_target);

private:
    void draw_orbit_plot(VkCommandBuffer cmd,
                         EngineContext *ctx,
                         bool is_ldr_target);

    EngineContext *_context = nullptr;
    DeletionQueue _deletionQueue;
};
