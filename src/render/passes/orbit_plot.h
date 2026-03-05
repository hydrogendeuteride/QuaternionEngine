#pragma once

#include <render/renderpass.h>
#include <render/graph/types.h>

#include "orbit_plot_generate.h"

#include <array>
#include <cstddef>

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
    static constexpr std::size_t k_upload_ring_slots = 3;

    struct UploadSlot
    {
        AllocatedBuffer buffer{};
        std::size_t capacity_bytes = 0;
    };

    void draw_orbit_plot(VkCommandBuffer cmd,
                         EngineContext *ctx,
                         bool is_ldr_target);
    void draw_orbit_plot_indirect(VkCommandBuffer cmd,
                                  EngineContext *ctx,
                                  bool is_ldr_target,
                                  const OrbitPlotGenerate::PreparedFrame &generated_frame);

    EngineContext *_context = nullptr;
    DeletionQueue _deletionQueue;
    std::array<UploadSlot, k_upload_ring_slots> _upload_ring{};
    OrbitPlotGenerate _gpu_generate{};
};
