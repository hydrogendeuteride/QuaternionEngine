#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

class RenderGraph;
class RGPassResources;

class OceanPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void execute(VkCommandBuffer cmd) override;
    void cleanup() override;
    const char *getName() const override { return "Ocean"; }

    void register_graph(RenderGraph *graph,
                        RGImageHandle drawHandle,
                        RGImageHandle depthHandle,
                        RGImageHandle transmittanceLut = {});

private:
    void draw_ocean(VkCommandBuffer cmd,
                    EngineContext *context,
                    const RGPassResources &resources,
                    RGImageHandle drawHandle,
                    RGImageHandle depthHandle,
                    RGImageHandle transmittanceLut) const;

    EngineContext *_context{};
    VkDescriptorSetLayout _materialSetLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _atmosphereSetLayout = VK_NULL_HANDLE;
    mutable AllocatedImage _fallbackIbl2D{};
    mutable AllocatedImage _fallbackBrdf2D{};
    mutable AllocatedImage _fallbackTransmittanceLut{};
};
