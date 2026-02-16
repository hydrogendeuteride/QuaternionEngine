#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

class MeshVfxPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void execute(VkCommandBuffer cmd) override;
    void cleanup() override;
    const char *getName() const override { return "MeshVFX"; }

    void register_graph(class RenderGraph *graph,
                        RGImageHandle drawHandle,
                        RGImageHandle depthHandle);

private:
    void draw_mesh_vfx(VkCommandBuffer cmd,
                       EngineContext *context,
                       const class RGPassResources &resources,
                       RGImageHandle drawHandle,
                       RGImageHandle depthHandle) const;

    EngineContext *_context{};
};

