#pragma once

#include "render/renderpass.h"
#include <render/graph/types.h>

class DecalPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void execute(VkCommandBuffer cmd) override;
    void cleanup() override;

    const char *getName() const override { return "Decal"; }

    void register_graph(class RenderGraph *graph,
                        RGImageHandle gbufferPosition,
                        RGImageHandle gbufferNormal,
                        RGImageHandle gbufferAlbedo);

private:
    void draw_decals(VkCommandBuffer cmd,
                     EngineContext *context,
                     const class RGPassResources &resources,
                     RGImageHandle gbufferPosition,
                     RGImageHandle gbufferNormal,
                     RGImageHandle gbufferAlbedo);

    EngineContext *_context = nullptr;
    VkDescriptorSetLayout _gbufferInputLayout = VK_NULL_HANDLE; // set=1
    VkDescriptorSetLayout _decalMaterialLayout = VK_NULL_HANDLE; // set=2

    AllocatedImage _fallbackAlbedo{};
    AllocatedImage _fallbackNormal{};
};
