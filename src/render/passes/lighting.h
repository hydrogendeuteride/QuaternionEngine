#pragma once
#include "render/renderpass.h"
#include <render/graph/types.h>
#include <span>

class LightingPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;

    void cleanup() override;

    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "Lighting"; }

    // Register lighting; consumes GBuffer + CSM cascades.
    void register_graph(class RenderGraph *graph,
                        RGImageHandle drawHandle,
                        RGImageHandle gbufferPosition,
                        RGImageHandle gbufferNormal,
                        RGImageHandle gbufferAlbedo,
                        RGImageHandle gbufferExtra,
                        std::span<RGImageHandle> shadowCascades);

private:
    EngineContext *_context = nullptr;

    VkDescriptorSetLayout _gBufferInputDescriptorLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _shadowDescriptorLayout = VK_NULL_HANDLE; // set=2 (array)
    // Fallbacks if IBL is not loaded
    AllocatedImage _fallbackIbl2D{};       // 1x1 black
    AllocatedImage _fallbackBrdfLut2D{};   // 1x1 RG, black

    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkDescriptorSetLayout _emptySetLayout = VK_NULL_HANDLE; // placeholder if IBL layout missing

    void draw_lighting(VkCommandBuffer cmd,
                       EngineContext *context,
                       const class RGPassResources &resources,
                       RGImageHandle drawHandle,
                       RGImageHandle gbufferPosition,
                       RGImageHandle gbufferNormal,
                       RGImageHandle gbufferAlbedo,
                       RGImageHandle gbufferExtra,
                       std::span<RGImageHandle> shadowCascades);

    DeletionQueue _deletionQueue;
};
