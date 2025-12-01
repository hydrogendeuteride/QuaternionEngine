#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

class RenderGraph;
class RGPassResources;

// Screen Space Reflections (SSR) pass.
// In v1 this is a lightweight stub wired into the RenderGraph by the engine.
// The full pipeline and shader implementation is added in later plan steps.
class SSRPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "SSR"; }

    // Register SSR in the render graph.
    // hdrInput   : HDR color buffer produced by deferred lighting.
    // gbufPos    : G-Buffer world-space position (RGBA32F).
    // gbufNorm   : G-Buffer world-space normal + roughness.
    // gbufAlbedo : G-Buffer albedo + metallic.
    // hdrOutput  : HDR color buffer that will carry lighting + SSR.
    void register_graph(RenderGraph *graph,
                        RGImageHandle hdrInput,
                        RGImageHandle gbufPos,
                        RGImageHandle gbufNorm,
                        RGImageHandle gbufAlbedo,
                        RGImageHandle hdrOutput);

private:
    EngineContext *_context = nullptr;
    VkDescriptorSetLayout _inputSetLayout = VK_NULL_HANDLE; // set=1: HDR + GBuffer inputs
    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;

    void draw_ssr(VkCommandBuffer cmd,
                  EngineContext *context,
                  const class RGPassResources &resources,
                  RGImageHandle hdrInput,
                  RGImageHandle gbufPos,
                  RGImageHandle gbufNorm,
                  RGImageHandle gbufAlbedo);

    DeletionQueue _deletionQueue;
};
