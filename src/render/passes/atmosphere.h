#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

class RenderGraph;
class RGPassResources;

// Single-scattering Rayleigh/Mie atmosphere as an HDR fullscreen post-process.
// No LUTs: integrates per-pixel along view ray through a planet atmosphere sphere.
class AtmospherePass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "Atmosphere"; }

    // Register atmosphere scattering into the render graph.
    // hdrInput: HDR color buffer to composite onto.
    // gbufPos : G-Buffer world/local position (w > 0 for geometry, w == 0 for sky).
    // Returns a new HDR image handle with atmosphere applied.
    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos);

private:
    void draw_atmosphere(VkCommandBuffer cmd,
                         EngineContext *context,
                         const RGPassResources &resources,
                         RGImageHandle hdrInput,
                         RGImageHandle gbufPos,
                         RGImageHandle transmittanceLut);

    EngineContext *_context = nullptr;
    VkDescriptorSetLayout _inputSetLayout = VK_NULL_HANDLE; // set=1: hdr input + gbuffer position + transmittance LUT

    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;
};
