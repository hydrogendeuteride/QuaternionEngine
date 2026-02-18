#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

#include <string>

class RenderGraph;
class RGPassResources;

// Analytic rocket plume raymarching: render a set of plume-local procedural volumes and composite onto HDR.
class RocketPlumePass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "RocketPlume"; }

    // Composite rocket plumes on top of hdrInput using gbuffer position for depth clamping.
    // Returns a new HDR image handle with plumes composited.
    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos);

private:
    void update_noise_texture(EngineContext *ctxLocal);

    void draw_plumes(VkCommandBuffer cmd,
                     EngineContext *context,
                     const RGPassResources &resources,
                     RGImageHandle hdrInput,
                     RGImageHandle gbufPos,
                     VkBuffer plumeBuffer,
                     VkDeviceSize plumeBufferSize,
                     uint32_t plumeCount);

    EngineContext *_context = nullptr;

    VkDescriptorSetLayout _inputSetLayout = VK_NULL_HANDLE; // set=1: hdr input + gbuffer + plume instance SSBO

    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;

    uint32_t _noiseHandle = 0xFFFFFFFFu; // TextureCache::InvalidHandle
    std::string _noisePath{};            // asset-relative, e.g. "vfx/simplex.ktx2"

    DeletionQueue _deletionQueue;
};
