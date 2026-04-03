#pragma once

#include "render/renderpass.h"
#include "render/graph/types.h"

#include <core/types.h>
#include <core/world.h>

#include <array>
#include <string>

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
    void preload_cloud_textures();

    const char *getName() const override { return "Atmosphere"; }

    // Register atmosphere scattering into the render graph.
    // hdrInput: HDR color buffer to composite onto.
    // gbufPos : G-Buffer world/local position (w > 0 for geometry, w == 0 for sky).
    // Returns a new HDR image handle with atmosphere applied.
    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos);

private:
    struct HistoryImageState
    {
        VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
        VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
        VkAccessFlags2 access = 0;
    };

    struct HistorySnapshot
    {
        bool valid = false;
        VkExtent2D drawExtent{0, 0};
        VkExtent2D lowExtent{0, 0};
        uint64_t originRevision = 0;
        glm::mat4 projection{1.0f};
        glm::vec4 planetCenterRadius{0.0f};
        glm::vec4 sunDirection{0.0f};
        glm::vec4 sunColor{0.0f};
        glm::vec4 ambientColor{0.0f};
        std::string bodyName;
        std::string overlayPath;
        std::string noisePath;
        float cloudBaseM = 0.0f;
        float cloudThicknessM = 0.0f;
        float cloudDensityScale = 0.0f;
        glm::vec4 cloudColor{1.0f};
        float cloudCoverage = 0.0f;
        float cloudNoiseScale = 0.0f;
        float cloudDetailScale = 0.0f;
        float cloudNoiseBlend = 0.0f;
        float cloudDetailErode = 0.0f;
        float cloudWindSpeed = 0.0f;
        float cloudWindAngle = 0.0f;
        float cloudOverlayRotation = 0.0f;
        bool cloudOverlayFlipV = false;
    };

    void draw_cloud_low_res(VkCommandBuffer cmd,
                            EngineContext *context,
                            const RGPassResources &resources,
                            VkExtent2D extent,
                            RGImageHandle gbufPos,
                            RGImageHandle transmittanceLut,
                            RGImageHandle cloudLighting,
                            RGImageHandle cloudSegment);
    void draw_cloud_temporal(VkCommandBuffer cmd,
                             EngineContext *context,
                             const RGPassResources &resources,
                             VkExtent2D extent,
                             RGImageHandle cloudLightingCurrent,
                             RGImageHandle cloudSegmentCurrent,
                             RGImageHandle cloudLightingHistoryPrev,
                             RGImageHandle cloudSegmentHistoryPrev,
                             RGImageHandle cloudLightingHistoryNext,
                             RGImageHandle cloudSegmentHistoryNext,
                             const glm::mat4 &previousViewProj,
                             const glm::vec3 &originDeltaLocal,
                             bool historyValid);
    void draw_cloud_upscale(VkCommandBuffer cmd,
                            EngineContext *context,
                            const RGPassResources &resources,
                            VkExtent2D extent,
                            RGImageHandle gbufPos,
                            RGImageHandle cloudLightingLowRes,
                            RGImageHandle cloudSegmentLowRes,
                            RGImageHandle cloudLightingResolved,
                            RGImageHandle cloudSegmentResolved);
    void draw_composite(VkCommandBuffer cmd,
                        EngineContext *context,
                        const RGPassResources &resources,
                        VkExtent2D extent,
                        RGImageHandle hdrInput,
                        RGImageHandle gbufPos,
                        RGImageHandle transmittanceLut,
                        RGImageHandle cloudLightingResolved,
                        RGImageHandle cloudSegmentResolved);

    void ensure_cloud_textures(EngineContext *context);
    void release_history_images();
    void ensure_history_images(VkExtent2D extent);

    EngineContext *_context = nullptr;
    VkDescriptorSetLayout _lowResSetLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _temporalSetLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _upscaleSetLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _compositeSetLayout = VK_NULL_HANDLE;

    VkPipeline _cloudLowResPipeline = VK_NULL_HANDLE;
    VkPipelineLayout _cloudLowResPipelineLayout = VK_NULL_HANDLE;
    VkPipeline _cloudTemporalPipeline = VK_NULL_HANDLE;
    VkPipelineLayout _cloudTemporalPipelineLayout = VK_NULL_HANDLE;
    VkPipeline _cloudUpscalePipeline = VK_NULL_HANDLE;
    VkPipelineLayout _cloudUpscalePipelineLayout = VK_NULL_HANDLE;
    VkPipeline _compositePipeline = VK_NULL_HANDLE;
    VkPipelineLayout _compositePipelineLayout = VK_NULL_HANDLE;

    AllocatedImage _cloudOverlayTex{};
    std::string _cloudOverlayLoadedPath;
    AllocatedImage _cloudNoiseTex{};
    std::string _cloudNoiseLoadedPath;

    std::array<AllocatedImage, 2> _cloudLightingHistory{};
    std::array<AllocatedImage, 2> _cloudSegmentHistory{};
    std::array<HistoryImageState, 2> _cloudLightingHistoryState{};
    std::array<HistoryImageState, 2> _cloudSegmentHistoryState{};
    uint32_t _historyActiveIndex = 0;
    bool _historyValid = false;
    glm::mat4 _previousViewProj{1.0f};
    WorldVec3 _previousOriginWorld{0.0, 0.0, 0.0};
    HistorySnapshot _historySnapshot{};
};
