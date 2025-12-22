#pragma once

#include "core/world.h"
#include "render/renderpass.h"
#include "render/graph/types.h"

#include <array>

class RenderGraph;
class RGPassResources;
struct VoxelVolumeSettings;

// Volumetric voxel clouds: raymarch a bounded volume and sample density from an SSBO voxel grid.
class CloudPass : public IRenderPass
{
public:
    void init(EngineContext *context) override;
    void cleanup() override;
    void execute(VkCommandBuffer cmd) override;

    const char *getName() const override { return "Volumetrics"; }

    // Register the cloud pass into the render graph.
    // hdrInput: HDR color buffer to composite on top of.
    // gbufPos : G-Buffer world/local position (w=1 for geometry, w=0 for sky).
    // Returns a new HDR image handle with clouds composited.
    RGImageHandle register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos);

private:
    EngineContext *_context = nullptr;
    VkDescriptorSetLayout _inputSetLayout = VK_NULL_HANDLE; // set=1: hdr input + gbuffer + voxel density buffer

    VkPipeline _pipeline = VK_NULL_HANDLE;
    VkPipelineLayout _pipelineLayout = VK_NULL_HANDLE;

    static constexpr uint32_t MAX_VOLUMES = 4;

    struct VolumeBuffers
    {
        AllocatedBuffer voxelDensity[2]{};
        uint32_t voxelReadIndex = 0;
        VkDeviceSize voxelDensitySize = 0;
        uint32_t gridResolution = 0;
    };

    std::array<VolumeBuffers, MAX_VOLUMES> _volumes{};

    void rebuild_voxel_density(uint32_t volume_index, uint32_t resolution, const VoxelVolumeSettings &settings);

    void update_time_and_origin_delta();

    void draw_volume(VkCommandBuffer cmd,
                     EngineContext *context,
                     const RGPassResources &resources,
                     RGImageHandle hdrInput,
                     RGImageHandle gbufPos,
                     const VoxelVolumeSettings &settings,
                     uint32_t grid_resolution,
                     VkBuffer voxelBuffer,
                     VkDeviceSize voxelSize);

    // Per-frame sim time (used when animateVoxels is enabled).
    float _dt_sec = 0.0f;
    float _time_sec = 0.0f;

    // Floating-origin tracking (used to keep the volume stable when not following camera).
    bool _has_prev_origin = false;
    WorldVec3 _prev_origin_world{0.0, 0.0, 0.0};
    glm::vec3 _origin_delta_local{0.0f, 0.0f, 0.0f};

    DeletionQueue _deletionQueue;
};
