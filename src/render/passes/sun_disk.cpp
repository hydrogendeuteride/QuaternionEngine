#include "sun_disk.h"

#include "core/assets/manager.h"
#include "core/context.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/device/swapchain.h"
#include "core/frame/resources.h"
#include "core/pipeline/manager.h"

#include "render/graph/graph.h"
#include "render/graph/resources.h"
#include "render/pipelines.h"

#include <algorithm>

namespace
{
    struct SunDiskPush
    {
        glm::vec4 params0; // x: disk intensity, y: halo intensity, z: starburst intensity, w: halo radius (deg)
        glm::vec4 params1; // x: starburst radius (deg), y: spikes, z: sharpness, w: reserved
    };

    static_assert(sizeof(SunDiskPush) % 16 == 0);
} // namespace

void SunDiskPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getSwapchain() || !_context->getAssets())
    {
        return;
    }

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("sun_disk.frag.spv");
    info.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(), // set = 0
    };

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(SunDiskPush);
    info.pushConstants = {pcr};

    info.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.enable_blending_additive();
        if (_context && _context->getSwapchain())
        {
            b.set_color_attachment_format(_context->getSwapchain()->drawImage().imageFormat);
        }
    };

    _context->pipelines->createGraphicsPipeline("sun_disk", info);
}

void SunDiskPass::cleanup()
{
    // Pipelines are owned/destroyed by PipelineManager; clear cached handles.
    _pipeline = VK_NULL_HANDLE;
    _pipelineLayout = VK_NULL_HANDLE;
}

void SunDiskPass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void SunDiskPass::register_graph(RenderGraph *graph, RGImageHandle hdrTarget)
{
    if (!graph || !hdrTarget.valid() || !_context)
    {
        return;
    }

    const AtmosphereSettings &s = _context->atmosphere;
    const float diskIntensity = std::max(0.0f, s.sunDiskIntensity);
    const float haloIntensity = std::max(0.0f, s.sunHaloIntensity);
    const float starIntensity = std::max(0.0f, s.sunStarburstIntensity);
    if (diskIntensity <= 0.0f && haloIntensity <= 0.0f && starIntensity <= 0.0f)
    {
        return;
    }

    graph->add_pass(
        "SunDisk",
        RGPassType::Graphics,
        [hdrTarget](RGPassBuilder &builder, EngineContext *)
        {
            builder.write_color(hdrTarget);
        },
        [this, hdrTarget](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx)
        {
            draw_sun_disk(cmd, ctx, res, hdrTarget);
        });
}

void SunDiskPass::draw_sun_disk(VkCommandBuffer cmd,
                                EngineContext *context,
                                const RGPassResources &,
                                RGImageHandle)
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    PipelineManager *pipelineManager = ctxLocal->pipelines;
    if (!resourceManager || !deviceManager || !descriptorLayouts || !pipelineManager) return;

    const AtmosphereSettings &s = ctxLocal->atmosphere;
    const float diskIntensity = std::max(0.0f, s.sunDiskIntensity);
    const float haloIntensity = std::max(0.0f, s.sunHaloIntensity);
    const float starIntensity = std::max(0.0f, s.sunStarburstIntensity);
    const float haloRadiusDeg = std::max(0.0f, s.sunHaloRadiusDeg);
    const float starRadiusDeg = std::max(0.0f, s.sunStarburstRadiusDeg);
    const float starSharpness = std::max(1.0f, s.sunStarburstSharpness);
    const int starSpikesInt = std::clamp(s.sunStarburstSpikes, 2, 64);
    const float starSpikes = static_cast<float>(starSpikesInt);

    if (diskIntensity <= 0.0f && haloIntensity <= 0.0f && starIntensity <= 0.0f)
    {
        return;
    }

    if (!pipelineManager->getGraphics("sun_disk", _pipeline, _pipelineLayout))
    {
        return;
    }

    // Global scene UBO (set = 0).
    AllocatedBuffer gpuSceneDataBuffer = resourceManager->create_buffer(
        sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, gpuSceneDataBuffer]()
    {
        resourceManager->destroy_buffer(gpuSceneDataBuffer);
    });

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(deviceManager->allocator(), gpuSceneDataBuffer.allocation, &allocInfo);
    auto *sceneUniformData = static_cast<GPUSceneData *>(allocInfo.pMappedData);
    *sceneUniformData = ctxLocal->getSceneData();
    vmaFlushAllocation(deviceManager->allocator(), gpuSceneDataBuffer.allocation, 0, sizeof(GPUSceneData));

    VkDescriptorSet globalDescriptor = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    {
        DescriptorWriter writer;
        writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        writer.update_set(deviceManager->device(), globalDescriptor);
    }

    SunDiskPush pc{};
    pc.params0 = glm::vec4(diskIntensity, haloIntensity, starIntensity, haloRadiusDeg);
    pc.params1 = glm::vec4(starRadiusDeg, starSpikes, starSharpness, 0.0f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _pipelineLayout, 0, 1, &globalDescriptor, 0, nullptr);
    vkCmdPushConstants(cmd, _pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport vp{0.f, 0.f, float(extent.width), float(extent.height), 0.f, 1.f};
    VkRect2D sc{{0,0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
