#include "punctual_shadow.h"

#include <string>
#include <unordered_set>

#include "core/context.h"
#include "core/config.h"
#include "core/device/resource.h"
#include "core/device/device.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"
#include "render/pipelines.h"
#include "render/graph/graph.h"
#include "render/graph/builder.h"
#include "scene/vk_scene.h"

namespace
{
struct PunctualShadowPushConstants
{
    glm::mat4 light_mvp;
    VkDeviceAddress vertexBuffer;
    uint32_t objectID;
    uint32_t _pad;
};
static_assert(offsetof(PunctualShadowPushConstants, light_mvp) == 0);
static_assert(offsetof(PunctualShadowPushConstants, vertexBuffer) == 64);
static_assert(offsetof(PunctualShadowPushConstants, objectID) == 72);
static_assert(sizeof(PunctualShadowPushConstants) == 80);
} // namespace

void PunctualShadowPass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->pipelines || !_context->getAssets())
    {
        return;
    }

    VkPushConstantRange pc{};
    pc.offset = 0;
    pc.size = static_cast<uint32_t>(sizeof(PunctualShadowPushConstants));
    pc.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    GraphicsPipelineCreateInfo info{};
    info.vertexShaderPath = _context->getAssets()->shaderPath("punctual_shadow.vert.spv");
    info.fragmentShaderPath = _context->getAssets()->shaderPath("punctual_shadow.frag.spv");
    info.pushConstants = {pc};
    info.configure = [](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        b.enable_depthtest(true, VK_COMPARE_OP_LESS_OR_EQUAL);
        b.set_depth_format(VK_FORMAT_D32_SFLOAT);
        b._rasterizer.depthBiasEnable = VK_TRUE;
        b._rasterizer.depthBiasConstantFactor = kShadowDepthBiasConstant;
        b._rasterizer.depthBiasSlopeFactor = kShadowDepthBiasSlope;
        b._rasterizer.depthBiasClamp = 0.0f;
    };

    _context->pipelines->createGraphicsPipeline("mesh.punctual_shadow", info);
}

void PunctualShadowPass::cleanup()
{
    Logger::info("PunctualShadowPass::cleanup()");
}

void PunctualShadowPass::execute(VkCommandBuffer)
{
    // RenderGraph-driven.
}

void PunctualShadowPass::register_graph(RenderGraph *graph,
                                        std::span<RGImageHandle> spotShadowMaps,
                                        std::span<RGImageHandle> pointShadowFaces,
                                        VkExtent2D spotExtent,
                                        VkExtent2D pointExtent)
{
    if (!graph)
    {
        return;
    }

    auto add_common_reads = [](RGPassBuilder &builder, EngineContext *ctx)
    {
        if (!ctx)
        {
            return;
        }
        const DrawContext &dc = ctx->getMainDrawContext();
        std::unordered_set<VkBuffer> indexSet;
        std::unordered_set<VkBuffer> vertexSet;
        for (const auto &r : dc.OpaqueSurfaces)
        {
            if (!r.sourceMesh)
            {
                continue;
            }
            if (r.indexBuffer)
            {
                indexSet.insert(r.indexBuffer);
            }
            if (r.vertexBuffer)
            {
                vertexSet.insert(r.vertexBuffer);
            }
        }
        for (VkBuffer b : indexSet)
        {
            builder.read_buffer(b, RGBufferUsage::IndexRead, 0, "punctual_shadow.index");
        }
        for (VkBuffer b : vertexSet)
        {
            builder.read_buffer(b, RGBufferUsage::StorageRead, 0, "punctual_shadow.vertex");
        }
    };

    for (uint32_t i = 0; i < spotShadowMaps.size(); ++i)
    {
        if (!spotShadowMaps[i].valid())
        {
            continue;
        }

        const std::string passName = std::string("PunctualShadow.Spot[") + std::to_string(i) + "]";
        graph->add_pass(
            passName.c_str(),
            RGPassType::Graphics,
            [add_common_reads, shadowDepth = spotShadowMaps[i]](RGPassBuilder &builder, EngineContext *ctx)
            {
                VkClearValue clear{};
                clear.depthStencil = {1.0f, 0};
                builder.write_depth(shadowDepth, true, clear);
                add_common_reads(builder, ctx);
            },
            [this, i, spotExtent](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx)
            {
                draw_shadow(cmd, ctx, spotExtent, false, i, 0u);
            });
    }

    for (uint32_t idx = 0; idx < pointShadowFaces.size(); ++idx)
    {
        if (!pointShadowFaces[idx].valid())
        {
            continue;
        }

        const uint32_t lightIndex = idx / kPointShadowFaceCount;
        const uint32_t faceIndex = idx % kPointShadowFaceCount;
        const std::string passName = std::string("PunctualShadow.Point[")
                                     + std::to_string(lightIndex)
                                     + "."
                                     + std::to_string(faceIndex)
                                     + "]";
        graph->add_pass(
            passName.c_str(),
            RGPassType::Graphics,
            [add_common_reads, shadowDepth = pointShadowFaces[idx]](RGPassBuilder &builder, EngineContext *ctx)
            {
                VkClearValue clear{};
                clear.depthStencil = {1.0f, 0};
                builder.write_depth(shadowDepth, true, clear);
                add_common_reads(builder, ctx);
            },
            [this, lightIndex, faceIndex, pointExtent](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx)
            {
                draw_shadow(cmd, ctx, pointExtent, true, lightIndex, faceIndex);
            });
    }
}

void PunctualShadowPass::draw_shadow(VkCommandBuffer cmd,
                                     EngineContext *context,
                                     VkExtent2D extent,
                                     bool pointLight,
                                     uint32_t lightIndex,
                                     uint32_t faceIndex) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame || !ctxLocal->pipelines)
    {
        return;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (!ctxLocal->pipelines->getGraphics("mesh.punctual_shadow", pipeline, layout))
    {
        return;
    }

    const GPUSceneData &sd = ctxLocal->getSceneData();
    glm::mat4 lightVP(1.0f);
    if (pointLight)
    {
        const uint32_t matrixIndex = lightIndex * kPointShadowFaceCount + faceIndex;
        if (matrixIndex >= kMaxPointShadowFaces)
        {
            return;
        }
        lightVP = sd.pointLightShadowViewProj[matrixIndex];
    }
    else
    {
        if (lightIndex >= kMaxShadowedSpotLights)
        {
            return;
        }
        lightVP = sd.spotLightShadowViewProj[lightIndex];
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = extent;
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    const DrawContext &dc = ctxLocal->getMainDrawContext();
    VkBuffer lastIndexBuffer = VK_NULL_HANDLE;
    for (const auto &r : dc.OpaqueSurfaces)
    {
        if (!r.sourceMesh)
        {
            continue;
        }
        if (r.indexBuffer != lastIndexBuffer)
        {
            lastIndexBuffer = r.indexBuffer;
            vkCmdBindIndexBuffer(cmd, r.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }

        PunctualShadowPushConstants spc{};
        spc.light_mvp = lightVP * r.transform;
        spc.vertexBuffer = r.vertexBufferAddress;
        spc.objectID = r.objectID;
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PunctualShadowPushConstants), &spc);
        vkCmdDrawIndexed(cmd, r.indexCount, 1, r.firstIndex, 0, 0);
    }
}
