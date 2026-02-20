#include "mesh_vfx.h"

#include <algorithm>
#include <unordered_set>

#include "core/assets/texture_cache.h"
#include "scene/vk_scene.h"
#include "core/context.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/descriptor/manager.h"
#include "core/frame/resources.h"
#include "render/graph/graph.h"

void MeshVfxPass::init(EngineContext *context)
{
    _context = context;
}

void MeshVfxPass::execute(VkCommandBuffer)
{
    // Executed through render graph.
}

void MeshVfxPass::register_graph(RenderGraph *graph, RGImageHandle drawHandle, RGImageHandle depthHandle)
{
    if (!graph || !drawHandle.valid() || !depthHandle.valid()) return;

    graph->add_pass(
        "MeshVFX",
        RGPassType::Graphics,
        [drawHandle, depthHandle](RGPassBuilder &builder, EngineContext *ctx) {
            builder.write_color(drawHandle);
            builder.write_depth(depthHandle, false /*load existing depth*/);

            if (ctx)
            {
                const DrawContext &dc = ctx->getMainDrawContext();
                std::unordered_set<VkBuffer> indexSet;
                std::unordered_set<VkBuffer> vertexSet;
                for (const auto &r : dc.MeshVfxSurfaces)
                {
                    if (r.indexBuffer) indexSet.insert(r.indexBuffer);
                    if (r.vertexBuffer) vertexSet.insert(r.vertexBuffer);
                }
                for (VkBuffer b : indexSet) builder.read_buffer(b, RGBufferUsage::IndexRead, 0, "meshvfx.index");
                for (VkBuffer b : vertexSet) builder.read_buffer(b, RGBufferUsage::StorageRead, 0, "meshvfx.vertex");
            }
        },
        [this, drawHandle, depthHandle](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            draw_mesh_vfx(cmd, ctx, res, drawHandle, depthHandle);
        }
    );
}

void MeshVfxPass::draw_mesh_vfx(VkCommandBuffer cmd,
                                EngineContext *context,
                                const RGPassResources &,
                                RGImageHandle,
                                RGImageHandle) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    if (!resourceManager || !deviceManager || !descriptorLayouts) return;

    const auto &dc = ctxLocal->getMainDrawContext();
    const auto &sceneData = ctxLocal->getSceneData();
    if (dc.MeshVfxSurfaces.empty()) return;

    AllocatedBuffer gpuSceneDataBuffer = resourceManager->create_buffer(
        sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, gpuSceneDataBuffer]() {
        resourceManager->destroy_buffer(gpuSceneDataBuffer);
    });

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(deviceManager->allocator(), gpuSceneDataBuffer.allocation, &allocInfo);
    auto *sceneUniformData = static_cast<GPUSceneData *>(allocInfo.pMappedData);
    *sceneUniformData = sceneData;
    vmaFlushAllocation(deviceManager->allocator(), gpuSceneDataBuffer.allocation, 0, sizeof(GPUSceneData));

    VkDescriptorSet globalDescriptor = ctxLocal->currentFrame->_frameDescriptors.allocate(
        deviceManager->device(), descriptorLayouts->gpuSceneDataLayout());
    DescriptorWriter writer;
    writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.update_set(deviceManager->device(), globalDescriptor);

    std::vector<const RenderObject *> draws;
    draws.reserve(dc.MeshVfxSurfaces.size());
    for (const auto &r : dc.MeshVfxSurfaces)
    {
        if (r.material != nullptr)
        {
            draws.push_back(&r);
        }
    }
    if (draws.empty()) return;

    auto view = sceneData.view;
    auto depth_of = [&](const RenderObject *r) {
        glm::vec4 c = r->transform * glm::vec4(r->bounds.origin, 1.f);
        float z = (view * c).z;
        return -z;
    };
    std::sort(draws.begin(), draws.end(), [&](const RenderObject *a, const RenderObject *b) {
        return depth_of(a) > depth_of(b);
    });

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport viewport{0.f, 0.f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.f, 1.f};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{{0, 0}, extent};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    MaterialPipeline *lastPipeline = nullptr;
    MaterialInstance *lastMaterial = nullptr;
    VkBuffer lastIndexBuffer = VK_NULL_HANDLE;

    auto draw = [&](const RenderObject &r) {
        if (r.material != lastMaterial)
        {
            lastMaterial = r.material;
            if (r.material->pipeline != lastPipeline)
            {
                lastPipeline = r.material->pipeline;
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->pipeline);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->layout, 0, 1,
                                        &globalDescriptor, 0, nullptr);
            }

            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->layout, 1, 1,
                                    &r.material->materialSet, 0, nullptr);
            if (ctxLocal->textures)
            {
                ctxLocal->textures->markSetUsed(r.material->materialSet, ctxLocal->frameIndex);
            }
        }

        if (r.indexBuffer != lastIndexBuffer)
        {
            lastIndexBuffer = r.indexBuffer;
            vkCmdBindIndexBuffer(cmd, r.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }

        GPUDrawPushConstants push{};
        push.worldMatrix = r.transform;
        {
            const glm::mat3 n = glm::transpose(glm::inverse(glm::mat3(r.transform)));
            push.normalMatrix[0] = glm::vec4(n[0], 0.0f);
            push.normalMatrix[1] = glm::vec4(n[1], 0.0f);
            push.normalMatrix[2] = glm::vec4(n[2], 0.0f);
        }
        push.vertexBuffer = r.vertexBufferAddress;
        push.objectID = r.objectID;
        vkCmdPushConstants(cmd, r.material->pipeline->layout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(GPUDrawPushConstants), &push);
        vkCmdDrawIndexed(cmd, r.indexCount, 1, r.firstIndex, 0, 0);
        if (ctxLocal->stats)
        {
            ctxLocal->stats->drawcall_count++;
            ctxLocal->stats->triangle_count += r.indexCount / 3;
        }
    };

    for (const RenderObject *obj : draws)
    {
        draw(*obj);
    }
}

void MeshVfxPass::cleanup()
{
    Logger::info("MeshVfxPass::cleanup()");
}

