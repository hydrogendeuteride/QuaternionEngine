#include "transparent.h"

#include <algorithm>
#include <unordered_set>

#include "core/assets/ibl_manager.h"
#include "core/assets/texture_cache.h"
#include "core/pipeline/sampler.h"
#include "scene/vk_scene.h"
#include "core/device/swapchain.h"
#include "core/context.h"
#include "core/device/resource.h"
#include "core/device/device.h"
#include "core/descriptor/manager.h"
#include "core/frame/resources.h"
#include "render/graph/graph.h"

void TransparentPass::init(EngineContext *context)
{
    _context = context;
    // Create fallback images
    const uint32_t pixel = 0x00000000u;
    _fallbackIbl2D = _context->getResources()->create_image(&pixel, VkExtent3D{1,1,1},
                                                            VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);
    _fallbackBrdf2D = _context->getResources()->create_image(&pixel, VkExtent3D{1,1,1},
                                                             VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);
}

void TransparentPass::execute(VkCommandBuffer)
{
    // Executed through render graph.
}

void TransparentPass::register_graph(RenderGraph *graph, RGImageHandle drawHandle, RGImageHandle depthHandle)
{
    if (!graph || !drawHandle.valid() || !depthHandle.valid()) return;

    graph->add_pass(
        "Transparent",
        RGPassType::Graphics,
        [drawHandle, depthHandle](RGPassBuilder &builder, EngineContext *ctx) {
            // Draw transparent to the HDR target with depth testing against the existing depth buffer.
            builder.write_color(drawHandle);
            builder.write_depth(depthHandle, false /*load existing depth*/);

            // Register external buffers used by draws
            if (ctx)
            {
                const DrawContext &dc = ctx->getMainDrawContext();
                std::unordered_set<VkBuffer> indexSet;
                std::unordered_set<VkBuffer> vertexSet;
                auto collect = [&](const std::vector<RenderObject> &v) {
                    for (const auto &r: v)
                    {
                        if (r.indexBuffer) indexSet.insert(r.indexBuffer);
                        if (r.vertexBuffer) vertexSet.insert(r.vertexBuffer);
                    }
                };
                collect(dc.TransparentSurfaces);
                for (VkBuffer b: indexSet) builder.read_buffer(b, RGBufferUsage::IndexRead, 0, "trans.index");
                for (VkBuffer b: vertexSet) builder.read_buffer(b, RGBufferUsage::StorageRead, 0, "trans.vertex");
            }
        },
        [this, drawHandle, depthHandle](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *ctx) {
            draw_transparent(cmd, ctx, res, drawHandle, depthHandle);
        }
    );
}

void TransparentPass::draw_transparent(VkCommandBuffer cmd,
                                       EngineContext *context,
                                       const RGPassResources &resources,
                                       RGImageHandle /*drawHandle*/,
                                       RGImageHandle /*depthHandle*/) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    if (!resourceManager || !deviceManager || !descriptorLayouts) return;

    const auto &dc = ctxLocal->getMainDrawContext();
    const auto &sceneData = ctxLocal->getSceneData();

    // Prepare per-frame scene UBO
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

    // Build IBL descriptor set (set=3) once for this pass
    VkDescriptorSet iblSet = VK_NULL_HANDLE;
    VkDescriptorSetLayout iblLayout = ctxLocal->ibl ? ctxLocal->ibl->descriptorLayout() : VK_NULL_HANDLE;
    VkImageView specView = VK_NULL_HANDLE, brdfView = VK_NULL_HANDLE;
    VkBuffer shBuf = VK_NULL_HANDLE; VkDeviceSize shSize = sizeof(glm::vec4)*9;
    if (iblLayout)
    {
        // Fallbacks: use black if any missing
        specView = (ctxLocal->ibl && ctxLocal->ibl->specular().imageView) ? ctxLocal->ibl->specular().imageView
                                                                          : _fallbackIbl2D.imageView;
        brdfView = (ctxLocal->ibl && ctxLocal->ibl->brdf().imageView)    ? ctxLocal->ibl->brdf().imageView
                                                                          : _fallbackBrdf2D.imageView;
        if (ctxLocal->ibl && ctxLocal->ibl->hasSH()) shBuf = ctxLocal->ibl->shBuffer().buffer;

        // If SH missing, allocate zero UBO for this frame
        AllocatedBuffer shZero{};
        if (shBuf == VK_NULL_HANDLE)
        {
            shZero = resourceManager->create_buffer(shSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
            std::memset(shZero.info.pMappedData, 0, shSize);
            vmaFlushAllocation(deviceManager->allocator(), shZero.allocation, 0, shSize);
            shBuf = shZero.buffer;
            ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, shZero]() { resourceManager->destroy_buffer(shZero); });
        }

        iblSet = ctxLocal->currentFrame->_frameDescriptors.allocate(deviceManager->device(), iblLayout);
        DescriptorWriter iw;
        iw.write_image(0, specView, ctxLocal->getSamplers()->defaultLinear(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                       VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        iw.write_image(1, brdfView, ctxLocal->getSamplers()->defaultLinear(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                       VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        iw.write_buffer(2, shBuf, shSize, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        iw.update_set(deviceManager->device(), iblSet);
    }

    // Sort transparent back-to-front using camera-space depth.
    // We approximate object depth by transforming the mesh bounds origin.
    // For better results consider using per-object center or per-draw depth range.
    std::vector<const RenderObject *> draws;
    draws.reserve(dc.TransparentSurfaces.size());
    for (const auto &r: dc.TransparentSurfaces) draws.push_back(&r);

    auto view = sceneData.view; // world -> view
    auto depthOf = [&](const RenderObject *r) {
        glm::vec4 c = r->transform * glm::vec4(r->bounds.origin, 1.f);
        float z = (view * c).z;
        return -z; // positive depth; larger = further
    };

    std::sort(draws.begin(), draws.end(), [&](const RenderObject *A, const RenderObject *B) {
        return depthOf(A) > depthOf(B); // far to near
    });

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport viewport{0.f, 0.f, (float) extent.width, (float) extent.height, 0.f, 1.f};
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
                if (iblSet)
                {
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->layout, 3, 1,
                                            &iblSet, 0, nullptr);
                }
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

    for (auto *pObj: draws) draw(*pObj);
}

void TransparentPass::cleanup()
{
    if (_context && _context->getResources())
    {
        if (_fallbackIbl2D.image) _context->getResources()->destroy_image(_fallbackIbl2D);
        if (_fallbackBrdf2D.image) _context->getResources()->destroy_image(_fallbackBrdf2D);
    }
    fmt::print("TransparentPass::cleanup()\n");
}
