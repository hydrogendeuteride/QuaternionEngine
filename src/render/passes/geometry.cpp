#include "geometry.h"

#include <chrono>
#include <unordered_set>

#include "core/frame/resources.h"
#include "core/assets/texture_cache.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/context.h"
#include "core/util/initializers.h"
#include "core/device/resource.h"

#include "vk_mem_alloc.h"
#include "scene/vk_scene.h"
#include "core/device/swapchain.h"
#include "render/graph/graph.h"

// Basic conservative frustum test against RenderObject AABB.
// Clip space uses Vulkan Z0 (0..w). Returns true if any part of the box is inside.
bool is_visible(const RenderObject &obj, const glm::mat4 &viewproj)
{
    const std::array<glm::vec3, 8> corners{
        glm::vec3{+1, +1, +1}, glm::vec3{+1, +1, -1}, glm::vec3{+1, -1, +1}, glm::vec3{+1, -1, -1},
        glm::vec3{-1, +1, +1}, glm::vec3{-1, +1, -1}, glm::vec3{-1, -1, +1}, glm::vec3{-1, -1, -1},
    };

    const glm::vec3 o = obj.bounds.origin;
    const glm::vec3 e = obj.bounds.extents;
    const glm::mat4 m = viewproj * obj.transform; // world -> clip

    glm::vec4 clip[8];
    for (int i = 0; i < 8; ++i)
    {
        const glm::vec3 p = o + corners[i] * e;
        clip[i] = m * glm::vec4(p, 1.f);
    }

    auto all_out = [&](auto pred) {
        for (int i = 0; i < 8; ++i)
        {
            if (!pred(clip[i])) return false;
        }
        return true;
    };

    // Clip volume in Vulkan (ZO): -w<=x<=w, -w<=y<=w, 0<=z<=w
    if (all_out([](const glm::vec4 &v) { return v.x < -v.w; })) return false; // left
    if (all_out([](const glm::vec4 &v) { return v.x >  v.w; })) return false; // right
    if (all_out([](const glm::vec4 &v) { return v.y < -v.w; })) return false; // bottom
    if (all_out([](const glm::vec4 &v) { return v.y >  v.w; })) return false; // top
    if (all_out([](const glm::vec4 &v) { return v.z <  0.0f; })) return false; // near (ZO)
    if (all_out([](const glm::vec4 &v) { return v.z >  v.w; })) return false; // far

    return true; // intersects or is fully inside
}

void GeometryPass::init(EngineContext *context)
{
    _context = context;
}

void GeometryPass::execute(VkCommandBuffer)
{
    // Geometry is executed via the render graph now.
}

void GeometryPass::register_graph(RenderGraph *graph,
                                  RGImageHandle gbufferPosition,
                                  RGImageHandle gbufferNormal,
                                  RGImageHandle gbufferAlbedo,
                                  RGImageHandle gbufferExtra,
                                  RGImageHandle idHandle,
                                  RGImageHandle depthHandle)
{
    if (!graph || !gbufferPosition.valid() || !gbufferNormal.valid() || !gbufferAlbedo.valid() ||
        !gbufferExtra.valid() ||
        !idHandle.valid() || !depthHandle.valid())
    {
        return;
    }

    graph->add_pass(
        "Geometry",
        RGPassType::Graphics,
        [gbufferPosition, gbufferNormal, gbufferAlbedo, gbufferExtra, idHandle, depthHandle](RGPassBuilder &builder, EngineContext *ctx)
        {
            VkClearValue clear{};
            clear.color = {{0.f, 0.f, 0.f, 0.f}};

            builder.write_color(gbufferPosition, true, clear);
            builder.write_color(gbufferNormal, true, clear);
            builder.write_color(gbufferAlbedo, true, clear);
            VkClearValue clearID{};
            clearID.color.uint32[0] = 0u;
            builder.write_color(idHandle, true, clearID);
            VkClearValue clearExtra{};
            clearExtra.color = {{1.f, 0.f, 0.f, 0.f}}; // AO=1, emissive=0
            builder.write_color(gbufferExtra, true, clearExtra);

            // Reverse-Z: clear depth to 0.0
            VkClearValue depthClear{};
            depthClear.depthStencil = {0.f, 0};
            builder.write_depth(depthHandle, true, depthClear);

            // Register read buffers used by all draw calls (index + vertex SSBO)
            if (ctx)
            {
                const DrawContext &dc = ctx->getMainDrawContext();
                // Collect unique buffers to avoid duplicates
                std::unordered_set<VkBuffer> indexSet;
                std::unordered_set<VkBuffer> vertexSet;
                indexSet.reserve(dc.OpaqueSurfaces.size() + dc.TransparentSurfaces.size());
                vertexSet.reserve(dc.OpaqueSurfaces.size() + dc.TransparentSurfaces.size());
                auto collect = [&](const std::vector<RenderObject>& v){
                    for (const auto &r : v)
                    {
                        if (r.indexBuffer) indexSet.insert(r.indexBuffer);
                        if (r.vertexBuffer) vertexSet.insert(r.vertexBuffer);
                    }
                };
                collect(dc.OpaqueSurfaces);
                collect(dc.TransparentSurfaces);

                for (VkBuffer b : indexSet)
                    builder.read_buffer(b, RGBufferUsage::IndexRead, 0, "geom.index");
                for (VkBuffer b : vertexSet)
                    builder.read_buffer(b, RGBufferUsage::StorageRead, 0, "geom.vertex");
            }
        },
        [this, gbufferPosition, gbufferNormal, gbufferAlbedo, gbufferExtra, idHandle, depthHandle](VkCommandBuffer cmd,
                                                                                     const RGPassResources &res,
                                                                                     EngineContext *ctx)
        {
            draw_geometry(cmd, ctx, res, gbufferPosition, gbufferNormal, gbufferAlbedo, gbufferExtra, idHandle, depthHandle);
        });
}

void GeometryPass::draw_geometry(VkCommandBuffer cmd,
                                 EngineContext *context,
                                 const RGPassResources &resources,
                                 RGImageHandle gbufferPosition,
                                 RGImageHandle gbufferNormal,
                                 RGImageHandle gbufferAlbedo,
                                 RGImageHandle /*gbufferExtra*/,
                                 RGImageHandle /*idHandle*/,
                                 RGImageHandle depthHandle) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame) return;

    ResourceManager *resourceManager = ctxLocal->getResources();
    DeviceManager *deviceManager = ctxLocal->getDevice();
    DescriptorManager *descriptorLayouts = ctxLocal->getDescriptorLayouts();
    if (!resourceManager || !deviceManager || !descriptorLayouts) return;

    VkImageView positionView = resources.image_view(gbufferPosition);
    VkImageView normalView = resources.image_view(gbufferNormal);
    VkImageView albedoView = resources.image_view(gbufferAlbedo);
    VkImageView depthView = resources.image_view(depthHandle);

    if (positionView == VK_NULL_HANDLE || normalView == VK_NULL_HANDLE ||
        albedoView == VK_NULL_HANDLE || depthView == VK_NULL_HANDLE)
    {
        return;
    }

    const auto& mainDrawContext = ctxLocal->getMainDrawContext();
    const auto& sceneData = ctxLocal->getSceneData();
    VkExtent2D drawExtent = ctxLocal->getDrawExtent();

    auto start = std::chrono::system_clock::now();

    std::vector<uint32_t> opaque_draws;
    opaque_draws.reserve(mainDrawContext.OpaqueSurfaces.size());

    for (int i = 0; i < mainDrawContext.OpaqueSurfaces.size(); i++)
    {
        if (is_visible(mainDrawContext.OpaqueSurfaces[i], sceneData.viewproj))
        {
            opaque_draws.push_back(i);
        }
    }

    // Texture visibility-driven residency
    if (ctxLocal->textures && !opaque_draws.empty())
    {
        std::unordered_set<VkDescriptorSet> seen;
        seen.reserve(opaque_draws.size());
        for (uint32_t idx : opaque_draws)
        {
            const RenderObject &r = mainDrawContext.OpaqueSurfaces[idx];
            VkDescriptorSet set = r.material ? r.material->materialSet : VK_NULL_HANDLE;
            if (set != VK_NULL_HANDLE && seen.insert(set).second)
            {
                ctxLocal->textures->markSetUsed(set, ctxLocal->frameIndex);
            }
        }
    }

    std::sort(opaque_draws.begin(), opaque_draws.end(), [&](const auto &iA, const auto &iB)
    {
        const RenderObject &A = mainDrawContext.OpaqueSurfaces[iA];
        const RenderObject &B = mainDrawContext.OpaqueSurfaces[iB];
        if (A.material == B.material)
        {
            return A.indexBuffer < B.indexBuffer;
        }
        return A.material < B.material;
    });

    // Dynamic rendering is now begun by the RenderGraph using the declared attachments.

    AllocatedBuffer gpuSceneDataBuffer = resourceManager->create_buffer(sizeof(GPUSceneData),
                                                                        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                        VMA_MEMORY_USAGE_CPU_TO_GPU);

    ctxLocal->currentFrame->_deletionQueue.push_function([resourceManager, gpuSceneDataBuffer]()
    {
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

    MaterialPipeline *lastPipeline = nullptr;
    MaterialInstance *lastMaterial = nullptr;
    VkBuffer lastIndexBuffer = VK_NULL_HANDLE;

    auto draw = [&](const RenderObject &r)
    {
        if (r.material != lastMaterial)
        {
            lastMaterial = r.material;
            if (r.material->pipeline != lastPipeline)
            {
                lastPipeline = r.material->pipeline;
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->pipeline);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->layout, 0, 1,
                                        &globalDescriptor, 0, nullptr);

                VkViewport viewport{};
                viewport.x = 0;
                viewport.y = 0;
                viewport.width = static_cast<float>(drawExtent.width);
                viewport.height = static_cast<float>(drawExtent.height);
                viewport.minDepth = 0.f;
                viewport.maxDepth = 1.f;

                vkCmdSetViewport(cmd, 0, 1, &viewport);

                VkRect2D scissor{};
                scissor.offset.x = 0;
                scissor.offset.y = 0;
                scissor.extent.width = drawExtent.width;
                scissor.extent.height = drawExtent.height;

                vkCmdSetScissor(cmd, 0, 1, &scissor);
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
        GPUDrawPushConstants push_constants{};
        push_constants.worldMatrix = r.transform;
        {
            const glm::mat3 n = glm::transpose(glm::inverse(glm::mat3(r.transform)));
            push_constants.normalMatrix[0] = glm::vec4(n[0], 0.0f);
            push_constants.normalMatrix[1] = glm::vec4(n[1], 0.0f);
            push_constants.normalMatrix[2] = glm::vec4(n[2], 0.0f);
        }
        push_constants.vertexBuffer = r.vertexBufferAddress;
        push_constants.objectID = r.objectID;

        vkCmdPushConstants(cmd, r.material->pipeline->layout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(GPUDrawPushConstants), &push_constants);

        vkCmdDrawIndexed(cmd, r.indexCount, 1, r.firstIndex, 0, 0);

        if (ctxLocal->stats)
        {
            ctxLocal->stats->drawcall_count++;
            ctxLocal->stats->triangle_count += r.indexCount / 3;
        }
    };

    if (ctxLocal->stats)
    {
        ctxLocal->stats->drawcall_count = 0;
        ctxLocal->stats->triangle_count = 0;
    }

    for (auto &r: opaque_draws)
    {
        draw(mainDrawContext.OpaqueSurfaces[r]);
    }

    // Transparent surfaces are rendered in a separate Transparent pass after lighting.

    // RenderGraph will end dynamic rendering for this pass.

    auto end = std::chrono::system_clock::now();

    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    if (ctxLocal->stats)
    {
        ctxLocal->stats->mesh_draw_time = elapsed.count() / 1000.f;
    }
}

void GeometryPass::cleanup()
{
    fmt::print("GeometryPass::cleanup()\n");
}
