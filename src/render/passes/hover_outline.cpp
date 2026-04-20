#include "hover_outline.h"

#include <algorithm>
#include <unordered_set>

#include "core/assets/manager.h"
#include "core/context.h"
#include "core/descriptor/descriptors.h"
#include "core/descriptor/manager.h"
#include "core/device/device.h"
#include "core/device/swapchain.h"
#include "core/frame/resources.h"
#include "core/picking/picking_system.h"
#include "core/pipeline/manager.h"
#include "core/pipeline/sampler.h"
#include "render/graph/graph.h"
#include "render/pipelines.h"
#include "scene/planet/planet_system.h"
#include "scene/vk_scene.h"

namespace
{
    constexpr VkFormat k_hover_outline_mask_format = VK_FORMAT_R8_UNORM;

    VkExtent2D half_extent(VkExtent2D extent)
    {
        extent.width = std::max(1u, (extent.width + 1u) / 2u);
        extent.height = std::max(1u, (extent.height + 1u) / 2u);
        return extent;
    }

    bool is_planet_owner(const EngineContext *context, RenderObject::OwnerType owner_type, std::string_view owner_name)
    {
        if (!context || !context->scene || owner_type != RenderObject::OwnerType::MeshInstance || owner_name.empty())
        {
            return false;
        }

        PlanetSystem *planets = context->scene->get_planet_system();
        return planets && planets->find_body_by_name(owner_name) != nullptr;
    }

    bool is_supported_hover_owner(RenderObject::OwnerType owner_type)
    {
        return owner_type == RenderObject::OwnerType::GLTFInstance ||
               owner_type == RenderObject::OwnerType::MeshInstance;
    }

    HoverOutlinePass::TargetMode target_mode_from_selection_level(PickingSystem::SelectionLevel level,
                                                                  HoverOutlinePass::TargetMode fallback)
    {
        switch (level)
        {
            case PickingSystem::SelectionLevel::Object:
                return HoverOutlinePass::TargetMode::Object;
            case PickingSystem::SelectionLevel::Member:
                return HoverOutlinePass::TargetMode::Member;
            case PickingSystem::SelectionLevel::Node:
                return HoverOutlinePass::TargetMode::Node;
            case PickingSystem::SelectionLevel::Primitive:
                return HoverOutlinePass::TargetMode::Primitive;
            case PickingSystem::SelectionLevel::None:
            default:
                return fallback;
        }
    }

    bool matches_instance_target(const RenderObject &draw, const PickingSystem::PickInfo &pick)
    {
        return draw.ownerType == pick.ownerType &&
               !draw.ownerName.empty() &&
               draw.ownerName == pick.ownerName;
    }

    bool matches_object_target(const RenderObject &draw,
                               const PickingSystem &picking,
                               const PickingSystem::PickInfo &pick)
    {
        if (pick.objectName.empty() || draw.ownerName.empty())
        {
            return false;
        }

        const PickingSystem::OwnerBindingView binding = picking.resolve_owner_binding(draw.ownerType, draw.ownerName);
        return !binding.object_name.empty() && binding.object_name == pick.objectName;
    }

    bool matches_member_target(const RenderObject &draw,
                               const PickingSystem &picking,
                               const PickingSystem::PickInfo &pick)
    {
        if (pick.objectName.empty() || pick.memberName.empty() || draw.ownerName.empty())
        {
            return false;
        }

        const PickingSystem::OwnerBindingView binding = picking.resolve_owner_binding(draw.ownerType, draw.ownerName);
        return !binding.object_name.empty() &&
               !binding.member_name.empty() &&
               binding.object_name == pick.objectName &&
               binding.member_name == pick.memberName;
    }

    bool matches_node_target(const RenderObject &draw, const PickingSystem::PickInfo &pick)
    {
        return matches_instance_target(draw, pick) &&
               pick.ownerType == RenderObject::OwnerType::GLTFInstance &&
               pick.node != nullptr &&
               draw.sourceNode == pick.node;
    }

    bool matches_primitive_target(const RenderObject &draw, const PickingSystem::PickInfo &pick)
    {
        if (!matches_instance_target(draw, pick))
        {
            return false;
        }

        switch (pick.ownerType)
        {
            case RenderObject::OwnerType::MeshInstance:
                return draw.surfaceIndex == pick.surfaceIndex;

            case RenderObject::OwnerType::GLTFInstance:
            {
                if (pick.node != nullptr && draw.sourceNode != pick.node)
                {
                    return false;
                }

                const bool has_explicit_surface =
                    pick.indexCount > 0 || pick.firstIndex > 0 || pick.surfaceIndex > 0;
                return !has_explicit_surface || draw.surfaceIndex == pick.surfaceIndex;
            }

            default:
                break;
        }

        return false;
    }

    HoverOutlinePass::TargetMode resolve_channel_target_mode(const PickingSystem::PickInfo &pick,
                                                             const HoverOutlinePass::ChannelSettings &channel)
    {
        return channel.use_pick_selection_level
                   ? target_mode_from_selection_level(pick.selectionLevel, channel.target_mode)
                   : channel.target_mode;
    }

    std::vector<const RenderObject *> collect_outline_draws(const EngineContext *context,
                                                            const DrawContext &draw_context,
                                                            const PickingSystem &picking,
                                                            const PickingSystem::PickInfo &pick,
                                                            const HoverOutlinePass::ChannelSettings &channel)
    {
        std::vector<const RenderObject *> draws{};
        if (!pick.valid ||
            pick.kind != PickingSystem::PickInfo::Kind::SceneObject ||
            !is_supported_hover_owner(pick.ownerType) ||
            pick.ownerName.empty() ||
            is_planet_owner(context, pick.ownerType, pick.ownerName))
        {
            return draws;
        }

        const HoverOutlinePass::TargetMode mode = resolve_channel_target_mode(pick, channel);
        draws.reserve(draw_context.OpaqueSurfaces.size());
        for (const RenderObject &draw : draw_context.OpaqueSurfaces)
        {
            if (is_planet_owner(context, draw.ownerType, draw.ownerName))
            {
                continue;
            }

            bool matches = false;
            switch (mode)
            {
                case HoverOutlinePass::TargetMode::Primitive:
                    matches = matches_primitive_target(draw, pick);
                    break;
                case HoverOutlinePass::TargetMode::Node:
                    matches = matches_node_target(draw, pick);
                    break;
                case HoverOutlinePass::TargetMode::Member:
                    matches = matches_member_target(draw, picking, pick);
                    break;
                case HoverOutlinePass::TargetMode::Object:
                    matches = matches_object_target(draw, picking, pick);
                    break;
            }
            if (matches)
            {
                draws.push_back(&draw);
            }
        }

        return draws;
    }
} // namespace

void HoverOutlinePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getSwapchain() || !_context->getAssets() || _context->pbrMaterialLayout == VK_NULL_HANDLE)
    {
        return;
    }

    _singleImageLayout = _context->getDescriptorLayouts()->singleImageLayout();

    VkPushConstantRange mask_pcr{};
    mask_pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    mask_pcr.offset = 0;
    mask_pcr.size = sizeof(GPUDrawPushConstants);

    GraphicsPipelineCreateInfo mask_info{};
    mask_info.vertexShaderPath = _context->getAssets()->shaderPath("mesh.vert.spv");
    mask_info.fragmentShaderPath = _context->getAssets()->shaderPath("hover_outline_mask.frag.spv");
    mask_info.setLayouts = {
        _context->getDescriptorLayouts()->gpuSceneDataLayout(),
        _context->pbrMaterialLayout,
    };
    mask_info.pushConstants = {mask_pcr};
    mask_info.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        b.enable_depthtest(false, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_color_attachment_format(k_hover_outline_mask_format);
        if (_context && _context->getSwapchain())
        {
            b.set_depth_format(_context->getSwapchain()->depthImage().imageFormat);
        }
    };
    _context->pipelines->createGraphicsPipeline("hover_outline.mask", mask_info);

    VkPushConstantRange blur_pcr{};
    blur_pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    blur_pcr.offset = 0;
    blur_pcr.size = sizeof(BlurPush);

    GraphicsPipelineCreateInfo blur_info{};
    blur_info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    blur_info.fragmentShaderPath = _context->getAssets()->shaderPath("hover_outline_blur.frag.spv");
    blur_info.setLayouts = {_singleImageLayout};
    blur_info.pushConstants = {blur_pcr};
    blur_info.configure = [](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.disable_blending();
        b.set_color_attachment_format(k_hover_outline_mask_format);
    };
    _context->pipelines->createGraphicsPipeline("hover_outline.blur", blur_info);

    VkPushConstantRange composite_pcr{};
    composite_pcr.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    composite_pcr.offset = 0;
    composite_pcr.size = sizeof(CompositePush);

    GraphicsPipelineCreateInfo composite_info{};
    composite_info.vertexShaderPath = _context->getAssets()->shaderPath("fullscreen.vert.spv");
    composite_info.fragmentShaderPath = _context->getAssets()->shaderPath("hover_outline_composite.frag.spv");
    composite_info.setLayouts = {
        _singleImageLayout,
        _singleImageLayout,
        _singleImageLayout,
        _singleImageLayout,
        _singleImageLayout,
    };
    composite_info.pushConstants = {composite_pcr};
    composite_info.configure = [this](PipelineBuilder &b)
    {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_depthtest();
        b.disable_blending();
        if (_context && _context->getSwapchain())
        {
            b.set_color_attachment_format(_context->getSwapchain()->swapchainImageFormat());
        }
    };
    _context->pipelines->createGraphicsPipeline("hover_outline.composite", composite_info);
}

void HoverOutlinePass::cleanup()
{
    _maskPipeline = VK_NULL_HANDLE;
    _maskPipelineLayout = VK_NULL_HANDLE;
    _blurPipeline = VK_NULL_HANDLE;
    _blurPipelineLayout = VK_NULL_HANDLE;
    _compositePipeline = VK_NULL_HANDLE;
    _compositePipelineLayout = VK_NULL_HANDLE;
    _singleImageLayout = VK_NULL_HANDLE;
}

void HoverOutlinePass::execute(VkCommandBuffer)
{
    // Executed through the render graph.
}

RGImageHandle HoverOutlinePass::register_graph(RenderGraph *graph, RGImageHandle ldrInput, RGImageHandle depthHandle)
{
    if (!graph || !ldrInput.valid() || !depthHandle.valid() || !_context || !_settings.enabled || !_context->picking)
    {
        return ldrInput;
    }

    PickingSystem &picking = *_context->picking;
    const DrawContext &draw_context = _context->getMainDrawContext();
    const PickingSystem::PickInfo &hover_pick = picking.hover_pick();
    const PickingSystem::PickInfo &selection_pick = picking.last_pick();

    const bool hover_suppressed = _settings.suppress_hover_when_selected && selection_pick.valid;
    const std::vector<const RenderObject *> hover_draws =
        (_settings.hover.enabled && !hover_suppressed)
            ? collect_outline_draws(_context, draw_context, picking, hover_pick, _settings.hover)
            : std::vector<const RenderObject *>{};
    const std::vector<const RenderObject *> selection_draws =
        _settings.selection.enabled
            ? collect_outline_draws(_context, draw_context, picking, selection_pick, _settings.selection)
            : std::vector<const RenderObject *>{};

    if (hover_draws.empty() && selection_draws.empty())
    {
        return ldrInput;
    }

    const VkExtent2D draw_extent = _context->getDrawExtent();
    const VkExtent2D blur_extent = _settings.half_resolution_blur ? half_extent(draw_extent) : draw_extent;

    RGImageDesc mask_desc{};
    mask_desc.name = "hover_outline.mask";
    mask_desc.format = k_hover_outline_mask_format;
    mask_desc.extent = draw_extent;
    mask_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

    RGImageDesc blur_desc = mask_desc;
    blur_desc.extent = blur_extent;

    RGImageDesc ldr_desc{};
    ldr_desc.name = "ldr.hover_outline";
    ldr_desc.format = _context->getSwapchain()->swapchainImageFormat();
    ldr_desc.extent = draw_extent;
    ldr_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                     VK_IMAGE_USAGE_SAMPLED_BIT |
                     VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    mask_desc.name = "hover_outline.hover.mask";
    RGImageHandle hover_raw_mask = graph->create_image(mask_desc);
    blur_desc.name = "hover_outline.hover.blur_x";
    RGImageHandle hover_blur_x = graph->create_image(blur_desc);
    blur_desc.name = "hover_outline.hover.blur_y";
    RGImageHandle hover_blur_y = graph->create_image(blur_desc);

    mask_desc.name = "hover_outline.selection.mask";
    RGImageHandle selection_raw_mask = graph->create_image(mask_desc);
    blur_desc.name = "hover_outline.selection.blur_x";
    RGImageHandle selection_blur_x = graph->create_image(blur_desc);
    blur_desc.name = "hover_outline.selection.blur_y";
    RGImageHandle selection_blur_y = graph->create_image(blur_desc);

    RGImageHandle composite_output = graph->create_image(ldr_desc);

    graph->add_pass(
        "HoverOutline.HoverMask",
        RGPassType::Graphics,
        [hover_raw_mask, depthHandle, hover_draws](RGPassBuilder &builder, EngineContext *)
        {
            builder.write_color(hover_raw_mask, true);
            builder.write_depth(depthHandle, false);

            std::unordered_set<VkBuffer> index_set{};
            std::unordered_set<VkBuffer> vertex_set{};
            index_set.reserve(hover_draws.size());
            vertex_set.reserve(hover_draws.size());

            for (const RenderObject *draw : hover_draws)
            {
                if (!draw)
                {
                    continue;
                }
                if (draw->indexBuffer) index_set.insert(draw->indexBuffer);
                if (draw->vertexBuffer) vertex_set.insert(draw->vertexBuffer);
            }

            for (VkBuffer buffer : index_set)
            {
                builder.read_buffer(buffer, RGBufferUsage::IndexRead, 0, "hover_outline.index");
            }
            for (VkBuffer buffer : vertex_set)
            {
                builder.read_buffer(buffer, RGBufferUsage::StorageRead, 0, "hover_outline.vertex");
            }
        },
        [this, hover_draws](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx)
        {
            draw_mask(cmd, ctx, hover_draws);
        });

    graph->add_pass(
        "HoverOutline.HoverBlurX",
        RGPassType::Graphics,
        [hover_raw_mask, hover_blur_x](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(hover_raw_mask, RGImageUsage::SampledFragment);
            builder.write_color(hover_blur_x, true);
        },
        [this, hover_raw_mask, blur_extent](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
        {
            draw_blur(cmd, ctx, resources, hover_raw_mask, glm::vec2(1.0f, 0.0f), blur_extent, _settings.hover.blur_radius_px);
        });

    graph->add_pass(
        "HoverOutline.HoverBlurY",
        RGPassType::Graphics,
        [hover_blur_x, hover_blur_y](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(hover_blur_x, RGImageUsage::SampledFragment);
            builder.write_color(hover_blur_y, true);
        },
        [this, hover_blur_x, blur_extent](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
        {
            draw_blur(cmd, ctx, resources, hover_blur_x, glm::vec2(0.0f, 1.0f), blur_extent, _settings.hover.blur_radius_px);
        });

    graph->add_pass(
        "HoverOutline.SelectionMask",
        RGPassType::Graphics,
        [selection_raw_mask, depthHandle, selection_draws](RGPassBuilder &builder, EngineContext *)
        {
            builder.write_color(selection_raw_mask, true);
            builder.write_depth(depthHandle, false);

            std::unordered_set<VkBuffer> index_set{};
            std::unordered_set<VkBuffer> vertex_set{};
            index_set.reserve(selection_draws.size());
            vertex_set.reserve(selection_draws.size());

            for (const RenderObject *draw : selection_draws)
            {
                if (!draw)
                {
                    continue;
                }
                if (draw->indexBuffer) index_set.insert(draw->indexBuffer);
                if (draw->vertexBuffer) vertex_set.insert(draw->vertexBuffer);
            }

            for (VkBuffer buffer : index_set)
            {
                builder.read_buffer(buffer, RGBufferUsage::IndexRead, 0, "hover_outline.index");
            }
            for (VkBuffer buffer : vertex_set)
            {
                builder.read_buffer(buffer, RGBufferUsage::StorageRead, 0, "hover_outline.vertex");
            }
        },
        [this, selection_draws](VkCommandBuffer cmd, const RGPassResources &, EngineContext *ctx)
        {
            draw_mask(cmd, ctx, selection_draws);
        });

    graph->add_pass(
        "HoverOutline.SelectionBlurX",
        RGPassType::Graphics,
        [selection_raw_mask, selection_blur_x](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(selection_raw_mask, RGImageUsage::SampledFragment);
            builder.write_color(selection_blur_x, true);
        },
        [this, selection_raw_mask, blur_extent](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
        {
            draw_blur(cmd,
                      ctx,
                      resources,
                      selection_raw_mask,
                      glm::vec2(1.0f, 0.0f),
                      blur_extent,
                      _settings.selection.blur_radius_px);
        });

    graph->add_pass(
        "HoverOutline.SelectionBlurY",
        RGPassType::Graphics,
        [selection_blur_x, selection_blur_y](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(selection_blur_x, RGImageUsage::SampledFragment);
            builder.write_color(selection_blur_y, true);
        },
        [this, selection_blur_x, blur_extent](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
        {
            draw_blur(cmd,
                      ctx,
                      resources,
                      selection_blur_x,
                      glm::vec2(0.0f, 1.0f),
                      blur_extent,
                      _settings.selection.blur_radius_px);
        });

    graph->add_pass(
        "HoverOutline.Composite",
        RGPassType::Graphics,
        [ldrInput, hover_raw_mask, hover_blur_y, selection_raw_mask, selection_blur_y, composite_output](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(ldrInput, RGImageUsage::SampledFragment);
            builder.read(hover_raw_mask, RGImageUsage::SampledFragment);
            builder.read(hover_blur_y, RGImageUsage::SampledFragment);
            builder.read(selection_raw_mask, RGImageUsage::SampledFragment);
            builder.read(selection_blur_y, RGImageUsage::SampledFragment);
            builder.write_color(composite_output, true);
        },
        [this, ldrInput, hover_raw_mask, hover_blur_y, selection_raw_mask, selection_blur_y](VkCommandBuffer cmd,
                                                                                               const RGPassResources &resources,
                                                                                               EngineContext *ctx)
        {
            draw_composite(cmd, ctx, resources, ldrInput, hover_raw_mask, hover_blur_y, selection_raw_mask, selection_blur_y);
        });

    return composite_output;
}

void HoverOutlinePass::draw_mask(VkCommandBuffer cmd,
                                 EngineContext *context,
                                 const std::vector<const RenderObject *> &draws) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->pipelines || draws.empty())
    {
        return;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (!ctxLocal->pipelines->getGraphics("hover_outline.mask", pipeline, layout))
    {
        return;
    }

    const VkDescriptorSet scene_set = ctxLocal->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &scene_set, 0, nullptr);

    VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport viewport{0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f};
    VkRect2D scissor{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    MaterialInstance *last_material = nullptr;
    VkBuffer last_index_buffer = VK_NULL_HANDLE;

    for (const RenderObject *draw : draws)
    {
        if (!draw || !draw->material || draw->material->materialSet == VK_NULL_HANDLE || draw->indexBuffer == VK_NULL_HANDLE)
        {
            continue;
        }

        if (draw->material != last_material)
        {
            last_material = draw->material;
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 1, 1, &draw->material->materialSet, 0, nullptr);
        }

        if (draw->indexBuffer != last_index_buffer)
        {
            last_index_buffer = draw->indexBuffer;
            vkCmdBindIndexBuffer(cmd, draw->indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }

        GPUDrawPushConstants push{};
        push.worldMatrix = draw->transform;
        {
            const glm::mat3 normal = glm::transpose(glm::inverse(glm::mat3(draw->transform)));
            push.normalMatrix[0] = glm::vec4(normal[0], 0.0f);
            push.normalMatrix[1] = glm::vec4(normal[1], 0.0f);
            push.normalMatrix[2] = glm::vec4(normal[2], 0.0f);
        }
        push.vertexBuffer = draw->vertexBufferAddress;
        push.objectID = draw->objectID;

        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(GPUDrawPushConstants), &push);
        vkCmdDrawIndexed(cmd, draw->indexCount, 1, draw->firstIndex, 0, 0);
    }
}

void HoverOutlinePass::draw_blur(VkCommandBuffer cmd,
                                 EngineContext *context,
                                 const RGPassResources &resources,
                                 RGImageHandle inputHandle,
                                 const glm::vec2 &direction,
                                 VkExtent2D render_extent,
                                 float radius_px) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame || !ctxLocal->getDevice() || !ctxLocal->getSamplers() || !_singleImageLayout)
    {
        return;
    }

    const VkImageView src_view = resources.image_view(inputHandle);
    if (src_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (!ctxLocal->pipelines || !ctxLocal->pipelines->getGraphics("hover_outline.blur", pipeline, layout))
    {
        return;
    }

    const VkDevice device = ctxLocal->getDevice()->device();
    VkDescriptorSet set = ctxLocal->currentFrame->_frameDescriptors.allocate(device, _singleImageLayout);
    DescriptorWriter writer;
    writer.write_image(0, src_view, ctxLocal->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, set);

    BlurPush push{};
    const VkExtent2D draw_extent = ctxLocal->getDrawExtent();
    push.inverse_extent.x = draw_extent.width > 0 ? 1.0f / static_cast<float>(draw_extent.width) : 0.0f;
    push.inverse_extent.y = draw_extent.height > 0 ? 1.0f / static_cast<float>(draw_extent.height) : 0.0f;
    push.direction = direction;
    push.radius_px = std::max(radius_px, 0.5f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &set, 0, nullptr);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(BlurPush), &push);

    VkViewport viewport{0.0f, 0.0f, static_cast<float>(render_extent.width), static_cast<float>(render_extent.height),
                        0.0f, 1.0f};
    VkRect2D scissor{{0, 0}, render_extent};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void HoverOutlinePass::draw_composite(VkCommandBuffer cmd,
                                      EngineContext *context,
                                      const RGPassResources &resources,
                                      RGImageHandle ldrInput,
                                      RGImageHandle hoverRawMask,
                                      RGImageHandle hoverBlurredMask,
                                      RGImageHandle selectionRawMask,
                                      RGImageHandle selectionBlurredMask) const
{
    EngineContext *ctxLocal = context ? context : _context;
    if (!ctxLocal || !ctxLocal->currentFrame || !ctxLocal->getDevice() || !ctxLocal->getSamplers() || !_singleImageLayout)
    {
        return;
    }

    const VkImageView ldr_view = resources.image_view(ldrInput);
    const VkImageView hover_raw_view = resources.image_view(hoverRawMask);
    const VkImageView hover_blurred_view = resources.image_view(hoverBlurredMask);
    const VkImageView selection_raw_view = resources.image_view(selectionRawMask);
    const VkImageView selection_blurred_view = resources.image_view(selectionBlurredMask);
    if (ldr_view == VK_NULL_HANDLE ||
        hover_raw_view == VK_NULL_HANDLE ||
        hover_blurred_view == VK_NULL_HANDLE ||
        selection_raw_view == VK_NULL_HANDLE ||
        selection_blurred_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    if (!ctxLocal->pipelines || !ctxLocal->pipelines->getGraphics("hover_outline.composite", pipeline, layout))
    {
        return;
    }

    const VkDevice device = ctxLocal->getDevice()->device();
    VkDescriptorSet base_set = ctxLocal->currentFrame->_frameDescriptors.allocate(device, _singleImageLayout);
    VkDescriptorSet hover_raw_set = ctxLocal->currentFrame->_frameDescriptors.allocate(device, _singleImageLayout);
    VkDescriptorSet hover_blurred_set = ctxLocal->currentFrame->_frameDescriptors.allocate(device, _singleImageLayout);
    VkDescriptorSet selection_raw_set = ctxLocal->currentFrame->_frameDescriptors.allocate(device, _singleImageLayout);
    VkDescriptorSet selection_blurred_set = ctxLocal->currentFrame->_frameDescriptors.allocate(device, _singleImageLayout);

    DescriptorWriter writer;
    writer.write_image(0, ldr_view, ctxLocal->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, base_set);

    writer.clear();
    writer.write_image(0, hover_raw_view, ctxLocal->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, hover_raw_set);

    writer.clear();
    writer.write_image(0, hover_blurred_view, ctxLocal->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, hover_blurred_set);

    writer.clear();
    writer.write_image(0, selection_raw_view, ctxLocal->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, selection_raw_set);

    writer.clear();
    writer.write_image(0, selection_blurred_view, ctxLocal->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(device, selection_blurred_set);

    CompositePush push{};
    push.hover_color_intensity = glm::vec4(_settings.hover.color, std::max(_settings.hover.intensity, 0.0f));
    push.selection_color_intensity = glm::vec4(_settings.selection.color, std::max(_settings.selection.intensity, 0.0f));
    push.params.x = std::max(_settings.hover.outline_width_px, 0.25f);
    push.params.y = std::max(_settings.hover.blur_radius_px, 0.5f);
    push.params.z = std::max(_settings.selection.outline_width_px, 0.25f);
    push.params.w = std::max(_settings.selection.blur_radius_px, 0.5f);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &base_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 1, 1, &hover_raw_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 2, 1, &hover_blurred_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 3, 1, &selection_raw_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 4, 1, &selection_blurred_set, 0, nullptr);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(CompositePush), &push);

    const VkExtent2D extent = ctxLocal->getDrawExtent();
    VkViewport viewport{0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f};
    VkRect2D scissor{{0, 0}, extent};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
