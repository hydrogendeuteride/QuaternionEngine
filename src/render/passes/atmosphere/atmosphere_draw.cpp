#include "render/passes/atmosphere/atmosphere_internal.h"

using namespace atmosphere::detail;

void AtmospherePass::draw_cloud_low_res(VkCommandBuffer cmd,
                                        EngineContext *context,
                                        const RGPassResources &resources,
                                        VkExtent2D extent,
                                        RGImageHandle gbufPos,
                                        RGImageHandle transmittanceLut,
                                        RGImageHandle cloudLighting,
                                        RGImageHandle cloudSegment)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    VkImageView pos_view = resources.image_view(gbufPos);
    VkImageView lut_view = resources.image_view(transmittanceLut);
    VkImageView cloud_lighting_view = resources.image_view(cloudLighting);
    VkImageView cloud_segment_view = resources.image_view(cloudSegment);
    if (pos_view == VK_NULL_HANDLE || lut_view == VK_NULL_HANDLE ||
        cloud_lighting_view == VK_NULL_HANDLE || cloud_segment_view == VK_NULL_HANDLE)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    AtmosphereBodySelection body_selection{};
    if (ctx->scene)
    {
        if (PlanetSystem *planets = ctx->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctx, *ctx->scene, *planets, body_selection);
        }
    }

    const std::string_view terrain_height_dir =
        (body_selection.body && body_selection.body->terrain) ? body_selection.body->terrain_height_dir : std::string_view{};
    ensure_planet_height_textures(ctx, terrain_height_dir);

    VkImageView overlay_view = _cloudOverlayTex.imageView != VK_NULL_HANDLE ? _cloudOverlayTex.imageView : assets->fallbackBlackView();
    VkImageView noise_view = _cloudNoiseTex.imageView != VK_NULL_HANDLE ? _cloudNoiseTex.imageView : assets->fallbackWhiteView();
    VkImageView noise_view_3d = _cloudNoiseTex3D.imageView != VK_NULL_HANDLE ? _cloudNoiseTex3D.imageView : _cloudNoiseFallback3D.imageView;
    VkImageView jitter_view = _jitterNoiseTex.imageView != VK_NULL_HANDLE ? _jitterNoiseTex.imageView : _cloudNoiseFallback3D.imageView;
    std::array<VkImageView, 6> terrain_height_views{};
    terrain_height_views.fill(assets->fallbackBlackView());
    if (ctx->textures)
    {
        for (size_t i = 0; i < _planetHeightHandles.size(); ++i)
        {
            const uint32_t handle = _planetHeightHandles[i];
            if (handle == UINT32_MAX)
            {
                continue;
            }
            ctx->textures->markUsed(handle, ctx->frameIndex);
            VkImageView view = ctx->textures->imageView(handle);
            if (view != VK_NULL_HANDLE)
            {
                terrain_height_views[i] = view;
            }
        }
    }
    if (overlay_view == VK_NULL_HANDLE || noise_view == VK_NULL_HANDLE || noise_view_3d == VK_NULL_HANDLE ||
        jitter_view == VK_NULL_HANDLE || terrain_height_views[0] == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _lowResSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, pos_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, lut_view, ctx->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, overlay_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, noise_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(4, noise_view_3d, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(5, jitter_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    for (uint32_t i = 0; i < terrain_height_views.size(); ++i)
    {
        writer.write_image(6 + static_cast<int>(i), terrain_height_views[i], ctx->getSamplers()->linearClampEdge(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    }
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.cloud_lowres", _cloudLowResPipeline, _cloudLowResPipelineLayout))
    {
        return;
    }

    const AtmospherePush push = build_atmosphere_push(
        *ctx,
        body_selection,
        _jitterNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudOverlayTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex3D.imageView != VK_NULL_HANDLE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudLowResPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudLowResPipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudLowResPipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _cloudLowResPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void AtmospherePass::draw_cloud_temporal(VkCommandBuffer cmd,
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
                                         bool historyValid)
{
    (void)cloudLightingHistoryNext;
    (void)cloudSegmentHistoryNext;

    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    VkImageView cloud_lighting_current_view = resources.image_view(cloudLightingCurrent);
    VkImageView cloud_segment_current_view = resources.image_view(cloudSegmentCurrent);
    if (cloud_lighting_current_view == VK_NULL_HANDLE || cloud_segment_current_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkImageView cloud_lighting_history_prev_view = resources.image_view(cloudLightingHistoryPrev);
    VkImageView cloud_segment_history_prev_view = resources.image_view(cloudSegmentHistoryPrev);
    if (cloud_lighting_history_prev_view == VK_NULL_HANDLE)
    {
        cloud_lighting_history_prev_view = assets->fallbackBlackView();
    }
    if (cloud_segment_history_prev_view == VK_NULL_HANDLE)
    {
        cloud_segment_history_prev_view = assets->fallbackBlackView();
    }
    if (cloud_lighting_history_prev_view == VK_NULL_HANDLE || cloud_segment_history_prev_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _temporalSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, cloud_lighting_current_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, cloud_segment_current_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, cloud_lighting_history_prev_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, cloud_segment_history_prev_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.cloud_temporal", _cloudTemporalPipeline, _cloudTemporalPipelineLayout))
    {
        return;
    }

    CloudTemporalPush push{};
    push.previous_view_proj = previousViewProj;
    push.origin_delta_blend = glm::vec4(originDeltaLocal, 0.85f);
    push.viewport_params = glm::vec4(
        static_cast<float>(extent.width),
        static_cast<float>(extent.height),
        1.0f / static_cast<float>(std::max(1u, extent.width)),
        1.0f / static_cast<float>(std::max(1u, extent.height)));
    push.misc = glm::ivec4(historyValid ? 1 : 0, 0, 0, 0);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudTemporalPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudTemporalPipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudTemporalPipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _cloudTemporalPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void AtmospherePass::draw_cloud_upscale(VkCommandBuffer cmd,
                                        EngineContext *context,
                                        const RGPassResources &resources,
                                        VkExtent2D extent,
                                        RGImageHandle gbufPos,
                                        RGImageHandle cloudLightingLowRes,
                                        RGImageHandle cloudSegmentLowRes,
                                        RGImageHandle cloudLightingResolved,
                                        RGImageHandle cloudSegmentResolved)
{
    (void)cloudLightingResolved;
    (void)cloudSegmentResolved;

    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    VkImageView pos_view = resources.image_view(gbufPos);
    VkImageView cloud_lighting_lowres_view = resources.image_view(cloudLightingLowRes);
    VkImageView cloud_segment_lowres_view = resources.image_view(cloudSegmentLowRes);
    if (pos_view == VK_NULL_HANDLE || cloud_lighting_lowres_view == VK_NULL_HANDLE || cloud_segment_lowres_view == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    AtmosphereBodySelection body_selection{};
    if (ctx->scene)
    {
        if (PlanetSystem *planets = ctx->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctx, *ctx->scene, *planets, body_selection);
        }
    }

    const std::string_view terrain_height_dir =
        (body_selection.body && body_selection.body->terrain) ? body_selection.body->terrain_height_dir : std::string_view{};
    ensure_planet_height_textures(ctx, terrain_height_dir);
    std::array<VkImageView, 6> terrain_height_views{};
    terrain_height_views.fill(assets->fallbackBlackView());
    if (ctx->textures)
    {
        for (size_t i = 0; i < _planetHeightHandles.size(); ++i)
        {
            const uint32_t handle = _planetHeightHandles[i];
            if (handle == UINT32_MAX)
            {
                continue;
            }
            ctx->textures->markUsed(handle, ctx->frameIndex);
            VkImageView view = ctx->textures->imageView(handle);
            if (view != VK_NULL_HANDLE)
            {
                terrain_height_views[i] = view;
            }
        }
    }
    if (terrain_height_views[0] == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _upscaleSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, pos_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, cloud_lighting_lowres_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, cloud_segment_lowres_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    for (uint32_t i = 0; i < terrain_height_views.size(); ++i)
    {
        writer.write_image(3 + static_cast<int>(i), terrain_height_views[i], ctx->getSamplers()->linearClampEdge(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    }
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.cloud_upscale", _cloudUpscalePipeline, _cloudUpscalePipelineLayout))
    {
        return;
    }

    const AtmospherePush push = build_atmosphere_push(
        *ctx,
        body_selection,
        _jitterNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudOverlayTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex3D.imageView != VK_NULL_HANDLE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudUpscalePipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudUpscalePipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _cloudUpscalePipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _cloudUpscalePipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}

void AtmospherePass::draw_composite(VkCommandBuffer cmd,
                                    EngineContext *context,
                                    const RGPassResources &resources,
                                    VkExtent2D extent,
                                    RGImageHandle hdrInput,
                                    RGImageHandle gbufPos,
                                    RGImageHandle transmittanceLut,
                                    RGImageHandle cloudLightingResolved,
                                    RGImageHandle cloudSegmentResolved)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->currentFrame)
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    if (!assets)
    {
        return;
    }

    VkImageView hdr_view = resources.image_view(hdrInput);
    VkImageView pos_view = resources.image_view(gbufPos);
    VkImageView lut_view = resources.image_view(transmittanceLut);
    if (hdr_view == VK_NULL_HANDLE || pos_view == VK_NULL_HANDLE || lut_view == VK_NULL_HANDLE)
    {
        return;
    }

    AtmosphereBodySelection body_selection{};
    if (ctx->scene)
    {
        if (PlanetSystem *planets = ctx->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*ctx, *ctx->scene, *planets, body_selection);
        }
    }

    const std::string_view terrain_height_dir =
        (body_selection.body && body_selection.body->terrain) ? body_selection.body->terrain_height_dir : std::string_view{};
    ensure_planet_height_textures(ctx, terrain_height_dir);

    VkImageView overlay_view = _cloudOverlayTex.imageView != VK_NULL_HANDLE ? _cloudOverlayTex.imageView : assets->fallbackBlackView();
    VkImageView noise_view = _cloudNoiseTex.imageView != VK_NULL_HANDLE ? _cloudNoiseTex.imageView : assets->fallbackWhiteView();
    VkImageView noise_view_3d = _cloudNoiseTex3D.imageView != VK_NULL_HANDLE ? _cloudNoiseTex3D.imageView : _cloudNoiseFallback3D.imageView;
    VkImageView jitter_view = _jitterNoiseTex.imageView != VK_NULL_HANDLE ? _jitterNoiseTex.imageView : _cloudNoiseFallback3D.imageView;
    std::array<VkImageView, 6> terrain_height_views{};
    terrain_height_views.fill(assets->fallbackBlackView());
    if (ctx->textures)
    {
        for (size_t i = 0; i < _planetHeightHandles.size(); ++i)
        {
            const uint32_t handle = _planetHeightHandles[i];
            if (handle == UINT32_MAX)
            {
                continue;
            }
            ctx->textures->markUsed(handle, ctx->frameIndex);
            VkImageView view = ctx->textures->imageView(handle);
            if (view != VK_NULL_HANDLE)
            {
                terrain_height_views[i] = view;
            }
        }
    }
    VkImageView cloud_lighting_resolved_view = resources.image_view(cloudLightingResolved);
    VkImageView cloud_segment_resolved_view = resources.image_view(cloudSegmentResolved);
    if (cloud_lighting_resolved_view == VK_NULL_HANDLE)
    {
        cloud_lighting_resolved_view = assets->fallbackBlackView();
    }
    if (cloud_segment_resolved_view == VK_NULL_HANDLE)
    {
        cloud_segment_resolved_view = assets->fallbackBlackView();
    }
    if (overlay_view == VK_NULL_HANDLE || noise_view == VK_NULL_HANDLE || noise_view_3d == VK_NULL_HANDLE ||
        jitter_view == VK_NULL_HANDLE ||
        cloud_lighting_resolved_view == VK_NULL_HANDLE || cloud_segment_resolved_view == VK_NULL_HANDLE ||
        terrain_height_views[0] == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet scene_set = ctx->getOrCreateSceneDataDescriptor();
    if (scene_set == VK_NULL_HANDLE)
    {
        return;
    }

    VkDescriptorSet input_set = ctx->currentFrame->_frameDescriptors.allocate(ctx->getDevice()->device(), _compositeSetLayout);
    DescriptorWriter writer;
    writer.write_image(0, hdr_view, ctx->getSamplers()->defaultLinear(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(1, pos_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, lut_view, ctx->getSamplers()->linearClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, overlay_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(4, noise_view, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(5, cloud_lighting_resolved_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(6, cloud_segment_resolved_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(7, noise_view_3d, ctx->getSamplers()->linearRepeatClampEdge(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(8, jitter_view, ctx->getSamplers()->defaultNearest(),
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    for (uint32_t i = 0; i < terrain_height_views.size(); ++i)
    {
        writer.write_image(9 + static_cast<int>(i), terrain_height_views[i], ctx->getSamplers()->linearClampEdge(),
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    }
    writer.update_set(ctx->getDevice()->device(), input_set);

    if (!ctx->pipelines->getGraphics("atmosphere.composite", _compositePipeline, _compositePipelineLayout))
    {
        return;
    }

    const AtmospherePush push = build_atmosphere_push(
        *ctx,
        body_selection,
        _jitterNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudOverlayTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex.imageView != VK_NULL_HANDLE,
        _cloudNoiseTex3D.imageView != VK_NULL_HANDLE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _compositePipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _compositePipelineLayout, 0, 1, &scene_set, 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, _compositePipelineLayout, 1, 1, &input_set, 0, nullptr);
    vkCmdPushConstants(cmd, _compositePipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    set_fullscreen_viewport(cmd, extent);
    vkCmdDraw(cmd, 3, 1, 0, 0);
}
