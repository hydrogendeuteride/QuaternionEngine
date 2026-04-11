#include "render/passes/atmosphere/atmosphere_internal.h"

using namespace atmosphere::detail;

RGImageHandle AtmospherePass::register_graph(RenderGraph *graph, RGImageHandle hdrInput, RGImageHandle gbufPos)
{
    if (!graph || !hdrInput.valid() || !gbufPos.valid())
    {
        return hdrInput;
    }
    if (!_context || !(_context->enableAtmosphere || _context->enablePlanetClouds))
    {
        return hdrInput;
    }

    ensure_cloud_textures(_context);

    AtmosphereBodySelection body_selection{};
    if (_context->scene)
    {
        if (PlanetSystem *planets = _context->scene->get_planet_system(); planets && planets->enabled())
        {
            (void)find_atmosphere_body(*_context, *_context->scene, *planets, body_selection);
        }
    }

    ensure_transmittance_lut_image();
    if (_transmittanceLut.image == VK_NULL_HANDLE || _transmittanceLut.imageView == VK_NULL_HANDLE)
    {
        return hdrInput;
    }

    const AtmosphereSettings &settings = _context->atmosphere;
    const float atmosphere_height = std::max(0.0f, settings.atmosphereHeightM);
    const float atmosphere_radius = (body_selection.radius_m > 0.0f && atmosphere_height > 0.0f)
        ? (body_selection.radius_m + atmosphere_height)
        : 0.0f;

    AtmosphereLutPush lut_push{};
    lut_push.radii_heights = glm::vec4(
        body_selection.radius_m,
        atmosphere_radius,
        std::max(1.0f, settings.rayleighScaleHeightM),
        std::max(1.0f, settings.mieScaleHeightM));
    lut_push.misc = glm::ivec4(std::clamp(settings.lightSteps, 2, 256), 0, 0, 0);

    TransmittanceLutSnapshot next_lut_snapshot{};
    next_lut_snapshot.radiiHeights = lut_push.radii_heights;
    next_lut_snapshot.lightSteps = lut_push.misc.x;

    auto lut_snapshot_matches = [](const TransmittanceLutSnapshot &lhs, const TransmittanceLutSnapshot &rhs) {
        return nearly_equal_vec4(lhs.radiiHeights, rhs.radiiHeights) && lhs.lightSteps == rhs.lightSteps;
    };

    const bool transmittance_dirty = !_transmittanceLutValid || !lut_snapshot_matches(_transmittanceLutParams, next_lut_snapshot);

    RGImportedImageDesc lut_desc{};
    lut_desc.name = "atmosphere.lut.transmittance";
    lut_desc.image = _transmittanceLut.image;
    lut_desc.imageView = _transmittanceLut.imageView;
    lut_desc.format = _transmittanceLut.imageFormat;
    lut_desc.extent = VkExtent2D{_transmittanceLut.imageExtent.width, _transmittanceLut.imageExtent.height};
    lut_desc.currentLayout = _transmittanceLutState.layout;
    lut_desc.currentStage = _transmittanceLutState.stage;
    lut_desc.currentAccess = _transmittanceLutState.access;
    RGImageHandle transmittance_lut = graph->import_image(lut_desc);

    if (transmittance_dirty)
    {
        graph->add_pass(
            "AtmosphereLUT.Transmittance",
            RGPassType::Compute,
            [transmittance_lut](RGPassBuilder &builder, EngineContext *)
            {
                builder.write(transmittance_lut, RGImageUsage::ComputeWrite);
            },
            [this, transmittance_lut, lut_push](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
            {
                EngineContext *ctx_local = ctx ? ctx : _context;
                if (!ctx_local || !ctx_local->pipelines)
                {
                    return;
                }

                VkImageView lut_view = resources.image_view(transmittance_lut);
                if (lut_view == VK_NULL_HANDLE)
                {
                    return;
                }

                ctx_local->pipelines->setComputeInstanceStorageImage("atmosphere.transmittance_lut", 0, lut_view);

                ComputeDispatchInfo dispatch = ComputeManager::createDispatch2D(
                    k_transmittance_lut_width,
                    k_transmittance_lut_height);
                dispatch.pushConstants = &lut_push;
                dispatch.pushConstantSize = sizeof(lut_push);

                ctx_local->pipelines->dispatchComputeInstance(cmd, "atmosphere.transmittance_lut", dispatch);
            });
    }

    const bool cloud_overlay_ready = (_cloudOverlayTex.imageView != VK_NULL_HANDLE);
    const VkExtent2D draw_extent = _context->getDrawExtent();
    const VkExtent2D low_extent = half_extent(draw_extent);
    const bool run_cloud_pipeline = _context->enablePlanetClouds && cloud_overlay_ready;

    if (run_cloud_pipeline)
    {
        ensure_history_images(low_extent);
    }

    HistorySnapshot next_snapshot{};
    next_snapshot.valid = run_cloud_pipeline;
    next_snapshot.drawExtent = draw_extent;
    next_snapshot.lowExtent = low_extent;
    next_snapshot.originRevision = _context->origin_revision;
    next_snapshot.projection = _context->getSceneData().proj;
    next_snapshot.planetCenterRadius = glm::vec4(body_selection.center_local, body_selection.radius_m);
    next_snapshot.sunDirection = _context->getSceneData().sunlightDirection;
    next_snapshot.sunColor = _context->getSceneData().sunlightColor;
    next_snapshot.ambientColor = _context->getSceneData().ambientColor;
    next_snapshot.bodyName = _context->atmosphere.bodyName;
    next_snapshot.jitterPath = _context->atmosphere.jitterTexturePath;
    next_snapshot.overlayPath = _context->planetClouds.overlayTexturePath;
    next_snapshot.noisePath = _context->planetClouds.noiseTexturePath;
    next_snapshot.noise3DPath = _context->planetClouds.noiseTexture3DPath;
    next_snapshot.terrainHeightPath =
        (body_selection.body && body_selection.body->terrain) ? body_selection.body->terrain_height_dir : std::string{};
    next_snapshot.terrainHeightMaxM =
        (body_selection.body && body_selection.body->terrain)
            ? static_cast<float>(std::max(0.0, body_selection.body->terrain_height_max_m))
            : 0.0f;
    next_snapshot.terrainHeightOffsetM =
        (body_selection.body && body_selection.body->terrain)
            ? static_cast<float>(std::max(0.0, body_selection.body->terrain_height_offset_m))
            : 0.0f;
    next_snapshot.cloudBaseM = _context->planetClouds.baseHeightM;
    next_snapshot.cloudThicknessM = _context->planetClouds.thicknessM;
    next_snapshot.cloudDensityScale = _context->planetClouds.densityScale;
    next_snapshot.cloudColor = glm::vec4(_context->planetClouds.color, 1.0f);
    next_snapshot.cloudCoverage = _context->planetClouds.coverage;
    next_snapshot.cloudNoiseScale = _context->planetClouds.noiseScale;
    next_snapshot.cloudDetailScale = _context->planetClouds.detailScale;
    next_snapshot.cloudNoiseBlend = _context->planetClouds.noiseBlend;
    next_snapshot.cloudDetailErode = _context->planetClouds.detailErode;
    next_snapshot.cloudWindSpeed = _context->planetClouds.windSpeed;
    next_snapshot.cloudWindAngle = _context->planetClouds.windAngleRad;
    next_snapshot.cloudOverlayRotation = _context->planetClouds.overlayRotationRad;
    next_snapshot.cloudOverlayFlipV = _context->planetClouds.overlayFlipV;

    bool history_reset = !_historyValid || !_historySnapshot.valid || !run_cloud_pipeline;
    if (!history_reset)
    {
        history_reset = _historySnapshot.drawExtent.width != next_snapshot.drawExtent.width ||
                        _historySnapshot.drawExtent.height != next_snapshot.drawExtent.height ||
                        _historySnapshot.lowExtent.width != next_snapshot.lowExtent.width ||
                        _historySnapshot.lowExtent.height != next_snapshot.lowExtent.height ||
                        _historySnapshot.originRevision != next_snapshot.originRevision ||
                        !nearly_equal_mat4(_historySnapshot.projection, next_snapshot.projection) ||
                        !nearly_equal_vec4(_historySnapshot.planetCenterRadius, next_snapshot.planetCenterRadius, 1.0e-2f) ||
                        !nearly_equal_vec4(_historySnapshot.sunDirection, next_snapshot.sunDirection, 1.0e-3f) ||
                        !nearly_equal_vec4(_historySnapshot.sunColor, next_snapshot.sunColor, 2.0e-2f) ||
                        !nearly_equal_vec4(_historySnapshot.ambientColor, next_snapshot.ambientColor, 2.0e-2f) ||
                        _historySnapshot.bodyName != next_snapshot.bodyName ||
                        _historySnapshot.jitterPath != next_snapshot.jitterPath ||
                        _historySnapshot.overlayPath != next_snapshot.overlayPath ||
                        _historySnapshot.noisePath != next_snapshot.noisePath ||
                        _historySnapshot.noise3DPath != next_snapshot.noise3DPath ||
                        _historySnapshot.terrainHeightPath != next_snapshot.terrainHeightPath ||
                        !nearly_equal(_historySnapshot.terrainHeightMaxM, next_snapshot.terrainHeightMaxM) ||
                        !nearly_equal(_historySnapshot.terrainHeightOffsetM, next_snapshot.terrainHeightOffsetM) ||
                        !nearly_equal(_historySnapshot.cloudBaseM, next_snapshot.cloudBaseM) ||
                        !nearly_equal(_historySnapshot.cloudThicknessM, next_snapshot.cloudThicknessM) ||
                        !nearly_equal(_historySnapshot.cloudDensityScale, next_snapshot.cloudDensityScale) ||
                        !nearly_equal_vec4(_historySnapshot.cloudColor, next_snapshot.cloudColor, 2.0e-3f) ||
                        !nearly_equal(_historySnapshot.cloudCoverage, next_snapshot.cloudCoverage) ||
                        !nearly_equal(_historySnapshot.cloudNoiseScale, next_snapshot.cloudNoiseScale) ||
                        !nearly_equal(_historySnapshot.cloudDetailScale, next_snapshot.cloudDetailScale) ||
                        !nearly_equal(_historySnapshot.cloudNoiseBlend, next_snapshot.cloudNoiseBlend) ||
                        !nearly_equal(_historySnapshot.cloudDetailErode, next_snapshot.cloudDetailErode) ||
                        !nearly_equal(_historySnapshot.cloudWindSpeed, next_snapshot.cloudWindSpeed) ||
                        !nearly_equal(_historySnapshot.cloudWindAngle, next_snapshot.cloudWindAngle) ||
                        !nearly_equal(_historySnapshot.cloudOverlayRotation, next_snapshot.cloudOverlayRotation) ||
                        _historySnapshot.cloudOverlayFlipV != next_snapshot.cloudOverlayFlipV;
    }

    const glm::dvec3 origin_delta_world = _context->origin_world - _previousOriginWorld;
    const glm::vec3 origin_delta_local = glm::vec3(origin_delta_world);
    const glm::mat4 previous_view_proj = _previousViewProj;

    RGImageHandle cloud_lighting_resolved{};
    RGImageHandle cloud_segment_resolved{};

    if (run_cloud_pipeline)
    {
        const uint32_t previous_index = _historyActiveIndex;
        const uint32_t next_index = _historyActiveIndex ^ 1u;

        auto import_history = [&](const char *name,
                                  const AllocatedImage &image,
                                  const HistoryImageState &state) -> RGImageHandle
        {
            if (image.image == VK_NULL_HANDLE || image.imageView == VK_NULL_HANDLE)
            {
                return {};
            }

            RGImportedImageDesc desc{};
            desc.name = name;
            desc.image = image.image;
            desc.imageView = image.imageView;
            desc.format = image.imageFormat;
            desc.extent = VkExtent2D{image.imageExtent.width, image.imageExtent.height};
            desc.currentLayout = state.layout;
            desc.currentStage = state.stage;
            desc.currentAccess = state.access;
            return graph->import_image(desc);
        };

        RGImageDesc low_lighting_desc{};
        low_lighting_desc.name = "atmosphere.cloud.lowres_lighting";
        low_lighting_desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
        low_lighting_desc.extent = low_extent;
        low_lighting_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        RGImageHandle cloud_lighting_current = graph->create_image(low_lighting_desc);

        RGImageDesc low_segment_desc{};
        low_segment_desc.name = "atmosphere.cloud.lowres_segment";
        low_segment_desc.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        low_segment_desc.extent = low_extent;
        low_segment_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        RGImageHandle cloud_segment_current = graph->create_image(low_segment_desc);

        RGImageHandle cloud_lighting_history_prev = import_history(
            "atmosphere.cloud.history_prev_lighting",
            _cloudLightingHistory[previous_index],
            _cloudLightingHistoryState[previous_index]);
        RGImageHandle cloud_segment_history_prev = import_history(
            "atmosphere.cloud.history_prev_segment",
            _cloudSegmentHistory[previous_index],
            _cloudSegmentHistoryState[previous_index]);
        RGImageHandle cloud_lighting_history_next = import_history(
            "atmosphere.cloud.history_next_lighting",
            _cloudLightingHistory[next_index],
            _cloudLightingHistoryState[next_index]);
        RGImageHandle cloud_segment_history_next = import_history(
            "atmosphere.cloud.history_next_segment",
            _cloudSegmentHistory[next_index],
            _cloudSegmentHistoryState[next_index]);

        graph->add_pass(
            "Atmosphere.CloudLowRes",
            RGPassType::Graphics,
            [gbufPos, transmittance_lut, cloud_lighting_current, cloud_segment_current](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(gbufPos, RGImageUsage::SampledFragment);
                builder.read(transmittance_lut, RGImageUsage::SampledFragment);
                builder.write_color(cloud_lighting_current, true);
                builder.write_color(cloud_segment_current, true);
            },
            [this, low_extent, gbufPos, transmittance_lut, cloud_lighting_current, cloud_segment_current](
                VkCommandBuffer cmd,
                const RGPassResources &resources,
                EngineContext *ctx)
            {
                draw_cloud_low_res(
                    cmd,
                    ctx,
                    resources,
                    low_extent,
                    gbufPos,
                    transmittance_lut,
                    cloud_lighting_current,
                    cloud_segment_current);
            });

        graph->add_pass(
            "Atmosphere.CloudTemporal",
            RGPassType::Graphics,
            [cloud_lighting_current,
             cloud_segment_current,
             cloud_lighting_history_prev,
             cloud_segment_history_prev,
             cloud_lighting_history_next,
             cloud_segment_history_next](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(cloud_lighting_current, RGImageUsage::SampledFragment);
                builder.read(cloud_segment_current, RGImageUsage::SampledFragment);
                if (cloud_lighting_history_prev.valid()) builder.read(cloud_lighting_history_prev, RGImageUsage::SampledFragment);
                if (cloud_segment_history_prev.valid()) builder.read(cloud_segment_history_prev, RGImageUsage::SampledFragment);
                builder.write_color(cloud_lighting_history_next, false);
                builder.write_color(cloud_segment_history_next, false);
            },
            [this,
             low_extent,
             cloud_lighting_current,
             cloud_segment_current,
             cloud_lighting_history_prev,
             cloud_segment_history_prev,
             cloud_lighting_history_next,
             cloud_segment_history_next,
             previous_view_proj,
             origin_delta_local,
             history_reset,
             run_cloud_pipeline](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
            {
                draw_cloud_temporal(
                    cmd,
                    ctx,
                    resources,
                    low_extent,
                    cloud_lighting_current,
                    cloud_segment_current,
                    cloud_lighting_history_prev,
                    cloud_segment_history_prev,
                    cloud_lighting_history_next,
                    cloud_segment_history_next,
                    previous_view_proj,
                    origin_delta_local,
                    run_cloud_pipeline && !history_reset);
            });

        RGImageDesc resolved_lighting_desc{};
        resolved_lighting_desc.name = "atmosphere.cloud.resolved_lighting";
        resolved_lighting_desc.format = VK_FORMAT_R16G16B16A16_SFLOAT;
        resolved_lighting_desc.extent = draw_extent;
        resolved_lighting_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        cloud_lighting_resolved = graph->create_image(resolved_lighting_desc);

        RGImageDesc resolved_segment_desc{};
        resolved_segment_desc.name = "atmosphere.cloud.resolved_segment";
        resolved_segment_desc.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        resolved_segment_desc.extent = draw_extent;
        resolved_segment_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        cloud_segment_resolved = graph->create_image(resolved_segment_desc);

        graph->add_pass(
            "Atmosphere.CloudUpscale",
            RGPassType::Graphics,
            [gbufPos,
             cloud_lighting_history_next,
             cloud_segment_history_next,
             cloud_lighting_resolved,
             cloud_segment_resolved](RGPassBuilder &builder, EngineContext *)
            {
                builder.read(gbufPos, RGImageUsage::SampledFragment);
                builder.read(cloud_lighting_history_next, RGImageUsage::SampledFragment);
                builder.read(cloud_segment_history_next, RGImageUsage::SampledFragment);
                builder.write_color(cloud_lighting_resolved, true);
                builder.write_color(cloud_segment_resolved, true);
            },
            [this,
             draw_extent,
             gbufPos,
             cloud_lighting_history_next,
             cloud_segment_history_next,
             cloud_lighting_resolved,
             cloud_segment_resolved](VkCommandBuffer cmd, const RGPassResources &resources, EngineContext *ctx)
            {
                draw_cloud_upscale(
                    cmd,
                    ctx,
                    resources,
                    draw_extent,
                    gbufPos,
                    cloud_lighting_history_next,
                    cloud_segment_history_next,
                    cloud_lighting_resolved,
                    cloud_segment_resolved);
            });

        _cloudLightingHistoryState[previous_index] = {
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
        _cloudSegmentHistoryState[previous_index] = {
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
        _cloudLightingHistoryState[next_index] = {
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT};
        _cloudSegmentHistoryState[next_index] = {
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT};
        _historyActiveIndex = next_index;
        _historyValid = true;
    }
    else
    {
        _historyValid = false;
    }

    _previousViewProj = _context->getSceneData().viewproj;
    _previousOriginWorld = _context->origin_world;
    _historySnapshot = next_snapshot;

    RGImageDesc output_desc{};
    output_desc.name = "hdr.atmosphere";
    output_desc.format = _context->getSwapchain()
        ? _context->getSwapchain()->drawImage().imageFormat
        : VK_FORMAT_R16G16B16A16_SFLOAT;
    output_desc.extent = draw_extent;
    output_desc.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    RGImageHandle hdr_output = graph->create_image(output_desc);

    graph->add_pass(
        "Atmosphere.Composite",
        RGPassType::Graphics,
        [hdrInput, gbufPos, transmittance_lut, cloud_lighting_resolved, cloud_segment_resolved, hdr_output](
            RGPassBuilder &builder,
            EngineContext *)
        {
            builder.read(hdrInput, RGImageUsage::SampledFragment);
            builder.read(gbufPos, RGImageUsage::SampledFragment);
            builder.read(transmittance_lut, RGImageUsage::SampledFragment);
            if (cloud_lighting_resolved.valid()) builder.read(cloud_lighting_resolved, RGImageUsage::SampledFragment);
            if (cloud_segment_resolved.valid()) builder.read(cloud_segment_resolved, RGImageUsage::SampledFragment);
            builder.write_color(hdr_output, false);
        },
        [this, draw_extent, hdrInput, gbufPos, transmittance_lut, cloud_lighting_resolved, cloud_segment_resolved](
            VkCommandBuffer cmd,
            const RGPassResources &resources,
            EngineContext *ctx)
        {
            draw_composite(
                cmd,
                ctx,
                resources,
                draw_extent,
                hdrInput,
                gbufPos,
                transmittance_lut,
                cloud_lighting_resolved,
                cloud_segment_resolved);
        });

    _transmittanceLutState = {
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
        VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
    _transmittanceLutParams = next_lut_snapshot;
    _transmittanceLutValid = true;

    return hdr_output;
}
