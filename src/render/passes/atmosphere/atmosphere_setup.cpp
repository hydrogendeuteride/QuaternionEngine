#include "render/passes/atmosphere/atmosphere_internal.h"

using namespace atmosphere::detail;

void AtmospherePass::init(EngineContext *context)
{
    _context = context;
    if (!_context || !_context->getDevice() || !_context->getDescriptorLayouts() || !_context->pipelines ||
        !_context->getResources() || !_context->getAssets())
    {
        return;
    }

    VkDevice device = _context->getDevice()->device();

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(11, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _lowResSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _temporalSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _upscaleSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(11, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(12, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(13, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        builder.add_binding(14, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _compositeSetLayout = builder.build(
            device,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr,
            VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

    const std::string fullscreen_vertex = _context->getAssets()->shaderPath("atmosphere/atmosphere.vert.spv");

    {
        std::array<uint8_t, 8> white3d{};
        white3d.fill(0xFFu);
        _cloudNoiseFallback3D = _context->getResources()->create_image(
            white3d.data(),
            VkExtent3D{2, 2, 2},
            VK_FORMAT_R8_UNORM,
            VK_IMAGE_USAGE_SAMPLED_BIT,
            false);
    }

    const VkFormat hdr_format = _context->getSwapchain()
        ? _context->getSwapchain()->drawImage().imageFormat
        : VK_FORMAT_R16G16B16A16_SFLOAT;
    auto configure_cloud_outputs = [](PipelineBuilder &builder)
    {
        std::array<VkFormat, 2> cloud_formats{VK_FORMAT_R16G16B16A16_SFLOAT, VK_FORMAT_R32G32B32A32_SFLOAT};
        builder.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        builder.set_polygon_mode(VK_POLYGON_MODE_FILL);
        builder.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        builder.set_multisampling_none();
        builder.disable_depthtest();
        builder.disable_blending();
        builder.set_color_attachment_formats(std::span<VkFormat>(cloud_formats));
    };

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere/cloud_lowres.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _lowResSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(AtmospherePush);
        info.pushConstants = {push_range};
        info.configure = configure_cloud_outputs;
        _context->pipelines->createGraphicsPipeline("atmosphere.cloud_lowres", info);
    }

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere/cloud_temporal.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _temporalSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(CloudTemporalPush);
        info.pushConstants = {push_range};
        info.configure = configure_cloud_outputs;
        _context->pipelines->createGraphicsPipeline("atmosphere.cloud_temporal", info);
    }

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere/cloud_upscale.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _upscaleSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(AtmospherePush);
        info.pushConstants = {push_range};
        info.configure = configure_cloud_outputs;
        _context->pipelines->createGraphicsPipeline("atmosphere.cloud_upscale", info);
    }

    {
        GraphicsPipelineCreateInfo info{};
        info.vertexShaderPath = fullscreen_vertex;
        info.fragmentShaderPath = _context->getAssets()->shaderPath("atmosphere/atmosphere.frag.spv");
        info.setLayouts = {_context->getDescriptorLayouts()->gpuSceneDataLayout(), _compositeSetLayout};

        VkPushConstantRange push_range{};
        push_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        push_range.offset = 0;
        push_range.size = sizeof(AtmospherePush);
        info.pushConstants = {push_range};
        info.configure = [hdr_format](PipelineBuilder &builder)
        {
            builder.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
            builder.set_polygon_mode(VK_POLYGON_MODE_FILL);
            builder.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
            builder.set_multisampling_none();
            builder.disable_depthtest();
            builder.disable_blending();
            builder.set_color_attachment_format(hdr_format);
        };
        _context->pipelines->createGraphicsPipeline("atmosphere.composite", info);
    }

    {
        ComputePipelineCreateInfo info{};
        info.shaderPath = _context->getAssets()->shaderPath("atmosphere/transmittance_lut.comp.spv");
        info.descriptorTypes = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE};
        info.pushConstantSize = sizeof(AtmosphereLutPush);
        info.pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;
        _context->pipelines->createComputePipeline("atmosphere.transmittance_lut", info);
        _context->pipelines->createComputeInstance("atmosphere.transmittance_lut", "atmosphere.transmittance_lut");
    }

    ensure_cloud_textures(_context);
}

void AtmospherePass::cleanup()
{
    release_history_images();
    release_planet_height_textures();
    release_transmittance_lut_image();

    if (_context && _context->getResources())
    {
        ResourceManager *resources = _context->getResources();
        if (_cloudOverlayTex.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudOverlayTex);
            _cloudOverlayTex = {};
        }
        if (_cloudNoiseTex.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudNoiseTex);
            _cloudNoiseTex = {};
        }
        if (_jitterNoiseTex.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_jitterNoiseTex);
            _jitterNoiseTex = {};
        }
        if (_cloudNoiseTex3D.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudNoiseTex3D);
            _cloudNoiseTex3D = {};
        }
        if (_cloudNoiseFallback3D.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(_cloudNoiseFallback3D);
            _cloudNoiseFallback3D = {};
        }
    }

    _cloudOverlayLoadedPath.clear();
    _cloudNoiseLoadedPath.clear();
    _jitterNoiseLoadedPath.clear();
    _cloudNoise3DLoadedPath.clear();
    _planetHeightLoadedDir.clear();
    _historySnapshot = {};
    _historyValid = false;

    if (_context && _context->pipelines)
    {
        _context->pipelines->destroyComputeInstance("atmosphere.transmittance_lut");
        _context->pipelines->destroyComputePipeline("atmosphere.transmittance_lut");
    }

    if (_context && _context->getDevice())
    {
        VkDevice device = _context->getDevice()->device();
        if (_lowResSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _lowResSetLayout, nullptr);
            _lowResSetLayout = VK_NULL_HANDLE;
        }
        if (_temporalSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _temporalSetLayout, nullptr);
            _temporalSetLayout = VK_NULL_HANDLE;
        }
        if (_upscaleSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _upscaleSetLayout, nullptr);
            _upscaleSetLayout = VK_NULL_HANDLE;
        }
        if (_compositeSetLayout != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device, _compositeSetLayout, nullptr);
            _compositeSetLayout = VK_NULL_HANDLE;
        }
    }
}

void AtmospherePass::execute(VkCommandBuffer)
{
    // Executed via render graph.
}

void AtmospherePass::preload_cloud_textures()
{
    ensure_cloud_textures(_context);
}
