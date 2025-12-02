#include "materials.h"

#include "core/engine.h"
#include "render/pipelines.h"
#include "core/util/initializers.h"
#include "core/pipeline/manager.h"
#include "core/assets/manager.h"

namespace vkutil { bool load_shader_module(const char*, VkDevice, VkShaderModule*); }

void GLTFMetallic_Roughness::build_pipelines(VulkanEngine *engine)
{
    VkPushConstantRange matrixRange{};
    matrixRange.offset = 0;
    matrixRange.size = sizeof(GPUDrawPushConstants);
    matrixRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    DescriptorLayoutBuilder layoutBuilder;
    layoutBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    layoutBuilder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    materialLayout = layoutBuilder.build(engine->_deviceManager->device(),
                                         VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                         nullptr, VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);

    // Ensure IBL layout exists; add placeholder for set=2
    // Create a persistent empty set layout placeholder (lifetime = GLTFMetallic_Roughness)
    {
        VkDescriptorSetLayoutCreateInfo info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        VK_CHECK(vkCreateDescriptorSetLayout(engine->_deviceManager->device(), &info, nullptr, &emptySetLayout));
    }
    VkDescriptorSetLayout iblLayout = emptySetLayout;
    if (engine->_context->ibl && engine->_context->ibl->ensureLayout())
        iblLayout = engine->_context->ibl->descriptorLayout();

    VkDescriptorSetLayout layouts[] = {
        engine->_descriptorManager->gpuSceneDataLayout(), // set=0
        materialLayout,                                   // set=1
        emptySetLayout,                                   // set=2 (unused)
        iblLayout                                         // set=3
    };

    // Register pipelines with the central PipelineManager
    GraphicsPipelineCreateInfo opaqueInfo{};
    opaqueInfo.vertexShaderPath = engine->_context->getAssets()->shaderPath("mesh.vert.spv");
    opaqueInfo.fragmentShaderPath = engine->_context->getAssets()->shaderPath("mesh.frag.spv");
    opaqueInfo.setLayouts.assign(std::begin(layouts), std::end(layouts));
    opaqueInfo.pushConstants = {matrixRange};
    opaqueInfo.configure = [engine](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        // Reverse-Z depth test configuration
        b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_color_attachment_format(engine->_swapchainManager->drawImage().imageFormat);
        b.set_depth_format(engine->_swapchainManager->depthImage().imageFormat);
    };
    engine->_pipelineManager->registerGraphics("mesh.opaque", opaqueInfo);

    GraphicsPipelineCreateInfo transparentInfo = opaqueInfo;
    transparentInfo.configure = [engine](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        // Physically-based transparency uses standard alpha blending
        b.enable_blending_alphablend();
        // Transparent pass: keep reverse-Z test (no writes)
        b.enable_depthtest(false, VK_COMPARE_OP_GREATER_OR_EQUAL);
        b.set_color_attachment_format(engine->_swapchainManager->drawImage().imageFormat);
        b.set_depth_format(engine->_swapchainManager->depthImage().imageFormat);
    };
    engine->_pipelineManager->registerGraphics("mesh.transparent", transparentInfo);

    GraphicsPipelineCreateInfo gbufferInfo{};
    gbufferInfo.vertexShaderPath = engine->_context->getAssets()->shaderPath("mesh.vert.spv");
    gbufferInfo.fragmentShaderPath = engine->_context->getAssets()->shaderPath("gbuffer.frag.spv");
    gbufferInfo.setLayouts.assign(std::begin(layouts), std::end(layouts));
    gbufferInfo.pushConstants = {matrixRange};
    gbufferInfo.configure = [engine](PipelineBuilder &b) {
        b.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
        b.set_polygon_mode(VK_POLYGON_MODE_FILL);
        b.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
        b.set_multisampling_none();
        b.disable_blending();
        // GBuffer uses reverse-Z depth
        b.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);
        VkFormat gFormats[] = {
            engine->_swapchainManager->gBufferPosition().imageFormat,
            engine->_swapchainManager->gBufferNormal().imageFormat,
            engine->_swapchainManager->gBufferAlbedo().imageFormat,
            engine->_swapchainManager->idBuffer().imageFormat,
            engine->_swapchainManager->gBufferExtra().imageFormat
        };
        b.set_color_attachment_formats(std::span<VkFormat>(gFormats, 5));
        b.set_depth_format(engine->_swapchainManager->depthImage().imageFormat);
    };
    engine->_pipelineManager->registerGraphics("mesh.gbuffer", gbufferInfo);

    // Keep emptySetLayout until clear_resources()

    engine->_pipelineManager->getMaterialPipeline("mesh.opaque", opaquePipeline);
    engine->_pipelineManager->getMaterialPipeline("mesh.transparent", transparentPipeline);
    engine->_pipelineManager->getMaterialPipeline("mesh.gbuffer", gBufferPipeline);
}

void GLTFMetallic_Roughness::clear_resources(VkDevice device) const
{
    vkDestroyDescriptorSetLayout(device, materialLayout, nullptr);
    if (emptySetLayout) vkDestroyDescriptorSetLayout(device, emptySetLayout, nullptr);
}

MaterialInstance GLTFMetallic_Roughness::write_material(VkDevice device, MaterialPass pass,
                                                        const MaterialResources &resources,
                                                        DescriptorAllocatorGrowable &descriptorAllocator)
{
    MaterialInstance matData{};
    matData.passType = pass;
    if (pass == MaterialPass::Transparent)
    {
        matData.pipeline = &transparentPipeline;
    }
    else
    {
        matData.pipeline = &gBufferPipeline;
    }

    matData.materialSet = descriptorAllocator.allocate(device, materialLayout);

    writer.clear();
    writer.write_buffer(0, resources.dataBuffer, sizeof(MaterialConstants), resources.dataBufferOffset,
                        VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.write_image(1, resources.colorImage.imageView, resources.colorSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, resources.metalRoughImage.imageView, resources.metalRoughSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(3, resources.normalImage.imageView, resources.normalSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(4, resources.occlusionImage.imageView, resources.occlusionSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(5, resources.emissiveImage.imageView, resources.emissiveSampler,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    writer.update_set(device, matData.materialSet);

    return matData;
}
