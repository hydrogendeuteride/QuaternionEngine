#pragma once

#include <core/vk_types.h>
#include <core/vk_descriptors.h>

class VulkanEngine;

struct GLTFMetallic_Roughness
{
    MaterialPipeline opaquePipeline;
    MaterialPipeline transparentPipeline;
    MaterialPipeline gBufferPipeline;

    VkDescriptorSetLayout materialLayout;
    VkDescriptorSetLayout emptySetLayout = VK_NULL_HANDLE; // placeholder for set=2

    struct MaterialConstants
    {
        glm::vec4 colorFactors;
        glm::vec4 metal_rough_factors;

        glm::vec4 extra[14];
    };

    struct MaterialResources
    {
        AllocatedImage colorImage;
        VkSampler colorSampler;
        AllocatedImage metalRoughImage;
        VkSampler metalRoughSampler;
        AllocatedImage normalImage;
        VkSampler normalSampler;
        VkBuffer dataBuffer;
        uint32_t dataBufferOffset;
    };

    DescriptorWriter writer;

    void build_pipelines(VulkanEngine *engine);

    void clear_resources(VkDevice device) const;

    MaterialInstance write_material(VkDevice device, MaterialPass pass, const MaterialResources &resources,
                                    DescriptorAllocatorGrowable &descriptorAllocator);
};
