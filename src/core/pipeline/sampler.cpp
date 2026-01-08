#include "sampler.h"
#include "core/device/device.h"

void SamplerManager::init(DeviceManager *deviceManager)
{
    _deviceManager = deviceManager;

    VkSamplerCreateInfo sampl{.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};

    // Sensible, cross-vendor defaults
    sampl.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampl.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampl.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampl.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampl.minLod       = 0.0f;
    sampl.maxLod       = VK_LOD_CLAMP_NONE;
    sampl.mipLodBias   = 0.0f;
    sampl.anisotropyEnable = VK_FALSE; // set true + maxAnisotropy if feature enabled
    sampl.borderColor  = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampl.unnormalizedCoordinates = VK_FALSE;

    // Nearest defaults
    sampl.magFilter = VK_FILTER_NEAREST;
    sampl.minFilter = VK_FILTER_NEAREST;
    vkCreateSampler(_deviceManager->device(), &sampl, nullptr, &_defaultSamplerNearest);

    // Linear defaults
    sampl.magFilter = VK_FILTER_LINEAR;
    sampl.minFilter = VK_FILTER_LINEAR;
    vkCreateSampler(_deviceManager->device(), &sampl, nullptr, &_defaultSamplerLinear);

    // Linear clamp-to-edge (useful for tiled textures)
    VkSamplerCreateInfo clampEdge = sampl;
    clampEdge.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    clampEdge.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    clampEdge.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    vkCreateSampler(_deviceManager->device(), &clampEdge, nullptr, &_linearClampEdge);

    // Nearest clamp-to-edge (useful for LUTs / non-filterable formats)
    VkSamplerCreateInfo clampEdgeNearest = clampEdge;
    clampEdgeNearest.magFilter = VK_FILTER_NEAREST;
    clampEdgeNearest.minFilter = VK_FILTER_NEAREST;
    clampEdgeNearest.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    vkCreateSampler(_deviceManager->device(), &clampEdgeNearest, nullptr, &_nearestClampEdge);

    // Shadow linear clamp sampler (border=white)
    VkSamplerCreateInfo sh = sampl;
    sh.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sh.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sh.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    sh.compareEnable = VK_FALSE; // manual PCF
    sh.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    vkCreateSampler(_deviceManager->device(), &sh, nullptr, &_shadowLinearClamp);

}

void SamplerManager::cleanup()
{
    if (!_deviceManager) return;

    if (_defaultSamplerNearest)
    {
        vkDestroySampler(_deviceManager->device(), _defaultSamplerNearest, nullptr);
        _defaultSamplerNearest = VK_NULL_HANDLE;
    }
    if (_defaultSamplerLinear)
    {
        vkDestroySampler(_deviceManager->device(), _defaultSamplerLinear, nullptr);
        _defaultSamplerLinear = VK_NULL_HANDLE;
    }

    if (_shadowLinearClamp)
    {
        vkDestroySampler(_deviceManager->device(), _shadowLinearClamp, nullptr);
        _shadowLinearClamp = VK_NULL_HANDLE;
    }

    if (_linearClampEdge)
    {
        vkDestroySampler(_deviceManager->device(), _linearClampEdge, nullptr);
        _linearClampEdge = VK_NULL_HANDLE;
    }
    if (_nearestClampEdge)
    {
        vkDestroySampler(_deviceManager->device(), _nearestClampEdge, nullptr);
        _nearestClampEdge = VK_NULL_HANDLE;
    }
}
