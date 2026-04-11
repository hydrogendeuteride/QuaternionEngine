#include "render/passes/atmosphere/atmosphere_internal.h"

using namespace atmosphere::detail;

void AtmospherePass::release_transmittance_lut_image()
{
    if (!_context || !_context->getResources())
    {
        return;
    }

    if (_transmittanceLut.image != VK_NULL_HANDLE)
    {
        _context->getResources()->destroy_image(_transmittanceLut);
        _transmittanceLut = {};
    }

    _transmittanceLutState = {};
    _transmittanceLutParams = {};
    _transmittanceLutValid = false;
}

void AtmospherePass::ensure_transmittance_lut_image()
{
    if (!_context || !_context->getResources())
    {
        return;
    }

    const bool needs_recreate =
        _transmittanceLut.image == VK_NULL_HANDLE ||
        _transmittanceLut.imageView == VK_NULL_HANDLE ||
        _transmittanceLut.imageFormat != VK_FORMAT_R16G16B16A16_SFLOAT ||
        _transmittanceLut.imageExtent.width != k_transmittance_lut_width ||
        _transmittanceLut.imageExtent.height != k_transmittance_lut_height;

    if (!needs_recreate)
    {
        return;
    }

    release_transmittance_lut_image();

    const VkExtent3D extent{k_transmittance_lut_width, k_transmittance_lut_height, 1u};
    const VkImageUsageFlags usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    _transmittanceLut = _context->getResources()->create_image(extent, VK_FORMAT_R16G16B16A16_SFLOAT, usage);
}

void AtmospherePass::release_history_images()
{
    if (!_context || !_context->getResources())
    {
        return;
    }

    ResourceManager *resources = _context->getResources();
    for (AllocatedImage &image : _cloudLightingHistory)
    {
        if (image.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(image);
        }
        image = {};
    }
    for (AllocatedImage &image : _cloudSegmentHistory)
    {
        if (image.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(image);
        }
        image = {};
    }

    _cloudLightingHistoryState = {};
    _cloudSegmentHistoryState = {};
    _historyActiveIndex = 0;
    _historyValid = false;
}

void AtmospherePass::ensure_history_images(VkExtent2D extent)
{
    if (!_context || !_context->getResources())
    {
        return;
    }

    const bool need_recreate = [&]() {
        for (uint32_t i = 0; i < 2; ++i)
        {
            if (_cloudLightingHistory[i].image == VK_NULL_HANDLE || _cloudSegmentHistory[i].image == VK_NULL_HANDLE)
            {
                return true;
            }
            if (_cloudLightingHistory[i].imageFormat != VK_FORMAT_R16G16B16A16_SFLOAT ||
                _cloudSegmentHistory[i].imageFormat != VK_FORMAT_R32G32B32A32_SFLOAT)
            {
                return true;
            }
            if (_cloudLightingHistory[i].imageExtent.width != extent.width ||
                _cloudLightingHistory[i].imageExtent.height != extent.height ||
                _cloudSegmentHistory[i].imageExtent.width != extent.width ||
                _cloudSegmentHistory[i].imageExtent.height != extent.height)
            {
                return true;
            }
        }
        return false;
    }();

    if (!need_recreate)
    {
        return;
    }

    release_history_images();

    ResourceManager *resources = _context->getResources();
    const VkExtent3D image_extent{extent.width, extent.height, 1u};
    const VkImageUsageFlags usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

    for (uint32_t i = 0; i < 2; ++i)
    {
        _cloudLightingHistory[i] = resources->create_image(image_extent, VK_FORMAT_R16G16B16A16_SFLOAT, usage);
        _cloudSegmentHistory[i] = resources->create_image(image_extent, VK_FORMAT_R32G32B32A32_SFLOAT, usage);
    }
}
