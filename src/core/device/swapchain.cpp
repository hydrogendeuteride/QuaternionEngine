#include "swapchain.h"

#include <SDL_video.h>
#include "SDL2/SDL_vulkan.h"

#include "device.h"
#include "core/util/initializers.h"
#include "resource.h"

// Swapchain + per-frame targets (HDR draw, depth, GBuffer) management.
//
// Create/resize/destroy logic keeps per-frame images in a local deletion queue
// so they are cleaned up with the swapchain. The engine imports those images
// into the Render Graph each frame.
void SwapchainManager::init_swapchain()
{
    create_swapchain(_windowExtent.width, _windowExtent.height);

    // Create images used across the frame (draw, depth, GBuffer).
    // These are sized to _renderExtent (independent of the swapchain extent)
    // so the engine can render at a different internal resolution and then
    // upscale/letterbox into the swapchain.
    if (_renderExtent.width == 0 || _renderExtent.height == 0)
    {
        _renderExtent = _windowExtent;
    }
    resize_render_targets(_renderExtent);
}

void SwapchainManager::set_window_extent_from_window(struct SDL_Window *window)
{
    if (!window)
    {
        return;
    }

    int w = 0, h = 0;
    SDL_Vulkan_GetDrawableSize(window, &w, &h);
    if (w <= 0 || h <= 0)
    {
        SDL_GetWindowSize(window, &w, &h);
    }
    if (w > 0 && h > 0)
    {
        _windowExtent.width = static_cast<uint32_t>(w);
        _windowExtent.height = static_cast<uint32_t>(h);
    }
}

void SwapchainManager::cleanup()
{
    _deletionQueue.flush();
    destroy_swapchain();
    fmt::print("SwapchainManager::cleanup()\n");
}

void SwapchainManager::resize_render_targets(VkExtent2D renderExtent)
{
    if (!_deviceManager || !_resourceManager) return;
    if (renderExtent.width == 0 || renderExtent.height == 0) return;

    // Avoid doing work when nothing changes (common when called every frame).
    if (_renderExtent.width == renderExtent.width &&
        _renderExtent.height == renderExtent.height &&
        _drawImage.image != VK_NULL_HANDLE &&
        _depthImage.image != VK_NULL_HANDLE)
    {
        return;
    }

    // Ensure no in-flight work references these images before we destroy them.
    vkDeviceWaitIdle(_deviceManager->device());

    // Destroy previous targets (if any), then recreate at the new extent.
    _deletionQueue.flush();
    _renderExtent = renderExtent;

    VkExtent3D drawImageExtent = { _renderExtent.width, _renderExtent.height, 1 };

    // Draw HDR target
    _drawImage = {};
    _drawImage.imageFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    _drawImage.imageExtent = drawImageExtent;

    VkImageUsageFlags drawImageUsages{};
    drawImageUsages |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_STORAGE_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    // Post-processing (tonemap) samples HDR; allow sampling.
    drawImageUsages |= VK_IMAGE_USAGE_SAMPLED_BIT;

    VkImageCreateInfo rimg_info = vkinit::image_create_info(_drawImage.imageFormat, drawImageUsages, drawImageExtent);

    VmaAllocationCreateInfo rimg_allocinfo = {};
    rimg_allocinfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    rimg_allocinfo.requiredFlags = static_cast<VkMemoryPropertyFlags>(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    vmaCreateImage(_deviceManager->allocator(), &rimg_info, &rimg_allocinfo,
                   &_drawImage.image, &_drawImage.allocation, nullptr);

    VkImageViewCreateInfo rview_info = vkinit::imageview_create_info(_drawImage.imageFormat, _drawImage.image,
                                                                     VK_IMAGE_ASPECT_COLOR_BIT);
    VK_CHECK(vkCreateImageView(_deviceManager->device(), &rview_info, nullptr, &_drawImage.imageView));

    // Depth
    _depthImage = {};
    _depthImage.imageFormat = VK_FORMAT_D32_SFLOAT;
    _depthImage.imageExtent = drawImageExtent;
    VkImageUsageFlags depthImageUsages{};
    depthImageUsages |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    VkImageCreateInfo dimg_info = vkinit::image_create_info(_depthImage.imageFormat, depthImageUsages, drawImageExtent);
    vmaCreateImage(_deviceManager->allocator(), &dimg_info, &rimg_allocinfo, &_depthImage.image,
                   &_depthImage.allocation, nullptr);
    VkImageViewCreateInfo dview_info = vkinit::imageview_create_info(_depthImage.imageFormat, _depthImage.image,
                                                                     VK_IMAGE_ASPECT_DEPTH_BIT);
    VK_CHECK(vkCreateImageView(_deviceManager->device(), &dview_info, nullptr, &_depthImage.imageView));

    // GBuffer (SRGB not used to keep linear lighting)
    _gBufferPosition = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R32G32B32A32_SFLOAT,
                                                      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    _gBufferNormal = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R16G16B16A16_SFLOAT,
                                                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    _gBufferAlbedo = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R8G8B8A8_UNORM,
                                                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    _gBufferExtra = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R16G16B16A16_SFLOAT,
                                                   VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    _idBuffer = _resourceManager->create_image(drawImageExtent, VK_FORMAT_R32_UINT,
                                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                                               VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                                               VK_IMAGE_USAGE_SAMPLED_BIT);

    _deletionQueue.push_function([this]() {
        if (_drawImage.imageView) vkDestroyImageView(_deviceManager->device(), _drawImage.imageView, nullptr);
        if (_drawImage.image) vmaDestroyImage(_deviceManager->allocator(), _drawImage.image, _drawImage.allocation);

        if (_depthImage.imageView) vkDestroyImageView(_deviceManager->device(), _depthImage.imageView, nullptr);
        if (_depthImage.image) vmaDestroyImage(_deviceManager->allocator(), _depthImage.image, _depthImage.allocation);

        if (_gBufferPosition.image) _resourceManager->destroy_image(_gBufferPosition);
        if (_gBufferNormal.image) _resourceManager->destroy_image(_gBufferNormal);
        if (_gBufferAlbedo.image) _resourceManager->destroy_image(_gBufferAlbedo);
        if (_gBufferExtra.image) _resourceManager->destroy_image(_gBufferExtra);
        if (_idBuffer.image) _resourceManager->destroy_image(_idBuffer);

        _drawImage = {};
        _depthImage = {};
        _gBufferPosition = {};
        _gBufferNormal = {};
        _gBufferAlbedo = {};
        _gBufferExtra = {};
        _idBuffer = {};
    });
}

void SwapchainManager::create_swapchain(uint32_t width, uint32_t height)
{
    vkb::SwapchainBuilder swapchainBuilder{
        _deviceManager->physicalDevice(), _deviceManager->device(), _deviceManager->surface()
    };

    _swapchainImageFormat = VK_FORMAT_B8G8R8A8_UNORM;

    vkb::Swapchain vkbSwapchain = swapchainBuilder
            //.use_default_format_selection()
            .set_desired_format(VkSurfaceFormatKHR{
                .format = _swapchainImageFormat, .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
            })
            //use vsync present mode
            .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
            .set_desired_extent(width, height)
            .add_image_usage_flags(VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
            .build()
            .value();

    _swapchainExtent = vkbSwapchain.extent;
    //store swapchain and its related images
    _swapchain = vkbSwapchain.swapchain;
    _swapchainImages = vkbSwapchain.get_images().value();
    _swapchainImageViews = vkbSwapchain.get_image_views().value();
    _swapchainImageLayouts.assign(_swapchainImages.size(), VK_IMAGE_LAYOUT_UNDEFINED);
}

void SwapchainManager::destroy_swapchain() const
{
    // Destroy image views before the swapchain for stricter driver orderliness.
    // (Most drivers tolerate either order, but views reference swapchain images.)
    for (auto view : _swapchainImageViews)
    {
        vkDestroyImageView(_deviceManager->device(), view, nullptr);
    }
    vkDestroySwapchainKHR(_deviceManager->device(), _swapchain, nullptr);
}

void SwapchainManager::resize_swapchain(struct SDL_Window *window)
{
    int w, h;
    // HiDPI-aware drawable size for correct pixel dimensions
    SDL_Vulkan_GetDrawableSize(window, &w, &h);
    if (w <= 0 || h <= 0)
    {
        // Window may be minimized or in a transient resize state; keep current swapchain.
        resize_requested = true;
        return;
    }

    vkDeviceWaitIdle(_deviceManager->device());

    destroy_swapchain();

    _windowExtent.width = w;
    _windowExtent.height = h;

    create_swapchain(_windowExtent.width, _windowExtent.height);

    resize_requested = false;
}

VkImageLayout SwapchainManager::swapchain_image_layout(uint32_t index) const
{
    if (index >= _swapchainImageLayouts.size()) return VK_IMAGE_LAYOUT_UNDEFINED;
    return _swapchainImageLayouts[index];
}

void SwapchainManager::set_swapchain_image_layout(uint32_t index, VkImageLayout layout)
{
    if (index >= _swapchainImageLayouts.size()) return;
    _swapchainImageLayouts[index] = layout;
}
