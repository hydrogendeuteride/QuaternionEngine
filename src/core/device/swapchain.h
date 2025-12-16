#pragma once

#include <core/types.h>
#include <core/config.h>

class ResourceManager;
class DeviceManager;

class SwapchainManager
{
public:
    void init(DeviceManager *deviceManager, ResourceManager* resourceManager)
    {this->_deviceManager = deviceManager;this->_resourceManager = resourceManager;};

    void cleanup();

    void set_window_extent_from_window(struct SDL_Window *window);
    void init_swapchain();
    void create_swapchain(uint32_t width, uint32_t height);
    void destroy_swapchain() const;
    void resize_swapchain(struct SDL_Window *window);
    void resize_render_targets(VkExtent2D renderExtent);

    VkSwapchainKHR swapchain() const { return _swapchain; }
    VkFormat swapchainImageFormat() const { return _swapchainImageFormat; }
    VkExtent2D swapchainExtent() const { return _swapchainExtent; }
    const std::vector<VkImage> &swapchainImages() const { return _swapchainImages; }
    const std::vector<VkImageView> &swapchainImageViews() const { return _swapchainImageViews; }
    VkImageLayout swapchain_image_layout(uint32_t index) const;
    void set_swapchain_image_layout(uint32_t index, VkImageLayout layout);

    AllocatedImage drawImage() const { return _drawImage; }
    AllocatedImage depthImage() const { return _depthImage; }
    AllocatedImage gBufferPosition() const { return _gBufferPosition; }
    AllocatedImage gBufferNormal() const { return _gBufferNormal; }
    AllocatedImage gBufferAlbedo() const { return _gBufferAlbedo; }
    AllocatedImage gBufferExtra() const { return _gBufferExtra; }
    AllocatedImage idBuffer() const { return _idBuffer; }
    VkExtent2D windowExtent() const { return _windowExtent; }
    VkExtent2D renderExtent() const { return _renderExtent; }
    void set_render_extent(VkExtent2D extent) { _renderExtent = extent; }

    bool resize_requested{false};

private:
    DeviceManager *_deviceManager = nullptr;
    ResourceManager* _resourceManager = nullptr;

    VkSwapchainKHR _swapchain = nullptr;
    VkFormat _swapchainImageFormat = {};
    VkExtent2D _swapchainExtent = {};
    VkExtent2D _windowExtent{kRenderWidth, kRenderHeight};
    VkExtent2D _renderExtent{kRenderWidth, kRenderHeight};

    std::vector<VkImage> _swapchainImages;
    std::vector<VkImageView> _swapchainImageViews;
    std::vector<VkImageLayout> _swapchainImageLayouts;

    AllocatedImage _drawImage = {};
    AllocatedImage _depthImage = {};
    AllocatedImage _gBufferPosition = {};
    AllocatedImage _gBufferNormal = {};
    AllocatedImage _gBufferAlbedo = {};
    AllocatedImage _gBufferExtra = {};
    AllocatedImage _idBuffer = {};

    DeletionQueue _deletionQueue;
};
