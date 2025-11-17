#pragma once

#include <core/vk_types.h>

class ResourceManager;
class DeviceManager;

class SwapchainManager
{
public:
    void init(DeviceManager *deviceManager, ResourceManager* resourceManager)
    {this->_deviceManager = deviceManager;this->_resourceManager = resourceManager;};

    void cleanup();

    void init_swapchain();
    void create_swapchain(uint32_t width, uint32_t height);
    void destroy_swapchain() const;
    void resize_swapchain(struct SDL_Window *window);

    VkSwapchainKHR swapchain() const { return _swapchain; }
    VkFormat swapchainImageFormat() const { return _swapchainImageFormat; }
    VkExtent2D swapchainExtent() const { return _swapchainExtent; }
    const std::vector<VkImage> &swapchainImages() const { return _swapchainImages; }
    const std::vector<VkImageView> &swapchainImageViews() const { return _swapchainImageViews; }

    AllocatedImage drawImage() const { return _drawImage; }
    AllocatedImage depthImage() const { return _depthImage; }
    AllocatedImage gBufferPosition() const { return _gBufferPosition; }
    AllocatedImage gBufferNormal() const { return _gBufferNormal; }
    AllocatedImage gBufferAlbedo() const { return _gBufferAlbedo; }
    AllocatedImage idBuffer() const { return _idBuffer; }
    VkExtent2D windowExtent() const { return _windowExtent; }

    bool resize_requested{false};

private:
    DeviceManager *_deviceManager = nullptr;
    ResourceManager* _resourceManager = nullptr;

    VkSwapchainKHR _swapchain = nullptr;
    VkFormat _swapchainImageFormat = {};
    VkExtent2D _swapchainExtent = {};
    VkExtent2D _windowExtent{1920, 1080};

    std::vector<VkImage> _swapchainImages;
    std::vector<VkImageView> _swapchainImageViews;

    AllocatedImage _drawImage = {};
    AllocatedImage _depthImage = {};
    AllocatedImage _gBufferPosition = {};
    AllocatedImage _gBufferNormal = {};
    AllocatedImage _gBufferAlbedo = {};
    AllocatedImage _idBuffer = {};

    DeletionQueue _deletionQueue;
};
