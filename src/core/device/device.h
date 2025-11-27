#pragma once
#include <core/types.h>
#include "VkBootstrap.h"


class DeviceManager
{
public:
    void init_vulkan(struct SDL_Window *window);

    void cleanup();

    VkDevice device() const { return _device; }
    VkInstance instance() const { return _instance; }
    VkPhysicalDevice physicalDevice() const { return _chosenGPU; }
    VkSurfaceKHR surface() const { return _surface; }
    VkQueue graphicsQueue() const { return _graphicsQueue; }
    uint32_t graphicsQueueFamily() const { return _graphicsQueueFamily; }
    VmaAllocator allocator() const { return _allocator; }
    VkDebugUtilsMessengerEXT debugMessenger() { return _debug_messenger; }

    // Ray tracing capabilities (queried at init; not necessarily enabled)
    bool supportsRayQuery() const { return _rayQuerySupported; }
    bool supportsAccelerationStructure() const { return _accelStructSupported; }

private:
    VkInstance _instance = nullptr;
    VkDebugUtilsMessengerEXT _debug_messenger = nullptr;
    VkPhysicalDevice _chosenGPU = nullptr;
    VkDevice _device = nullptr;
    VkSurfaceKHR _surface = nullptr;
    VkQueue _graphicsQueue = nullptr;
    uint32_t _graphicsQueueFamily = 0;
    VmaAllocator _allocator = nullptr;

    DeletionQueue _deletionQueue;

    // Cached feature support flags
    bool _rayQuerySupported{false};
    bool _accelStructSupported{false};
};
