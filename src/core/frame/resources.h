#pragma once

#include <core/types.h>
#include <core/descriptor/descriptors.h>

class DeviceManager;

// Per-frame state used by the renderer and passes.
// Owns a command buffer, sync primitives, a transient descriptor pool, and a
// deletion queue for resources that should be destroyed when the frame is done.
struct FrameResources
{
    VkSemaphore _swapchainSemaphore = VK_NULL_HANDLE;
    VkSemaphore _renderSemaphore = VK_NULL_HANDLE;
    VkFence _renderFence = VK_NULL_HANDLE;

    VkCommandPool _commandPool = VK_NULL_HANDLE;
    VkCommandBuffer _mainCommandBuffer = VK_NULL_HANDLE;

    DeletionQueue _deletionQueue;
    DescriptorAllocatorGrowable _frameDescriptors;

    void init(DeviceManager *deviceManager,
              std::span<DescriptorAllocatorGrowable::PoolSizeRatio> framePoolSizes);

    void cleanup(DeviceManager *deviceManager);
};
