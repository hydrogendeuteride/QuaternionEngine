#include "resources.h"
#include <span>
#include "core/descriptor/descriptors.h"
#include "core/device/device.h"
#include "core/util/initializers.h"
#include "core/types.h"

void FrameResources::init(DeviceManager *deviceManager,
                          std::span<DescriptorAllocatorGrowable::PoolSizeRatio> framePoolSizes)
{
    VkCommandPoolCreateInfo commandPoolInfo = vkinit::command_pool_create_info(
        deviceManager->graphicsQueueFamily(), VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);
    VK_CHECK(vkCreateCommandPool(deviceManager->device(), &commandPoolInfo, nullptr, &_commandPool));

    VkCommandBufferAllocateInfo cmdAllocInfo = vkinit::command_buffer_allocate_info(_commandPool, 1);
    VK_CHECK(vkAllocateCommandBuffers(deviceManager->device(), &cmdAllocInfo, &_mainCommandBuffer));

    VkFenceCreateInfo fenceCreateInfo = vkinit::fence_create_info(VK_FENCE_CREATE_SIGNALED_BIT);
    VkSemaphoreCreateInfo semaphoreCreateInfo = vkinit::semaphore_create_info();

    VK_CHECK(vkCreateFence(deviceManager->device(), &fenceCreateInfo, nullptr, &_renderFence));
    VK_CHECK(vkCreateSemaphore(deviceManager->device(), &semaphoreCreateInfo, nullptr, &_swapchainSemaphore));
    VK_CHECK(vkCreateSemaphore(deviceManager->device(), &semaphoreCreateInfo, nullptr, &_renderSemaphore));

    _frameDescriptors.init(deviceManager->device(), 1000, framePoolSizes);
}

void FrameResources::cleanup(DeviceManager *deviceManager)
{
    _frameDescriptors.destroy_pools(deviceManager->device());

    if (_commandPool)
    {
        vkDestroyCommandPool(deviceManager->device(), _commandPool, nullptr);
        _commandPool = VK_NULL_HANDLE;
    }

    if (_renderFence)
    {
        vkDestroyFence(deviceManager->device(), _renderFence, nullptr);
        _renderFence = VK_NULL_HANDLE;
    }
    if (_renderSemaphore)
    {
        vkDestroySemaphore(deviceManager->device(), _renderSemaphore, nullptr);
        _renderSemaphore = VK_NULL_HANDLE;
    }
    if (_swapchainSemaphore)
    {
        vkDestroySemaphore(deviceManager->device(), _swapchainSemaphore, nullptr);
        _swapchainSemaphore = VK_NULL_HANDLE;
    }
}
