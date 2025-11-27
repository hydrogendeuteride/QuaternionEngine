#pragma once

#include <core/types.h>
#include <core/descriptor/descriptors.h>

#include "device/device.h"

class DeviceManager;

class DescriptorManager
{
public:
    void init(DeviceManager *deviceManager);

    void cleanup();

    VkDescriptorSetLayout gpuSceneDataLayout() const { return _gpuSceneDataDescriptorLayout; }
    VkDescriptorSetLayout singleImageLayout() const { return _singleImageDescriptorLayout; }

private:
    DeviceManager *_deviceManager = nullptr;
    VkDescriptorSetLayout _singleImageDescriptorLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout _gpuSceneDataDescriptorLayout = VK_NULL_HANDLE;
};
