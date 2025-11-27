#include "manager.h"
#include "core/device/device.h"
#include "descriptors.h"

void DescriptorManager::init(DeviceManager *deviceManager)
{
    _deviceManager = deviceManager;

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _singleImageDescriptorLayout = builder.build(
            _deviceManager->device(), VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr, VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    } {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        if (_deviceManager->supportsAccelerationStructure())
        {
            // TLAS for ray query (set=0,binding=1)
            builder.add_binding(1, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR);
        }
        _gpuSceneDataDescriptorLayout = builder.build(
            _deviceManager->device(), VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr, VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    }

}

void DescriptorManager::cleanup()
{
    if (!_deviceManager) return;

    if (_singleImageDescriptorLayout)
    {
        vkDestroyDescriptorSetLayout(_deviceManager->device(), _singleImageDescriptorLayout, nullptr);
        _singleImageDescriptorLayout = VK_NULL_HANDLE;
    }
    if (_gpuSceneDataDescriptorLayout)
    {
        vkDestroyDescriptorSetLayout(_deviceManager->device(), _gpuSceneDataDescriptorLayout, nullptr);
        _gpuSceneDataDescriptorLayout = VK_NULL_HANDLE;
    }
}
