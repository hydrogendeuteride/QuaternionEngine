#include "context.h"
#include "scene/vk_scene.h"
#include "core/device/device.h"
#include "core/device/resource.h"
#include "core/frame/resources.h"
#include "core/descriptor/manager.h"
#include "core/descriptor/descriptors.h"

#include <algorithm>
#include <vk_mem_alloc.h>

const GPUSceneData &EngineContext::getSceneData() const
{
    return scene->getSceneData();
}

const DrawContext &EngineContext::getMainDrawContext() const
{
    return const_cast<SceneManager *>(scene)->getMainDrawContext();
}

uint32_t EngineContext::getShadowMapResolution() const
{
    uint32_t res = shadowSettings.shadowMapResolution;
    if (res == 0)
    {
        res = kShadowMapResolution;
    }

    // Safety clamp to avoid accidental giant allocations; also clamp to device limits when available.
    uint32_t maxDim = 8192u;
    if (DeviceManager *dev = getDevice())
    {
        if (VkPhysicalDevice gpu = dev->physicalDevice())
        {
            VkPhysicalDeviceProperties props{};
            vkGetPhysicalDeviceProperties(gpu, &props);
            maxDim = std::min<uint32_t>(maxDim, props.limits.maxImageDimension2D);
        }
    }

    res = std::clamp(res, 256u, maxDim);
    return res;
}

void EngineContext::setShadowMapResolution(uint32_t resolution)
{
    shadowSettings.shadowMapResolution = resolution;
    // Normalize/clamp immediately so other systems see a valid value.
    shadowSettings.shadowMapResolution = getShadowMapResolution();
}

VkDescriptorSet EngineContext::getOrCreateSceneDataDescriptor()
{
    if (_cachedSceneDataFrame == frameIndex && _cachedSceneDataSet != VK_NULL_HANDLE)
    {
        return _cachedSceneDataSet;
    }

    DeviceManager *dev = getDevice();
    ResourceManager *res = getResources();
    DescriptorManager *layouts = getDescriptorLayouts();
    if (!dev || !res || !layouts || !currentFrame) return VK_NULL_HANDLE;

    AllocatedBuffer buf = res->create_buffer(
        sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
    currentFrame->_deletionQueue.push_function([res, buf]() { res->destroy_buffer(buf); });

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(dev->allocator(), buf.allocation, &allocInfo);
    *static_cast<GPUSceneData *>(allocInfo.pMappedData) = getSceneData();
    vmaFlushAllocation(dev->allocator(), buf.allocation, 0, sizeof(GPUSceneData));

    VkDescriptorSet set = currentFrame->_frameDescriptors.allocate(
        dev->device(), layouts->gpuSceneDataLayout());
    DescriptorWriter writer;
    writer.write_buffer(0, buf.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.update_set(dev->device(), set);

    _cachedSceneDataSet = set;
    _cachedSceneDataBuffer = buf;
    _cachedSceneDataFrame = frameIndex;
    return set;
}
