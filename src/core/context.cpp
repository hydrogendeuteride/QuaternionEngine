#include "context.h"
#include "scene/vk_scene.h"
#include "core/device/device.h"

#include <algorithm>

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
