#include "vk_device.h"
#include "config.h"
#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

void DeviceManager::init_vulkan(SDL_Window *window)
{
   vkb::InstanceBuilder builder;

    //make the vulkan instance, with basic debug features
    auto inst_ret = builder.set_app_name("Example Vulkan Application")
            .request_validation_layers(kUseValidationLayers)
            .use_default_debug_messenger()
            .require_api_version(1, 3, 0)
            .build();

    vkb::Instance vkb_inst = inst_ret.value();

    //grab the instance
    _instance = vkb_inst.instance;
    _debug_messenger = vkb_inst.debug_messenger;

    SDL_Vulkan_CreateSurface(window, _instance, &_surface);

    VkPhysicalDeviceVulkan13Features features{.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
    features.dynamicRendering = true;
    features.synchronization2 = true;

    VkPhysicalDeviceVulkan12Features features12{.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};
    features12.bufferDeviceAddress = true;
    features12.descriptorIndexing = true;

    //use vkbootstrap to select a gpu.
    //We want a gpu that can write to the SDL surface and supports vulkan 1.3
    vkb::PhysicalDeviceSelector selector{vkb_inst};
    vkb::PhysicalDevice physicalDevice = selector
            .set_minimum_version(1, 3)
            .set_required_features_13(features)
            .set_required_features_12(features12)
            .set_surface(_surface)
            .select()
            .value();

    //physicalDevice.features.
    // Enable ray tracing extensions on the physical device if supported (before creating the DeviceBuilder)
    // Query ray tracing capability on the chosen physical device
    {
        VkPhysicalDeviceAccelerationStructureFeaturesKHR accelFeat{
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
        VkPhysicalDeviceRayQueryFeaturesKHR rayqFeat{
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR,
            .pNext = &accelFeat };
        VkPhysicalDeviceFeatures2 feats2{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2, .pNext = &rayqFeat };
        vkGetPhysicalDeviceFeatures2(physicalDevice.physical_device, &feats2);
        _rayQuerySupported      = (rayqFeat.rayQuery == VK_TRUE);
        _accelStructSupported   = (accelFeat.accelerationStructure == VK_TRUE);
        fmt::print("[Device] RayQuery support: {} | AccelStruct: {}\n",
                   _rayQuerySupported ? "yes" : "no",
                   _accelStructSupported ? "yes" : "no");

        if (_rayQuerySupported && _accelStructSupported)
        {
            physicalDevice.enable_extension_if_present(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
            physicalDevice.enable_extension_if_present(VK_KHR_RAY_QUERY_EXTENSION_NAME);
            physicalDevice.enable_extension_if_present(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
        }
    }

    //create the final vulkan device
    vkb::DeviceBuilder deviceBuilder{physicalDevice};

    // Enable ray query + accel struct features in device create pNext if supported
    if (_rayQuerySupported && _accelStructSupported)
    {
        VkPhysicalDeviceAccelerationStructureFeaturesKHR accelReq{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
        accelReq.accelerationStructure = VK_TRUE;
        VkPhysicalDeviceRayQueryFeaturesKHR rayqReq{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR };
        rayqReq.pNext = &accelReq;
        rayqReq.rayQuery = VK_TRUE;
        deviceBuilder.add_pNext(&rayqReq);
    }

    vkb::Device vkbDevice = deviceBuilder.build().value();

    // Get the VkDevice handle used in the rest of a vulkan application
    _device = vkbDevice.device;
    _chosenGPU = physicalDevice.physical_device;

    // use vkbootstrap to get a Graphics queue
    _graphicsQueue = vkbDevice.get_queue(vkb::QueueType::graphics).value();

    _graphicsQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::graphics).value();

    //> vma_init
    //initialize the memory allocator
    VmaAllocatorCreateInfo allocatorInfo = {};
    allocatorInfo.physicalDevice = _chosenGPU;
    allocatorInfo.device = _device;
    allocatorInfo.instance = _instance;
    allocatorInfo.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
    vmaCreateAllocator(&allocatorInfo, &_allocator);

    _deletionQueue.push_function([&]() {
        vmaDestroyAllocator(_allocator);
    });
    //< vma_init
}

void DeviceManager::cleanup()
{
    vkDestroySurfaceKHR(_instance, _surface, nullptr);
    _deletionQueue.flush();
    vkDestroyDevice(_device, nullptr);
    vkb::destroy_debug_utils_messenger(_instance, _debug_messenger);
    vkDestroyInstance(_instance, nullptr);
    fmt::print("DeviceManager::cleanup()\n");
}
