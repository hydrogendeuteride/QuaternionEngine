#include <core/util/debug.h>

#include <cstring>

namespace vkdebug {

static inline PFN_vkCmdBeginDebugUtilsLabelEXT get_begin_fn(VkDevice device)
{
    static PFN_vkCmdBeginDebugUtilsLabelEXT fn = nullptr;
    static VkDevice cached = VK_NULL_HANDLE;
    if (device != cached)
    {
        cached = device;
        fn = reinterpret_cast<PFN_vkCmdBeginDebugUtilsLabelEXT>(
            vkGetDeviceProcAddr(device, "vkCmdBeginDebugUtilsLabelEXT"));
    }
    return fn;
}

static inline PFN_vkCmdEndDebugUtilsLabelEXT get_end_fn(VkDevice device)
{
    static PFN_vkCmdEndDebugUtilsLabelEXT fn = nullptr;
    static VkDevice cached = VK_NULL_HANDLE;
    if (device != cached)
    {
        cached = device;
        fn = reinterpret_cast<PFN_vkCmdEndDebugUtilsLabelEXT>(
            vkGetDeviceProcAddr(device, "vkCmdEndDebugUtilsLabelEXT"));
    }
    return fn;
}

void cmd_begin_label(VkDevice device, VkCommandBuffer cmd, const char* name,
                     float r, float g, float b, float a)
{
    auto fn = get_begin_fn(device);
    if (!fn) return;
    VkDebugUtilsLabelEXT label{};
    label.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
    label.pLabelName = name;
    label.color[0] = r; label.color[1] = g; label.color[2] = b; label.color[3] = a;
    fn(cmd, &label);
}

void cmd_end_label(VkDevice device, VkCommandBuffer cmd)
{
    auto fn = get_end_fn(device);
    if (!fn) return;
    fn(cmd);
}

} // namespace vkdebug

