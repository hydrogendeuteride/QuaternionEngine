#pragma once

#include <core/types.h>

namespace vkdebug
{
    // Begin a debug label on a command buffer if VK_EXT_debug_utils is available.
    void cmd_begin_label(VkDevice device, VkCommandBuffer cmd, const char *name,
                         float r = 0.2f, float g = 0.6f, float b = 0.9f, float a = 1.0f);

    // End a debug label on a command buffer if VK_EXT_debug_utils is available.
    void cmd_end_label(VkDevice device, VkCommandBuffer cmd);
}

