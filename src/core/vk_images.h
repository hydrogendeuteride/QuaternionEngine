#pragma once
#include <core/vk_types.h>

namespace vkutil {

    void transition_image(VkCommandBuffer cmd, VkImage image, VkImageLayout currentLayout, VkImageLayout newLayout);

    void copy_image_to_image(VkCommandBuffer cmd, VkImage source, VkImage destination, VkExtent2D srcSize, VkExtent2D dstSize);
    void generate_mipmaps(VkCommandBuffer cmd, VkImage image, VkExtent2D imageSize);
    // Variant that generates exactly mipLevels levels (starting at base level 0).
    void generate_mipmaps_levels(VkCommandBuffer cmd, VkImage image, VkExtent2D imageSize, int mipLevels);
};
