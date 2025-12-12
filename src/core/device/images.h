#pragma once
#include <core/types.h>
#include <glm/vec2.hpp>

namespace vkutil {

    void transition_image(VkCommandBuffer cmd, VkImage image, VkImageLayout currentLayout, VkImageLayout newLayout);

    // Compute a letterboxed destination rect inside dstSize that preserves srcSize aspect ratio.
    VkRect2D compute_letterbox_rect(VkExtent2D srcSize, VkExtent2D dstSize);

    // Map a window-space pixel position (in dstSize, top-left origin) into pixel coordinates
    // inside the letterboxed srcSize view. Returns false if the position lies in black bars.
    bool map_window_to_letterbox_src(const glm::vec2 &windowPosPixels,
                                     VkExtent2D srcSize,
                                     VkExtent2D dstSize,
                                     glm::vec2 &outSrcPosPixels);

    void copy_image_to_image(VkCommandBuffer cmd, VkImage source, VkImage destination, VkExtent2D srcSize, VkExtent2D dstSize);
    // Blit source into a letterboxed rect in destination (preserves aspect ratio).
    void copy_image_to_image_letterboxed(VkCommandBuffer cmd,
                                         VkImage source,
                                         VkImage destination,
                                         VkExtent2D srcSize,
                                         VkExtent2D dstSize,
                                         VkFilter filter = VK_FILTER_LINEAR);
    void generate_mipmaps(VkCommandBuffer cmd, VkImage image, VkExtent2D imageSize);
    // Variant that generates exactly mipLevels levels (starting at base level 0).
    void generate_mipmaps_levels(VkCommandBuffer cmd, VkImage image, VkExtent2D imageSize, int mipLevels);
};
