#pragma once

#include <core/types.h>
#include <vector>

// Simple KTX2 helpers focused on IBL assets.
// Uses libktx to open and (if needed) transcode to GPU-ready BC formats.
namespace ktxutil
{
    struct KtxCubemap
    {
        VkFormat fmt{VK_FORMAT_UNDEFINED};
        uint32_t baseW{0};
        uint32_t baseH{0};
        uint32_t mipLevels{0};
        uint32_t layers{0}; // total array layers in the Vulkan image (faces × layers)
        std::vector<uint8_t> bytes; // full file data block returned by libktx
        std::vector<VkBufferImageCopy> copies; // one per (mip × layer)
        VkImageCreateFlags imgFlags{0}; // e.g., VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT
    };

    // Loads a .ktx2 cubemap (or cubemap array) and prepares copy regions for upload.
    // - Prefers BC6H UFLOAT for HDR content; leaves existing GPU BC formats intact.
    // - Returns true on success and fills 'out'.
    bool load_ktx2_cubemap(const char* path, KtxCubemap& out);

    // Optional: minimal 2D loader for BRDF LUTs (RG/BC5 etc.). Returns VkFormat and copies per mip.
    struct Ktx2D
    {
        VkFormat fmt{VK_FORMAT_UNDEFINED};
        uint32_t baseW{0};
        uint32_t baseH{0};
        uint32_t mipLevels{0};
        std::vector<uint8_t> bytes;
        std::vector<VkBufferImageCopy> copies;
    };
    bool load_ktx2_2d(const char* path, Ktx2D& out);
}

