#include "ktx_loader.h"

#include <ktx.h>
#include <ktxvulkan.h>
#include <filesystem>

namespace ktxutil
{
    static inline bool is_bc_format(VkFormat f)
    {
        switch (f)
        {
            case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
            case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
            case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
            case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
            case VK_FORMAT_BC2_UNORM_BLOCK:
            case VK_FORMAT_BC2_SRGB_BLOCK:
            case VK_FORMAT_BC3_UNORM_BLOCK:
            case VK_FORMAT_BC3_SRGB_BLOCK:
            case VK_FORMAT_BC4_UNORM_BLOCK:
            case VK_FORMAT_BC4_SNORM_BLOCK:
            case VK_FORMAT_BC5_UNORM_BLOCK:
            case VK_FORMAT_BC5_SNORM_BLOCK:
            case VK_FORMAT_BC6H_UFLOAT_BLOCK:
            case VK_FORMAT_BC6H_SFLOAT_BLOCK:
            case VK_FORMAT_BC7_UNORM_BLOCK:
            case VK_FORMAT_BC7_SRGB_BLOCK:
                return true;
            default: return false;
        }
    }

    static inline bool exists_file(const char* path)
    {
        std::error_code ec; return std::filesystem::exists(path, ec) && !ec;
    }

    bool load_ktx2_cubemap(const char* path, KtxCubemap& out)
    {
        out = KtxCubemap{};
        if (path == nullptr || !exists_file(path)) return false;

        ktxTexture2* ktex = nullptr;
        ktxResult kres = ktxTexture2_CreateFromNamedFile(path, KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &ktex);
        if (kres != KTX_SUCCESS || !ktex) return false;

        // Ensure it is a cubemap or cubemap array
        if (ktex->numFaces != 6)
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        // Transcoding path: for IBL HDR cubemaps we expect GPU-ready formats (e.g., BC6H or R16G16B16A16).
        // BasisU does not support BC6H transcoding. If the KTX2 requires transcoding, we bail out here
        // and expect assets to be pre-encoded to a GPU format.
        if (ktxTexture2_NeedsTranscoding(ktex))
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        VkFormat vkfmt = static_cast<VkFormat>(ktex->vkFormat);
        // Accept any GPU format (BC6H preferred). Non-BC formats like R16G16B16A16 are valid too.

        const uint32_t mipLevels = ktex->numLevels;
        const uint32_t baseW = ktex->baseWidth;
        const uint32_t baseH = ktex->baseHeight;
        const uint32_t layers = std::max(1u, ktex->numLayers) * 6u; // arrayLayers = layers × faces

        ktx_size_t totalSize = ktxTexture_GetDataSize(ktxTexture(ktex));
        const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(ktxTexture_GetData(ktxTexture(ktex)));

        out.fmt = vkfmt;
        out.baseW = baseW;
        out.baseH = baseH;
        out.mipLevels = mipLevels;
        out.layers = layers;
        out.imgFlags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        out.bytes.assign(dataPtr, dataPtr + totalSize);

        out.copies.clear();
        out.copies.reserve(static_cast<size_t>(mipLevels) * layers);

        for (uint32_t mip = 0; mip < mipLevels; ++mip)
        {
            const uint32_t w = std::max(1u, baseW >> mip);
            const uint32_t h = std::max(1u, baseH >> mip);
            for (uint32_t layer = 0; layer < std::max(1u, ktex->numLayers); ++layer)
            {
                for (uint32_t face = 0; face < 6; ++face)
                {
                    ktx_size_t off = 0;
                    ktxTexture_GetImageOffset(ktxTexture(ktex), mip, layer, face, &off);

                    VkBufferImageCopy r{};
                    r.bufferOffset = static_cast<VkDeviceSize>(off);
                    r.bufferRowLength = 0; // tightly packed
                    r.bufferImageHeight = 0;
                    r.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    r.imageSubresource.mipLevel = mip;
                    r.imageSubresource.baseArrayLayer = layer * 6u + face;
                    r.imageSubresource.layerCount = 1;
                    r.imageExtent = { w, h, 1 };
                    out.copies.push_back(r);
                }
            }
        }

        ktxTexture_Destroy(ktxTexture(ktex));
        return true;
    }

    bool load_ktx2_2d(const char* path, Ktx2D& out)
    {
        out = Ktx2D{};
        if (path == nullptr || !exists_file(path)) return false;

        ktxTexture2* ktex = nullptr;
        ktxResult kres = ktxTexture2_CreateFromNamedFile(path, KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &ktex);
        if (kres != KTX_SUCCESS || !ktex) return false;

        if (ktxTexture2_NeedsTranscoding(ktex))
        {
            // Common for BRDF LUTs: BC5 RG UNORM
            kres = ktxTexture2_TranscodeBasis(ktex, KTX_TTF_BC5_RG, 0);
            if (kres != KTX_SUCCESS)
            {
                ktxTexture_Destroy(ktxTexture(ktex));
                return false;
            }
        }

        VkFormat vkfmt = static_cast<VkFormat>(ktex->vkFormat);
        if (!is_bc_format(vkfmt))
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        const uint32_t mipLevels = ktex->numLevels;
        const uint32_t baseW = ktex->baseWidth;
        const uint32_t baseH = ktex->baseHeight;

        ktx_size_t totalSize = ktxTexture_GetDataSize(ktxTexture(ktex));
        const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(ktxTexture_GetData(ktxTexture(ktex)));

        out.fmt = vkfmt;
        out.baseW = baseW;
        out.baseH = baseH;
        out.mipLevels = mipLevels;
        out.bytes.assign(dataPtr, dataPtr + totalSize);

        out.copies.clear();
        out.copies.reserve(mipLevels);
        for (uint32_t mip = 0; mip < mipLevels; ++mip)
        {
            ktx_size_t off = 0;
            ktxTexture_GetImageOffset(ktxTexture(ktex), mip, 0, 0, &off);
            const uint32_t w = std::max(1u, baseW >> mip);
            const uint32_t h = std::max(1u, baseH >> mip);
            VkBufferImageCopy r{};
            r.bufferOffset = static_cast<VkDeviceSize>(off);
            r.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            r.imageSubresource.mipLevel = mip;
            r.imageSubresource.baseArrayLayer = 0;
            r.imageSubresource.layerCount = 1;
            r.imageExtent = { w, h, 1 };
            out.copies.push_back(r);
        }

        ktxTexture_Destroy(ktxTexture(ktex));
        return true;
    }
}
