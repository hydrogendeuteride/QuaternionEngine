#include "planet_heightmap.h"

#include <cmath>
#include <filesystem>

#include <glm/glm.hpp>
#include <ktx.h>
#include <vulkan/vulkan.h>

namespace planet
{
    namespace
    {
        bool file_exists(const std::string &path)
        {
            std::error_code ec;
            return std::filesystem::exists(path, ec) && !ec;
        }

        void decode_bc4_unorm_block(const uint8_t *block, uint8_t out_texels[16])
        {
            const uint8_t r0 = block[0];
            const uint8_t r1 = block[1];

            uint8_t lut[8] = {};
            lut[0] = r0;
            lut[1] = r1;
            if (r0 > r1)
            {
                // 6 interpolated values between r0 and r1.
                lut[2] = static_cast<uint8_t>((6u * r0 + 1u * r1 + 3u) / 7u);
                lut[3] = static_cast<uint8_t>((5u * r0 + 2u * r1 + 3u) / 7u);
                lut[4] = static_cast<uint8_t>((4u * r0 + 3u * r1 + 3u) / 7u);
                lut[5] = static_cast<uint8_t>((3u * r0 + 4u * r1 + 3u) / 7u);
                lut[6] = static_cast<uint8_t>((2u * r0 + 5u * r1 + 3u) / 7u);
                lut[7] = static_cast<uint8_t>((1u * r0 + 6u * r1 + 3u) / 7u);
            }
            else
            {
                // 4 interpolated values; then 0 and 255.
                lut[2] = static_cast<uint8_t>((4u * r0 + 1u * r1 + 2u) / 5u);
                lut[3] = static_cast<uint8_t>((3u * r0 + 2u * r1 + 2u) / 5u);
                lut[4] = static_cast<uint8_t>((2u * r0 + 3u * r1 + 2u) / 5u);
                lut[5] = static_cast<uint8_t>((1u * r0 + 4u * r1 + 2u) / 5u);
                lut[6] = 0u;
                lut[7] = 255u;
            }

            uint64_t bits = 0;
            for (int i = 0; i < 6; ++i)
            {
                bits |= (static_cast<uint64_t>(block[2 + i]) << (8 * i));
            }

            for (int i = 0; i < 16; ++i)
            {
                const uint8_t code = static_cast<uint8_t>((bits >> (3 * i)) & 0x7ull);
                out_texels[i] = lut[code];
            }
        }
    } // namespace

    bool load_heightmap_bc4(const std::string &path, HeightFace &out_face)
    {
        out_face = {};
        if (path.empty() || !file_exists(path))
        {
            return false;
        }

        ktxTexture2 *ktex = nullptr;
        const ktxResult kres =
            ktxTexture2_CreateFromNamedFile(path.c_str(), KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &ktex);
        if (kres != KTX_SUCCESS || !ktex)
        {
            return false;
        }

        if (ktxTexture2_NeedsTranscoding(ktex))
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        const VkFormat fmt = static_cast<VkFormat>(ktex->vkFormat);
        if (fmt != VK_FORMAT_BC4_UNORM_BLOCK)
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        if (ktex->numFaces != 1 || ktex->numLayers != 1)
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        const uint32_t w = ktex->baseWidth;
        const uint32_t h = ktex->baseHeight;
        if (w == 0 || h == 0)
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        ktx_size_t off = 0;
        ktxTexture_GetImageOffset(ktxTexture(ktex), 0, 0, 0, &off);
        const size_t image_size = static_cast<size_t>(ktxTexture_GetImageSize(ktxTexture(ktex), 0));
        const uint8_t *data = reinterpret_cast<const uint8_t *>(ktxTexture_GetData(ktxTexture(ktex)));
        if (!data || image_size == 0)
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        const uint32_t blocks_x = (w + 3u) / 4u;
        const uint32_t blocks_y = (h + 3u) / 4u;
        const size_t expected_bytes = static_cast<size_t>(blocks_x) * static_cast<size_t>(blocks_y) * 8u;
        if (image_size < expected_bytes)
        {
            ktxTexture_Destroy(ktxTexture(ktex));
            return false;
        }

        out_face.width = w;
        out_face.height = h;
        out_face.texels.resize(static_cast<size_t>(w) * static_cast<size_t>(h));

        const uint8_t *src = data + off;
        uint8_t block_out[16] = {};
        for (uint32_t by = 0; by < blocks_y; ++by)
        {
            for (uint32_t bx = 0; bx < blocks_x; ++bx)
            {
                const size_t block_index = static_cast<size_t>(by) * blocks_x + bx;
                const uint8_t *block = src + block_index * 8u;
                decode_bc4_unorm_block(block, block_out);

                for (uint32_t iy = 0; iy < 4u; ++iy)
                {
                    const uint32_t py = by * 4u + iy;
                    if (py >= h) continue;
                    for (uint32_t ix = 0; ix < 4u; ++ix)
                    {
                        const uint32_t px = bx * 4u + ix;
                        if (px >= w) continue;
                        out_face.texels[static_cast<size_t>(py) * w + px] = block_out[iy * 4u + ix];
                    }
                }
            }
        }

        ktxTexture_Destroy(ktxTexture(ktex));
        return true;
    }

    float sample_height(const HeightFace &face, float u, float v)
    {
        if (face.width == 0 || face.height == 0 || face.texels.empty())
        {
            return 0.0f;
        }

        const float uu = glm::clamp(u, 0.0f, 1.0f);
        const float vv = glm::clamp(v, 0.0f, 1.0f);

        const float x = uu * static_cast<float>(face.width - 1u);
        const float y = vv * static_cast<float>(face.height - 1u);

        const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
        const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
        const uint32_t x1 = std::min(x0 + 1u, face.width - 1u);
        const uint32_t y1 = std::min(y0 + 1u, face.height - 1u);

        const float tx = x - static_cast<float>(x0);
        const float ty = y - static_cast<float>(y0);

        auto texel = [&](uint32_t xi, uint32_t yi) -> float
        {
            const uint8_t v8 = face.texels[static_cast<size_t>(yi) * face.width + xi];
            return static_cast<float>(v8) * (1.0f / 255.0f);
        };

        const float h00 = texel(x0, y0);
        const float h10 = texel(x1, y0);
        const float h01 = texel(x0, y1);
        const float h11 = texel(x1, y1);

        const float hx0 = glm::mix(h00, h10, tx);
        const float hx1 = glm::mix(h01, h11, tx);
        return glm::mix(hx0, hx1, ty);
    }

} // namespace planet
