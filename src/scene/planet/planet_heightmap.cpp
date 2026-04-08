#include "planet_heightmap.h"

#include <cstring>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <unordered_map>
#include <utility>

#include <glm/glm.hpp>
#if !defined(PLANET_HEIGHTMAP_DISABLE_KTX_IO)
#include <ktx.h>
#include <vulkan/vulkan.h>
#endif

namespace planet
{
    namespace
    {
        struct HeightView
        {
            uint32_t width = 0;
            uint32_t height = 0;
            const std::vector<uint16_t> *texels = nullptr;
        };

        struct HeightVarianceView
        {
            uint32_t width = 0;
            uint32_t height = 0;
            const std::vector<float> *texels = nullptr;
        };

        std::mutex g_preloaded_heightmap_mutex;
        std::unordered_map<std::string, HeightFaceSet> g_preloaded_heightmaps;

        HeightView height_view_for_mip(const HeightFace &face, uint32_t mip_level)
        {
            if (mip_level == 0u)
            {
                return HeightView{face.width, face.height, &face.texels};
            }

            const uint32_t index = mip_level - 1u;
            if (index >= face.mips.size())
            {
                return HeightView{};
            }

            const HeightMip &mip = face.mips[index];
            return HeightView{mip.width, mip.height, &mip.texels};
        }

        HeightVarianceView height_variance_view_for_mip(const HeightFace &face, uint32_t mip_level)
        {
            if (mip_level == 0u)
            {
                return HeightVarianceView{};
            }

            const uint32_t index = mip_level - 1u;
            if (index >= face.variance_mips.size())
            {
                return HeightVarianceView{};
            }

            const HeightVarianceMip &mip = face.variance_mips[index];
            return HeightVarianceView{mip.width, mip.height, &mip.texels};
        }

        uint32_t height_level_count(const HeightFace &face)
        {
            return 1u + static_cast<uint32_t>(face.mips.size());
        }

        void build_height_mips_internal(HeightFace &face)
        {
            face.mips.clear();
            face.variance_mips.clear();
            if (face.width == 0u || face.height == 0u || face.texels.empty())
            {
                return;
            }

            uint32_t src_w = face.width;
            uint32_t src_h = face.height;
            std::vector<float> src_mean(static_cast<size_t>(src_w) * src_h);
            std::vector<float> src_variance(static_cast<size_t>(src_w) * src_h, 0.0f);
            for (size_t i = 0; i < src_mean.size(); ++i)
            {
                src_mean[i] = static_cast<float>(face.texels[i]) * (1.0f / 65535.0f);
            }

            while (src_w > 1u || src_h > 1u)
            {
                const uint32_t dst_w = std::max(1u, (src_w + 1u) >> 1u);
                const uint32_t dst_h = std::max(1u, (src_h + 1u) >> 1u);

                HeightMip mip{};
                mip.width = dst_w;
                mip.height = dst_h;
                mip.texels.resize(static_cast<size_t>(dst_w) * dst_h);

                HeightVarianceMip variance_mip{};
                variance_mip.width = dst_w;
                variance_mip.height = dst_h;
                variance_mip.texels.resize(static_cast<size_t>(dst_w) * dst_h);

                std::vector<float> dst_mean(static_cast<size_t>(dst_w) * dst_h, 0.0f);
                std::vector<float> dst_variance(static_cast<size_t>(dst_w) * dst_h, 0.0f);

                for (uint32_t y = 0; y < dst_h; ++y)
                {
                    const uint32_t y0 = std::min(src_h - 1u, y * 2u);
                    const uint32_t y1 = std::min(src_h - 1u, y0 + 1u);
                    for (uint32_t x = 0; x < dst_w; ++x)
                    {
                        const uint32_t x0 = std::min(src_w - 1u, x * 2u);
                        const uint32_t x1 = std::min(src_w - 1u, x0 + 1u);

                        const size_t idx_a = static_cast<size_t>(y0) * src_w + x0;
                        const size_t idx_b = static_cast<size_t>(y0) * src_w + x1;
                        const size_t idx_c = static_cast<size_t>(y1) * src_w + x0;
                        const size_t idx_d = static_cast<size_t>(y1) * src_w + x1;

                        const float mean_a = src_mean[idx_a];
                        const float mean_b = src_mean[idx_b];
                        const float mean_c = src_mean[idx_c];
                        const float mean_d = src_mean[idx_d];

                        const float var_a = src_variance[idx_a];
                        const float var_b = src_variance[idx_b];
                        const float var_c = src_variance[idx_c];
                        const float var_d = src_variance[idx_d];

                        const float mean = 0.25f * (mean_a + mean_b + mean_c + mean_d);
                        const float mean_square = 0.25f * (
                            (var_a + mean_a * mean_a) +
                            (var_b + mean_b * mean_b) +
                            (var_c + mean_c * mean_c) +
                            (var_d + mean_d * mean_d));
                        const float variance = std::max(0.0f, mean_square - mean * mean);
                        const float normalized_variance = glm::clamp(variance * 4.0f, 0.0f, 1.0f);

                        const size_t dst_index = static_cast<size_t>(y) * dst_w + x;
                        dst_mean[dst_index] = mean;
                        dst_variance[dst_index] = variance;
                        mip.texels[dst_index] =
                            static_cast<uint16_t>(glm::clamp(std::round(mean * 65535.0f), 0.0f, 65535.0f));
                        variance_mip.texels[dst_index] = normalized_variance;
                    }
                }

                src_w = dst_w;
                src_h = dst_h;
                face.mips.push_back(std::move(mip));
                face.variance_mips.push_back(std::move(variance_mip));
                src_mean = std::move(dst_mean);
                src_variance = std::move(dst_variance);
            }
        }

#if !defined(PLANET_HEIGHTMAP_DISABLE_KTX_IO)
        void decode_bc4_unorm_block(const uint8_t *block, uint16_t out_texels[16])
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
                out_texels[i] = static_cast<uint16_t>(lut[code]) * 257u;
            }
        }
#endif
    } // namespace

    bool load_heightmap_bc4(const std::string &path, HeightFace &out_face)
    {
        out_face = {};
#if defined(PLANET_HEIGHTMAP_DISABLE_KTX_IO)
        (void) path;
        return false;
#else
        std::error_code ec;
        if (path.empty() || !std::filesystem::exists(path, ec) || ec)
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
        if (fmt != VK_FORMAT_BC4_UNORM_BLOCK && fmt != VK_FORMAT_R16_UNORM)
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

        out_face.width = w;
        out_face.height = h;
        out_face.texels.resize(static_cast<size_t>(w) * static_cast<size_t>(h));

        const uint8_t *src = data + off;
        if (fmt == VK_FORMAT_R16_UNORM)
        {
            const size_t expected_bytes = static_cast<size_t>(w) * static_cast<size_t>(h) * sizeof(uint16_t);
            if (image_size < expected_bytes)
            {
                ktxTexture_Destroy(ktxTexture(ktex));
                return false;
            }
            std::memcpy(out_face.texels.data(), src, expected_bytes);
        }
        else
        {
            const uint32_t blocks_x = (w + 3u) / 4u;
            const uint32_t blocks_y = (h + 3u) / 4u;
            const size_t expected_bytes = static_cast<size_t>(blocks_x) * static_cast<size_t>(blocks_y) * 8u;
            if (image_size < expected_bytes)
            {
                ktxTexture_Destroy(ktxTexture(ktex));
                return false;
            }

            uint16_t block_out[16] = {};
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
        }

        ktxTexture_Destroy(ktxTexture(ktex));
        build_height_mips_internal(out_face);
        return true;
#endif
    }

    bool load_heightmap_cube_faces_bc4(const std::string &directory_path, HeightFaceSet &out_faces)
    {
        out_faces = {};
#if defined(PLANET_HEIGHTMAP_DISABLE_KTX_IO)
        (void) directory_path;
        return false;
#else
        std::error_code ec;
        if (directory_path.empty() || !std::filesystem::exists(directory_path, ec) || ec)
        {
            return false;
        }

        HeightFaceSet loaded_faces{};
        for (size_t face_index = 0; face_index < loaded_faces.size(); ++face_index)
        {
            const CubeFace face = static_cast<CubeFace>(face_index);
            const std::string path =
                    directory_path + "/" + std::string(cube_face_name(face)) + ".ktx2";
            if (!load_heightmap_bc4(path, loaded_faces[face_index]))
            {
                return false;
            }
        }

        out_faces = std::move(loaded_faces);
        return true;
#endif
    }

    void retain_preloaded_heightmap_faces(const std::string &height_dir_key, HeightFaceSet faces)
    {
        if (height_dir_key.empty())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(g_preloaded_heightmap_mutex);
        g_preloaded_heightmaps[height_dir_key] = std::move(faces);
    }

    bool take_preloaded_heightmap_faces(const std::string &height_dir_key, HeightFaceSet &out_faces)
    {
        out_faces = {};
        if (height_dir_key.empty())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(g_preloaded_heightmap_mutex);
        const auto it = g_preloaded_heightmaps.find(height_dir_key);
        if (it == g_preloaded_heightmaps.end())
        {
            return false;
        }

        out_faces = std::move(it->second);
        g_preloaded_heightmaps.erase(it);
        return true;
    }

    void clear_preloaded_heightmap_faces()
    {
        std::lock_guard<std::mutex> lock(g_preloaded_heightmap_mutex);
        g_preloaded_heightmaps.clear();
    }

    void rebuild_height_mips(HeightFace &face)
    {
        build_height_mips_internal(face);
    }

    float sample_height(const HeightFace &face, float u, float v, uint32_t mip_level)
    {
        const HeightView view = height_view_for_mip(face, mip_level);
        if (view.width == 0 || view.height == 0 || !view.texels || view.texels->empty())
        {
            return 0.0f;
        }

        const float uu = glm::clamp(u, 0.0f, 1.0f);
        const float vv = glm::clamp(v, 0.0f, 1.0f);

        const float x = uu * static_cast<float>(view.width - 1u);
        const float y = vv * static_cast<float>(view.height - 1u);

        const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
        const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
        const uint32_t x1 = std::min(x0 + 1u, view.width - 1u);
        const uint32_t y1 = std::min(y0 + 1u, view.height - 1u);

        const float tx = x - static_cast<float>(x0);
        const float ty = y - static_cast<float>(y0);

        auto texel = [&](uint32_t xi, uint32_t yi) -> float
        {
            const uint16_t v16 = (*view.texels)[static_cast<size_t>(yi) * view.width + xi];
            return static_cast<float>(v16) * (1.0f / 65535.0f);
        };

        const float h00 = texel(x0, y0);
        const float h10 = texel(x1, y0);
        const float h01 = texel(x0, y1);
        const float h11 = texel(x1, y1);

        const float hx0 = glm::mix(h00, h10, tx);
        const float hx1 = glm::mix(h01, h11, tx);
        return glm::mix(hx0, hx1, ty);
    }

    float sample_height_variance(const HeightFace &face, float u, float v, uint32_t mip_level)
    {
        const HeightVarianceView view = height_variance_view_for_mip(face, mip_level);
        if (view.width == 0 || view.height == 0 || !view.texels || view.texels->empty())
        {
            return 0.0f;
        }

        const float uu = glm::clamp(u, 0.0f, 1.0f);
        const float vv = glm::clamp(v, 0.0f, 1.0f);

        const float x = uu * static_cast<float>(view.width - 1u);
        const float y = vv * static_cast<float>(view.height - 1u);

        const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
        const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
        const uint32_t x1 = std::min(x0 + 1u, view.width - 1u);
        const uint32_t y1 = std::min(y0 + 1u, view.height - 1u);

        const float tx = x - static_cast<float>(x0);
        const float ty = y - static_cast<float>(y0);

        auto texel = [&](uint32_t xi, uint32_t yi) -> float
        {
            return (*view.texels)[static_cast<size_t>(yi) * view.width + xi];
        };

        const float h00 = texel(x0, y0);
        const float h10 = texel(x1, y0);
        const float h01 = texel(x0, y1);
        const float h11 = texel(x1, y1);

        const float hx0 = glm::mix(h00, h10, tx);
        const float hx1 = glm::mix(h01, h11, tx);
        return glm::mix(hx0, hx1, ty);
    }

    uint32_t choose_height_mip_level(const HeightFace &face, uint32_t patch_level, uint32_t patch_resolution)
    {
        if (face.width == 0u || face.height == 0u || face.texels.empty())
        {
            return 0u;
        }

        const uint32_t safe_res = std::max(2u, patch_resolution);
        const double texels_per_patch = std::max(1.0, std::ldexp(static_cast<double>(face.width), -static_cast<int>(patch_level)));
        const double texels_per_segment = texels_per_patch / static_cast<double>(safe_res - 1u);
        if (!(texels_per_segment > 1.0))
        {
            return 0u;
        }

        const uint32_t max_mip = height_level_count(face) - 1u;
        const double mip_f = std::floor(std::log2(texels_per_segment));
        return std::min(max_mip, static_cast<uint32_t>(std::max(0.0, mip_f)));
    }

} // namespace planet
