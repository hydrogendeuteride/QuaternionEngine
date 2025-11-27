#include "ibl_manager.h"
#include <core/context.h>
#include <core/device/resource.h>
#include <core/assets/ktx_loader.h>
#include <core/pipeline/sampler.h>
#include <core/descriptor/descriptors.h>
#include <cmath>
#include <algorithm>
#include <ktx.h>
#include <SDL_stdinc.h>

#include "core/device/device.h"
#include "core/assets/texture_cache.h"

bool IBLManager::load(const IBLPaths &paths)
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr) return false;
    ResourceManager *rm = _ctx->getResources();

    // When uploads are deferred into the RenderGraph, any previously queued
    // image uploads might still reference VkImage handles owned by this
    // manager. Before destroying or recreating IBL images, flush those
    // uploads via the immediate path so we never record barriers or copies
    // for images that have been destroyed.
    if (rm->deferred_uploads() && rm->has_pending_uploads())
    {
        rm->process_queued_uploads_immediate();
    }

    // Allow reloading at runtime: destroy previous images/SH but keep layout.
    destroy_images_and_sh();
    ensureLayout();

    // Load specular environment: prefer cubemap; fallback to 2D equirect with mips.
    // Also hint the TextureCache (if present) so future switches are cheap.
    if (!paths.specularCube.empty())
    {
        // Try as cubemap first
        ktxutil::KtxCubemap kcm{};
        if (ktxutil::load_ktx2_cubemap(paths.specularCube.c_str(), kcm))
        {
            _spec = rm->create_image_compressed_layers(
                kcm.bytes.data(), kcm.bytes.size(),
                kcm.fmt, kcm.mipLevels, kcm.layers,
                kcm.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT,
                kcm.imgFlags
            );
        }
        else
        {
            ktxutil::Ktx2D k2d{};
            if (ktxutil::load_ktx2_2d(paths.specularCube.c_str(), k2d))
            {
                std::vector<ResourceManager::MipLevelCopy> lv;
                lv.reserve(k2d.mipLevels);
                for (uint32_t mip = 0; mip < k2d.mipLevels; ++mip)
                {
                    const auto &r = k2d.copies[mip];
                    lv.push_back(ResourceManager::MipLevelCopy{
                        .offset = r.bufferOffset,
                        .length = 0,
                        .width = r.imageExtent.width,
                        .height = r.imageExtent.height,
                    });
                }
                _spec = rm->create_image_compressed(k2d.bytes.data(), k2d.bytes.size(), k2d.fmt, lv,
                                                    VK_IMAGE_USAGE_SAMPLED_BIT);

                ktxTexture2 *ktex = nullptr;
                if (ktxTexture2_CreateFromNamedFile(paths.specularCube.c_str(), KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT,
                                                    &ktex) == KTX_SUCCESS && ktex)
                {
                    const VkFormat fmt = static_cast<VkFormat>(ktex->vkFormat);
                    const bool isFloat16 = fmt == VK_FORMAT_R16G16B16A16_SFLOAT;
                    const bool isFloat32 = fmt == VK_FORMAT_R32G32B32A32_SFLOAT;
                    if (!ktxTexture2_NeedsTranscoding(ktex) && (isFloat16 || isFloat32) && ktex->baseWidth == 2 * ktex->
                        baseHeight)
                    {
                        const uint32_t W = ktex->baseWidth;
                        const uint32_t H = ktex->baseHeight;
                        const uint8_t *dataPtr = reinterpret_cast<const uint8_t *>(
                            ktxTexture_GetData(ktxTexture(ktex)));

                        // Compute 9 SH coefficients (irradiance) from equirect HDR
                        struct Vec3
                        {
                            float x, y, z;
                        };
                        auto half_to_float = [](uint16_t h)-> float {
                            uint16_t h_exp = (h & 0x7C00u) >> 10;
                            uint16_t h_sig = h & 0x03FFu;
                            uint32_t sign = (h & 0x8000u) << 16;
                            uint32_t f_e, f_sig;
                            if (h_exp == 0)
                            {
                                if (h_sig == 0)
                                {
                                    f_e = 0;
                                    f_sig = 0;
                                }
                                else
                                {
                                    // subnormals
                                    int e = -1;
                                    uint16_t sig = h_sig;
                                    while ((sig & 0x0400u) == 0)
                                    {
                                        sig <<= 1;
                                        --e;
                                    }
                                    sig &= 0x03FFu;
                                    f_e = uint32_t(127 - 15 + e) << 23;
                                    f_sig = uint32_t(sig) << 13;
                                }
                            }
                            else if (h_exp == 0x1Fu)
                            {
                                f_e = 0xFFu << 23;
                                f_sig = uint32_t(h_sig) << 13;
                            }
                            else
                            {
                                f_e = uint32_t(h_exp - 15 + 127) << 23;
                                f_sig = uint32_t(h_sig) << 13;
                            }
                            uint32_t f = sign | f_e | f_sig;
                            float out;
                            std::memcpy(&out, &f, 4);
                            return out;
                        };

                        auto sample_at = [&](uint32_t x, uint32_t y)-> Vec3 {
                            if (isFloat32)
                            {
                                const float *px = reinterpret_cast<const float *>(dataPtr) + 4ull * (y * W + x);
                                return {px[0], px[1], px[2]};
                            }
                            else
                            {
                                const uint16_t *px = reinterpret_cast<const uint16_t *>(dataPtr) + 4ull * (y * W + x);
                                return {half_to_float(px[0]), half_to_float(px[1]), half_to_float(px[2])};
                            }
                        };

                        constexpr int L = 2; // 2nd order (9 coeffs)
                        const float dtheta = float(M_PI) / float(H);
                        const float dphi = 2.f * float(M_PI) / float(W);
                        // Accumulate RGB SH coeffs
                        std::array<glm::vec3, 9> c{};
                        for (auto &v: c) v = glm::vec3(0);

                        auto sh_basis = [](const glm::vec3 &d)-> std::array<float, 9> {
                            const float x = d.x, y = d.y, z = d.z;
                            // Real SH, unnormalized constants
                            const float c0 = 0.2820947918f;
                            const float c1 = 0.4886025119f;
                            const float c2 = 1.0925484306f;
                            const float c3 = 0.3153915653f;
                            const float c4 = 0.5462742153f;
                            return {
                                c0,
                                c1 * y,
                                c1 * z,
                                c1 * x,
                                c2 * x * y,
                                c2 * y * z,
                                c3 * (3.f * z * z - 1.f),
                                c2 * x * z,
                                c4 * (x * x - y * y)
                            };
                        };

                        for (uint32_t y = 0; y < H; ++y)
                        {
                            float theta = (y + 0.5f) * dtheta; // [0,pi]
                            float sinT = std::sin(theta);
                            for (uint32_t x = 0; x < W; ++x)
                            {
                                float phi = (x + 0.5f) * dphi; // [0,2pi]
                                glm::vec3 dir = glm::vec3(std::cos(phi) * sinT, std::cos(theta), std::sin(phi) * sinT);
                                auto Lrgb = sample_at(x, y);
                                glm::vec3 Lvec(Lrgb.x, Lrgb.y, Lrgb.z);
                                auto Y = sh_basis(dir);
                                float dOmega = dphi * dtheta * sinT; // solid angle per pixel
                                for (int i = 0; i < 9; ++i)
                                {
                                    c[i] += Lvec * (Y[i] * dOmega);
                                }
                            }
                        }
                        // Convolve with Lambert kernel via per-band scale
                        const float A0 = float(M_PI);
                        const float A1 = 2.f * float(M_PI) / 3.f;
                        const float A2 = float(M_PI) / 4.f;
                        const float Aband[3] = {A0, A1, A2};
                        for (int i = 0; i < 9; ++i)
                        {
                            int band = (i == 0) ? 0 : (i < 4 ? 1 : 2);
                            c[i] *= Aband[band];
                        }

                        _shBuffer = rm->create_buffer(sizeof(glm::vec4) * 9, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                      VMA_MEMORY_USAGE_CPU_TO_GPU);
                        for (int i = 0; i < 9; ++i)
                        {
                            glm::vec4 v(c[i], 0.0f);
                            std::memcpy(reinterpret_cast<char *>(_shBuffer.info.pMappedData) + i * sizeof(glm::vec4),
                                        &v, sizeof(glm::vec4));
                        }
                        vmaFlushAllocation(_ctx->getDevice()->allocator(), _shBuffer.allocation, 0,
                                           sizeof(glm::vec4) * 9);
                    }
                    ktxTexture_Destroy(ktxTexture(ktex));
                }
            }
        }
    }

    // Diffuse cubemap (optional; if missing, reuse specular)
    if (!paths.diffuseCube.empty())
    {
        ktxutil::KtxCubemap kcm{};
        if (ktxutil::load_ktx2_cubemap(paths.diffuseCube.c_str(), kcm))
        {
            _diff = rm->create_image_compressed_layers(
                kcm.bytes.data(), kcm.bytes.size(),
                kcm.fmt, kcm.mipLevels, kcm.layers,
                kcm.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT,
                kcm.imgFlags
            );
        }
    }
    if (_diff.image == VK_NULL_HANDLE && _spec.image != VK_NULL_HANDLE)
    {
        _diff = _spec;
    }

    // If background is still missing but specular is valid, reuse the specular environment.
    if (_background.image == VK_NULL_HANDLE && _spec.image != VK_NULL_HANDLE)
    {
        _background = _spec;
    }

    // BRDF LUT
    if (!paths.brdfLut2D.empty())
    {
        ktxutil::Ktx2D lut{};
        if (ktxutil::load_ktx2_2d(paths.brdfLut2D.c_str(), lut))
        {
            std::vector<ResourceManager::MipLevelCopy> lv;
            lv.reserve(lut.mipLevels);
            for (uint32_t mip = 0; mip < lut.mipLevels; ++mip)
            {
                const auto &r = lut.copies[mip];
                lv.push_back(ResourceManager::MipLevelCopy{
                    .offset = r.bufferOffset,
                    .length = 0,
                    .width = r.imageExtent.width,
                    .height = r.imageExtent.height,
                });
            }
            _brdf = rm->create_image_compressed(lut.bytes.data(), lut.bytes.size(), lut.fmt, lv,
                                                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    }

    return (_spec.image != VK_NULL_HANDLE) && (_diff.image != VK_NULL_HANDLE);
}

void IBLManager::unload()
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr) return;

    // Destroy images and SH buffer first.
    destroy_images_and_sh();

    // Then release descriptor layout.
    if (_iblSetLayout && _ctx && _ctx->getDevice())
    {
        vkDestroyDescriptorSetLayout(_ctx->getDevice()->device(), _iblSetLayout, nullptr);
        _iblSetLayout = VK_NULL_HANDLE;
    }
}

bool IBLManager::ensureLayout()
{
    if (_iblSetLayout != VK_NULL_HANDLE) return true;
    if (!_ctx || !_ctx->getDevice()) return false;

    DescriptorLayoutBuilder builder;
    // binding 0: environment/specular as 2D equirect with mips
    builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // binding 1: BRDF LUT 2D
    builder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // binding 2: SH coefficients UBO (vec4[9])
    builder.add_binding(2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    // binding 3: optional background environment texture (2D equirect)
    builder.add_binding(3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    _iblSetLayout = builder.build(
        _ctx->getDevice()->device(), VK_SHADER_STAGE_FRAGMENT_BIT,
        nullptr, VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT);
    return _iblSetLayout != VK_NULL_HANDLE;
}

void IBLManager::destroy_images_and_sh()
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr) return;
    auto *rm = _ctx->getResources();

    if (_spec.image)
    {
        rm->destroy_image(_spec);
    }
    // Handle potential aliasing: _diff may have been set to _spec in load().
    if (_diff.image && _diff.image != _spec.image)
    {
        rm->destroy_image(_diff);
    }
    // _background may alias _spec or _diff; only destroy when unique.
    if (_background.image &&
        _background.image != _spec.image &&
        _background.image != _diff.image)
    {
        rm->destroy_image(_background);
    }
    if (_brdf.image)
    {
        rm->destroy_image(_brdf);
    }

    if (_shBuffer.buffer)
    {
        rm->destroy_buffer(_shBuffer);
        _shBuffer = {};
    }

    _spec = {};
    _diff = {};
    _background = {};
    _brdf = {};
}
