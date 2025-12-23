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
#include <thread>
#include <mutex>
#include <condition_variable>

#include "core/device/device.h"
#include "core/assets/texture_cache.h"

struct PreparedIBLData
{
    IBLPaths paths{};

    bool has_spec{false};
    bool spec_is_cubemap{false};
    ktxutil::KtxCubemap spec_cubemap{};
    ktxutil::Ktx2D spec_2d{};

    bool has_diffuse{false};
    ktxutil::KtxCubemap diff_cubemap{};

    bool has_background{false};
    ktxutil::Ktx2D background_2d{};

    bool has_brdf{false};
    ktxutil::Ktx2D brdf_2d{};

    bool has_sh{false};
    glm::vec4 sh[9]{};
};

namespace
{
    static bool compute_sh_from_ktx2_equirect(const char *path, glm::vec4 out_sh[9])
    {
        if (path == nullptr) return false;

        ktxTexture2 *ktex = nullptr;
        if (ktxTexture2_CreateFromNamedFile(path, KTX_TEXTURE_CREATE_LOAD_IMAGE_DATA_BIT, &ktex) != KTX_SUCCESS || !ktex)
        {
            return false;
        }

        bool ok = false;
        const VkFormat fmt = static_cast<VkFormat>(ktex->vkFormat);
        const bool isFloat16 = fmt == VK_FORMAT_R16G16B16A16_SFLOAT;
        const bool isFloat32 = fmt == VK_FORMAT_R32G32B32A32_SFLOAT;
        if (!ktxTexture2_NeedsTranscoding(ktex) && (isFloat16 || isFloat32) && ktex->baseWidth == 2 * ktex->baseHeight)
        {
            const uint32_t W = ktex->baseWidth;
            const uint32_t H = ktex->baseHeight;
            const uint8_t *dataPtr = reinterpret_cast<const uint8_t *>(ktxTexture_GetData(ktxTexture(ktex)));

            struct Vec3
            {
                float x, y, z;
            };

            auto half_to_float = [](uint16_t h) -> float {
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

            auto sample_at = [&](uint32_t x, uint32_t y) -> Vec3 {
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

            const float dtheta = float(M_PI) / float(H);
            const float dphi = 2.f * float(M_PI) / float(W);

            std::array<glm::vec3, 9> c{};
            for (auto &v : c) v = glm::vec3(0.0f);

            auto sh_basis = [](const glm::vec3 &d) -> std::array<float, 9> {
                const float x = d.x, y = d.y, z = d.z;
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
                float theta = (y + 0.5f) * dtheta;
                float sinT = std::sin(theta);
                for (uint32_t x = 0; x < W; ++x)
                {
                    float phi = (x + 0.5f) * dphi;
                    glm::vec3 dir = glm::vec3(std::cos(phi) * sinT, std::cos(theta), std::sin(phi) * sinT);
                    auto Lrgb = sample_at(x, y);
                    glm::vec3 Lvec(Lrgb.x, Lrgb.y, Lrgb.z);
                    auto Y = sh_basis(dir);
                    float dOmega = dphi * dtheta * sinT;
                    for (int i = 0; i < 9; ++i)
                    {
                        c[i] += Lvec * (Y[i] * dOmega);
                    }
                }
            }

            const float A0 = float(M_PI);
            const float A1 = 2.f * float(M_PI) / 3.f;
            const float A2 = float(M_PI) / 4.f;
            const float Aband[3] = {A0, A1, A2};
            for (int i = 0; i < 9; ++i)
            {
                int band = (i == 0) ? 0 : (i < 4 ? 1 : 2);
                c[i] *= Aband[band];
                out_sh[i] = glm::vec4(c[i], 0.0f);
            }

            ok = true;
        }

        ktxTexture_Destroy(ktxTexture(ktex));
        return ok;
    }

    static bool prepare_ibl_cpu(const IBLPaths &paths, PreparedIBLData &outData, std::string &outError)
    {
        outData = PreparedIBLData{};
        outData.paths = paths;
        outError.clear();

        if (!paths.specularCube.empty())
        {
            ktxutil::KtxCubemap cube{};
            if (ktxutil::load_ktx2_cubemap(paths.specularCube.c_str(), cube))
            {
                outData.has_spec = true;
                outData.spec_is_cubemap = true;
                outData.spec_cubemap = std::move(cube);
            }
            else
            {
                ktxutil::Ktx2D k2d{};
                if (ktxutil::load_ktx2_2d(paths.specularCube.c_str(), k2d))
                {
                    outData.has_spec = true;
                    outData.spec_is_cubemap = false;
                    outData.spec_2d = std::move(k2d);

                    glm::vec4 sh[9]{};
                    if (compute_sh_from_ktx2_equirect(paths.specularCube.c_str(), sh))
                    {
                        outData.has_sh = true;
                        for (int i = 0; i < 9; ++i)
                        {
                            outData.sh[i] = sh[i];
                        }
                    }
                }
                else
                {
                    outError = "Failed to load specular IBL as cubemap or 2D KTX2";
                }
            }
        }

        if (!paths.diffuseCube.empty())
        {
            ktxutil::KtxCubemap diff{};
            if (ktxutil::load_ktx2_cubemap(paths.diffuseCube.c_str(), diff))
            {
                outData.has_diffuse = true;
                outData.diff_cubemap = std::move(diff);
            }
        }

        if (!paths.background2D.empty())
        {
            ktxutil::Ktx2D bg{};
            if (ktxutil::load_ktx2_2d(paths.background2D.c_str(), bg))
            {
                outData.has_background = true;
                outData.background_2d = std::move(bg);
            }
        }

        if (!paths.brdfLut2D.empty())
        {
            ktxutil::Ktx2D lut{};
            if (ktxutil::load_ktx2_2d(paths.brdfLut2D.c_str(), lut))
            {
                outData.has_brdf = true;
                outData.brdf_2d = std::move(lut);
            }
        }

        // Success is defined by having a specular environment; diffuse/background/BRDF are optional.
        if (!outData.has_spec)
        {
            if (outError.empty())
            {
                outError = "Specular IBL KTX2 not found or invalid";
            }
            return false;
        }
        return true;
    }
}

struct IBLManager::AsyncStateData
{
    std::mutex mutex;
    std::condition_variable cv;
    bool shutdown{false};

    bool requestPending{false};
    IBLPaths requestPaths{};
    uint64_t requestId{0};

    bool resultReady{false};
    bool resultSuccess{false};
    PreparedIBLData readyData{};
    std::string lastError;
    uint64_t resultId{0};

    std::thread worker;
};

IBLManager::~IBLManager()
{
    shutdown_async();
}

void IBLManager::init(EngineContext *ctx)
{
    _ctx = ctx;

    if (_async != nullptr)
    {
        return;
    }

    _async = new AsyncStateData();
    AsyncStateData *state = _async;

    state->worker = std::thread([this, state]() {
        for (;;)
        {
            IBLPaths paths{};
            uint64_t jobId = 0;
            {
                std::unique_lock<std::mutex> lock(state->mutex);
                state->cv.wait(lock, [state]() { return state->shutdown || state->requestPending; });
                if (state->shutdown)
                {
                    break;
                }
                paths = state->requestPaths;
                jobId = state->requestId;
                state->requestPending = false;
            }

            PreparedIBLData data{};
            std::string error;
            bool ok = prepare_ibl_cpu(paths, data, error);

            {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (state->shutdown)
                {
                    break;
                }
                // Drop results for superseded jobs.
                if (jobId != state->requestId)
                {
                    continue;
                }

                state->readyData = std::move(data);
                state->lastError = std::move(error);
                state->resultSuccess = ok;
                state->resultReady = true;
                state->resultId = jobId;
            }
        }
    });
}

bool IBLManager::load(const IBLPaths &paths)
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr) return false;

    PreparedIBLData data{};
    std::string error;
    if (!prepare_ibl_cpu(paths, data, error))
    {
        if (!error.empty())
        {
            fmt::println("[IBL] load failed: {}", error);
        }
        return false;
    }

    return commit_prepared(data);
}

bool IBLManager::load_async(const IBLPaths &paths)
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr)
    {
        return false;
    }

    if (_async == nullptr)
    {
        init(_ctx);
    }

    AsyncStateData *state = _async;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->requestPaths = paths;
        state->requestPending = true;
        state->requestId++;
        // Invalidate any previous ready result; it will be superseded by this job.
        state->resultReady = false;
    }
    state->cv.notify_one();
    return true;
}

IBLManager::AsyncResult IBLManager::pump_async()
{
    AsyncResult out{};

    if (_async == nullptr || _ctx == nullptr || _ctx->getResources() == nullptr)
    {
        return out;
    }

    AsyncStateData *state = _async;

    PreparedIBLData data{};
    bool success = false;
    std::string error;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->resultReady)
        {
            return out;
        }
        data = std::move(state->readyData);
        success = state->resultSuccess;
        error = std::move(state->lastError);
        state->resultReady = false;
    }

    out.completed = true;
    if (!success)
    {
        if (!error.empty())
        {
            fmt::println("[IBL] async load failed: {}", error);
        }
        out.success = false;
        return out;
    }

    // Commit GPU resources on the main thread.
    out.success = commit_prepared(data);
    return out;
}

void IBLManager::unload()
{
    shutdown_async();

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

void IBLManager::shutdown_async()
{
    if (_async == nullptr) return;

    AsyncStateData *state = _async;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->shutdown = true;
        state->requestPending = false;
    }
    state->cv.notify_all();
    if (state->worker.joinable())
    {
        state->worker.join();
    }

    delete _async;
    _async = nullptr;
}

bool IBLManager::commit_prepared(const PreparedIBLData &data)
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr)
    {
        return false;
    }

    ResourceManager *rm = _ctx->getResources();

    if (rm->deferred_uploads() && rm->has_pending_uploads())
    {
        rm->process_queued_uploads_immediate();
    }

    destroy_images_and_sh();
    ensureLayout();

    if (data.has_spec)
    {
        if (data.spec_is_cubemap)
        {
            const auto &kcm = data.spec_cubemap;
            _spec = rm->create_image_compressed_layers(
                kcm.bytes.data(), kcm.bytes.size(),
                kcm.fmt, kcm.mipLevels, kcm.layers,
                kcm.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT,
                kcm.imgFlags);
        }
        else
        {
            const auto &k2d = data.spec_2d;
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
            _spec = rm->create_image_compressed(
                k2d.bytes.data(), k2d.bytes.size(), k2d.fmt, lv,
                VK_IMAGE_USAGE_SAMPLED_BIT);

            if (data.has_sh)
            {
                _shBuffer = rm->create_buffer(sizeof(glm::vec4) * 9,
                                              VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                              VMA_MEMORY_USAGE_CPU_TO_GPU);
                for (int i = 0; i < 9; ++i)
                {
                    std::memcpy(reinterpret_cast<char *>(_shBuffer.info.pMappedData) + i * sizeof(glm::vec4),
                                &data.sh[i], sizeof(glm::vec4));
                }
                vmaFlushAllocation(_ctx->getDevice()->allocator(), _shBuffer.allocation, 0,
                                   sizeof(glm::vec4) * 9);
            }
        }
    }

    if (data.has_diffuse)
    {
        const auto &kcm = data.diff_cubemap;
        _diff = rm->create_image_compressed_layers(
            kcm.bytes.data(), kcm.bytes.size(),
            kcm.fmt, kcm.mipLevels, kcm.layers,
            kcm.copies,
            VK_IMAGE_USAGE_SAMPLED_BIT,
            kcm.imgFlags);
    }
    if (_diff.image == VK_NULL_HANDLE && _spec.image != VK_NULL_HANDLE)
    {
        _diff = _spec;
    }

    if (data.has_background)
    {
        const auto &bg = data.background_2d;
        std::vector<ResourceManager::MipLevelCopy> lv;
        lv.reserve(bg.mipLevels);
        for (uint32_t mip = 0; mip < bg.mipLevels; ++mip)
        {
            const auto &r = bg.copies[mip];
            lv.push_back(ResourceManager::MipLevelCopy{
                .offset = r.bufferOffset,
                .length = 0,
                .width = r.imageExtent.width,
                .height = r.imageExtent.height,
            });
        }
        _background = rm->create_image_compressed(
            bg.bytes.data(), bg.bytes.size(), bg.fmt, lv,
            VK_IMAGE_USAGE_SAMPLED_BIT);
    }

    if (_background.image == VK_NULL_HANDLE && _spec.image != VK_NULL_HANDLE)
    {
        _background = _spec;
    }

    if (data.has_brdf)
    {
        const auto &lut = data.brdf_2d;
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
        _brdf = rm->create_image_compressed(
            lut.bytes.data(), lut.bytes.size(), lut.fmt, lv,
            VK_IMAGE_USAGE_SAMPLED_BIT);
    }

    return (_spec.image != VK_NULL_HANDLE) && (_diff.image != VK_NULL_HANDLE);
}
