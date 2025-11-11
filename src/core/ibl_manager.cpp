#include "ibl_manager.h"
#include <core/engine_context.h>
#include <core/vk_resource.h>
#include <core/ktx_loader.h>
#include <core/vk_sampler_manager.h>

bool IBLManager::load(const IBLPaths &paths)
{
    if (_ctx == nullptr || _ctx->getResources() == nullptr) return false;
    ResourceManager* rm = _ctx->getResources();

    // Specular cubemap
    if (!paths.specularCube.empty())
    {
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
    }

    // Diffuse cubemap
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

    // BRDF LUT (optional)
    if (!paths.brdfLut2D.empty())
    {
        ktxutil::Ktx2D lut{};
        if (ktxutil::load_ktx2_2d(paths.brdfLut2D.c_str(), lut))
        {
            // Build regions into ResourceManager::MipLevelCopy to reuse compressed 2D helper
            std::vector<ResourceManager::MipLevelCopy> lv;
            lv.reserve(lut.mipLevels);
            for (uint32_t mip = 0; mip < lut.mipLevels; ++mip)
            {
                const auto &r = lut.copies[mip];
                lv.push_back(ResourceManager::MipLevelCopy{
                    .offset = r.bufferOffset,
                    .length = 0, // not needed for copy scheduling
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
    auto* rm = _ctx->getResources();
    if (_spec.image) { rm->destroy_image(_spec); _spec = {}; }
    if (_diff.image) { rm->destroy_image(_diff); _diff = {}; }
    if (_brdf.image) { rm->destroy_image(_brdf); _brdf = {}; }
}

