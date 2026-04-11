#include "render/passes/atmosphere/atmosphere_internal.h"

using namespace atmosphere::detail;

void AtmospherePass::release_planet_height_textures()
{
    if (_context && _context->textures)
    {
        for (uint32_t &handle : _planetHeightHandles)
        {
            if (handle != UINT32_MAX)
            {
                _context->textures->unpin(handle);
                handle = UINT32_MAX;
            }
        }
    }
    else
    {
        _planetHeightHandles.fill(UINT32_MAX);
    }

    _planetHeightLoadedDir.clear();
}

void AtmospherePass::ensure_planet_height_textures(EngineContext *context, std::string_view height_dir)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->textures || !ctx->getAssets() || !ctx->getSamplers())
    {
        release_planet_height_textures();
        return;
    }

    const std::string desired_dir{height_dir};
    if (desired_dir == _planetHeightLoadedDir)
    {
        return;
    }

    release_planet_height_textures();
    _planetHeightLoadedDir = desired_dir;
    if (desired_dir.empty())
    {
        return;
    }

    AssetManager *assets = ctx->getAssets();
    TextureCache *textures = ctx->textures;
    VkSampler sampler = ctx->getSamplers()->linearClampEdge();
    if (!assets || !textures || sampler == VK_NULL_HANDLE)
    {
        return;
    }

    for (uint32_t face_index = 0; face_index < _planetHeightHandles.size(); ++face_index)
    {
        const auto face = static_cast<planet::CubeFace>(face_index);
        const std::string abs_path = resolve_optional_face_texture_path(*assets, desired_dir, face);
        if (abs_path.empty())
        {
            continue;
        }

        TextureCache::TextureKey key{};
        key.kind = TextureCache::TextureKey::SourceKind::FilePath;
        key.path = abs_path;
        key.srgb = false;
        key.mipmapped = true;
        key.channels = TextureCache::TextureKey::ChannelsHint::R;

        const uint32_t handle = textures->request(key, sampler);
        textures->pin(handle);
        _planetHeightHandles[face_index] = handle;
    }
}

void AtmospherePass::ensure_cloud_textures(EngineContext *context)
{
    EngineContext *ctx = context ? context : _context;
    if (!ctx || !ctx->getResources() || !ctx->getAssets())
    {
        return;
    }

    ResourceManager *resources = ctx->getResources();

    auto reload_cloud_tex_2d = [&](AllocatedImage &dst, std::string &loaded_path, const std::string &wanted_path)
    {
        if (wanted_path == loaded_path)
        {
            return;
        }

        if (dst.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(dst);
            dst = {};
        }

        loaded_path = wanted_path;
        if (wanted_path.empty())
        {
            return;
        }

        const std::string absolute_path = ctx->getAssets()->assetPath(wanted_path);
        ktxutil::Ktx2D ktx{};
        if (!absolute_path.empty() && ktxutil::load_ktx2_2d(absolute_path.c_str(), ktx))
        {
            dst = resources->create_image_compressed_layers(
                ktx.bytes.data(),
                ktx.bytes.size(),
                ktx.fmt,
                ktx.mipLevels,
                1,
                ktx.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    };

    auto reload_cloud_tex_3d = [&](AllocatedImage &dst, std::string &loaded_path, const std::string &wanted_path)
    {
        if (wanted_path == loaded_path)
        {
            return;
        }

        if (dst.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(dst);
            dst = {};
        }

        loaded_path = wanted_path;
        if (wanted_path.empty())
        {
            return;
        }

        const std::string absolute_path = ctx->getAssets()->assetPath(wanted_path);
        ktxutil::Ktx3D ktx{};
        if (!absolute_path.empty() && ktxutil::load_ktx2_3d(absolute_path.c_str(), ktx))
        {
            dst = resources->create_image_compressed_layers(
                ktx.bytes.data(),
                ktx.bytes.size(),
                ktx.fmt,
                ktx.mipLevels,
                1,
                ktx.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    };

    auto reload_jitter_tex_3d = [&](AllocatedImage &dst, std::string &loaded_path, const std::string &wanted_path)
    {
        if (wanted_path == loaded_path)
        {
            return;
        }

        if (dst.image != VK_NULL_HANDLE)
        {
            resources->destroy_image(dst);
            dst = {};
        }

        loaded_path = wanted_path;
        if (wanted_path.empty())
        {
            return;
        }

        const std::string absolute_path = ctx->getAssets()->assetPath(wanted_path);
        ktxutil::Ktx3D ktx{};
        if (!absolute_path.empty() && ktxutil::load_ktx2_3d(absolute_path.c_str(), ktx))
        {
            dst = resources->create_image_compressed_layers(
                ktx.bytes.data(),
                ktx.bytes.size(),
                ktx.fmt,
                ktx.mipLevels,
                1,
                ktx.copies,
                VK_IMAGE_USAGE_SAMPLED_BIT);
        }
    };

    reload_jitter_tex_3d(_jitterNoiseTex, _jitterNoiseLoadedPath, ctx->atmosphere.jitterTexturePath);
    reload_cloud_tex_2d(_cloudOverlayTex, _cloudOverlayLoadedPath, ctx->planetClouds.overlayTexturePath);
    reload_cloud_tex_2d(_cloudNoiseTex, _cloudNoiseLoadedPath, ctx->planetClouds.noiseTexturePath);
    reload_cloud_tex_3d(_cloudNoiseTex3D, _cloudNoise3DLoadedPath, ctx->planetClouds.noiseTexture3DPath);
}

