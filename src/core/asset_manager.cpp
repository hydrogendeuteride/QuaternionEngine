#include "asset_manager.h"

#include <cstdlib>
#include <iostream>

#include <core/vk_engine.h>
#include <core/vk_resource.h>
#include <render/vk_materials.h>
#include <render/primitives.h>
#include <scene/tangent_space.h>
#include <stb_image.h>
#include "asset_locator.h"
#include <core/texture_cache.h>

using std::filesystem::path;

void AssetManager::init(VulkanEngine *engine)
{
    _engine = engine;
    _locator.init();
}

void AssetManager::cleanup()
{
    if (_engine && _engine->_resourceManager)
    {
        for (auto &kv: _meshCache)
        {
            if (kv.second)
            {
                _engine->_resourceManager->destroy_buffer(kv.second->meshBuffers.indexBuffer);
                _engine->_resourceManager->destroy_buffer(kv.second->meshBuffers.vertexBuffer);
            }
        }
        for (auto &kv: _meshMaterialBuffers)
        {
            _engine->_resourceManager->destroy_buffer(kv.second);
        }
        for (auto &kv: _meshOwnedImages)
        {
            for (const auto &img: kv.second)
            {
                _engine->_resourceManager->destroy_image(img);
            }
        }
    }
    _meshCache.clear();
    _meshMaterialBuffers.clear();
    _meshOwnedImages.clear();
    _gltfCacheByPath.clear();
}

std::string AssetManager::shaderPath(std::string_view name) const
{
    return _locator.shaderPath(name);
}

std::string AssetManager::assetPath(std::string_view name) const
{
    return _locator.assetPath(name);
}

std::string AssetManager::modelPath(std::string_view name) const
{
    return _locator.modelPath(name);
}

std::optional<std::shared_ptr<LoadedGLTF> > AssetManager::loadGLTF(std::string_view nameOrPath)
{
    if (!_engine) return {};
    if (nameOrPath.empty()) return {};

    std::string resolved = assetPath(nameOrPath);

    path keyPath = resolved;
    std::error_code ec;
    keyPath = std::filesystem::weakly_canonical(keyPath, ec);
    std::string key = (ec ? resolved : keyPath.string());

    if (auto it = _gltfCacheByPath.find(key); it != _gltfCacheByPath.end())
    {
        if (auto sp = it->second.lock()) return sp;
    }

    auto loaded = loadGltf(_engine, resolved);
    if (!loaded.has_value()) return {};
    _gltfCacheByPath[key] = loaded.value();
    return loaded;
}

std::shared_ptr<MeshAsset> AssetManager::getPrimitive(std::string_view name) const
{
    if (name.empty()) return {};
    auto findBy = [&](const std::string &key) -> std::shared_ptr<MeshAsset> {
        auto it = _meshCache.find(key);
        return (it != _meshCache.end()) ? it->second : nullptr;
    };

    if (name == std::string_view("cube") || name == std::string_view("Cube"))
    {
        if (auto m = findBy("cube")) return m;
        if (auto m = findBy("Cube")) return m;
        return {};
    }
    if (name == std::string_view("sphere") || name == std::string_view("Sphere"))
    {
        if (auto m = findBy("sphere")) return m;
        if (auto m = findBy("Sphere")) return m;
        return {};
    }
    return {};
}

std::shared_ptr<MeshAsset> AssetManager::createMesh(const MeshCreateInfo &info)
{
    if (!_engine || !_engine->_resourceManager) return {};
    if (info.name.empty()) return {};

    if (auto it = _meshCache.find(info.name); it != _meshCache.end())
    {
        return it->second;
    }

    std::vector<Vertex> tmpVerts;
    std::vector<uint32_t> tmpInds;
    std::span<Vertex> vertsSpan{};
    std::span<uint32_t> indsSpan{};

    switch (info.geometry.type)
    {
        case MeshGeometryDesc::Type::Provided:
            vertsSpan = info.geometry.vertices;
            indsSpan = info.geometry.indices;
            break;
        case MeshGeometryDesc::Type::Cube:
            primitives::buildCube(tmpVerts, tmpInds);
            vertsSpan = tmpVerts;
            indsSpan = tmpInds;
            break;
        case MeshGeometryDesc::Type::Sphere:
            primitives::buildSphere(tmpVerts, tmpInds, info.geometry.sectors, info.geometry.stacks);
            vertsSpan = tmpVerts;
            indsSpan = tmpInds;
            break;
    }

    // Ensure tangents exist for primitives (and provided geometry if needed)
    if (!tmpVerts.empty() && !tmpInds.empty())
    {
        geom::generate_tangents(tmpVerts, tmpInds);
    }

    if (info.material.kind == MeshMaterialDesc::Kind::Default)
    {
        return createMesh(info.name, vertsSpan, indsSpan, {});
    }

    const auto &opt = info.material.options;

    // Fallbacks are bound now; real textures will patch in via TextureCache
    AllocatedBuffer matBuffer = createMaterialBufferWithConstants(opt.constants);

    GLTFMetallic_Roughness::MaterialResources res{};
    res.colorImage = _engine->_errorCheckerboardImage; // visible fallback for albedo
    res.colorSampler = _engine->_samplerManager->defaultLinear();
    res.metalRoughImage = _engine->_whiteImage;
    res.metalRoughSampler = _engine->_samplerManager->defaultLinear();
    res.normalImage = _engine->_flatNormalImage;
    res.normalSampler = _engine->_samplerManager->defaultLinear();
    res.dataBuffer = matBuffer.buffer;
    res.dataBufferOffset = 0;

    auto mat = createMaterial(opt.pass, res);

    // Register dynamic texture bindings using the central TextureCache
    if (_engine && _engine->_context && _engine->_context->textures)
    {
        TextureCache *cache = _engine->_context->textures;
        auto buildKey = [&](std::string_view path, bool srgb) -> TextureCache::TextureKey {
            TextureCache::TextureKey k{};
            if (!path.empty())
            {
                k.kind = TextureCache::TextureKey::SourceKind::FilePath;
                k.path = assetPath(path);
                k.srgb = srgb;
                k.mipmapped = true;
                std::string id = std::string("PRIM:") + k.path + (srgb ? "#sRGB" : "#UNORM");
                k.hash = texcache::fnv1a64(id);
            }
            return k;
        };

        if (!opt.albedoPath.empty())
        {
            auto key = buildKey(opt.albedoPath, opt.albedoSRGB);
            if (key.hash != 0)
            {
                VkSampler samp = _engine->_samplerManager->defaultLinear();
                auto handle = cache->request(key, samp);
                cache->watchBinding(handle, mat->data.materialSet, 1u, samp, _engine->_errorCheckerboardImage.imageView);
            }
        }
        if (!opt.metalRoughPath.empty())
        {
            auto key = buildKey(opt.metalRoughPath, opt.metalRoughSRGB);
            if (key.hash != 0)
            {
                VkSampler samp = _engine->_samplerManager->defaultLinear();
                auto handle = cache->request(key, samp);
                cache->watchBinding(handle, mat->data.materialSet, 2u, samp, _engine->_whiteImage.imageView);
            }
        }
        if (!opt.normalPath.empty())
        {
            auto key = buildKey(opt.normalPath, opt.normalSRGB);
            if (key.hash != 0)
            {
                VkSampler samp = _engine->_samplerManager->defaultLinear();
                auto handle = cache->request(key, samp);
                cache->watchBinding(handle, mat->data.materialSet, 3u, samp, _engine->_flatNormalImage.imageView);
            }
        }
    }

    auto mesh = createMesh(info.name, vertsSpan, indsSpan, mat);
    _meshMaterialBuffers.emplace(info.name, matBuffer);
    return mesh;
}

static Bounds compute_bounds(std::span<Vertex> vertices)
{
    Bounds b{};
    if (vertices.empty())
    {
        b.origin = glm::vec3(0.0f);
        b.extents = glm::vec3(0.5f);
        b.sphereRadius = glm::length(b.extents);
        return b;
    }
    glm::vec3 minpos = vertices[0].position;
    glm::vec3 maxpos = vertices[0].position;
    for (const auto &v: vertices)
    {
        minpos = glm::min(minpos, v.position);
        maxpos = glm::max(maxpos, v.position);
    }
    b.origin = (maxpos + minpos) / 2.f;
    b.extents = (maxpos - minpos) / 2.f;
    b.sphereRadius = glm::length(b.extents);
    return b;
}

AllocatedBuffer AssetManager::createMaterialBufferWithConstants(
    const GLTFMetallic_Roughness::MaterialConstants &constants) const
{
    AllocatedBuffer matBuffer = _engine->_resourceManager->create_buffer(
        sizeof(GLTFMetallic_Roughness::MaterialConstants),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);

    VmaAllocationInfo allocInfo{};
    vmaGetAllocationInfo(_engine->_deviceManager->allocator(), matBuffer.allocation, &allocInfo);
    auto *matConstants = (GLTFMetallic_Roughness::MaterialConstants *) allocInfo.pMappedData;
    *matConstants = constants;
    if (matConstants->colorFactors == glm::vec4(0))
    {
        matConstants->colorFactors = glm::vec4(1.0f);
    }
    if (matConstants->extra[0].x == 0.0f)
    {
        matConstants->extra[0].x = 1.0f; // normal scale default
    }
    // Ensure writes are visible on non-coherent memory
    vmaFlushAllocation(_engine->_deviceManager->allocator(), matBuffer.allocation, 0,
                       sizeof(GLTFMetallic_Roughness::MaterialConstants));
    return matBuffer;
}

std::shared_ptr<GLTFMaterial> AssetManager::createMaterial(
    MaterialPass pass, const GLTFMetallic_Roughness::MaterialResources &res) const
{
    auto mat = std::make_shared<GLTFMaterial>();
    mat->data = _engine->metalRoughMaterial.write_material(
        _engine->_deviceManager->device(), pass, res, *_engine->_context->descriptors);
    return mat;
}

std::pair<AllocatedImage, bool> AssetManager::loadImageFromAsset(std::string_view imgPath, bool srgb) const
{
    AllocatedImage out{};
    bool created = false;
    if (!imgPath.empty())
    {
        std::string resolved = assetPath(imgPath);
        int w = 0, h = 0, comp = 0;
        stbi_uc *pixels = stbi_load(resolved.c_str(), &w, &h, &comp, 4);
        if (pixels && w > 0 && h > 0)
        {
            VkFormat fmt = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
            out = _engine->_resourceManager->create_image(pixels,
                                                          VkExtent3D{static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1},
                                                          fmt,
                                                          VK_IMAGE_USAGE_SAMPLED_BIT,
                                                          false);
            created = true;
        }
        if (pixels) stbi_image_free(pixels);
    }
    return {out, created};
}

std::shared_ptr<MeshAsset> AssetManager::createMesh(const std::string &name,
                                                    std::span<Vertex> vertices,
                                                    std::span<uint32_t> indices,
                                                    std::shared_ptr<GLTFMaterial> material)
{
    if (!_engine || !_engine->_resourceManager) return {};
    if (name.empty()) return {};

    auto it = _meshCache.find(name);
    if (it != _meshCache.end()) return it->second;

    if (!material)
    {
        GLTFMetallic_Roughness::MaterialResources matResources{};
        matResources.colorImage = _engine->_whiteImage;
        matResources.colorSampler = _engine->_samplerManager->defaultLinear();
        matResources.metalRoughImage = _engine->_whiteImage;
        matResources.metalRoughSampler = _engine->_samplerManager->defaultLinear();
        matResources.normalImage = _engine->_flatNormalImage;
        matResources.normalSampler = _engine->_samplerManager->defaultLinear();

        AllocatedBuffer matBuffer = createMaterialBufferWithConstants({});
        matResources.dataBuffer = matBuffer.buffer;
        matResources.dataBufferOffset = 0;

        material = createMaterial(MaterialPass::MainColor, matResources);
        _meshMaterialBuffers.emplace(name, matBuffer);
    }

    auto mesh = std::make_shared<MeshAsset>();
    mesh->name = name;
    mesh->meshBuffers = _engine->_resourceManager->uploadMesh(indices, vertices);
    // Build BLAS for the mesh if ray tracing manager is available
    if (_engine->_rayManager)
    {
        _engine->_rayManager->getOrBuildBLAS(mesh);
    }

    GeoSurface surf{};
    surf.startIndex = 0;
    surf.count = (uint32_t) indices.size();
    surf.material = material;
    surf.bounds = compute_bounds(vertices);
    mesh->surfaces.push_back(surf);

    _meshCache.emplace(name, mesh);
    return mesh;
}

std::shared_ptr<MeshAsset> AssetManager::getMesh(const std::string &name) const
{
    auto it = _meshCache.find(name);
    return (it != _meshCache.end()) ? it->second : nullptr;
}

bool AssetManager::removeMesh(const std::string &name)
{
    auto it = _meshCache.find(name);
    if (it == _meshCache.end()) return false;
    if (_engine && _engine->_rayManager)
    {
        // Clean up BLAS cached for this mesh (if ray tracing is enabled)
        _engine->_rayManager->removeBLASForBuffer(it->second->meshBuffers.vertexBuffer.buffer);
    }
    if (_engine && _engine->_resourceManager)
    {
        _engine->_resourceManager->destroy_buffer(it->second->meshBuffers.indexBuffer);
        _engine->_resourceManager->destroy_buffer(it->second->meshBuffers.vertexBuffer);
    }
    _meshCache.erase(it);
    auto itb = _meshMaterialBuffers.find(name);
    if (itb != _meshMaterialBuffers.end())
    {
        if (_engine && _engine->_resourceManager)
        {
            _engine->_resourceManager->destroy_buffer(itb->second);
        }
        _meshMaterialBuffers.erase(itb);
    }
    auto iti = _meshOwnedImages.find(name);
    if (iti != _meshOwnedImages.end())
    {
        if (_engine && _engine->_resourceManager)
        {
            for (const auto &img: iti->second)
            {
                _engine->_resourceManager->destroy_image(img);
            }
        }
        _meshOwnedImages.erase(iti);
    }
    return true;
}
