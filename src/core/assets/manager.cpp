#include "manager.h"

#include <cstdlib>
#include <iostream>

#include <core/engine.h>
#include <core/device/resource.h>
#include <render/materials.h>
#include <render/primitives.h>
#include <scene/tangent_space.h>
#include <scene/mesh_bvh.h>
#include <stb_image.h>
#include "locator.h"
#include <core/assets/texture_cache.h>
#include <fastgltf/parser.hpp>
#include <fastgltf/util.hpp>
#include <fastgltf/tools.hpp>
#include <fmt/core.h>

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
    {
        std::lock_guard<std::mutex> lock(_gltfMutex);
        _gltfCacheByPath.clear();
    }
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
    return loadGLTF(nameOrPath, nullptr);
}

std::optional<std::shared_ptr<LoadedGLTF> > AssetManager::loadGLTF(std::string_view nameOrPath,
                                                                   const GLTFLoadCallbacks *cb)
{
    if (!_engine) return {};
    if (nameOrPath.empty()) return {};

    std::string resolved = assetPath(nameOrPath);

    path keyPath = resolved;
    std::error_code ec;
    keyPath = std::filesystem::weakly_canonical(keyPath, ec);
    std::string key = (ec ? resolved : keyPath.string());

    {
        std::lock_guard<std::mutex> lock(_gltfMutex);
        if (auto it = _gltfCacheByPath.find(key); it != _gltfCacheByPath.end())
        {
            if (auto sp = it->second.lock())
            {
                fmt::println("[AssetManager] loadGLTF cache hit key='{}' path='{}' ptr={}", key, resolved,
                             static_cast<const void *>(sp.get()));
                return sp;
            }
            fmt::println("[AssetManager] loadGLTF cache expired key='{}' path='{}' (reloading)", key, resolved);
            _gltfCacheByPath.erase(it);
        }
    }

    auto loaded = loadGltf(_engine, resolved, cb);
    if (!loaded.has_value()) return {};

    if (loaded.value())
    {
        fmt::println("[AssetManager] loadGLTF loaded new scene key='{}' path='{}' ptr={}", key, resolved,
                     static_cast<const void *>(loaded.value().get()));
    }
    else
    {
        fmt::println("[AssetManager] loadGLTF got empty scene for key='{}' path='{}'", key, resolved);
    }

    {
        std::lock_guard<std::mutex> lock(_gltfMutex);
        _gltfCacheByPath[key] = loaded.value();
    }
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
    if (name == std::string_view("plane") || name == std::string_view("Plane"))
    {
        if (auto m = findBy("plane")) return m;
        if (auto m = findBy("Plane")) return m;
        return {};
    }
    if (name == std::string_view("capsule") || name == std::string_view("Capsule"))
    {
        if (auto m = findBy("capsule")) return m;
        if (auto m = findBy("Capsule")) return m;
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
    case MeshGeometryDesc::Type::Plane:
        primitives::buildPlane(tmpVerts, tmpInds);
        vertsSpan = tmpVerts;
        indsSpan = tmpInds;
        break;
    case MeshGeometryDesc::Type::Capsule:
        primitives::buildCapsule(tmpVerts, tmpInds);
        vertsSpan = tmpVerts;
        indsSpan = tmpInds;
        break;
    }

    // Ensure tangents exist for primitives (and provided geometry if needed)
    if (!tmpVerts.empty() && !tmpInds.empty())
    {
        geom::generate_tangents(tmpVerts, tmpInds);
    }

    std::shared_ptr<MeshAsset> mesh;

    if (info.material.kind == MeshMaterialDesc::Kind::Default)
    {
        mesh = createMesh(info.name, vertsSpan, indsSpan, {});
    }
    else
    {
        const auto &opt = info.material.options;

        GLTFMetallic_Roughness::MaterialConstants constants = opt.constants;

        if (!opt.occlusionPath.empty())
        {
            if (constants.extra[0].y == 0.0f && constants.extra[0].z == 0.0f)
            {
                constants.extra[0].y = 1.0f; // AO strength
                constants.extra[0].z = 1.0f; // hasAO flag
            }
        }

        if (!opt.emissivePath.empty())
        {
            if (constants.extra[1].x == 0.0f &&
                constants.extra[1].y == 0.0f &&
                constants.extra[1].z == 0.0f)
            {
                constants.extra[1] = glm::vec4(1.0f, 1.0f, 1.0f, constants.extra[1].w);
            }
        }

        AllocatedBuffer matBuffer = createMaterialBufferWithConstants(constants);

        GLTFMetallic_Roughness::MaterialResources res{};
        res.colorImage = _engine->_errorCheckerboardImage;
        res.colorSampler = _engine->_samplerManager->defaultLinear();
        res.metalRoughImage = _engine->_whiteImage;
        res.metalRoughSampler = _engine->_samplerManager->defaultLinear();
        res.normalImage = _engine->_flatNormalImage;
        res.normalSampler = _engine->_samplerManager->defaultLinear();
        res.occlusionImage = _engine->_whiteImage;
        res.occlusionSampler = _engine->_samplerManager->defaultLinear();
        res.emissiveImage = _engine->_blackImage;
        res.emissiveSampler = _engine->_samplerManager->defaultLinear();
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
            if (!opt.occlusionPath.empty())
            {
                auto key = buildKey(opt.occlusionPath, opt.occlusionSRGB);
                key.channels = TextureCache::TextureKey::ChannelsHint::R;
                if (key.hash != 0)
                {
                    VkSampler samp = _engine->_samplerManager->defaultLinear();
                    auto handle = cache->request(key, samp);
                    cache->watchBinding(handle, mat->data.materialSet, 4u, samp, _engine->_whiteImage.imageView);
                }
            }
            if (!opt.emissivePath.empty())
            {
                auto key = buildKey(opt.emissivePath, opt.emissiveSRGB);
                if (key.hash != 0)
                {
                    VkSampler samp = _engine->_samplerManager->defaultLinear();
                    auto handle = cache->request(key, samp);
                    cache->watchBinding(handle, mat->data.materialSet, 5u, samp, _engine->_blackImage.imageView);
                }
            }
        }

        mesh = createMesh(info.name, vertsSpan, indsSpan, mat);
        _meshMaterialBuffers.emplace(info.name, matBuffer);
    }

    if (!mesh)
    {
        return {};
    }

    // Tag primitive meshes with more appropriate default bounds types for picking,
    // then apply any explicit override from MeshCreateInfo.
    for (auto &surf : mesh->surfaces)
    {
        switch (info.geometry.type)
        {
        case MeshGeometryDesc::Type::Sphere:
            surf.bounds.type = BoundsType::Sphere;
            break;
        case MeshGeometryDesc::Type::Capsule:
            surf.bounds.type = BoundsType::Capsule;
            break;
        case MeshGeometryDesc::Type::Cube:
            surf.bounds.type = BoundsType::Box;
            break;
        case MeshGeometryDesc::Type::Plane:
            surf.bounds.type = BoundsType::Box;
            break;
        case MeshGeometryDesc::Type::Provided:
        default:
            surf.bounds.type = BoundsType::Box;
            break;
        }

        if (info.boundsType.has_value())
        {
            surf.bounds.type = *info.boundsType;
        }
    }

    return mesh;
}

AssetManager::GLTFTexturePrefetchResult AssetManager::prefetchGLTFTexturesWithHandles(std::string_view nameOrPath)
{
    GLTFTexturePrefetchResult result{};
    if (!_engine || !_engine->_context || !_engine->_context->textures) return result;
    if (nameOrPath.empty()) return result;

    std::string resolved = assetPath(nameOrPath);
    std::filesystem::path path = resolved;

    fastgltf::Parser parser{};
    constexpr auto gltfOptions = fastgltf::Options::DontRequireValidAssetMember | fastgltf::Options::AllowDouble |
                                 fastgltf::Options::LoadGLBBuffers | fastgltf::Options::LoadExternalBuffers;
    fastgltf::GltfDataBuffer data;
    if (!data.loadFromFile(path)) return result;

    fastgltf::Asset gltf;

    auto type = fastgltf::determineGltfFileType(&data);
    if (type == fastgltf::GltfType::glTF)
    {
        auto load = parser.loadGLTF(&data, path.parent_path(), gltfOptions);
        if (load) gltf = std::move(load.get()); else return result;
    }
    else if (type == fastgltf::GltfType::GLB)
    {
        auto load = parser.loadBinaryGLTF(&data, path.parent_path(), gltfOptions);
        if (load) gltf = std::move(load.get()); else return result;
    }
    else
    {
        return result;
    }

    TextureCache *cache = _engine->_context->textures;
    const std::filesystem::path baseDir = path.parent_path();

    auto enqueueTex = [&](size_t imgIndex, bool srgb)
    {
        if (imgIndex >= gltf.images.size()) return;
        TextureCache::TextureKey key{};
        key.srgb = srgb;
        key.mipmapped = true;

        fastgltf::Image &image = gltf.images[imgIndex];
        std::visit(fastgltf::visitor{
            [&](fastgltf::sources::URI &filePath)
            {
                const std::string rel(filePath.uri.path().begin(), filePath.uri.path().end());
                std::filesystem::path resolvedImg = std::filesystem::path(rel);
                if (resolvedImg.is_relative())
                {
                    resolvedImg = baseDir / resolvedImg;
                }
                key.kind = TextureCache::TextureKey::SourceKind::FilePath;
                key.path = resolvedImg.string();
                std::string id = std::string("GLTF:") + key.path + (srgb ? "#sRGB" : "#UNORM");
                key.hash = texcache::fnv1a64(id);
            },
            [&](fastgltf::sources::Vector &vector)
            {
                key.kind = TextureCache::TextureKey::SourceKind::Bytes;
                key.bytes.assign(vector.bytes.begin(), vector.bytes.end());
                uint64_t h = texcache::fnv1a64(key.bytes.data(), key.bytes.size());
                key.hash = h ^ (srgb ? 0x9E3779B97F4A7C15ull : 0ull);
            },
            [&](fastgltf::sources::BufferView &view)
            {
                auto &bufferView = gltf.bufferViews[view.bufferViewIndex];
                auto &buffer = gltf.buffers[bufferView.bufferIndex];
                std::visit(fastgltf::visitor{
                    [](auto &arg) {},
                    [&](fastgltf::sources::Vector &vec)
                    {
                        size_t off = bufferView.byteOffset;
                        size_t len = bufferView.byteLength;
                        key.kind = TextureCache::TextureKey::SourceKind::Bytes;
                        key.bytes.assign(vec.bytes.begin() + off, vec.bytes.begin() + off + len);
                        uint64_t h = texcache::fnv1a64(key.bytes.data(), key.bytes.size());
                        key.hash = h ^ (srgb ? 0x9E3779B97F4A7C15ull : 0ull);
                    }
                }, buffer.data);
            },
            [](auto &other) {}
        }, image.data);

        if (key.hash != 0)
        {
            VkSampler samp = _engine->_samplerManager->defaultLinear();
            TextureCache::TextureHandle handle = cache->request(key, samp);
            result.handles.push_back(handle);
            result.scheduled++;
        }
    };

    for (const auto &tex : gltf.textures)
    {
        if (tex.imageIndex.has_value())
        {
            // For baseColor we prefer sRGB; other maps requested later will reuse entry
            enqueueTex(tex.imageIndex.value(), true);
        }
    }

    // Proactively free big buffer vectors we no longer need.
    for (auto &buf : gltf.buffers)
    {
        std::visit(fastgltf::visitor{
            [](auto &arg) {},
            [&](fastgltf::sources::Vector &vec) {
                std::vector<uint8_t>().swap(vec.bytes);
            }
        }, buf.data);
    }

    return result;
}

size_t AssetManager::prefetchGLTFTextures(std::string_view nameOrPath)
{
    return prefetchGLTFTexturesWithHandles(nameOrPath).scheduled;
}

static Bounds compute_bounds(std::span<Vertex> vertices)
{
    Bounds b{};
    if (vertices.empty())
    {
        b.origin = glm::vec3(0.0f);
        b.extents = glm::vec3(0.5f);
        b.sphereRadius = glm::length(b.extents);
        b.type = BoundsType::Box;
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
    b.type = BoundsType::Box;
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
        else
        {
            fmt::println("[AssetManager] Failed to load texture '{}' (resolved='{}')",
                         imgPath,
                         resolved);
        }
        if (pixels) stbi_image_free(pixels);
    }
    return {out, created};
}

std::shared_ptr<MeshAsset> AssetManager::createMesh(const std::string &name,
                                                    std::span<Vertex> vertices,
                                                    std::span<uint32_t> indices,
                                                    std::shared_ptr<GLTFMaterial> material,
                                                    bool build_bvh)
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
        matResources.occlusionImage = _engine->_whiteImage;
        matResources.occlusionSampler = _engine->_samplerManager->defaultLinear();
        matResources.emissiveImage = _engine->_blackImage;
        matResources.emissiveSampler = _engine->_samplerManager->defaultLinear();

        AllocatedBuffer matBuffer = createMaterialBufferWithConstants({});
        matResources.dataBuffer = matBuffer.buffer;
        matResources.dataBufferOffset = 0;

        material = createMaterial(MaterialPass::MainColor, matResources);
        _meshMaterialBuffers.emplace(name, matBuffer);
    }

    auto mesh = std::make_shared<MeshAsset>();
    mesh->name = name;
    mesh->meshBuffers = _engine->_resourceManager->uploadMesh(indices, vertices);
    // BLAS for this mesh is built lazily when TLAS is constructed from the draw
    // context (RayTracingManager::buildTLASFromDrawContext). This keeps RT work
    // centralized and avoids redundant builds on load.

    GeoSurface surf{};
    surf.startIndex = 0;
    surf.count = (uint32_t) indices.size();
    surf.material = material;
    surf.bounds = compute_bounds(vertices);
    mesh->surfaces.push_back(surf);

    if (build_bvh)
    {
        // Build CPU-side BVH for precise ray picking over this mesh.
        // This uses the same mesh-local vertex/index data as the GPU upload.
        mesh->bvh = build_mesh_bvh(*mesh, vertices, indices);
    }

    _meshCache.emplace(name, mesh);
    return mesh;
}

std::shared_ptr<GLTFMaterial> AssetManager::createMaterialFromConstants(
    const std::string &name,
    const GLTFMetallic_Roughness::MaterialConstants &constants,
    MaterialPass pass)
{
    if (!_engine) return {};
    GLTFMetallic_Roughness::MaterialResources res{};
    res.colorImage = _engine->_whiteImage;
    res.colorSampler = _engine->_samplerManager->defaultLinear();
    res.metalRoughImage = _engine->_whiteImage;
    res.metalRoughSampler = _engine->_samplerManager->defaultLinear();
    res.normalImage = _engine->_flatNormalImage;
    res.normalSampler = _engine->_samplerManager->defaultLinear();
    res.occlusionImage = _engine->_whiteImage;
    res.occlusionSampler = _engine->_samplerManager->defaultLinear();
    res.emissiveImage = _engine->_blackImage;
    res.emissiveSampler = _engine->_samplerManager->defaultLinear();

    AllocatedBuffer buf = createMaterialBufferWithConstants(constants);
    res.dataBuffer = buf.buffer;
    res.dataBufferOffset = 0;
    _meshMaterialBuffers[name] = buf;

    return createMaterial(pass, res);
}

VkImageView AssetManager::fallback_checkerboard_view() const
{
    return (_engine) ? _engine->_errorCheckerboardImage.imageView : VK_NULL_HANDLE;
}

VkImageView AssetManager::fallback_white_view() const
{
    return (_engine) ? _engine->_whiteImage.imageView : VK_NULL_HANDLE;
}

VkImageView AssetManager::fallback_flat_normal_view() const
{
    return (_engine) ? _engine->_flatNormalImage.imageView : VK_NULL_HANDLE;
}

VkImageView AssetManager::fallback_black_view() const
{
    return (_engine) ? _engine->_blackImage.imageView : VK_NULL_HANDLE;
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

bool AssetManager::removeMeshDeferred(const std::string &name, DeletionQueue &dq)
{
    auto it = _meshCache.find(name);
    if (it == _meshCache.end()) return false;

    const std::shared_ptr<MeshAsset> mesh = it->second;
    if (!mesh) return false;

    // Remove from cache immediately so callers won't retrieve a mesh we plan to destroy.
    _meshCache.erase(it);

    if (_engine && _engine->_rayManager)
    {
        // Clean up BLAS cached for this mesh (if ray tracing is enabled).
        // RayTracingManager defers actual AS destruction internally.
        _engine->_rayManager->removeBLASForBuffer(mesh->meshBuffers.vertexBuffer.buffer);
    }

    ResourceManager *rm = (_engine && _engine->_resourceManager) ? _engine->_resourceManager.get() : nullptr;
    if (!rm)
    {
        return true;
    }

    const AllocatedBuffer indexBuffer = mesh->meshBuffers.indexBuffer;
    const AllocatedBuffer vertexBuffer = mesh->meshBuffers.vertexBuffer;

    std::optional<AllocatedBuffer> materialBuffer;
    auto itb = _meshMaterialBuffers.find(name);
    if (itb != _meshMaterialBuffers.end())
    {
        materialBuffer = itb->second;
        _meshMaterialBuffers.erase(itb);
    }

    std::vector<AllocatedImage> ownedImages;
    auto iti = _meshOwnedImages.find(name);
    if (iti != _meshOwnedImages.end())
    {
        ownedImages = std::move(iti->second);
        _meshOwnedImages.erase(iti);
    }

    dq.push_function([rm, indexBuffer, vertexBuffer, materialBuffer, ownedImages = std::move(ownedImages)]() mutable
    {
        if (indexBuffer.buffer) rm->destroy_buffer(indexBuffer);
        if (vertexBuffer.buffer) rm->destroy_buffer(vertexBuffer);

        if (materialBuffer.has_value() && materialBuffer->buffer)
        {
            rm->destroy_buffer(*materialBuffer);
        }

        for (const auto &img : ownedImages)
        {
            if (img.image) rm->destroy_image(img);
        }
    });

    return true;
}
