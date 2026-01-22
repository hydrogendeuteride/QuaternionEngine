#include "stb_image.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "vk_loader.h"
#include "core/assets/texture_cache.h"

#include "core/engine.h"
#include "render/materials.h"
#include "core/util/initializers.h"
#include "core/types.h"
#include "core/config.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include <fastgltf/glm_element_traits.hpp>
#include <fastgltf/parser.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/util.hpp>
#include <optional>
#include "tangent_space.h"
#include "mesh_bvh.h"
#include "physics/gltf_collider_parser.h"
//> loadimg
std::optional<AllocatedImage> load_image(VulkanEngine *engine, fastgltf::Asset &asset, fastgltf::Image &image, bool srgb)
{
    AllocatedImage newImage{};

    int width, height, nrChannels;

    std::visit(
        fastgltf::visitor{
            [](auto &arg) {
            },
            [&](fastgltf::sources::URI &filePath) {
                assert(filePath.fileByteOffset == 0); // We don't support offsets with stbi.
                assert(filePath.uri.isLocalPath()); // We're only capable of loading
                // local files.

                const std::string path(filePath.uri.path().begin(),
                                       filePath.uri.path().end()); // Thanks C++.
                unsigned char *data = stbi_load(path.c_str(), &width, &height, &nrChannels, 4);
                if (data)
                {
                    VkExtent3D imagesize;
                    imagesize.width = width;
                    imagesize.height = height;
                    imagesize.depth = 1;

                    VkFormat fmt = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
                    newImage = engine->_resourceManager->create_image(
                        data, imagesize, fmt, VK_IMAGE_USAGE_SAMPLED_BIT, false);
                    // Name the allocation for diagnostics
                    if (vmaDebugEnabled())
                        vmaSetAllocationName(engine->_deviceManager->allocator(), newImage.allocation, path.c_str());

                    stbi_image_free(data);
                }
            },
            [&](fastgltf::sources::Vector &vector) {
                unsigned char *data = stbi_load_from_memory(vector.bytes.data(), static_cast<int>(vector.bytes.size()),
                                                            &width, &height, &nrChannels, 4);
                if (data)
                {
                    VkExtent3D imagesize;
                    imagesize.width = width;
                    imagesize.height = height;
                    imagesize.depth = 1;

                    VkFormat fmt = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
                    newImage = engine->_resourceManager->create_image(
                        data, imagesize, fmt, VK_IMAGE_USAGE_SAMPLED_BIT, false);
                    if (vmaDebugEnabled())
                        vmaSetAllocationName(engine->_deviceManager->allocator(), newImage.allocation, "gltf.vector.image");

                    stbi_image_free(data);
                }
            },
            [&](fastgltf::sources::BufferView &view) {
                auto &bufferView = asset.bufferViews[view.bufferViewIndex];
                auto &buffer = asset.buffers[bufferView.bufferIndex];

                std::visit(fastgltf::visitor{
                               // We only care about VectorWithMime here, because we
                               // specify LoadExternalBuffers, meaning all buffers
                               // are already loaded into a vector.
                               [](auto &arg) {
                               },
                               [&](fastgltf::sources::Vector &vector) {
                                   unsigned char *data = stbi_load_from_memory(
                                       vector.bytes.data() + bufferView.byteOffset,
                                       static_cast<int>(bufferView.byteLength),
                                       &width, &height, &nrChannels, 4);
                                   if (data)
                                   {
                                       VkExtent3D imagesize;
                                       imagesize.width = width;
                                       imagesize.height = height;
                                       imagesize.depth = 1;

                                       VkFormat fmt = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
                    newImage = engine->_resourceManager->create_image(
                                           data, imagesize, fmt, VK_IMAGE_USAGE_SAMPLED_BIT, false);
                                       if (vmaDebugEnabled())
                                           vmaSetAllocationName(engine->_deviceManager->allocator(), newImage.allocation, "gltf.bufferview.image");

                                       stbi_image_free(data);
                                   }
                               }
                           },
                           buffer.data);
            },
        },
        image.data);

    // if any of the attempts to load the data failed, we havent written the image
    // so handle is null
    if (newImage.image == VK_NULL_HANDLE)
    {
        return {};
    }
    else
    {
        return newImage;
    }
}

//< loadimg
//> filters
VkFilter extract_filter(fastgltf::Filter filter)
{
    switch (filter)
    {
        // nearest samplers
        case fastgltf::Filter::Nearest:
        case fastgltf::Filter::NearestMipMapNearest:
        case fastgltf::Filter::NearestMipMapLinear:
            return VK_FILTER_NEAREST;

        // linear samplers
        case fastgltf::Filter::Linear:
        case fastgltf::Filter::LinearMipMapNearest:
        case fastgltf::Filter::LinearMipMapLinear:
        default:
            return VK_FILTER_LINEAR;
    }
}

VkSamplerMipmapMode extract_mipmap_mode(fastgltf::Filter filter)
{
    switch (filter)
    {
        case fastgltf::Filter::NearestMipMapNearest:
        case fastgltf::Filter::LinearMipMapNearest:
            return VK_SAMPLER_MIPMAP_MODE_NEAREST;

        case fastgltf::Filter::NearestMipMapLinear:
        case fastgltf::Filter::LinearMipMapLinear:
        default:
            return VK_SAMPLER_MIPMAP_MODE_LINEAR;
    }
}

//< filters

std::optional<std::shared_ptr<LoadedGLTF> > loadGltf(VulkanEngine *engine,
                                                     std::string_view filePath,
                                                     const GLTFLoadCallbacks *cb)
{
    //> load_1
    fmt::println("[GLTF] loadGltf begin: '{}'", filePath);

    std::shared_ptr<LoadedGLTF> scene = std::make_shared<LoadedGLTF>();
    scene->creator = engine;
    LoadedGLTF &file = *scene.get();

    fastgltf::Parser parser{};

    constexpr auto gltfOptions = fastgltf::Options::DontRequireValidAssetMember | fastgltf::Options::AllowDouble |
                                 fastgltf::Options::LoadGLBBuffers | fastgltf::Options::LoadExternalBuffers;
    // fastgltf::Options::LoadExternalImages;

    fastgltf::GltfDataBuffer data;
    data.loadFromFile(filePath);

    fastgltf::Asset gltf;

    std::filesystem::path path = filePath;

    auto type = fastgltf::determineGltfFileType(&data);
    if (type == fastgltf::GltfType::glTF)
    {
        auto load = parser.loadGLTF(&data, path.parent_path(), gltfOptions);
        if (load)
        {
            gltf = std::move(load.get());
        }
        else
        {
            std::cerr << "Failed to load glTF: " << fastgltf::to_underlying(load.error()) << std::endl;
            return {};
        }
    }
    else if (type == fastgltf::GltfType::GLB)
    {
        auto load = parser.loadBinaryGLTF(&data, path.parent_path(), gltfOptions);
        if (load)
        {
            gltf = std::move(load.get());
        }
        else
        {
            std::cerr << "Failed to load glTF: " << fastgltf::to_underlying(load.error()) << std::endl;
            return {};
        }
    }
    else
    {
        std::cerr << "Failed to determine glTF container" << std::endl;
        return {};
    }
    //< load_1
    // Simple helpers for progress/cancellation callbacks (if provided)
    auto report_progress = [&](float v)
    {
        if (cb && cb->on_progress)
        {
            float clamped = std::clamp(v, 0.0f, 1.0f);
            cb->on_progress(clamped);
        }
    };
    auto is_cancelled = [&]() -> bool
    {
        if (cb && cb->is_cancelled)
        {
            return cb->is_cancelled();
        }
        return false;
    };

    //> load_2
    // we can stimate the descriptors we will need accurately
    fmt::println("[GLTF] loadGltf: materials={} meshes={} images={} samplers={} (creating descriptor pool)",
                 gltf.materials.size(),
                 gltf.meshes.size(),
                 gltf.images.size(),
                 gltf.samplers.size());

    // One material descriptor set binds:
    // - 1x uniform buffer (material constants)
    // - 5x combined image samplers (baseColor, metalRough, normal, occlusion, emissive)
    //
    // The pool must have at least these counts per material. If the ratio is too low
    // (e.g. 3 samplers) and the asset has only 1 material, allocating the first set
    // can fail with VK_ERROR_OUT_OF_POOL_MEMORY.
    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 5},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
    };

    file.descriptorPool.init(engine->_deviceManager->device(), gltf.materials.size(), sizes);

    fmt::println("[GLTF] loadGltf: descriptor pool initialized for '{}' (materials={})",
                 filePath,
                 gltf.materials.size());

    report_progress(0.1f);
    //< load_2
    //> load_samplers

    // load samplers
    for (fastgltf::Sampler &sampler: gltf.samplers)
    {
        VkSamplerCreateInfo sampl = {.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO, .pNext = nullptr};
        sampl.maxLod = VK_LOD_CLAMP_NONE;
        sampl.minLod = 0.0f;

        sampl.magFilter = extract_filter(sampler.magFilter.value_or(fastgltf::Filter::Nearest));
        sampl.minFilter = extract_filter(sampler.minFilter.value_or(fastgltf::Filter::Nearest));
        sampl.mipmapMode = extract_mipmap_mode(sampler.minFilter.value_or(fastgltf::Filter::Nearest));

        // Address modes: default to glTF Repeat
        auto toAddress = [](fastgltf::Wrap w) -> VkSamplerAddressMode {
            switch (w) {
                case fastgltf::Wrap::ClampToEdge: return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                case fastgltf::Wrap::MirroredRepeat: return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
                case fastgltf::Wrap::Repeat:
                default: return VK_SAMPLER_ADDRESS_MODE_REPEAT;
            }
        };
        // fastgltf::Sampler::wrapS/wrapT are non-optional and already default to Repeat
        sampl.addressModeU = toAddress(sampler.wrapS);
        sampl.addressModeV = toAddress(sampler.wrapT);
        sampl.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sampl.unnormalizedCoordinates = VK_FALSE;

        VkSampler newSampler;
        vkCreateSampler(engine->_deviceManager->device(), &sampl, nullptr, &newSampler);

        file.samplers.push_back(newSampler);
    }
    //< load_samplers

    report_progress(0.2f);
    //> load_arrays
    // temporal arrays for all the objects to use while creating the GLTF data
    std::vector<std::shared_ptr<MeshAsset> > meshes;
    std::vector<std::shared_ptr<Node> > nodes;
    std::vector<std::shared_ptr<GLTFMaterial> > materials;
    //< load_arrays

    // Note: glTF images are now loaded on-demand via TextureCache.
    // Resolve external image paths relative to the source glTF file directory
    // to avoid failing to find textures when running from a different CWD.
    const std::filesystem::path baseDir = path.parent_path();
    auto buildTextureKey = [&](size_t imgIndex, bool srgb) -> TextureCache::TextureKey
    {
        TextureCache::TextureKey key{};
        key.srgb = srgb;
        key.mipmapped = true;
        if (imgIndex >= gltf.images.size())
        {
            key.hash = 0; // invalid
            return key;
        }
        fastgltf::Image &image = gltf.images[imgIndex];
        std::visit(fastgltf::visitor{
            [&](fastgltf::sources::URI &filePath)
            {
                const std::string rel(filePath.uri.path().begin(), filePath.uri.path().end());
                // Build an absolute (or at least baseDir-resolved) path for IO + stable keying
                std::filesystem::path resolved = std::filesystem::path(rel);
                if (resolved.is_relative())
                {
                    resolved = baseDir / resolved;
                }
                key.kind = TextureCache::TextureKey::SourceKind::FilePath;
                key.path = resolved.string();
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
        return key;
    };

    //> load_buffer
    // create buffer to hold the material data
    file.materialDataBuffer = engine->_resourceManager->create_buffer(
        sizeof(GLTFMetallic_Roughness::MaterialConstants) * gltf.materials.size(),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
    int data_index = 0;
    GLTFMetallic_Roughness::MaterialConstants *sceneMaterialConstants = (GLTFMetallic_Roughness::MaterialConstants *)
            file.materialDataBuffer.info.pMappedData;
    //< load_buffer
    //
    //> load_material
    for (fastgltf::Material &mat: gltf.materials)
    {
        if (is_cancelled()) return {};

        std::shared_ptr<GLTFMaterial> newMat = std::make_shared<GLTFMaterial>();
        materials.push_back(newMat);
        file.materials[mat.name.c_str()] = newMat;

        GLTFMetallic_Roughness::MaterialConstants constants{};
        constants.colorFactors.x = mat.pbrData.baseColorFactor[0];
        constants.colorFactors.y = mat.pbrData.baseColorFactor[1];
        constants.colorFactors.z = mat.pbrData.baseColorFactor[2];
        constants.colorFactors.w = mat.pbrData.baseColorFactor[3];

        constants.metal_rough_factors.x = mat.pbrData.metallicFactor;
        constants.metal_rough_factors.y = mat.pbrData.roughnessFactor;
        // extra[0].x: normalScale (default 1.0)
        constants.extra[0].x = 1.0f;
        // extra[0].y: occlusionStrength (0..1)
        // extra[0].z: hasAO flag (1.0 if an occlusionTexture is present, 0.0 otherwise)
        if (mat.occlusionTexture.has_value())
        {
            constants.extra[0].y = mat.occlusionTexture->strength;
            constants.extra[0].z = 1.0f;
        }
        else
        {
            constants.extra[0].y = 0.0f;
            constants.extra[0].z = 0.0f;
        }
        // extra[1].rgb: emissiveFactor
        constants.extra[1].x = mat.emissiveFactor[0];
        constants.extra[1].y = mat.emissiveFactor[1];
        constants.extra[1].z = mat.emissiveFactor[2];
        // If an emissive texture is present but the factor is left at 0,
        // default to white so the texture is visible (common authoring pattern).
        if (mat.emissiveTexture.has_value())
        {
            if (constants.extra[1].x == 0.0f &&
                constants.extra[1].y == 0.0f &&
                constants.extra[1].z == 0.0f)
            {
                constants.extra[1].x = 1.0f;
                constants.extra[1].y = 1.0f;
                constants.extra[1].z = 1.0f;
            }
        }
        // extra[2].x: alphaCutoff for MASK materials (>0 enables alpha test)
        constants.extra[2].x = 0.0f;
        if (mat.alphaMode == fastgltf::AlphaMode::Mask)
        {
            constants.extra[2].x = static_cast<float>(mat.alphaCutoff);
        }
        // write material parameters to buffer
        sceneMaterialConstants[data_index] = constants;

        MaterialPass passType = MaterialPass::MainColor;
        if (mat.alphaMode == fastgltf::AlphaMode::Blend)
        {
            passType = MaterialPass::Transparent;
        }

        GLTFMetallic_Roughness::MaterialResources materialResources;
        // default the material textures
        materialResources.colorImage = engine->_whiteImage;
        materialResources.colorSampler = engine->_samplerManager->defaultLinear();
        materialResources.metalRoughImage = engine->_whiteImage;
        materialResources.metalRoughSampler = engine->_samplerManager->defaultLinear();
        materialResources.normalImage = engine->_flatNormalImage;
        materialResources.normalSampler = engine->_samplerManager->defaultLinear();
        materialResources.occlusionImage = engine->_whiteImage;
        materialResources.occlusionSampler = engine->_samplerManager->defaultLinear();
        materialResources.emissiveImage = engine->_blackImage;
        materialResources.emissiveSampler = engine->_samplerManager->defaultLinear();

        // set the uniform buffer for the material data
        materialResources.dataBuffer = file.materialDataBuffer.buffer;
        materialResources.dataBufferOffset = data_index * sizeof(GLTFMetallic_Roughness::MaterialConstants);
        // Dynamic texture bindings via TextureCache (fallbacks are already set)
        TextureCache *cache = engine->_context->textures;
        TextureCache::TextureHandle hColor = TextureCache::InvalidHandle;
        TextureCache::TextureHandle hMRO  = TextureCache::InvalidHandle;
        TextureCache::TextureHandle hNorm = TextureCache::InvalidHandle;
        TextureCache::TextureHandle hOcc  = TextureCache::InvalidHandle;
        TextureCache::TextureHandle hEmissive = TextureCache::InvalidHandle;

        if (cache && mat.pbrData.baseColorTexture.has_value())
        {
            const auto &tex = gltf.textures[mat.pbrData.baseColorTexture.value().textureIndex];
            const size_t imgIndex = tex.imageIndex.value();
            const bool hasSampler = tex.samplerIndex.has_value();
            const VkSampler sampler = hasSampler ? file.samplers[tex.samplerIndex.value()] : engine->_samplerManager->defaultLinear();
            auto key = buildTextureKey(imgIndex, true);
            if (key.hash != 0)
            {
                hColor = cache->request(key, sampler);
                materialResources.colorSampler = sampler;
            }
        }

        if (cache && mat.pbrData.metallicRoughnessTexture.has_value())
        {
            const auto &tex = gltf.textures[mat.pbrData.metallicRoughnessTexture.value().textureIndex];
            const size_t imgIndex = tex.imageIndex.value();
            const bool hasSampler = tex.samplerIndex.has_value();
            const VkSampler sampler = hasSampler ? file.samplers[tex.samplerIndex.value()] : engine->_samplerManager->defaultLinear();
            auto key = buildTextureKey(imgIndex, false);
            if (key.hash != 0)
            {
                hMRO = cache->request(key, sampler);
                materialResources.metalRoughSampler = sampler;
            }
        }

        if (cache && mat.occlusionTexture.has_value())
        {
            const auto &tex = gltf.textures[mat.occlusionTexture->textureIndex];
            const size_t imgIndex = tex.imageIndex.value();
            const bool hasSampler = tex.samplerIndex.has_value();
            const VkSampler sampler = hasSampler ? file.samplers[tex.samplerIndex.value()] : engine->_samplerManager->defaultLinear();
            auto key = buildTextureKey(imgIndex, false);
            key.channels = TextureCache::TextureKey::ChannelsHint::R;
            if (key.hash != 0)
            {
                hOcc = cache->request(key, sampler);
                materialResources.occlusionSampler = sampler;
            }
        }

        if (cache && mat.emissiveTexture.has_value())
        {
            const auto &tex = gltf.textures[mat.emissiveTexture->textureIndex];
            const size_t imgIndex = tex.imageIndex.value();
            const bool hasSampler = tex.samplerIndex.has_value();
            const VkSampler sampler = hasSampler ? file.samplers[tex.samplerIndex.value()] : engine->_samplerManager->defaultLinear();
            auto key = buildTextureKey(imgIndex, true);
            if (key.hash != 0)
            {
                hEmissive = cache->request(key, sampler);
                materialResources.emissiveSampler = sampler;
            }
        }

        if (cache && mat.normalTexture.has_value())
        {
            const auto &tex = gltf.textures[mat.normalTexture.value().textureIndex];
            const size_t imgIndex = tex.imageIndex.value();
            const bool hasSampler = tex.samplerIndex.has_value();
            const VkSampler sampler = hasSampler ? file.samplers[tex.samplerIndex.value()] : engine->_samplerManager->defaultLinear();
            auto key = buildTextureKey(imgIndex, false);
            key.channels = TextureCache::TextureKey::ChannelsHint::RG; // prefer BC5 for normals
            if (key.hash != 0)
            {
                hNorm = cache->request(key, sampler);
                materialResources.normalSampler = sampler;
            }
            // Store normal scale if provided
            sceneMaterialConstants[data_index].extra[0].x = mat.normalTexture->scale;
        }
        // build material
        newMat->data = engine->metalRoughMaterial.write_material(engine->_deviceManager->device(), passType, materialResources,
                                                                 file.descriptorPool);

        // Register descriptor patches for dynamic textures
        if (cache)
        {
            if (hColor != TextureCache::InvalidHandle)
            {
                cache->watchBinding(hColor, newMat->data.materialSet, 1u, materialResources.colorSampler,
                                    engine->_whiteImage.imageView);
            }
            if (hMRO != TextureCache::InvalidHandle)
            {
                cache->watchBinding(hMRO, newMat->data.materialSet, 2u, materialResources.metalRoughSampler,
                                    engine->_whiteImage.imageView);
            }
            if (hNorm != TextureCache::InvalidHandle)
            {
                cache->watchBinding(hNorm, newMat->data.materialSet, 3u, materialResources.normalSampler,
                                    engine->_flatNormalImage.imageView);
            }
            if (hOcc != TextureCache::InvalidHandle)
            {
                cache->watchBinding(hOcc, newMat->data.materialSet, 4u, materialResources.occlusionSampler,
                                    engine->_whiteImage.imageView);
            }
            if (hEmissive != TextureCache::InvalidHandle)
            {
                cache->watchBinding(hEmissive, newMat->data.materialSet, 5u, materialResources.emissiveSampler,
                                    engine->_blackImage.imageView);
            }
        }

        data_index++;
    }
    //< load_material

    // Rough progress after materials and texture requests
    if (!gltf.meshes.empty())
    {
        report_progress(0.25f);
    }

    // Flush material constants buffer so GPU sees updated data on non-coherent memory
    if (!gltf.materials.empty())
    {
        VkDeviceSize totalSize = sizeof(GLTFMetallic_Roughness::MaterialConstants) * gltf.materials.size();
        vmaFlushAllocation(engine->_deviceManager->allocator(), file.materialDataBuffer.allocation, 0, totalSize);
    }

    // use the same vectors for all meshes so that the memory doesnt reallocate as
    // often
    std::vector<uint32_t> indices;
    std::vector<Vertex> vertices;

    for (size_t meshIndex = 0; meshIndex < gltf.meshes.size(); ++meshIndex)
    {
        if (is_cancelled()) return {};

        fastgltf::Mesh &mesh = gltf.meshes[meshIndex];
        std::shared_ptr<MeshAsset> newmesh = std::make_shared<MeshAsset>();
        meshes.push_back(newmesh);
        file.meshes[mesh.name.c_str()] = newmesh;
        newmesh->name = mesh.name;

        // clear the mesh arrays each mesh, we dont want to merge them by error
        indices.clear();
        vertices.clear();

        for (auto &&p: mesh.primitives)
        {
            GeoSurface newSurface;
            newSurface.startIndex = (uint32_t) indices.size();
            newSurface.count = (uint32_t) gltf.accessors[p.indicesAccessor.value()].count;

            size_t initial_vtx = vertices.size();

            // load indexes
            {
                fastgltf::Accessor &indexaccessor = gltf.accessors[p.indicesAccessor.value()];
                indices.reserve(indices.size() + indexaccessor.count);

                fastgltf::iterateAccessor<std::uint32_t>(gltf, indexaccessor,
                                                         [&](std::uint32_t idx) {
                                                             indices.push_back(idx + initial_vtx);
                                                         });
            }

            // load vertex positions
            {
                fastgltf::Accessor &posAccessor = gltf.accessors[p.findAttribute("POSITION")->second];
                vertices.resize(vertices.size() + posAccessor.count);

                fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, posAccessor,
                                                              [&](glm::vec3 v, size_t index) {
                                                                  Vertex newvtx{};
                                                                  newvtx.position = v;
                                                                  newvtx.normal = {1, 0, 0};
                                                                  newvtx.color = glm::vec4{1.f};
                                                                  newvtx.uv_x = 0;
                                                                  newvtx.uv_y = 0;
                                                                  newvtx.tangent = glm::vec4(1,0,0,1);
                                                                  vertices[initial_vtx + index] = newvtx;
                                                              });
            }

            // load vertex normals
            auto normals = p.findAttribute("NORMAL");
            if (normals != p.attributes.end())
            {
                fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, gltf.accessors[(*normals).second],
                                                              [&](glm::vec3 v, size_t index) {
                                                                  vertices[initial_vtx + index].normal = v;
                                                              });
            }

            // load UVs
            auto uv = p.findAttribute("TEXCOORD_0");
            if (uv != p.attributes.end())
            {
                fastgltf::iterateAccessorWithIndex<glm::vec2>(gltf, gltf.accessors[(*uv).second],
                                                              [&](glm::vec2 v, size_t index) {
                                                                  vertices[initial_vtx + index].uv_x = v.x;
                                                                  vertices[initial_vtx + index].uv_y = v.y;
                                                              });
            }

            // load vertex colors
            auto colors = p.findAttribute("COLOR_0");
            if (colors != p.attributes.end())
            {
                fastgltf::iterateAccessorWithIndex<glm::vec4>(gltf, gltf.accessors[(*colors).second],
                                                              [&](glm::vec4 v, size_t index) {
                                                                  vertices[initial_vtx + index].color = v;
                                                              });
            }

            // load tangents if present (vec4, w = sign)
            auto tangents = p.findAttribute("TANGENT");
            bool hasTangents = tangents != p.attributes.end();
            if (hasTangents)
            {
                fastgltf::iterateAccessorWithIndex<glm::vec4>(gltf, gltf.accessors[(*tangents).second],
                                                              [&](glm::vec4 v, size_t index) {
                                                                  vertices[initial_vtx + index].tangent = v;
                                                              });
            }

            // Generate tangents only when needed for normal mapping.
            // If the bound material has no normal map, we can skip TBN generation.
            bool materialHasNormalMap = false;
            if (p.materialIndex.has_value())
            {
                size_t matIndex = p.materialIndex.value();
                if (matIndex < gltf.materials.size())
                {
                    materialHasNormalMap = gltf.materials[matIndex].normalTexture.has_value();
                }
            }
            else if (!gltf.materials.empty())
            {
                // Primitives without an explicit material fall back to material 0.
                materialHasNormalMap = gltf.materials[0].normalTexture.has_value();
            }

            if (!hasTangents && materialHasNormalMap)
            {
                size_t primIndexStart = newSurface.startIndex;
                size_t primIndexCount = newSurface.count;
                size_t primVertexStart = initial_vtx;
                size_t primVertexCount = vertices.size() - initial_vtx;
                geom::generate_tangents_range(vertices, indices,
                                              primIndexStart, primIndexCount,
                                              primVertexStart, primVertexCount);
            }

            if (p.materialIndex.has_value())
            {
                newSurface.material = materials[p.materialIndex.value()];
            }
            else
            {
                newSurface.material = materials[0];
            }

            // Compute per-surface bounds using only the indices referenced by this primitive.
            if (newSurface.count > 0)
            {
                uint32_t firstIndex = newSurface.startIndex;
                uint32_t lastIndex = newSurface.startIndex + newSurface.count;
                uint32_t baseVertex = indices[firstIndex];
                glm::vec3 minpos = vertices[baseVertex].position;
                glm::vec3 maxpos = vertices[baseVertex].position;
                for (uint32_t i = firstIndex + 1; i < lastIndex; i++)
                {
                    uint32_t vi = indices[i];
                    const glm::vec3 &p = vertices[vi].position;
                    minpos = glm::min(minpos, p);
                    maxpos = glm::max(maxpos, p);
                }
                newSurface.bounds.origin = (maxpos + minpos) / 2.f;
                newSurface.bounds.extents = (maxpos - minpos) / 2.f;
                newSurface.bounds.sphereRadius = glm::length(newSurface.bounds.extents);
                newSurface.bounds.type = BoundsType::Mesh;
            }
            else
            {
                newSurface.bounds.origin = glm::vec3(0.0f);
                newSurface.bounds.extents = glm::vec3(0.5f);
                newSurface.bounds.sphereRadius = glm::length(newSurface.bounds.extents);
                newSurface.bounds.type = BoundsType::Mesh;
            }
            newmesh->surfaces.push_back(newSurface);
        }

        // Build CPU BVH for precise picking over this mesh (triangle-level).
        {
            std::span<const Vertex> vSpan(vertices.data(), vertices.size());
            std::span<const uint32_t> iSpan(indices.data(), indices.size());
            newmesh->bvh = build_mesh_bvh(*newmesh, vSpan, iSpan);
        }

        newmesh->meshBuffers = engine->_resourceManager->uploadMesh(indices, vertices);
        // BLAS for this mesh will be built lazily from RayTracingManager::buildTLASFromDrawContext()
        // when ray-traced shadows are enabled. This avoids redundant builds and concentrates
        // RT work in one place.

        // If CPU vectors ballooned for this mesh, release capacity back to the OS
        auto shrink_if_huge = [](auto &vec, size_t elemSizeBytes) {
            const size_t capBytes = vec.capacity() * elemSizeBytes;
            const size_t kThreshold = 64ull * 1024ull * 1024ull; // 64 MiB
            if (capBytes > kThreshold)
            {
                using Vec = std::remove_reference_t<decltype(vec)>;
                Vec empty;
                vec.swap(empty);
            }
        };
        shrink_if_huge(indices, sizeof(uint32_t));
        shrink_if_huge(vertices, sizeof(Vertex));

        // Update progress based on meshes built so far; meshes/BVH/uploads get 0.6 of the range.
        float meshFrac = static_cast<float>(meshIndex + 1) / static_cast<float>(gltf.meshes.size());
        report_progress(0.2f + meshFrac * 0.6f);
    }
    //> load_nodes
    // load all nodes and their meshes
    std::vector<std::string> stable_node_names;
    stable_node_names.resize(gltf.nodes.size());
    {
        std::unordered_set<std::string> used_names;
        used_names.reserve(gltf.nodes.size());

        for (size_t nodeIndex = 0; nodeIndex < gltf.nodes.size(); ++nodeIndex)
        {
            const fastgltf::Node &node = gltf.nodes[nodeIndex];
            std::string base_name = node.name.empty() ? fmt::format("__node_{}", nodeIndex) : std::string(node.name);

            std::string unique_name = base_name;
            if (used_names.contains(unique_name))
            {
                uint32_t suffix = 2;
                do
                {
                    unique_name = fmt::format("{}#{}", base_name, suffix++);
                }
                while (used_names.contains(unique_name));
            }

            used_names.insert(unique_name);
            stable_node_names[nodeIndex] = std::move(unique_name);
        }
    }

    for (size_t nodeIndex = 0; nodeIndex < gltf.nodes.size(); ++nodeIndex)
    {
        if (is_cancelled()) return {};

        fastgltf::Node &node = gltf.nodes[nodeIndex];
        std::shared_ptr<Node> newNode;

        // find if the node has a mesh, and if it does hook it to the mesh pointer and allocate it with the meshnode class
        if (node.meshIndex.has_value())
        {
            auto meshNode = std::make_shared<MeshNode>();
            meshNode->mesh = meshes[*node.meshIndex];
            meshNode->scene = &file;
            newNode = meshNode;
        }
        else
        {
            newNode = std::make_shared<Node>();
        }

        nodes.push_back(newNode);
        file.nodes[stable_node_names[nodeIndex]] = newNode;

        std::visit(fastgltf::visitor{
                       [&](fastgltf::Node::TransformMatrix matrix) {
                           glm::mat4 m(1.0f);
                           memcpy(&m, matrix.data(), sizeof(matrix));

                           glm::vec3 t;
                           glm::quat r;
                           glm::vec3 s;
                           decompose_trs_matrix(m, t, r, s);
                           newNode->setTRS(t, r, s);
                       },
                       [&](fastgltf::Node::TRS transform) {
                           glm::vec3 tl(transform.translation[0], transform.translation[1],
                                        transform.translation[2]);
                           glm::quat rot(transform.rotation[3], transform.rotation[0], transform.rotation[1],
                                         transform.rotation[2]);
                           glm::vec3 sc(transform.scale[0], transform.scale[1], transform.scale[2]);

                           newNode->setTRS(tl, rot, sc);
                       }
                   },
                   node.transform);

        // Node building and hierarchy wiring shares a small slice of progress.
        if (!gltf.nodes.empty())
        {
            float nodeFrac = static_cast<float>(nodeIndex + 1) / static_cast<float>(gltf.nodes.size());
            // Reserve 0.1 of the total range for nodes/animations/transforms.
            report_progress(0.8f + nodeFrac * 0.1f);
        }
    }
    //< load_nodes
    //> load_graph
    // run loop again to setup transform hierarchy
    for (int i = 0; i < gltf.nodes.size(); i++)
    {
        fastgltf::Node &node = gltf.nodes[i];
        std::shared_ptr<Node> &sceneNode = nodes[i];

        for (auto &c: node.children)
        {
            sceneNode->children.push_back(nodes[c]);
            nodes[c]->parent = sceneNode;
        }
    }

    // find the top nodes, with no parents
    for (auto &node: nodes)
    {
        if (node->parent.lock() == nullptr)
        {
            file.topNodes.push_back(node);
            node->refreshTransform(glm::mat4{1.f});
        }
    }

    // Load animations (if present)
    if (!gltf.animations.empty())
    {
        file.animations.reserve(gltf.animations.size());

        for (auto &anim: gltf.animations)
        {
            LoadedGLTF::Animation dstAnim;
            dstAnim.name = anim.name.c_str();
            dstAnim.duration = 0.0f;

            dstAnim.channels.reserve(anim.channels.size());

            for (auto &ch: anim.channels)
            {
                if (ch.nodeIndex >= nodes.size() || ch.samplerIndex >= anim.samplers.size())
                {
                    continue;
                }

                LoadedGLTF::AnimationChannel channel{};
                channel.node = nodes[ch.nodeIndex];

                switch (ch.path)
                {
                    case fastgltf::AnimationPath::Translation:
                        channel.target = LoadedGLTF::AnimationChannel::Target::Translation;
                        break;
                    case fastgltf::AnimationPath::Rotation:
                        channel.target = LoadedGLTF::AnimationChannel::Target::Rotation;
                        break;
                    case fastgltf::AnimationPath::Scale:
                        channel.target = LoadedGLTF::AnimationChannel::Target::Scale;
                        break;
                    default:
                        // Weights and other paths not yet supported
                        continue;
                }

                const fastgltf::AnimationSampler &sampler = anim.samplers[ch.samplerIndex];
                switch (sampler.interpolation)
                {
                    case fastgltf::AnimationInterpolation::Step:
                        channel.interpolation = LoadedGLTF::AnimationChannel::Interpolation::Step;
                        break;
                    case fastgltf::AnimationInterpolation::Linear:
                    case fastgltf::AnimationInterpolation::CubicSpline:
                    default:
                        channel.interpolation = LoadedGLTF::AnimationChannel::Interpolation::Linear;
                        break;
                }

                // Input times
                const auto &timeAccessor = gltf.accessors[sampler.inputAccessor];
                channel.times.reserve(timeAccessor.count);
                float maxTime = 0.0f;

                fastgltf::iterateAccessorWithIndex<float>(gltf, timeAccessor,
                    [&](float value, size_t) {
                        channel.times.push_back(value);
                        if (value > maxTime) maxTime = value;
                    });

                // Output values
                const auto &valueAccessor = gltf.accessors[sampler.outputAccessor];
                const bool isCubic = sampler.interpolation == fastgltf::AnimationInterpolation::CubicSpline;

                if (channel.target == LoadedGLTF::AnimationChannel::Target::Rotation)
                {
                    channel.vec4Values.clear();
                    channel.vec4Values.reserve(valueAccessor.count);

                    fastgltf::iterateAccessorWithIndex<glm::vec4>(gltf, valueAccessor,
                        [&](glm::vec4 v, size_t index) {
                            if (isCubic)
                            {
                                // For cubic-spline, values are [in, value, out]; keep only the middle one.
                                if (index % 3 != 1) return;
                            }
                            channel.vec4Values.push_back(v);
                        });
                }
                else
                {
                    channel.vec3Values.clear();
                    channel.vec3Values.reserve(valueAccessor.count);

                    fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, valueAccessor,
                        [&](glm::vec3 v, size_t index) {
                            if (isCubic)
                            {
                                if (index % 3 != 1) return;
                            }
                            channel.vec3Values.push_back(v);
                        });
                }

                if (!channel.times.empty())
                {
                    dstAnim.duration = std::max(dstAnim.duration, maxTime);
                    dstAnim.channels.push_back(std::move(channel));
                }
            }

            if (!dstAnim.channels.empty())
            {
                file.animations.push_back(std::move(dstAnim));
            }
        }

        // Default animation state is now owned by SceneManager per static scene / instance.
        // LoadedGLTF only stores shared animation clips.
    }

    report_progress(0.95f);

    // We no longer need glTF-owned buffer payloads; free any large vectors
    for (auto &buf : gltf.buffers)
    {
        std::visit(fastgltf::visitor{
            [](auto &arg) {},
            [&](fastgltf::sources::Vector &vec) {
                std::vector<uint8_t>().swap(vec.bytes);
            }
        }, buf.data);
    }
    for (auto &img : gltf.images)
    {
        std::visit(fastgltf::visitor{
            [](auto &arg) {},
            [&](fastgltf::sources::Vector &vec) {
                std::vector<uint8_t>().swap(vec.bytes);
            }
        }, img.data);
    }
    fmt::println("[GLTF] loadGltf done: meshes={} materials={} images={} samplers={} animations={} debugName='{}'",
                 file.meshes.size(),
                 file.materials.size(),
                 file.images.size(),
                 file.samplers.size(),
                 file.animations.size(),
                 file.debugName.empty() ? "<none>" : file.debugName);

    report_progress(1.0f);
    return scene;
    //< load_graph
}

void LoadedGLTF::Draw(const glm::mat4 &topMatrix, DrawContext &ctx)
{
    // create renderables from the scenenodes
    for (auto &n: topNodes)
    {
        n->Draw(topMatrix, ctx);
    }
}

std::shared_ptr<Node> LoadedGLTF::getNode(const std::string &name)
{
    auto it = nodes.find(name);
    return (it != nodes.end()) ? it->second : nullptr;
}

void LoadedGLTF::refreshAllTransforms()
{
    for (auto &n: topNodes)
    {
        if (n)
        {
            n->refreshTransform(glm::mat4{1.f});
        }
    }
}

void LoadedGLTF::build_colliders_from_markers(bool clear_existing)
{
    Physics::build_colliders_from_markers(collider_compounds, *this, clear_existing);
}

void LoadedGLTF::build_mesh_colliders_from_markers(bool clear_existing)
{
    Physics::build_mesh_colliders_from_markers(collider_mesh_instances, *this, clear_existing);
}

void LoadedGLTF::build_colliders_from_sidecar(const LoadedGLTF &sidecar, bool clear_existing)
{
    std::unordered_set<std::string_view> dst_names;
    dst_names.reserve(nodes.size());
    for (const auto &[name, ptr] : nodes)
    {
        (void) ptr;
        dst_names.insert(name);
    }

    Physics::build_colliders_from_sidecar(collider_compounds, sidecar, dst_names, clear_existing);
}

void LoadedGLTF::build_mesh_colliders_from_sidecar(const LoadedGLTF &sidecar, bool clear_existing)
{
    std::unordered_set<std::string_view> dst_names;
    dst_names.reserve(nodes.size());
    for (const auto &[name, ptr] : nodes)
    {
        (void) ptr;
        dst_names.insert(name);
    }

    Physics::build_mesh_colliders_from_sidecar(collider_mesh_instances, sidecar, dst_names, clear_existing);
}

void LoadedGLTF::ensureRestTransformsCached()
{
    if (_restTransformsCached)
    {
        return;
    }

    _restTransformsCached = true;
    _restTransforms.clear();
    _restTransforms.reserve(nodes.size());

    for (auto &[name, nodePtr] : nodes)
    {
        if (!nodePtr)
        {
            continue;
        }

        RestNodeTransform rest{};
        rest.localMatrix = nodePtr->localTransform;
        rest.hasTRS = nodePtr->hasTRS;
        rest.translation = nodePtr->translation;
        rest.rotation = glm::normalize(nodePtr->rotation);
        rest.scale = nodePtr->scale;

        if (!rest.hasTRS)
        {
            // Decompose so partial-channel animations (e.g. translation-only) have a stable base for R/S.
            glm::vec3 skew(0.0f);
            glm::vec4 perspective(0.0f);
            glm::vec3 scale(1.0f);
            glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 translation(0.0f);
            if (glm::decompose(rest.localMatrix, scale, orientation, translation, skew, perspective))
            {
                rest.translation = translation;
                rest.rotation = glm::normalize(orientation);
                rest.scale = scale;
            }
        }

        _restTransforms.emplace(nodePtr.get(), rest);
    }
}

void LoadedGLTF::restoreNodeToRest(Node &node) const
{
    auto it = _restTransforms.find(&node);
    if (it == _restTransforms.end())
    {
        return;
    }

    const RestNodeTransform &rest = it->second;
    node.localTransform = rest.localMatrix;
    node.translation = rest.translation;
    node.rotation = rest.rotation;
    node.scale = rest.scale;
    node.hasTRS = rest.hasTRS;
}

void LoadedGLTF::setActiveAnimation(AnimationState &state, int index, bool resetTime)
{
    if (animations.empty())
    {
        state.activeAnimation = -1;
        state.blending = false;
        state.blendFromAnimation = -1;
        return;
    }

    if (index < 0)
    {
        state.activeAnimation = -1;
        state.blending = false;
        state.blendFromAnimation = -1;
        if (resetTime)
        {
            state.animationTime = 0.0f;
        }
        return;
    }

    if (index >= static_cast<int>(animations.size()))
    {
        index = 0;
    }

    state.activeAnimation = index;
    state.blending = false;
    state.blendFromAnimation = -1;
    if (resetTime)
    {
        state.animationTime = 0.0f;
    }
}

void LoadedGLTF::setActiveAnimation(AnimationState &state, const std::string &name, bool resetTime)
{
    for (size_t i = 0; i < animations.size(); ++i)
    {
        if (animations[i].name == name)
        {
            setActiveAnimation(state, static_cast<int>(i), resetTime);
            return;
        }
    }
}

void LoadedGLTF::transitionAnimation(AnimationState &state, int index, float blendDurationSeconds, bool resetTime)
{
    if (animations.empty())
    {
        setActiveAnimation(state, -1, resetTime);
        return;
    }

    if (index < 0)
    {
        // For now, treat disable as an immediate stop (restoring happens in updateAnimation).
        setActiveAnimation(state, -1, resetTime);
        return;
    }

    if (index >= static_cast<int>(animations.size()))
    {
        index = 0;
    }

    const bool canBlend =
        (blendDurationSeconds > 0.0f) &&
        (state.activeAnimation >= 0) &&
        (state.activeAnimation < static_cast<int>(animations.size())) &&
        (state.activeAnimation != index);

    if (!canBlend)
    {
        setActiveAnimation(state, index, resetTime);
        return;
    }

    state.blending = true;
    state.blendFromAnimation = state.activeAnimation;
    state.blendFromTime = state.animationTime;
    state.blendFromLoop = state.animationLoop;
    state.blendTime = 0.0f;
    state.blendDuration = blendDurationSeconds;

    state.activeAnimation = index;
    if (resetTime)
    {
        state.animationTime = 0.0f;
    }
}

void LoadedGLTF::transitionAnimation(AnimationState &state, const std::string &name, float blendDurationSeconds, bool resetTime)
{
    for (size_t i = 0; i < animations.size(); ++i)
    {
        if (animations[i].name == name)
        {
            transitionAnimation(state, static_cast<int>(i), blendDurationSeconds, resetTime);
            return;
        }
    }
}

void LoadedGLTF::updateAnimation(float dt, AnimationState &state)
{
    ensureRestTransformsCached();

    const bool hasAnims = !animations.empty();
    const bool hasActive =
        hasAnims && state.activeAnimation >= 0 && state.activeAnimation < static_cast<int>(animations.size());
    const bool hasBlendFrom =
        hasAnims && state.blending &&
        state.blendFromAnimation >= 0 && state.blendFromAnimation < static_cast<int>(animations.size());

    // If animation is disabled, restore nodes affected by the previous update back to bind pose.
    if (!hasActive && !hasBlendFrom)
    {
        if (!state.touchedNodes.empty())
        {
            for (Node *node : state.touchedNodes)
            {
                if (node)
                {
                    restoreNodeToRest(*node);
                }
            }
            state.touchedNodes.clear();
            refreshAllTransforms();
        }
        return;
    }

    const float dtUnscaled = dt;
    const float dtScaled = dtUnscaled * state.playbackSpeed;

    auto advance_time = [](float &time, const Animation &clip, bool loop, float delta) {
        if (clip.duration <= 0.0f || delta == 0.0f)
        {
            return;
        }

        time += delta;

        if (loop)
        {
            time = std::fmod(time, clip.duration);
            if (time < 0.0f)
            {
                time += clip.duration;
            }
        }
        else
        {
            time = std::clamp(time, 0.0f, clip.duration);
        }
    };

    if (hasActive)
    {
        advance_time(state.animationTime, animations[state.activeAnimation], state.animationLoop, dtScaled);
    }
    if (hasBlendFrom)
    {
        advance_time(state.blendFromTime, animations[state.blendFromAnimation], state.blendFromLoop, dtScaled);
    }

    float blendAlpha = 1.0f;
    if (hasBlendFrom)
    {
        if (dtUnscaled > 0.0f)
        {
            state.blendTime += dtUnscaled;
        }

        if (state.blendDuration > 0.0f)
        {
            blendAlpha = std::clamp(state.blendTime / state.blendDuration, 0.0f, 1.0f);
        }

        if (blendAlpha >= 1.0f)
        {
            state.blending = false;
            state.blendFromAnimation = -1;
            state.blendTime = 0.0f;
            state.blendDuration = 0.0f;
        }
    }

    struct NodePoseOverride
    {
        bool hasT = false;
        bool hasR = false;
        bool hasS = false;
        glm::vec3 t{0.0f, 0.0f, 0.0f};
        glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 s{1.0f, 1.0f, 1.0f};
    };

    thread_local std::unordered_map<Node *, NodePoseOverride> poseTo;
    thread_local std::unordered_map<Node *, NodePoseOverride> poseFrom;
    thread_local std::unordered_set<Node *> touchedNow;

    poseTo.clear();
    poseFrom.clear();
    touchedNow.clear();

    auto sample_clip = [&](const Animation &clip, float t, std::unordered_map<Node *, NodePoseOverride> &out) {
        for (const auto &ch : clip.channels)
        {
            Node *node = ch.node.get();
            if (!node)
            {
                continue;
            }

            const size_t keyCount = ch.times.size();
            if (keyCount == 0)
            {
                continue;
            }

            touchedNow.insert(node);

            size_t k0 = 0;
            size_t k1 = 0;
            if (t <= ch.times.front())
            {
                k0 = k1 = 0;
            }
            else if (t >= ch.times.back())
            {
                k0 = k1 = keyCount - 1;
            }
            else
            {
                auto it = std::upper_bound(ch.times.begin(), ch.times.end(), t);
                k1 = static_cast<size_t>(std::distance(ch.times.begin(), it));
                k0 = (k1 > 0) ? (k1 - 1) : 0;
            }

            float t0 = ch.times[k0];
            float t1 = ch.times[k1];
            float alpha = 0.0f;
            if (k0 != k1 && t1 > t0)
            {
                alpha = (t - t0) / (t1 - t0);
                alpha = std::clamp(alpha, 0.0f, 1.0f);
            }

            NodePoseOverride &pose = out[node];

            switch (ch.target)
            {
                case AnimationChannel::Target::Translation:
                {
                    if (ch.vec3Values.size() != keyCount) break;
                    glm::vec3 v0 = ch.vec3Values[k0];
                    glm::vec3 v1 = ch.vec3Values[k1];
                    pose.t = (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                                 ? v0
                                 : (v0 * (1.0f - alpha) + v1 * alpha);
                    pose.hasT = true;
                    break;
                }
                case AnimationChannel::Target::Scale:
                {
                    if (ch.vec3Values.size() != keyCount) break;
                    glm::vec3 v0 = ch.vec3Values[k0];
                    glm::vec3 v1 = ch.vec3Values[k1];
                    pose.s = (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                                 ? v0
                                 : (v0 * (1.0f - alpha) + v1 * alpha);
                    pose.hasS = true;
                    break;
                }
                case AnimationChannel::Target::Rotation:
                {
                    if (ch.vec4Values.size() != keyCount) break;
                    glm::vec4 v0 = ch.vec4Values[k0];
                    glm::vec4 v1 = ch.vec4Values[k1];

                    glm::quat q0(v0.w, v0.x, v0.y, v0.z);
                    glm::quat q1(v1.w, v1.x, v1.y, v1.z);
                    pose.r = (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                                 ? q0
                                 : glm::slerp(q0, q1, alpha);
                    pose.r = glm::normalize(pose.r);
                    pose.hasR = true;
                    break;
                }
            }
        }
    };

    if (hasActive)
    {
        sample_clip(animations[state.activeAnimation], state.animationTime, poseTo);
    }
    if (hasBlendFrom)
    {
        sample_clip(animations[state.blendFromAnimation], state.blendFromTime, poseFrom);
    }

    bool anyChanged = false;

    // Restore nodes that were animated last update but are no longer affected by the current clips.
    if (!state.touchedNodes.empty())
    {
        for (Node *node : state.touchedNodes)
        {
            if (node && touchedNow.find(node) == touchedNow.end())
            {
                restoreNodeToRest(*node);
                anyChanged = true;
            }
        }
    }

    // Apply current pose(s) to all affected nodes.
    for (Node *node : touchedNow)
    {
        if (!node)
        {
            continue;
        }

        auto restIt = _restTransforms.find(node);
        if (restIt == _restTransforms.end())
        {
            continue;
        }

        const RestNodeTransform &rest = restIt->second;

        struct TRS
        {
            glm::vec3 t;
            glm::quat r;
            glm::vec3 s;
        };

        auto eval_pose = [&](const std::unordered_map<Node *, NodePoseOverride> &m) -> TRS {
            glm::vec3 t = rest.translation;
            glm::quat r = rest.rotation;
            glm::vec3 s = rest.scale;

            auto it = m.find(node);
            if (it != m.end())
            {
                const NodePoseOverride &p = it->second;
                if (p.hasT) t = p.t;
                if (p.hasR) r = p.r;
                if (p.hasS) s = p.s;
            }

            return TRS{t, r, s};
        };

        TRS to = eval_pose(poseTo);
        glm::vec3 outT = to.t;
        glm::quat outR = to.r;
        glm::vec3 outS = to.s;

        if (hasBlendFrom)
        {
            TRS from = eval_pose(poseFrom);
            outT = from.t * (1.0f - blendAlpha) + to.t * blendAlpha;
            outS = from.s * (1.0f - blendAlpha) + to.s * blendAlpha;
            outR = glm::normalize(glm::slerp(from.r, to.r, blendAlpha));
        }

        node->translation = outT;
        node->rotation = outR;
        node->scale = outS;
        node->hasTRS = true;
        node->updateLocalFromTRS();
        anyChanged = true;
    }

    state.touchedNodes.swap(touchedNow);
    touchedNow.clear();

    if (anyChanged)
    {
        refreshAllTransforms();
    }
}

void LoadedGLTF::clearAll()
{
    const char *name = debugName.empty() ? "<unnamed>" : debugName.c_str();
    fmt::println("[GLTF] clearAll begin for '{}' (meshes={} images={} materials={} samplers={})",
                 name,
                 meshes.size(),
                 images.size(),
                 materials.size(),
                 samplers.size());

    VkDevice dv = creator->_deviceManager->device();

    // Before destroying descriptor pools, unregister descriptor-set watches so
    // the TextureCache will not attempt to patch dead sets.
    if (creator && creator->_context && creator->_context->textures)
    {
        TextureCache *cache = creator->_context->textures;
        for (auto &[k, mat] : materials)
        {
            if (mat && mat->data.materialSet != VK_NULL_HANDLE)
            {
                cache->unwatchSet(mat->data.materialSet);
            }
        }
    }

    for (auto &[k, v]: meshes)
    {
        if (creator->_rayManager)
        {
            creator->_rayManager->removeBLASForMesh(v.get());
        }
        creator->_resourceManager->destroy_buffer(v->meshBuffers.indexBuffer);
        creator->_resourceManager->destroy_buffer(v->meshBuffers.vertexBuffer);
    }

    for (auto &[k, v]: images)
    {
        if (v.image == creator->_errorCheckerboardImage.image)
        {
            // dont destroy the default images
            continue;
        }
        creator->_resourceManager->destroy_image(v);
    }

    for (auto &sampler: samplers)
    {
        vkDestroySampler(dv, sampler, nullptr);
    }

    auto materialBuffer = materialDataBuffer;
    auto samplersToDestroy = samplers;

    descriptorPool.destroy_pools(dv);

    creator->_resourceManager->destroy_buffer(materialBuffer);

    fmt::println("[GLTF] clearAll done for '{}' (meshes={}, images={}, materials={}, samplers={})",
                 name,
                 meshes.size(),
                 images.size(),
                 materials.size(),
                 samplers.size());
}
