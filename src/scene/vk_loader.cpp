#include "stb_image.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "vk_loader.h"
#include "core/texture_cache.h"

#include "core/vk_engine.h"
#include "render/vk_materials.h"
#include "core/vk_initializers.h"
#include "core/vk_types.h"
#include "core/config.h"
#include <glm/gtx/quaternion.hpp>

#include <fastgltf/glm_element_traits.hpp>
#include <fastgltf/parser.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/util.hpp>
#include <optional>
#include "tangent_space.h"
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

std::optional<std::shared_ptr<LoadedGLTF> > loadGltf(VulkanEngine *engine, std::string_view filePath)
{
    //> load_1
    fmt::print("Loading GLTF: {}", filePath);

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
    //> load_2
    // we can stimate the descriptors we will need accurately
    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1}
    };

    file.descriptorPool.init(engine->_deviceManager->device(), gltf.materials.size(), sizes);
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
        std::shared_ptr<GLTFMaterial> newMat = std::make_shared<GLTFMaterial>();
        materials.push_back(newMat);
        file.materials[mat.name.c_str()] = newMat;

        GLTFMetallic_Roughness::MaterialConstants constants;
        // Defaults
        constants.extra[0].x = 1.0f; // normalScale
        constants.colorFactors.x = mat.pbrData.baseColorFactor[0];
        constants.colorFactors.y = mat.pbrData.baseColorFactor[1];
        constants.colorFactors.z = mat.pbrData.baseColorFactor[2];
        constants.colorFactors.w = mat.pbrData.baseColorFactor[3];

        constants.metal_rough_factors.x = mat.pbrData.metallicFactor;
        constants.metal_rough_factors.y = mat.pbrData.roughnessFactor;
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

        // set the uniform buffer for the material data
        materialResources.dataBuffer = file.materialDataBuffer.buffer;
        materialResources.dataBufferOffset = data_index * sizeof(GLTFMetallic_Roughness::MaterialConstants);
        // Dynamic texture bindings via TextureCache (fallbacks are already set)
        TextureCache *cache = engine->_context->textures;
        TextureCache::TextureHandle hColor = TextureCache::InvalidHandle;
        TextureCache::TextureHandle hMRO  = TextureCache::InvalidHandle;
        TextureCache::TextureHandle hNorm = TextureCache::InvalidHandle;

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
        }

        data_index++;
    }
    //< load_material

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

    for (fastgltf::Mesh &mesh: gltf.meshes)
    {
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

            // Generate tangents if missing and we have UVs
            if (!hasTangents)
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

            glm::vec3 minpos = vertices[initial_vtx].position;
            glm::vec3 maxpos = vertices[initial_vtx].position;
            for (int i = initial_vtx; i < vertices.size(); i++)
            {
                minpos = glm::min(minpos, vertices[i].position);
                maxpos = glm::max(maxpos, vertices[i].position);
            }

            newSurface.bounds.origin = (maxpos + minpos) / 2.f;
            newSurface.bounds.extents = (maxpos - minpos) / 2.f;
            newSurface.bounds.sphereRadius = glm::length(newSurface.bounds.extents);
            newmesh->surfaces.push_back(newSurface);
        }

        newmesh->meshBuffers = engine->_resourceManager->uploadMesh(indices, vertices);
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
        if (engine->_rayManager)
        {
            engine->_rayManager->getOrBuildBLAS(newmesh);
        }
    }
    //> load_nodes
    // load all nodes and their meshes
    for (fastgltf::Node &node: gltf.nodes)
    {
        std::shared_ptr<Node> newNode;

        // find if the node has a mesh, and if it does hook it to the mesh pointer and allocate it with the meshnode class
        if (node.meshIndex.has_value())
        {
            newNode = std::make_shared<MeshNode>();
            static_cast<MeshNode *>(newNode.get())->mesh = meshes[*node.meshIndex];
        }
        else
        {
            newNode = std::make_shared<Node>();
        }

        nodes.push_back(newNode);
        if (!node.name.empty())
        {
            file.nodes[std::string(node.name)] = newNode;
        }

        std::visit(fastgltf::visitor{
                       [&](fastgltf::Node::TransformMatrix matrix) {
                           glm::mat4 m(1.0f);
                           memcpy(&m, matrix.data(), sizeof(matrix));

                           glm::vec3 t = glm::vec3(m[3]);
                           glm::vec3 col0 = glm::vec3(m[0]);
                           glm::vec3 col1 = glm::vec3(m[1]);
                           glm::vec3 col2 = glm::vec3(m[2]);

                           glm::vec3 s(glm::length(col0), glm::length(col1), glm::length(col2));
                           if (s.x != 0.0f) col0 /= s.x;
                           if (s.y != 0.0f) col1 /= s.y;
                           if (s.z != 0.0f) col2 /= s.z;
                           glm::mat3 rotMat(col0, col1, col2);
                           glm::quat r = glm::quat_cast(rotMat);

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

        if (!file.animations.empty())
        {
            file.activeAnimation = 0;
            file.animationTime = 0.0f;
            file.animationLoop = true;
        }
    }

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

void LoadedGLTF::setActiveAnimation(int index, bool resetTime)
{
    if (animations.empty())
    {
        activeAnimation = -1;
        return;
    }

    if (index < 0 || index >= static_cast<int>(animations.size()))
    {
        index = 0;
    }

    activeAnimation = index;
    if (resetTime)
    {
        animationTime = 0.0f;
    }
}

void LoadedGLTF::setActiveAnimation(const std::string &name, bool resetTime)
{
    for (size_t i = 0; i < animations.size(); ++i)
    {
        if (animations[i].name == name)
        {
            setActiveAnimation(static_cast<int>(i), resetTime);
            return;
        }
    }
}

void LoadedGLTF::updateAnimation(float dt)
{
    if (animations.empty()) return;
    if (activeAnimation < 0 || activeAnimation >= static_cast<int>(animations.size())) return;
    if (dt <= 0.0f) return;

    Animation &clip = animations[activeAnimation];
    if (clip.duration <= 0.0f) return;

    animationTime += dt;
    if (animationLoop)
    {
        animationTime = std::fmod(animationTime, clip.duration);
        if (animationTime < 0.0f)
        {
            animationTime += clip.duration;
        }
    }
    else if (animationTime > clip.duration)
    {
        animationTime = clip.duration;
    }

    float t = animationTime;

    for (auto &ch: clip.channels)
    {
        if (!ch.node) continue;
        const size_t keyCount = ch.times.size();
        if (keyCount == 0) continue;

        size_t k1 = 0;
        while (k1 < keyCount && ch.times[k1] < t)
        {
            ++k1;
        }

        size_t k0;
        if (k1 == 0)
        {
            k0 = k1 = 0;
        }
        else if (k1 >= keyCount)
        {
            k0 = keyCount - 1;
            k1 = keyCount - 1;
        }
        else
        {
            k0 = k1 - 1;
        }

        float t0 = ch.times[k0];
        float t1 = ch.times[k1];
        float alpha = 0.0f;
        if (k0 != k1 && t1 > t0)
        {
            alpha = (t - t0) / (t1 - t0);
            alpha = std::clamp(alpha, 0.0f, 1.0f);
        }

        Node &node = *ch.node;

        switch (ch.target)
        {
            case AnimationChannel::Target::Translation:
            {
                if (ch.vec3Values.size() != keyCount) break;
                glm::vec3 v0 = ch.vec3Values[k0];
                glm::vec3 v1 = ch.vec3Values[k1];
                glm::vec3 v;
                if (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                {
                    v = v0;
                }
                else
                {
                    v = v0 * (1.0f - alpha) + v1 * alpha;
                }
                node.translation = v;
                node.hasTRS = true;
                break;
            }
            case AnimationChannel::Target::Scale:
            {
                if (ch.vec3Values.size() != keyCount) break;
                glm::vec3 v0 = ch.vec3Values[k0];
                glm::vec3 v1 = ch.vec3Values[k1];
                glm::vec3 v;
                if (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                {
                    v = v0;
                }
                else
                {
                    v = v0 * (1.0f - alpha) + v1 * alpha;
                }
                node.scale = v;
                node.hasTRS = true;
                break;
            }
            case AnimationChannel::Target::Rotation:
            {
                if (ch.vec4Values.size() != keyCount) break;
                glm::vec4 v0 = ch.vec4Values[k0];
                glm::vec4 v1 = ch.vec4Values[k1];

                glm::quat q0(v0.w, v0.x, v0.y, v0.z);
                glm::quat q1(v1.w, v1.x, v1.y, v1.z);
                glm::quat q;
                if (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                {
                    q = q0;
                }
                else
                {
                    q = glm::slerp(q0, q1, alpha);
                }
                node.rotation = glm::normalize(q);
                node.hasTRS = true;
                break;
            }
        }
    }

    // Rebuild local matrices from updated TRS and refresh world transforms
    for (auto &[name, nodePtr]: nodes)
    {
        if (nodePtr && nodePtr->hasTRS)
        {
            nodePtr->updateLocalFromTRS();
        }
    }

    refreshAllTransforms();
}

void LoadedGLTF::clearAll()
{
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
            creator->_rayManager->removeBLASForBuffer(v->meshBuffers.vertexBuffer.buffer);
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
}
