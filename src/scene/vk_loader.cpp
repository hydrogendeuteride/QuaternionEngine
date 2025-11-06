#include "stb_image.h"
#include <iostream>
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
        file.nodes[node.name.c_str()];

        std::visit(fastgltf::visitor{
                       [&](fastgltf::Node::TransformMatrix matrix) {
                           memcpy(&newNode->localTransform, matrix.data(), sizeof(matrix));
                       },
                       [&](fastgltf::Node::TRS transform) {
                           glm::vec3 tl(transform.translation[0], transform.translation[1],
                                        transform.translation[2]);
                           glm::quat rot(transform.rotation[3], transform.rotation[0], transform.rotation[1],
                                         transform.rotation[2]);
                           glm::vec3 sc(transform.scale[0], transform.scale[1], transform.scale[2]);

                           glm::mat4 tm = glm::translate(glm::mat4(1.f), tl);
                           glm::mat4 rm = glm::toMat4(rot);
                           glm::mat4 sm = glm::scale(glm::mat4(1.f), sc);

                           newNode->localTransform = tm * rm * sm;
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

void LoadedGLTF::clearAll()
{
    VkDevice dv = creator->_deviceManager->device();

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
