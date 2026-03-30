#include <algorithm>
#include <cmath>
#include <filesystem>
#include <optional>
#include <unordered_set>
#include <vector>

#include <fmt/format.h>
#include <glm/gtc/type_ptr.hpp>

#include <fastgltf/glm_element_traits.hpp>
#include <fastgltf/parser.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/util.hpp>

#include "vk_loader.h"

#include "core/assets/texture_cache.h"
#include "core/config.h"
#include "core/engine.h"
#include "core/types.h"
#include "mesh_bvh.h"
#include "render/materials.h"
#include "tangent_space.h"

namespace
{
constexpr auto kGltfLoadOptions =
    fastgltf::Options::DontRequireValidAssetMember |
    fastgltf::Options::AllowDouble |
    fastgltf::Options::LoadGLBBuffers |
    fastgltf::Options::LoadExternalBuffers;

struct ProgressReporter
{
    const GLTFLoadCallbacks *callbacks = nullptr;

    void report(float value) const
    {
        if (callbacks && callbacks->on_progress)
        {
            callbacks->on_progress(std::clamp(value, 0.0f, 1.0f));
        }
    }

    [[nodiscard]] bool cancelled() const
    {
        return callbacks && callbacks->is_cancelled && callbacks->is_cancelled();
    }
};

VkFilter extract_filter(fastgltf::Filter filter)
{
    switch (filter)
    {
        case fastgltf::Filter::Nearest:
        case fastgltf::Filter::NearestMipMapNearest:
        case fastgltf::Filter::NearestMipMapLinear:
            return VK_FILTER_NEAREST;
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

VkSamplerAddressMode extract_address_mode(fastgltf::Wrap wrap)
{
    switch (wrap)
    {
        case fastgltf::Wrap::ClampToEdge:
            return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        case fastgltf::Wrap::MirroredRepeat:
            return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
        case fastgltf::Wrap::Repeat:
        default:
            return VK_SAMPLER_ADDRESS_MODE_REPEAT;
    }
}

std::optional<fastgltf::Asset> load_gltf_asset(std::string_view file_path, const std::filesystem::path &base_path)
{
    fastgltf::Parser parser{};
    fastgltf::GltfDataBuffer data;
    data.loadFromFile(file_path);

    auto type = fastgltf::determineGltfFileType(&data);
    if (type == fastgltf::GltfType::glTF)
    {
        auto load = parser.loadGLTF(&data, base_path, kGltfLoadOptions);
        if (load)
        {
            return std::move(load.get());
        }

        Logger::error("Failed to load glTF: {}", fastgltf::to_underlying(load.error()));
        return std::nullopt;
    }

    if (type == fastgltf::GltfType::GLB)
    {
        auto load = parser.loadBinaryGLTF(&data, base_path, kGltfLoadOptions);
        if (load)
        {
            return std::move(load.get());
        }

        Logger::error("Failed to load glTF: {}", fastgltf::to_underlying(load.error()));
        return std::nullopt;
    }

    Logger::error("Failed to determine glTF container");
    return std::nullopt;
}

TextureCache::TextureKey build_texture_key(const fastgltf::Asset &gltf,
                                           const fastgltf::Image &image,
                                           const std::filesystem::path &base_dir,
                                           bool srgb)
{
    TextureCache::TextureKey key{};
    key.srgb = srgb;
    key.mipmapped = true;

    std::visit(fastgltf::visitor{
                   [&](const fastgltf::sources::URI &file_path)
                   {
                       const std::string relative_path(file_path.uri.path().begin(), file_path.uri.path().end());

                       std::filesystem::path resolved = std::filesystem::path(relative_path);
                       if (resolved.is_relative())
                       {
                           resolved = base_dir / resolved;
                       }

                       key.kind = TextureCache::TextureKey::SourceKind::FilePath;
                       key.path = resolved.string();

                       std::string id = std::string("GLTF:") + key.path + (srgb ? "#sRGB" : "#UNORM");
                       key.hash = texcache::fnv1a64(id);
                   },
                   [&](const fastgltf::sources::Vector &vector)
                   {
                       key.kind = TextureCache::TextureKey::SourceKind::Bytes;
                       key.bytes.assign(vector.bytes.begin(), vector.bytes.end());

                       uint64_t hash = texcache::fnv1a64(key.bytes.data(), key.bytes.size());
                       key.hash = hash ^ (srgb ? 0x9E3779B97F4A7C15ull : 0ull);
                   },
                   [&](const fastgltf::sources::BufferView &view)
                   {
                       const auto &buffer_view = gltf.bufferViews[view.bufferViewIndex];
                       const auto &buffer = gltf.buffers[buffer_view.bufferIndex];

                       std::visit(fastgltf::visitor{
                                      [](const auto &) {},
                                      [&](const fastgltf::sources::Vector &vector)
                                      {
                                          const size_t offset = buffer_view.byteOffset;
                                          const size_t length = buffer_view.byteLength;

                                          key.kind = TextureCache::TextureKey::SourceKind::Bytes;
                                          key.bytes.assign(vector.bytes.begin() + offset, vector.bytes.begin() + offset + length);

                                          uint64_t hash = texcache::fnv1a64(key.bytes.data(), key.bytes.size());
                                          key.hash = hash ^ (srgb ? 0x9E3779B97F4A7C15ull : 0ull);
                                      }
                                  },
                                  buffer.data);
                   },
                   [](const auto &) {}
               },
               image.data);

    return key;
}

std::vector<std::string> build_stable_node_names(const fastgltf::Asset &gltf)
{
    std::vector<std::string> stable_names(gltf.nodes.size());
    std::unordered_set<std::string> used_names;
    used_names.reserve(gltf.nodes.size());

    for (size_t node_index = 0; node_index < gltf.nodes.size(); ++node_index)
    {
        const fastgltf::Node &node = gltf.nodes[node_index];
        std::string base_name = node.name.empty() ? fmt::format("__node_{}", node_index) : std::string(node.name);

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
        stable_names[node_index] = std::move(unique_name);
    }

    return stable_names;
}

void release_gltf_payloads(fastgltf::Asset &gltf)
{
    for (auto &buffer : gltf.buffers)
    {
        std::visit(fastgltf::visitor{
                       [](auto &) {},
                       [&](fastgltf::sources::Vector &vector)
                       {
                           std::vector<uint8_t>().swap(vector.bytes);
                       }
                   },
                   buffer.data);
    }

    for (auto &image : gltf.images)
    {
        std::visit(fastgltf::visitor{
                       [](auto &) {},
                       [&](fastgltf::sources::Vector &vector)
                       {
                           std::vector<uint8_t>().swap(vector.bytes);
                       }
                   },
                   image.data);
    }
}

void initialize_material_descriptor_pool(VulkanEngine *engine,
                                         LoadedGLTF &file,
                                         const fastgltf::Asset &gltf,
                                         std::string_view file_path)
{
    const uint32_t material_set_capacity = std::max<uint32_t>(1u, static_cast<uint32_t>(gltf.materials.size()));

    Logger::info("[GLTF] loadGltf: materials={} meshes={} images={} samplers={} (creating descriptor pool with {} sets)",
                 gltf.materials.size(),
                 gltf.meshes.size(),
                 gltf.images.size(),
                 gltf.samplers.size(),
                 material_set_capacity);

    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> pool_sizes = {
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 5},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
    };

    file.descriptorPool.init(engine->_deviceManager->device(), material_set_capacity, pool_sizes);

    Logger::info("[GLTF] loadGltf: descriptor pool initialized for '{}' (materials={})",
                 file_path,
                 gltf.materials.size());
}

void load_samplers(VulkanEngine *engine, LoadedGLTF &file, const fastgltf::Asset &gltf)
{
    for (const fastgltf::Sampler &sampler : gltf.samplers)
    {
        VkSamplerCreateInfo sampler_info{};
        sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        sampler_info.maxLod = VK_LOD_CLAMP_NONE;
        sampler_info.minLod = 0.0f;
        sampler_info.magFilter = extract_filter(sampler.magFilter.value_or(fastgltf::Filter::Nearest));
        sampler_info.minFilter = extract_filter(sampler.minFilter.value_or(fastgltf::Filter::Nearest));
        sampler_info.mipmapMode = extract_mipmap_mode(sampler.minFilter.value_or(fastgltf::Filter::Nearest));
        sampler_info.addressModeU = extract_address_mode(sampler.wrapS);
        sampler_info.addressModeV = extract_address_mode(sampler.wrapT);
        sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sampler_info.unnormalizedCoordinates = VK_FALSE;

        VkSampler new_sampler = VK_NULL_HANDLE;
        vkCreateSampler(engine->_deviceManager->device(), &sampler_info, nullptr, &new_sampler);
        file.samplers.push_back(new_sampler);
    }
}

GLTFMetallic_Roughness::MaterialResources make_default_material_resources(VulkanEngine *engine,
                                                                          const LoadedGLTF &file,
                                                                          size_t data_buffer_offset)
{
    GLTFMetallic_Roughness::MaterialResources material_resources;
    material_resources.colorImage = engine->_whiteImage;
    material_resources.colorSampler = engine->_samplerManager->defaultLinear();
    material_resources.metalRoughImage = engine->_whiteImage;
    material_resources.metalRoughSampler = engine->_samplerManager->defaultLinear();
    material_resources.normalImage = engine->_flatNormalImage;
    material_resources.normalSampler = engine->_samplerManager->defaultLinear();
    material_resources.occlusionImage = engine->_whiteImage;
    material_resources.occlusionSampler = engine->_samplerManager->defaultLinear();
    material_resources.emissiveImage = engine->_blackImage;
    material_resources.emissiveSampler = engine->_samplerManager->defaultLinear();
    material_resources.dataBuffer = file.materialDataBuffer.buffer;
    material_resources.dataBufferOffset = data_buffer_offset;
    return material_resources;
}

bool load_materials(VulkanEngine *engine,
                    LoadedGLTF &file,
                    const fastgltf::Asset &gltf,
                    const std::filesystem::path &base_dir,
                    const ProgressReporter &progress,
                    std::vector<std::shared_ptr<GLTFMaterial> > &materials)
{
    const size_t material_count = std::max<size_t>(1, gltf.materials.size());
    file.materialDataBuffer = engine->_resourceManager->create_buffer(
        sizeof(GLTFMetallic_Roughness::MaterialConstants) * material_count,
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_CPU_TO_GPU);

    int data_index = 0;
    auto *scene_material_constants =
        static_cast<GLTFMetallic_Roughness::MaterialConstants *>(file.materialDataBuffer.info.pMappedData);

    for (const fastgltf::Material &mat : gltf.materials)
    {
        if (progress.cancelled())
        {
            return false;
        }

        std::shared_ptr<GLTFMaterial> new_mat = std::make_shared<GLTFMaterial>();
        materials.push_back(new_mat);
        file.materials[mat.name.c_str()] = new_mat;
        new_mat->constants_index = static_cast<uint32_t>(data_index);

        GLTFMetallic_Roughness::MaterialConstants constants{};
        constants.colorFactors.x = mat.pbrData.baseColorFactor[0];
        constants.colorFactors.y = mat.pbrData.baseColorFactor[1];
        constants.colorFactors.z = mat.pbrData.baseColorFactor[2];
        constants.colorFactors.w = mat.pbrData.baseColorFactor[3];
        constants.metal_rough_factors.x = mat.pbrData.metallicFactor;
        constants.metal_rough_factors.y = mat.pbrData.roughnessFactor;
        constants.extra[0].x = 1.0f;

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

        constants.extra[1].x = mat.emissiveFactor[0];
        constants.extra[1].y = mat.emissiveFactor[1];
        constants.extra[1].z = mat.emissiveFactor[2];
        if (mat.emissiveTexture.has_value() &&
            constants.extra[1].x == 0.0f &&
            constants.extra[1].y == 0.0f &&
            constants.extra[1].z == 0.0f)
        {
            constants.extra[1].x = 1.0f;
            constants.extra[1].y = 1.0f;
            constants.extra[1].z = 1.0f;
        }

        if (mat.alphaMode == fastgltf::AlphaMode::Mask)
        {
            constants.extra[2].x = static_cast<float>(mat.alphaCutoff);
        }

        scene_material_constants[data_index] = constants;

        MaterialPass pass_type = MaterialPass::MainColor;
        if (mat.alphaMode == fastgltf::AlphaMode::Blend)
        {
            pass_type = MaterialPass::Transparent;
        }

        GLTFMetallic_Roughness::MaterialResources material_resources = make_default_material_resources(
            engine,
            file,
            data_index * sizeof(GLTFMetallic_Roughness::MaterialConstants));

        TextureCache *cache = engine->_context->textures;
        TextureCache::TextureHandle color_handle = TextureCache::InvalidHandle;
        TextureCache::TextureHandle metal_rough_handle = TextureCache::InvalidHandle;
        TextureCache::TextureHandle normal_handle = TextureCache::InvalidHandle;
        TextureCache::TextureHandle occlusion_handle = TextureCache::InvalidHandle;
        TextureCache::TextureHandle emissive_handle = TextureCache::InvalidHandle;

        if (cache && mat.pbrData.baseColorTexture.has_value())
        {
            const auto &texture = gltf.textures[mat.pbrData.baseColorTexture->textureIndex];
            const size_t image_index = texture.imageIndex.value();
            const VkSampler sampler = texture.samplerIndex.has_value()
                                          ? file.samplers[texture.samplerIndex.value()]
                                          : engine->_samplerManager->defaultLinear();

            auto key = build_texture_key(gltf, gltf.images[image_index], base_dir, true);
            if (key.hash != 0)
            {
                color_handle = cache->request(key, sampler);
                material_resources.colorSampler = sampler;
            }
        }

        if (cache && mat.pbrData.metallicRoughnessTexture.has_value())
        {
            const auto &texture = gltf.textures[mat.pbrData.metallicRoughnessTexture->textureIndex];
            const size_t image_index = texture.imageIndex.value();
            const VkSampler sampler = texture.samplerIndex.has_value()
                                          ? file.samplers[texture.samplerIndex.value()]
                                          : engine->_samplerManager->defaultLinear();

            auto key = build_texture_key(gltf, gltf.images[image_index], base_dir, false);
            if (key.hash != 0)
            {
                metal_rough_handle = cache->request(key, sampler);
                material_resources.metalRoughSampler = sampler;
            }
        }

        if (cache && mat.occlusionTexture.has_value())
        {
            const auto &texture = gltf.textures[mat.occlusionTexture->textureIndex];
            const size_t image_index = texture.imageIndex.value();
            const VkSampler sampler = texture.samplerIndex.has_value()
                                          ? file.samplers[texture.samplerIndex.value()]
                                          : engine->_samplerManager->defaultLinear();

            auto key = build_texture_key(gltf, gltf.images[image_index], base_dir, false);
            key.channels = TextureCache::TextureKey::ChannelsHint::R;
            if (key.hash != 0)
            {
                occlusion_handle = cache->request(key, sampler);
                material_resources.occlusionSampler = sampler;
            }
        }

        if (cache && mat.emissiveTexture.has_value())
        {
            const auto &texture = gltf.textures[mat.emissiveTexture->textureIndex];
            const size_t image_index = texture.imageIndex.value();
            const VkSampler sampler = texture.samplerIndex.has_value()
                                          ? file.samplers[texture.samplerIndex.value()]
                                          : engine->_samplerManager->defaultLinear();

            auto key = build_texture_key(gltf, gltf.images[image_index], base_dir, true);
            if (key.hash != 0)
            {
                emissive_handle = cache->request(key, sampler);
                material_resources.emissiveSampler = sampler;
            }
        }

        if (cache && mat.normalTexture.has_value())
        {
            const auto &texture = gltf.textures[mat.normalTexture->textureIndex];
            const size_t image_index = texture.imageIndex.value();
            const VkSampler sampler = texture.samplerIndex.has_value()
                                          ? file.samplers[texture.samplerIndex.value()]
                                          : engine->_samplerManager->defaultLinear();

            auto key = build_texture_key(gltf, gltf.images[image_index], base_dir, false);
            key.channels = TextureCache::TextureKey::ChannelsHint::RG;
            if (key.hash != 0)
            {
                normal_handle = cache->request(key, sampler);
                material_resources.normalSampler = sampler;
            }

            scene_material_constants[data_index].extra[0].x = mat.normalTexture->scale;
        }

        new_mat->data = engine->metalRoughMaterial.write_material(
            engine->_deviceManager->device(),
            pass_type,
            material_resources,
            file.descriptorPool);

        if (cache)
        {
            if (color_handle != TextureCache::InvalidHandle)
            {
                cache->watchBinding(color_handle,
                                    new_mat->data.materialSet,
                                    1u,
                                    material_resources.colorSampler,
                                    engine->_whiteImage.imageView);
            }
            if (metal_rough_handle != TextureCache::InvalidHandle)
            {
                cache->watchBinding(metal_rough_handle,
                                    new_mat->data.materialSet,
                                    2u,
                                    material_resources.metalRoughSampler,
                                    engine->_whiteImage.imageView);
            }
            if (normal_handle != TextureCache::InvalidHandle)
            {
                cache->watchBinding(normal_handle,
                                    new_mat->data.materialSet,
                                    3u,
                                    material_resources.normalSampler,
                                    engine->_flatNormalImage.imageView);
            }
            if (occlusion_handle != TextureCache::InvalidHandle)
            {
                cache->watchBinding(occlusion_handle,
                                    new_mat->data.materialSet,
                                    4u,
                                    material_resources.occlusionSampler,
                                    engine->_whiteImage.imageView);
            }
            if (emissive_handle != TextureCache::InvalidHandle)
            {
                cache->watchBinding(emissive_handle,
                                    new_mat->data.materialSet,
                                    5u,
                                    material_resources.emissiveSampler,
                                    engine->_blackImage.imageView);
            }
        }

        ++data_index;
    }

    if (materials.empty())
    {
        std::shared_ptr<GLTFMaterial> default_material = std::make_shared<GLTFMaterial>();
        materials.push_back(default_material);
        file.materials["__default__"] = default_material;
        default_material->constants_index = 0u;

        GLTFMetallic_Roughness::MaterialConstants constants{};
        constants.colorFactors = glm::vec4(1.0f);
        constants.metal_rough_factors = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
        constants.extra[0].x = 1.0f;
        scene_material_constants[0] = constants;

        GLTFMetallic_Roughness::MaterialResources material_resources =
            make_default_material_resources(engine, file, 0);

        default_material->data = engine->metalRoughMaterial.write_material(
            engine->_deviceManager->device(),
            MaterialPass::MainColor,
            material_resources,
            file.descriptorPool);
    }

    if (!gltf.meshes.empty())
    {
        progress.report(0.25f);
    }

    const VkDeviceSize total_size = sizeof(GLTFMetallic_Roughness::MaterialConstants) * material_count;
    vmaFlushAllocation(engine->_deviceManager->allocator(), file.materialDataBuffer.allocation, 0, total_size);
    return true;
}

bool load_meshes(VulkanEngine *engine,
                 LoadedGLTF &file,
                 const fastgltf::Asset &gltf,
                 const ProgressReporter &progress,
                 const std::vector<std::shared_ptr<GLTFMaterial> > &materials,
                 std::vector<std::shared_ptr<MeshAsset> > &meshes)
{
    std::vector<uint32_t> indices;
    std::vector<Vertex> vertices;

    for (size_t mesh_index = 0; mesh_index < gltf.meshes.size(); ++mesh_index)
    {
        if (progress.cancelled())
        {
            return false;
        }

        const fastgltf::Mesh &mesh = gltf.meshes[mesh_index];
        std::shared_ptr<MeshAsset> new_mesh = std::make_shared<MeshAsset>();
        meshes.push_back(new_mesh);
        file.meshes[mesh.name.c_str()] = new_mesh;
        new_mesh->name = mesh.name;

        indices.clear();
        vertices.clear();

        for (auto &&primitive : mesh.primitives)
        {
            GeoSurface new_surface;
            new_surface.startIndex = static_cast<uint32_t>(indices.size());
            new_surface.count = static_cast<uint32_t>(gltf.accessors[primitive.indicesAccessor.value()].count);

            const size_t initial_vertex = vertices.size();

            {
                const fastgltf::Accessor &index_accessor = gltf.accessors[primitive.indicesAccessor.value()];
                indices.reserve(indices.size() + index_accessor.count);

                fastgltf::iterateAccessor<std::uint32_t>(
                    gltf,
                    index_accessor,
                    [&](std::uint32_t idx)
                    {
                        indices.push_back(idx + initial_vertex);
                    });
            }

            {
                const fastgltf::Accessor &position_accessor = gltf.accessors[primitive.findAttribute("POSITION")->second];
                vertices.resize(vertices.size() + position_accessor.count);

                fastgltf::iterateAccessorWithIndex<glm::vec3>(
                    gltf,
                    position_accessor,
                    [&](glm::vec3 value, size_t index)
                    {
                        Vertex vertex{};
                        vertex.position = value;
                        vertex.normal = {1, 0, 0};
                        vertex.color = glm::vec4{1.f};
                        vertex.uv_x = 0;
                        vertex.uv_y = 0;
                        vertex.tangent = glm::vec4(1, 0, 0, 1);
                        vertices[initial_vertex + index] = vertex;
                    });
            }

            auto normals = primitive.findAttribute("NORMAL");
            if (normals != primitive.attributes.end())
            {
                fastgltf::iterateAccessorWithIndex<glm::vec3>(
                    gltf,
                    gltf.accessors[normals->second],
                    [&](glm::vec3 value, size_t index)
                    {
                        vertices[initial_vertex + index].normal = value;
                    });
            }

            auto uv = primitive.findAttribute("TEXCOORD_0");
            if (uv != primitive.attributes.end())
            {
                fastgltf::iterateAccessorWithIndex<glm::vec2>(
                    gltf,
                    gltf.accessors[uv->second],
                    [&](glm::vec2 value, size_t index)
                    {
                        vertices[initial_vertex + index].uv_x = value.x;
                        vertices[initial_vertex + index].uv_y = value.y;
                    });
            }

            auto colors = primitive.findAttribute("COLOR_0");
            if (colors != primitive.attributes.end())
            {
                fastgltf::iterateAccessorWithIndex<glm::vec4>(
                    gltf,
                    gltf.accessors[colors->second],
                    [&](glm::vec4 value, size_t index)
                    {
                        vertices[initial_vertex + index].color = value;
                    });
            }

            auto tangents = primitive.findAttribute("TANGENT");
            bool has_tangents = tangents != primitive.attributes.end();
            if (has_tangents)
            {
                fastgltf::iterateAccessorWithIndex<glm::vec4>(
                    gltf,
                    gltf.accessors[tangents->second],
                    [&](glm::vec4 value, size_t index)
                    {
                        vertices[initial_vertex + index].tangent = value;
                    });
            }

            bool material_has_normal_map = false;
            if (primitive.materialIndex.has_value())
            {
                const size_t material_index = primitive.materialIndex.value();
                if (material_index < gltf.materials.size())
                {
                    material_has_normal_map = gltf.materials[material_index].normalTexture.has_value();
                }
            }
            else if (!gltf.materials.empty())
            {
                material_has_normal_map = gltf.materials[0].normalTexture.has_value();
            }

            if (!has_tangents && material_has_normal_map)
            {
                const size_t primitive_index_start = new_surface.startIndex;
                const size_t primitive_index_count = new_surface.count;
                const size_t primitive_vertex_start = initial_vertex;
                const size_t primitive_vertex_count = vertices.size() - initial_vertex;
                geom::generate_tangents_range(vertices,
                                              indices,
                                              primitive_index_start,
                                              primitive_index_count,
                                              primitive_vertex_start,
                                              primitive_vertex_count);
            }

            new_surface.material = primitive.materialIndex.has_value()
                                       ? materials[primitive.materialIndex.value()]
                                       : materials[0];

            if (new_surface.count > 0)
            {
                const uint32_t first_index = new_surface.startIndex;
                const uint32_t last_index = new_surface.startIndex + new_surface.count;
                const uint32_t base_vertex = indices[first_index];

                glm::vec3 min_position = vertices[base_vertex].position;
                glm::vec3 max_position = vertices[base_vertex].position;
                for (uint32_t i = first_index + 1; i < last_index; ++i)
                {
                    const glm::vec3 &position = vertices[indices[i]].position;
                    min_position = glm::min(min_position, position);
                    max_position = glm::max(max_position, position);
                }

                new_surface.bounds.origin = (max_position + min_position) / 2.f;
                new_surface.bounds.extents = (max_position - min_position) / 2.f;
                new_surface.bounds.sphereRadius = glm::length(new_surface.bounds.extents);
                new_surface.bounds.type = BoundsType::Mesh;
            }
            else
            {
                new_surface.bounds.origin = glm::vec3(0.0f);
                new_surface.bounds.extents = glm::vec3(0.5f);
                new_surface.bounds.sphereRadius = glm::length(new_surface.bounds.extents);
                new_surface.bounds.type = BoundsType::Mesh;
            }

            new_mesh->surfaces.push_back(new_surface);
        }

        {
            std::span<const Vertex> vertex_span(vertices.data(), vertices.size());
            std::span<const uint32_t> index_span(indices.data(), indices.size());
            new_mesh->bvh = build_mesh_bvh(*new_mesh, vertex_span, index_span);
        }

        new_mesh->meshBuffers = engine->_resourceManager->uploadMesh(indices, vertices);

        auto shrink_if_huge = [](auto &vec, size_t elem_size_bytes)
        {
            const size_t cap_bytes = vec.capacity() * elem_size_bytes;
            constexpr size_t kThreshold = 64ull * 1024ull * 1024ull;
            if (cap_bytes > kThreshold)
            {
                using Vec = std::remove_reference_t<decltype(vec)>;
                Vec empty;
                vec.swap(empty);
            }
        };

        shrink_if_huge(indices, sizeof(uint32_t));
        shrink_if_huge(vertices, sizeof(Vertex));

        const float mesh_fraction = static_cast<float>(mesh_index + 1) / static_cast<float>(gltf.meshes.size());
        progress.report(0.2f + mesh_fraction * 0.6f);
    }

    return true;
}

bool build_nodes_and_hierarchy(LoadedGLTF &file,
                               const fastgltf::Asset &gltf,
                               const std::vector<std::shared_ptr<MeshAsset> > &meshes,
                               const ProgressReporter &progress,
                               std::vector<std::shared_ptr<Node> > &nodes)
{
    const std::vector<std::string> stable_node_names = build_stable_node_names(gltf);

    for (size_t node_index = 0; node_index < gltf.nodes.size(); ++node_index)
    {
        if (progress.cancelled())
        {
            return false;
        }

        const fastgltf::Node &node = gltf.nodes[node_index];
        std::shared_ptr<Node> new_node;

        if (node.meshIndex.has_value())
        {
            auto mesh_node = std::make_shared<MeshNode>();
            mesh_node->mesh = meshes[*node.meshIndex];
            mesh_node->scene = &file;
            new_node = mesh_node;
        }
        else
        {
            new_node = std::make_shared<Node>();
        }

        nodes.push_back(new_node);
        file.nodes[stable_node_names[node_index]] = new_node;

        std::visit(
            fastgltf::visitor{
                [&](fastgltf::Node::TransformMatrix matrix)
                {
                    glm::mat4 transform = glm::make_mat4(matrix.data());

                    glm::vec3 translation;
                    glm::quat rotation;
                    glm::vec3 scale;
                    decompose_trs_matrix(transform, translation, rotation, scale);
                    new_node->setTRS(translation, rotation, scale);
                },
                [&](fastgltf::Node::TRS transform)
                {
                    glm::vec3 translation(transform.translation[0], transform.translation[1], transform.translation[2]);
                    glm::quat rotation(transform.rotation[3], transform.rotation[0], transform.rotation[1], transform.rotation[2]);
                    glm::vec3 scale(transform.scale[0], transform.scale[1], transform.scale[2]);

                    new_node->setTRS(translation, rotation, scale);
                }},
            node.transform);

        if (!gltf.nodes.empty())
        {
            const float node_fraction = static_cast<float>(node_index + 1) / static_cast<float>(gltf.nodes.size());
            progress.report(0.8f + node_fraction * 0.1f);
        }
    }

    for (size_t i = 0; i < gltf.nodes.size(); ++i)
    {
        const fastgltf::Node &node = gltf.nodes[i];
        std::shared_ptr<Node> &scene_node = nodes[i];

        for (auto &child : node.children)
        {
            scene_node->children.push_back(nodes[child]);
            nodes[child]->parent = scene_node;
        }
    }

    for (auto &node : nodes)
    {
        if (node->parent.lock() == nullptr)
        {
            file.topNodes.push_back(node);
            node->refreshTransform(glm::mat4{1.f});
        }
    }

    return true;
}

void load_animations(LoadedGLTF &file,
                     const fastgltf::Asset &gltf,
                     const std::vector<std::shared_ptr<Node> > &nodes)
{
    if (gltf.animations.empty())
    {
        return;
    }

    file.animations.reserve(gltf.animations.size());

    for (const auto &animation : gltf.animations)
    {
        LoadedGLTF::Animation dst_animation;
        dst_animation.name = animation.name.c_str();
        dst_animation.duration = 0.0f;
        dst_animation.channels.reserve(animation.channels.size());

        for (const auto &channel_src : animation.channels)
        {
            if (channel_src.nodeIndex >= nodes.size() || channel_src.samplerIndex >= animation.samplers.size())
            {
                continue;
            }

            LoadedGLTF::AnimationChannel channel{};
            channel.node = nodes[channel_src.nodeIndex];

            switch (channel_src.path)
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
                    continue;
            }

            const fastgltf::AnimationSampler &sampler = animation.samplers[channel_src.samplerIndex];
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

            const auto &time_accessor = gltf.accessors[sampler.inputAccessor];
            channel.times.reserve(time_accessor.count);
            float max_time = 0.0f;

            fastgltf::iterateAccessorWithIndex<float>(
                gltf,
                time_accessor,
                [&](float value, size_t)
                {
                    channel.times.push_back(value);
                    if (value > max_time)
                    {
                        max_time = value;
                    }
                });

            const auto &value_accessor = gltf.accessors[sampler.outputAccessor];
            const bool is_cubic = sampler.interpolation == fastgltf::AnimationInterpolation::CubicSpline;

            if (channel.target == LoadedGLTF::AnimationChannel::Target::Rotation)
            {
                channel.vec4Values.reserve(value_accessor.count);
                fastgltf::iterateAccessorWithIndex<glm::vec4>(
                    gltf,
                    value_accessor,
                    [&](glm::vec4 value, size_t index)
                    {
                        if (is_cubic && index % 3 != 1)
                        {
                            return;
                        }

                        channel.vec4Values.push_back(value);
                    });
            }
            else
            {
                channel.vec3Values.reserve(value_accessor.count);
                fastgltf::iterateAccessorWithIndex<glm::vec3>(
                    gltf,
                    value_accessor,
                    [&](glm::vec3 value, size_t index)
                    {
                        if (is_cubic && index % 3 != 1)
                        {
                            return;
                        }

                        channel.vec3Values.push_back(value);
                    });
            }

            if (!channel.times.empty())
            {
                dst_animation.duration = std::max(dst_animation.duration, max_time);
                dst_animation.channels.push_back(std::move(channel));
            }
        }

        if (!dst_animation.channels.empty())
        {
            file.animations.push_back(std::move(dst_animation));
        }
    }
}

void finalize_loaded_gltf(LoadedGLTF &file, fastgltf::Asset &gltf, const ProgressReporter &progress)
{
    progress.report(0.95f);
    release_gltf_payloads(gltf);

    Logger::info("[GLTF] loadGltf done: meshes={} materials={} images={} samplers={} animations={} debugName='{}'",
                 file.meshes.size(),
                 file.materials.size(),
                 file.images.size(),
                 file.samplers.size(),
                 file.animations.size(),
                 file.debugName.empty() ? "<none>" : file.debugName);

    progress.report(1.0f);
}
} // namespace

std::optional<std::shared_ptr<LoadedGLTF> > loadGltf(VulkanEngine *engine,
                                                     std::string_view filePath,
                                                     const GLTFLoadCallbacks *cb)
{
    Logger::info("[GLTF] loadGltf begin: '{}'", filePath);

    auto scene = std::make_shared<LoadedGLTF>();
    scene->creator = engine;
    LoadedGLTF &file = *scene.get();

    const std::filesystem::path path = filePath;
    auto gltf_result = load_gltf_asset(filePath, path.parent_path());
    if (!gltf_result.has_value())
    {
        return {};
    }

    fastgltf::Asset gltf = std::move(*gltf_result);
    ProgressReporter progress{cb};
    initialize_material_descriptor_pool(engine, file, gltf, filePath);
    progress.report(0.1f);
    load_samplers(engine, file, gltf);
    progress.report(0.2f);

    std::vector<std::shared_ptr<MeshAsset> > meshes;
    std::vector<std::shared_ptr<Node> > nodes;
    std::vector<std::shared_ptr<GLTFMaterial> > materials;

    const std::filesystem::path base_dir = path.parent_path();
    if (!load_materials(engine, file, gltf, base_dir, progress, materials))
    {
        return {};
    }
    if (!load_meshes(engine, file, gltf, progress, materials, meshes))
    {
        return {};
    }
    if (!build_nodes_and_hierarchy(file, gltf, meshes, progress, nodes))
    {
        return {};
    }
    load_animations(file, gltf, nodes);
    finalize_loaded_gltf(file, gltf, progress);
    return scene;
}
