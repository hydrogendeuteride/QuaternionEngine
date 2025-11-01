#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <filesystem>
#include <vector>
#include <utility>

#include <scene/vk_loader.h>
#include <core/vk_types.h>

#include "vk_materials.h"
#include "asset_locator.h"

class VulkanEngine;
class MeshAsset;

class AssetManager
{
public:
    struct MaterialOptions
    {
        std::string albedoPath;
        std::string metalRoughPath;
        // Optional tangent-space normal map for PBR (placeholder; not wired yet)
        // When enabled later, this will be sampled in shaders and requires tangents.
        std::string normalPath;

        bool albedoSRGB = true;
        bool metalRoughSRGB = false;
        bool normalSRGB = false; // normal maps are typically non-sRGB

        GLTFMetallic_Roughness::MaterialConstants constants{};

        MaterialPass pass = MaterialPass::MainColor;
    };

    struct MeshGeometryDesc
    {
        enum class Type { Provided, Cube, Sphere };

        Type type = Type::Provided;
        std::span<Vertex> vertices{};
        std::span<uint32_t> indices{};
        int sectors = 16;
        int stacks = 16;
    };

    struct MeshMaterialDesc
    {
        enum class Kind { Default, Textured };

        Kind kind = Kind::Default;
        MaterialOptions options{};
    };

    struct MeshCreateInfo
    {
        std::string name;
        MeshGeometryDesc geometry;
        MeshMaterialDesc material;
    };

    void init(VulkanEngine *engine);

    void cleanup();

    std::string shaderPath(std::string_view name) const;

    std::string modelPath(std::string_view name) const;

    std::string assetPath(std::string_view name) const;

    std::optional<std::shared_ptr<LoadedGLTF> > loadGLTF(std::string_view nameOrPath);

    std::shared_ptr<MeshAsset> createMesh(const MeshCreateInfo &info);

    std::shared_ptr<MeshAsset> getPrimitive(std::string_view name) const;

    std::shared_ptr<MeshAsset> createMesh(const std::string &name,
                                          std::span<Vertex> vertices,
                                          std::span<uint32_t> indices,
                                          std::shared_ptr<GLTFMaterial> material = {});

    std::shared_ptr<MeshAsset> getMesh(const std::string &name) const;

    bool removeMesh(const std::string &name);

    const AssetPaths &paths() const { return _locator.paths(); }
    void setPaths(const AssetPaths &p) { _locator.setPaths(p); }

private:
    VulkanEngine *_engine = nullptr;
    AssetLocator _locator;

    std::unordered_map<std::string, std::weak_ptr<LoadedGLTF> > _gltfCacheByPath;
    std::unordered_map<std::string, std::shared_ptr<MeshAsset> > _meshCache;
    std::unordered_map<std::string, AllocatedBuffer> _meshMaterialBuffers;
    std::unordered_map<std::string, std::vector<AllocatedImage> > _meshOwnedImages;

    AllocatedBuffer createMaterialBufferWithConstants(const GLTFMetallic_Roughness::MaterialConstants &constants) const;

    std::shared_ptr<GLTFMaterial> createMaterial(MaterialPass pass,
                                                 const GLTFMetallic_Roughness::MaterialResources &res) const;

    std::pair<AllocatedImage, bool> loadImageFromAsset(std::string_view path, bool srgb) const;
};
