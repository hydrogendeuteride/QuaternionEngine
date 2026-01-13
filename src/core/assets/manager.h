#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <filesystem>
#include <vector>
#include <utility>
#include <mutex>

#include <scene/vk_loader.h>
#include <core/types.h>
#include <core/assets/texture_cache.h>

#include "render/materials.h"
#include "locator.h"

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
        std::string occlusionPath;
        std::string emissivePath;

        bool albedoSRGB = true;
        bool metalRoughSRGB = false;
        bool normalSRGB = false; // normal maps are typically non-sRGB
        bool occlusionSRGB = false;
        bool emissiveSRGB = true;

        GLTFMetallic_Roughness::MaterialConstants constants{};

        MaterialPass pass = MaterialPass::MainColor;
    };

    struct MeshGeometryDesc
    {
        enum class Type { Provided, Cube, Sphere, Plane, Capsule };

        Type type = Type::Provided;
        std::span<Vertex> vertices{};
        std::span<uint32_t> indices{};
        int sectors = 24;
        int stacks = 24;
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
        // Optional override for collision / picking bounds type for this mesh.
        // When unset, a reasonable default is chosen based on geometry.type.
        std::optional<BoundsType> boundsType;
    };

    void init(VulkanEngine *engine);

    void cleanup();

    std::string shaderPath(std::string_view name) const;

    std::string modelPath(std::string_view name) const;

    std::string assetPath(std::string_view name) const;

    std::optional<std::shared_ptr<LoadedGLTF> > loadGLTF(std::string_view nameOrPath);
    std::optional<std::shared_ptr<LoadedGLTF> > loadGLTF(std::string_view nameOrPath,
                                                         const GLTFLoadCallbacks *cb);

    // Queue texture loads for a glTF file ahead of time. This parses the glTF,
    // builds TextureCache keys for referenced images (both external URIs and
    // embedded images in buffers), and issues TextureCache::request() calls.
    // Actual uploads happen via the normal per-frame pump.
    // Returns number of textures scheduled.
    size_t prefetchGLTFTextures(std::string_view nameOrPath);
    struct GLTFTexturePrefetchResult
    {
        size_t scheduled = 0;
        std::vector<TextureCache::TextureHandle> handles;
    };
    GLTFTexturePrefetchResult prefetchGLTFTexturesWithHandles(std::string_view nameOrPath);

    std::shared_ptr<MeshAsset> createMesh(const MeshCreateInfo &info);

    std::shared_ptr<MeshAsset> getPrimitive(std::string_view name) const;

    std::shared_ptr<MeshAsset> createMesh(const std::string &name,
                                          std::span<Vertex> vertices,
                                          std::span<uint32_t> indices,
                                          std::shared_ptr<GLTFMaterial> material = {},
                                          bool build_bvh = true);

    std::shared_ptr<MeshAsset> getMesh(const std::string &name) const;

    bool removeMesh(const std::string &name);
    bool removeMeshDeferred(const std::string &name, DeletionQueue &dq);

    // Convenience: create a PBR material from constants using engine default textures
    std::shared_ptr<GLTFMaterial> createMaterialFromConstants(const std::string &name,
                                                              const GLTFMetallic_Roughness::MaterialConstants &constants,
                                                              MaterialPass pass = MaterialPass::MainColor);

    // Access engine-provided fallback textures for procedural systems.
    VkImageView fallback_checkerboard_view() const;
    VkImageView fallback_white_view() const;
    VkImageView fallback_flat_normal_view() const;
    VkImageView fallback_black_view() const;

    const AssetPaths &paths() const { return _locator.paths(); }
    void setPaths(const AssetPaths &p) { _locator.setPaths(p); }

private:
    VulkanEngine *_engine = nullptr;
    AssetLocator _locator;

    std::unordered_map<std::string, std::weak_ptr<LoadedGLTF> > _gltfCacheByPath;
    mutable std::mutex _gltfMutex;
    std::unordered_map<std::string, std::shared_ptr<MeshAsset> > _meshCache;
    std::unordered_map<std::string, AllocatedBuffer> _meshMaterialBuffers;
    std::unordered_map<std::string, std::vector<AllocatedImage> > _meshOwnedImages;

    AllocatedBuffer createMaterialBufferWithConstants(const GLTFMetallic_Roughness::MaterialConstants &constants) const;

    std::shared_ptr<GLTFMaterial> createMaterial(MaterialPass pass,
                                                 const GLTFMetallic_Roughness::MaterialResources &res) const;

    std::pair<AllocatedImage, bool> loadImageFromAsset(std::string_view path, bool srgb) const;
};
