// vulkan_engine.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <core/vk_types.h>

#include "core/vk_descriptors.h"
#include <unordered_map>
#include <filesystem>

class VulkanEngine;

// Basic collision / selection shape associated with a render surface.
// origin/extents are always the local-space AABB center and half-size;
// sphereRadius is a conservative bounding-sphere radius in local space.
// type controls how precise ray tests should be.
enum class BoundsType : uint8_t
{
    None = 0,   // not pickable
    Box,        // oriented box using origin/extents (default)
    Sphere,     // sphere using origin + sphereRadius
    Capsule,    // capsule aligned with local Y (derived from extents)
    Mesh        // full mesh (BVH / ray query using mesh BVH when available)
};

struct Bounds
{
    glm::vec3 origin;
    float sphereRadius;
    glm::vec3 extents;
    BoundsType type = BoundsType::Box;
};

struct GLTFMaterial
{
    MaterialInstance data;
};

struct GeoSurface
{
    uint32_t startIndex;
    uint32_t count;
    Bounds bounds;
    std::shared_ptr<GLTFMaterial> material;
};

struct MeshBVH;

struct MeshAsset
{
    std::string name;


    std::vector<GeoSurface> surfaces;
    GPUMeshBuffers meshBuffers;

    // Optional CPU BVH for precise picking / queries.
    std::shared_ptr<MeshBVH> bvh;
};

struct LoadedGLTF : public IRenderable
{
    // storage for all the data on a given gltf file
    std::unordered_map<std::string, std::shared_ptr<MeshAsset> > meshes;
    std::unordered_map<std::string, std::shared_ptr<Node> > nodes;
    std::unordered_map<std::string, AllocatedImage> images;
    std::unordered_map<std::string, std::shared_ptr<GLTFMaterial> > materials;

    // nodes that dont have a parent, for iterating through the file in tree order
    std::vector<std::shared_ptr<Node> > topNodes;

    std::vector<VkSampler> samplers;

    DescriptorAllocatorGrowable descriptorPool;

    AllocatedBuffer materialDataBuffer;

    VulkanEngine *creator;

    struct AnimationChannel
    {
        enum class Target { Translation, Rotation, Scale };
        enum class Interpolation { Linear, Step };

        Target target = Target::Translation;
        Interpolation interpolation = Interpolation::Linear;
        std::shared_ptr<Node> node;
        std::vector<float> times;
        std::vector<glm::vec3> vec3Values; // translation / scale
        std::vector<glm::vec4> vec4Values; // rotation (x,y,z,w)
    };

    struct Animation
    {
        std::string name;
        float duration = 0.f;
        std::vector<AnimationChannel> channels;
    };

    std::vector<Animation> animations;
    int activeAnimation = -1;
    float animationTime = 0.f;
    bool animationLoop = true;

    // Optional debug name (e.g., key used when loaded into SceneManager)
    std::string debugName;

    // Animation helpers
    void updateAnimation(float dt);
    void refreshAllTransforms();
    std::shared_ptr<Node> getNode(const std::string &name);
    void setActiveAnimation(int index, bool resetTime = true);
    void setActiveAnimation(const std::string &name, bool resetTime = true);

    ~LoadedGLTF()
    {
        const char *name = debugName.empty() ? "<unnamed>" : debugName.c_str();
        fmt::println("[GLTF] ~LoadedGLTF destructor begin for '{}' ({})", name, static_cast<const void *>(this));
        clearAll();
        fmt::println("[GLTF] ~LoadedGLTF destructor end for '{}' ({})", name, static_cast<const void *>(this));
    };

    void clearMeshes(){ clearAll(); };

    virtual void Draw(const glm::mat4 &topMatrix, DrawContext &ctx);

private:
    void clearAll();
};

std::optional<std::shared_ptr<LoadedGLTF> > loadGltf(VulkanEngine *engine, std::string_view filePath);
