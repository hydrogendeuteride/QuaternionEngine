// vulkan_engine.h : Include file for standard system include files,
// or project specific include files.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <span>
#include <array>
#include <functional>
#include <deque>

#include <vulkan/vulkan.h>
#if __has_include(<vulkan/vk_enum_string_helper.h>)
#include <vulkan/vk_enum_string_helper.h>
#elif __has_include(<vk_enum_string_helper.h>)
#include <vk_enum_string_helper.h>
#else
// Fallback for environments without Vulkan-Hpp's enum string helpers.
// This keeps the build working; enum names may not be available.
inline const char *string_VkResult(VkResult) { return "VkResult"; }
inline const char *string_VkFormat(VkFormat) { return "VkFormat"; }
#endif

#include <vk_mem_alloc.h>

#include <fmt/core.h>

#include <glm/mat4x4.hpp>
#include <glm/mat3x4.hpp>
#include <glm/vec4.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Helper utilities for TRS (translation-rotation-scale) transforms using quaternions.
inline glm::mat4 make_trs_matrix(const glm::vec3 &translation,
                                 const glm::quat &rotation,
                                 const glm::vec3 &scale)
{
    glm::mat4 tm = glm::translate(glm::mat4(1.0f), translation);
    glm::mat4 rm = glm::mat4_cast(rotation);
    glm::mat4 sm = glm::scale(glm::mat4(1.0f), scale);
    return tm * rm * sm;
}

inline void decompose_trs_matrix(const glm::mat4 &m,
                                 glm::vec3 &out_translation,
                                 glm::quat &out_rotation,
                                 glm::vec3 &out_scale)
{
    out_translation = glm::vec3(m[3]);

    glm::vec3 col0 = glm::vec3(m[0]);
    glm::vec3 col1 = glm::vec3(m[1]);
    glm::vec3 col2 = glm::vec3(m[2]);

    out_scale = glm::vec3(glm::length(col0), glm::length(col1), glm::length(col2));
    if (out_scale.x != 0.0f) col0 /= out_scale.x;
    if (out_scale.y != 0.0f) col1 /= out_scale.y;
    if (out_scale.z != 0.0f) col2 /= out_scale.z;

    glm::mat3 rot_mat(col0, col1, col2);
    out_rotation = glm::quat_cast(rot_mat);
}


#define VK_CHECK(x)                                                     \
    do {                                                                \
        VkResult err = x;                                               \
        if (err) {                                                      \
            fmt::println("Detected Vulkan error: {}", string_VkResult(err)); \
            abort();                                                    \
        }                                                               \
    } while (0)

struct DeletionQueue
{
    std::deque<std::function<void()> > deletors;

    void push_function(std::function<void()> &&function)
    {
        deletors.push_back(function);
    }

    void flush()
    {
        // reverse iterate the deletion queue to execute all the functions
        for (auto it = deletors.rbegin(); it != deletors.rend(); it++)
        {
            (*it)(); //call functors
        }

        deletors.clear();
    }
};

struct AllocatedImage
{
    VkImage image;
    VkImageView imageView;
    VmaAllocation allocation;
    VkFormat imageFormat;
    VkExtent3D imageExtent;
};

struct AllocatedBuffer {
    VkBuffer buffer;
    VmaAllocation allocation;
    VmaAllocationInfo info;
};

struct GPUPunctualLight {
    glm::vec4 position_radius;
    glm::vec4 color_intensity;
};

static constexpr uint32_t kMaxPunctualLights = 64;

struct GPUSpotLight {
    glm::vec4 position_radius;        // xyz: position (local), w: radius
    glm::vec4 direction_cos_outer;    // xyz: direction (unit), w: cos(outer_angle)
    glm::vec4 color_intensity;        // rgb: color, a: intensity
    glm::vec4 cone;                   // x: cos(inner_angle), yzw: unused
};

static constexpr uint32_t kMaxSpotLights = 32;

struct GPUSceneData {
    glm::mat4 view;
    glm::mat4 proj;
    glm::mat4 viewproj;
    glm::mat4 lightViewProj; // legacy single-shadow; kept for transition
    glm::vec4 ambientColor;
    glm::vec4 sunlightDirection; // w for sun power
    glm::vec4 sunlightColor;

    glm::mat4 lightViewProjCascades[4];
    glm::vec4 cascadeSplitsView;
    // Hybrid ray-query / reflection options (match shaders/input_structures.glsl)
    // rtOptions.x = RT shadows enabled (1/0)
    // rtOptions.y = cascade bitmask (bit i => cascade i assisted)
    // rtOptions.z = shadow mode (0 = clipmap, 1 = hybrid, 2 = RT only)
    // rtOptions.w = reflection mode (SSR/RT)
    glm::uvec4 rtOptions;
    // rtParams.x = N·L threshold for hybrid shadows
    // rtParams.y = shadows enabled flag (1.0 = on, 0.0 = off)
    // rtParams.z = planet receiver clipmap shadow maps enabled flag (RT-only mode)
    // rtParams.w = sun angular radius (radians) for analytic planet shadow penumbra
    glm::vec4  rtParams;

    GPUPunctualLight punctualLights[kMaxPunctualLights];
    GPUSpotLight spotLights[kMaxSpotLights];
    // lightCounts.x = point light count
    // lightCounts.y = spot light count
    // lightCounts.z = planet occluder count (analytic directional sun shadow)
    glm::uvec4 lightCounts;

    // Analytic planet shadow occluders (max 4):
    // xyz = center in render-local space, w = radius in meters.
    glm::vec4 planetOccluders[4];
};

enum class MaterialPass :uint8_t {
    MainColor,
    Transparent,
    Other
};

struct MaterialPipeline {
    VkPipeline pipeline;
    VkPipelineLayout layout;
};

struct MaterialInstance {
    MaterialPipeline* pipeline;
    VkDescriptorSet materialSet;
    MaterialPass passType;
};

struct Vertex {

    glm::vec3 position;
    float uv_x;
    glm::vec3 normal;
    float uv_y;
    glm::vec4 color;
    // Tangent.xyz = tangent direction; Tangent.w = handedness sign for B = sign * cross(N, T)
    glm::vec4 tangent;
};

// holds the resources needed for a mesh
struct GPUMeshBuffers {

    AllocatedBuffer indexBuffer;
    AllocatedBuffer vertexBuffer;
    VkDeviceAddress vertexBufferAddress;
    VkDeviceAddress indexBufferAddress;
    uint32_t vertexCount{0};
    uint32_t indexCount{0};
};

// push constants for our mesh object draws
struct GPUDrawPushConstants {
    glm::mat4 worldMatrix;
    // std140-compatible representation of mat3 (3 x vec4 columns; w unused).
    glm::mat3x4 normalMatrix;
    VkDeviceAddress vertexBuffer;
    uint32_t objectID;
};
static_assert(offsetof(GPUDrawPushConstants, worldMatrix) == 0);
static_assert(offsetof(GPUDrawPushConstants, normalMatrix) == 64);
static_assert(offsetof(GPUDrawPushConstants, vertexBuffer) == 112);
static_assert(offsetof(GPUDrawPushConstants, objectID) == 120);
static_assert(sizeof(GPUDrawPushConstants) == 128);

struct DrawContext;

// base class for a renderable dynamic object
class IRenderable {

    virtual void Draw(const glm::mat4& topMatrix, DrawContext& ctx) = 0;
};

// implementation of a drawable scene node.
// the scene node can hold children and will also keep a transform to propagate
// to them
struct Node : public IRenderable {

    // parent pointer must be a weak pointer to avoid circular dependencies
    std::weak_ptr<Node> parent;
    std::vector<std::shared_ptr<Node>> children;

    glm::mat4 localTransform;
    glm::mat4 worldTransform;

    glm::vec3 translation{0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f, 1.0f, 1.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    bool hasTRS{false};

    void updateLocalFromTRS()
    {
        localTransform = make_trs_matrix(translation, rotation, scale);
    }

    void setTRS(const glm::vec3 &t, const glm::quat &r, const glm::vec3 &s)
    {
        translation = t;
        rotation = r;
        scale = s;
        hasTRS = true;
        updateLocalFromTRS();
    }

    void refreshTransform(const glm::mat4& parentMatrix)
    {
        worldTransform = parentMatrix * localTransform;
        for (auto c : children) {
            c->refreshTransform(worldTransform);
        }
    }

    virtual void Draw(const glm::mat4& topMatrix, DrawContext& ctx)
    {
        // draw children
        for (auto& c : children) {
            c->Draw(topMatrix, ctx);
        }
    }
};
