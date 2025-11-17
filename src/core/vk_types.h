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
#include <vulkan/vk_enum_string_helper.h>

#include <vk_mem_alloc.h>

#include <fmt/core.h>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>


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
    // Hybrid ray-query options (match shaders/input_structures.glsl)
    glm::uvec4 rtOptions; // x: enabled (1/0), y: cascade mask, z,w: reserved
    glm::vec4  rtParams;  // x: N·L threshold, yzw: reserved
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
    VkDeviceAddress vertexBuffer;
};

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
        glm::mat4 tm = glm::translate(glm::mat4(1.0f), translation);
        glm::mat4 rm = glm::mat4_cast(rotation);
        glm::mat4 sm = glm::scale(glm::mat4(1.0f), scale);
        localTransform = tm * rm * sm;
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
