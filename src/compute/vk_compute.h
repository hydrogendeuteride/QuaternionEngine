# pragma once

#include <core/types.h>
#include <core/descriptor/descriptors.h>
#include <functional>
#include <unordered_map>
#include <glm/glm.hpp>

// Common compute data structures used across passes
struct ComputePushConstants
{
    glm::vec4 data1;
    glm::vec4 data2;
    glm::vec4 data3;
    glm::vec4 data4;
};

struct ComputeEffect
{
    const char *name;
    ComputePushConstants data;
};

class EngineContext;

struct ComputeBinding
{
    uint32_t binding;
    VkDescriptorType type;

    union
    {
        struct
        {
            VkBuffer buffer;
            VkDeviceSize offset;
            VkDeviceSize size;
        } buffer;

        struct
        {
            VkImage image;
            VkImageLayout imageLayout;
            VkSampler sampler;
        } image;

        struct
        {
            VkImageView imageView;
            VkImageLayout layout;
        } storageImage;
    };

    static ComputeBinding uniformBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset = 0);

    static ComputeBinding storageBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size, VkDeviceSize offset = 0);

    static ComputeBinding sampledImage(uint32_t binding, VkImageView imageView, VkSampler sampler,
                                       VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    static ComputeBinding storeImage(uint32_t binding, VkImageView imageView,
                                       VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL);
};

struct ComputePipelineCreateInfo
{
    std::string shaderPath;
    std::vector<VkDescriptorType> descriptorTypes;
    uint32_t pushConstantSize = 0;
    VkShaderStageFlags pushConstantStages = VK_SHADER_STAGE_COMPUTE_BIT;

    std::vector<VkSpecializationMapEntry> specializationEntries;
    std::vector<uint32_t> specializationData;
};

struct ComputeDispatchInfo
{
    uint32_t groupCountX = 1;
    uint32_t groupCountY = 1;
    uint32_t groupCountZ = 1;

    std::vector<ComputeBinding> bindings;

    const void *pushConstants = nullptr;
    uint32_t pushConstantSize = 0;

    std::vector<VkMemoryBarrier2> memoryBarriers;
    std::vector<VkBufferMemoryBarrier2> bufferBarriers;
    std::vector<VkImageMemoryBarrier2> imageBarriers;
};

class ComputePipeline
{
public:
    ComputePipeline() = default;

    ~ComputePipeline();

    ComputePipeline(ComputePipeline &&other) noexcept;

    ComputePipeline &operator=(ComputePipeline &&other) noexcept;

    ComputePipeline(const ComputePipeline &) = delete;

    ComputePipeline &operator=(const ComputePipeline &) = delete;

    bool isValid() const { return pipeline != VK_NULL_HANDLE; }

    VkPipeline getPipeline() const { return pipeline; }
    VkPipelineLayout getLayout() const { return layout; }

private:
    friend class ComputeManager;

    VkDevice device = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout descriptorLayout = VK_NULL_HANDLE;

    void cleanup();
};

// Small compute manager for one-off pipelines and persistent instances.
// It owns a dedicated descriptor allocator and provides helpers to build
// pipelines, set bindings, and dispatch work (immediate or on a provided cmd).
class ComputeManager
{
public:
    ComputeManager() = default;

    ~ComputeManager();

    void init(EngineContext *context);

    void cleanup();

    bool registerPipeline(const std::string &name, const ComputePipelineCreateInfo &createInfo);

    bool createComputePipeline(const std::string &name, const ComputePipelineCreateInfo &createInfo) {
        return registerPipeline(name, createInfo);
    }
    void unregisterPipeline(const std::string &name);
    bool hasPipeline(const std::string &name) const;

    void dispatch(VkCommandBuffer cmd, const std::string &pipelineName, const ComputeDispatchInfo &dispatchInfo);

    void dispatchImmediate(const std::string &pipelineName, const ComputeDispatchInfo &dispatchInfo);

    static uint32_t calculateGroupCount(uint32_t workItems, uint32_t localSize);
    static ComputeDispatchInfo createDispatch2D(uint32_t width, uint32_t height, uint32_t localSizeX = 16,
                                                uint32_t localSizeY = 16);
    static ComputeDispatchInfo createDispatch3D(uint32_t width, uint32_t height, uint32_t depth,
                                                uint32_t localSizeX = 8, uint32_t localSizeY = 8,
                                                uint32_t localSizeZ = 8);

    void clearImage(VkCommandBuffer cmd, VkImageView imageView, const glm::vec4 &clearColor = {0, 0, 0, 0});
    void copyBuffer(VkCommandBuffer cmd, VkBuffer src, VkBuffer dst, VkDeviceSize size, VkDeviceSize srcOffset = 0,
                    VkDeviceSize dstOffset = 0);

    struct ComputeInstance
    {
        std::string pipelineName;
        VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
        std::vector<ComputeBinding> bindings;
        std::vector<AllocatedImage> ownedImages;
        std::vector<AllocatedBuffer> ownedBuffers;
    };

    bool createInstance(const std::string &instanceName, const std::string &pipelineName);
    void destroyInstance(const std::string &instanceName);
    bool hasInstance(const std::string &instanceName) const { return instances.find(instanceName) != instances.end(); }

    bool setInstanceBinding(const std::string &instanceName, const ComputeBinding &binding);

    bool setInstanceStorageImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                 VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL);
    bool setInstanceSampledImage(const std::string &instanceName, uint32_t binding, VkImageView view, VkSampler sampler,
                                 VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    bool setInstanceBuffer(const std::string &instanceName, uint32_t binding, VkBuffer buffer, VkDeviceSize size,
                           VkDescriptorType type, VkDeviceSize offset = 0);

    AllocatedImage createAndBindStorageImage(const std::string &instanceName, uint32_t binding, VkExtent3D extent,
                                             VkFormat format,
                                             VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL,
                                             VkImageUsageFlags usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    AllocatedBuffer createAndBindStorageBuffer(const std::string &instanceName, uint32_t binding, VkDeviceSize size,
                                               VkBufferUsageFlags usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                               VmaMemoryUsage memUsage = VMA_MEMORY_USAGE_GPU_ONLY);

    bool updateInstanceDescriptorSet(const std::string &instanceName);
    void dispatchInstance(VkCommandBuffer cmd, const std::string &instanceName, const ComputeDispatchInfo &dispatchInfo);

private:
    EngineContext *context = nullptr;
    std::unordered_map<std::string, ComputePipeline> pipelines;
    DescriptorAllocatorGrowable descriptorAllocator;

    std::unordered_map<std::string, ComputeInstance> instances;

    bool createPipeline(const std::string &name, const ComputePipelineCreateInfo &createInfo);
    VkDescriptorSet allocateDescriptorSet(const ComputePipeline &pipeline, const std::vector<ComputeBinding> &bindings);
    void updateDescriptorSet(VkDescriptorSet descriptorSet, const std::vector<ComputeBinding> &bindings);
    void insertBarriers(VkCommandBuffer cmd, const ComputeDispatchInfo &dispatchInfo);
};
