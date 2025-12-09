#pragma once

#include <core/types.h>
#include <render/pipelines.h>
#include <compute/vk_compute.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <filesystem>
#include <thread>

class EngineContext;

struct GraphicsPipelineCreateInfo
{
    std::string vertexShaderPath;
    std::string fragmentShaderPath;

    std::vector<VkDescriptorSetLayout> setLayouts;
    std::vector<VkPushConstantRange> pushConstants;

    // This function MUST set things like topology, rasterization, depth/blend state
    // and color/depth attachment formats on the builder.
    std::function<void(PipelineBuilder &)> configure;
};

// Graphics pipeline registry with hot-reload support.
// Stores specs keyed by name, builds on demand, and can rebuild when shader
// timestamps change. Also forwards a minimal Compute API to ComputeManager.
class PipelineManager
{
public:
    PipelineManager() = default;

    ~PipelineManager();

    void init(EngineContext *ctx);

    void cleanup();

    // Register and build a graphics pipeline under a unique name
    bool registerGraphics(const std::string &name, const GraphicsPipelineCreateInfo &info);

    // Convenience alias for registerGraphics to match desired API
    bool createGraphicsPipeline(const std::string &name, const GraphicsPipelineCreateInfo &info)
    {
        return registerGraphics(name, info);
    }

    // Compute wrappers (forward to ComputeManager for a unified API)
    bool createComputePipeline(const std::string &name, const ComputePipelineCreateInfo &info);

    void destroyComputePipeline(const std::string &name);

    bool hasComputePipeline(const std::string &name) const;

    void dispatchCompute(VkCommandBuffer cmd, const std::string &name, const ComputeDispatchInfo &info);

    void dispatchComputeImmediate(const std::string &name, const ComputeDispatchInfo &info);

    // Persistent compute instances (forwarded to ComputeManager)
    bool createComputeInstance(const std::string &instanceName, const std::string &pipelineName);

    void destroyComputeInstance(const std::string &instanceName);

    bool setComputeInstanceStorageImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                        VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL);

    bool setComputeInstanceSampledImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                        VkSampler sampler,
                                        VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    bool setComputeInstanceBuffer(const std::string &instanceName, uint32_t binding, VkBuffer buffer, VkDeviceSize size,
                                  VkDescriptorType type, VkDeviceSize offset = 0);

    AllocatedImage createAndBindComputeStorageImage(const std::string &instanceName, uint32_t binding,
                                                    VkExtent3D extent,
                                                    VkFormat format,
                                                    VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL,
                                                    VkImageUsageFlags usage =
                                                            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);

    AllocatedBuffer createAndBindComputeStorageBuffer(const std::string &instanceName, uint32_t binding,
                                                      VkDeviceSize size,
                                                      VkBufferUsageFlags usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                      VmaMemoryUsage memUsage = VMA_MEMORY_USAGE_GPU_ONLY);

    void dispatchComputeInstance(VkCommandBuffer cmd, const std::string &instanceName, const ComputeDispatchInfo &info);

    // Remove and destroy a graphics pipeline
    void unregisterGraphics(const std::string &name);

    // Get pipeline handles for binding
    bool getGraphics(const std::string &name, VkPipeline &pipeline, VkPipelineLayout &layout) const;

    // Convenience to interop with MaterialInstance
    bool getMaterialPipeline(const std::string &name, MaterialPipeline &out) const;

    // Rebuild pipelines whose shaders changed on disk (enqueue async rebuild jobs)
    void hotReloadChanged();

    // Apply any completed async rebuilds on the main thread.
    void pump_main_thread();

    // Debug helpers (graphics only)
    struct GraphicsPipelineDebugInfo
    {
        std::string name;
        std::string vertexShaderPath;
        std::string fragmentShaderPath;
        bool valid = false;
    };
    void debug_get_graphics(std::vector<GraphicsPipelineDebugInfo>& out) const;

private:
    struct GraphicsPipelineRecord
    {
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;

        GraphicsPipelineCreateInfo spec;

        std::filesystem::file_time_type vertTime{};
        std::filesystem::file_time_type fragTime{};
    };

    EngineContext *_context = nullptr;
    std::unordered_map<std::string, GraphicsPipelineRecord> _graphicsPipelines;

    // --- Async hot-reload state ---
    struct ReloadJob
    {
        std::string name;
        GraphicsPipelineRecord record;
    };

    std::atomic<bool> _running{false};
    std::thread _worker;

    std::mutex _jobs_mutex;
    std::condition_variable _jobs_cv;
    std::deque<ReloadJob> _pending_jobs;
    std::deque<ReloadJob> _completed_jobs;
    std::unordered_set<std::string> _inflight;

    bool buildGraphics(GraphicsPipelineRecord &rec) const;

    void destroyGraphics(GraphicsPipelineRecord &rec);

    void start_worker();

    void stop_worker();

    void worker_loop();
};
