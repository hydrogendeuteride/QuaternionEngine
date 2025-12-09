#include <core/pipeline/manager.h>

#include <core/context.h>
#include <core/util/initializers.h>

#include <render/pipelines.h>
#include <core/device/device.h>

#include <filesystem>

PipelineManager::~PipelineManager()
{
    cleanup();
}

void PipelineManager::init(EngineContext *ctx)
{
    _context = ctx;
    start_worker();
}

void PipelineManager::cleanup()
{
    // Stop async worker first so no background thread touches the device/context.
    stop_worker();

    for (auto &kv: _graphicsPipelines)
    {
        destroyGraphics(kv.second);
    }
    _graphicsPipelines.clear();
    _context = nullptr;
}

bool PipelineManager::registerGraphics(const std::string &name, const GraphicsPipelineCreateInfo &info)
{
    if (! _context || !_context->getDevice()) return false;

    auto it = _graphicsPipelines.find(name);
    if (it != _graphicsPipelines.end())
    {
        fmt::println("Graphics pipeline '{}' already exists", name);
        return false;
    }

    GraphicsPipelineRecord rec{};
    rec.spec = info;

    if (!buildGraphics(rec))
    {
        destroyGraphics(rec);
        return false;
    }

    _graphicsPipelines.emplace(name, std::move(rec));
    return true;
}

void PipelineManager::unregisterGraphics(const std::string &name)
{
    auto it = _graphicsPipelines.find(name);
    if (it == _graphicsPipelines.end()) return;
    destroyGraphics(it->second);
    _graphicsPipelines.erase(it);
}

bool PipelineManager::getGraphics(const std::string &name, VkPipeline &pipeline, VkPipelineLayout &layout) const
{
    auto it = _graphicsPipelines.find(name);
    if (it == _graphicsPipelines.end()) return false;
    pipeline = it->second.pipeline;
    layout = it->second.layout;
    return pipeline != VK_NULL_HANDLE && layout != VK_NULL_HANDLE;
}

bool PipelineManager::getMaterialPipeline(const std::string &name, MaterialPipeline &out) const
{
    VkPipeline p{}; VkPipelineLayout l{};
    if (!getGraphics(name, p, l)) return false;
    out.pipeline = p;
    out.layout = l;
    return true;
}

void PipelineManager::hotReloadChanged()
{
    if (!_context || !_context->getDevice()) return;

    // Discover pipelines whose shaders changed and enqueue async rebuild jobs.
    std::vector<ReloadJob> to_enqueue;
    to_enqueue.reserve(_graphicsPipelines.size());

    for (auto &kv: _graphicsPipelines)
    {
        auto &rec = kv.second;
        try
        {
            bool needReload = false;
            if (!rec.spec.vertexShaderPath.empty())
            {
                auto t = std::filesystem::last_write_time(rec.spec.vertexShaderPath);
                if (rec.vertTime != std::filesystem::file_time_type{} && t != rec.vertTime) needReload = true;
            }
            if (!rec.spec.fragmentShaderPath.empty())
            {
                auto t = std::filesystem::last_write_time(rec.spec.fragmentShaderPath);
                if (rec.fragTime != std::filesystem::file_time_type{} && t != rec.fragTime) needReload = true;
            }

            if (needReload)
            {
                GraphicsPipelineRecord fresh = rec;
                // Do not touch existing pipeline here; async worker will build into a fresh record.
                fresh.pipeline = VK_NULL_HANDLE;
                fresh.layout = VK_NULL_HANDLE;

                ReloadJob job{};
                job.name = kv.first;
                job.record = std::move(fresh);
                to_enqueue.push_back(std::move(job));
            }
        }
        catch (const std::exception &)
        {
            // ignore hot-reload errors to avoid spamming
        }
    }

    if (to_enqueue.empty()) return;

    {
        std::lock_guard<std::mutex> lock(_jobs_mutex);
        for (auto &job : to_enqueue)
        {
            // Avoid duplicate enqueues while a previous rebuild for this pipeline is in-flight.
            if (_inflight.find(job.name) != _inflight.end())
            {
                continue;
            }
            _inflight.insert(job.name);
            _pending_jobs.push_back(std::move(job));
        }
    }
    _jobs_cv.notify_all();
}

void PipelineManager::pump_main_thread()
{
    // Move completed jobs to a local queue so we don't hold the mutex while doing Vulkan work.
    std::deque<ReloadJob> completed;
    {
        std::lock_guard<std::mutex> lock(_jobs_mutex);
        if (_completed_jobs.empty()) return;
        completed.swap(_completed_jobs);
    }

    if (!_context || !_context->getDevice()) return;

    std::vector<std::string> finished_names;
    finished_names.reserve(completed.size());

    for (auto &job : completed)
    {
        auto it = _graphicsPipelines.find(job.name);
        if (it == _graphicsPipelines.end())
        {
            // Pipeline was unregistered while the job was in flight; just destroy the newly built pipeline.
            destroyGraphics(job.record);
        }
        else
        {
            // Replace existing pipeline with the freshly built one.
            destroyGraphics(it->second);
            it->second = std::move(job.record);
            fmt::println("Reloaded graphics pipeline '{}' (async)", job.name);
        }
        finished_names.push_back(job.name);
    }

    // Clear in-flight markers after commit so new reloads can be enqueued.
    {
        std::lock_guard<std::mutex> lock(_jobs_mutex);
        for (const auto &name : finished_names)
        {
            _inflight.erase(name);
        }
    }
}

void PipelineManager::debug_get_graphics(std::vector<GraphicsPipelineDebugInfo> &out) const
{
    out.clear();
    out.reserve(_graphicsPipelines.size());
    for (const auto &kv : _graphicsPipelines)
    {
        const auto &rec = kv.second;
        GraphicsPipelineDebugInfo info{};
        info.name = kv.first;
        info.vertexShaderPath = rec.spec.vertexShaderPath;
        info.fragmentShaderPath = rec.spec.fragmentShaderPath;
        info.valid = (rec.pipeline != VK_NULL_HANDLE) && (rec.layout != VK_NULL_HANDLE);
        out.push_back(std::move(info));
    }
}

bool PipelineManager::buildGraphics(GraphicsPipelineRecord &rec) const
{
    VkShaderModule vert = VK_NULL_HANDLE;
    VkShaderModule frag = VK_NULL_HANDLE;

    if (!rec.spec.vertexShaderPath.empty())
    {
        if (!vkutil::load_shader_module(rec.spec.vertexShaderPath.c_str(), _context->getDevice()->device(), &vert))
        {
            fmt::println("Failed to load vertex shader: {}", rec.spec.vertexShaderPath);
            return false;
        }
    }
    if (!rec.spec.fragmentShaderPath.empty())
    {
        if (!vkutil::load_shader_module(rec.spec.fragmentShaderPath.c_str(), _context->getDevice()->device(), &frag))
        {
            if (vert != VK_NULL_HANDLE) vkDestroyShaderModule(_context->getDevice()->device(), vert, nullptr);
            fmt::println("Failed to load fragment shader: {}", rec.spec.fragmentShaderPath);
            return false;
        }
    }

    VkPipelineLayoutCreateInfo layoutInfo = vkinit::pipeline_layout_create_info();
    layoutInfo.setLayoutCount = static_cast<uint32_t>(rec.spec.setLayouts.size());
    layoutInfo.pSetLayouts = rec.spec.setLayouts.empty() ? nullptr : rec.spec.setLayouts.data();
    layoutInfo.pushConstantRangeCount = static_cast<uint32_t>(rec.spec.pushConstants.size());
    layoutInfo.pPushConstantRanges = rec.spec.pushConstants.empty() ? nullptr : rec.spec.pushConstants.data();

    VK_CHECK(vkCreatePipelineLayout(_context->getDevice()->device(), &layoutInfo, nullptr, &rec.layout));

    PipelineBuilder builder;
    if (vert != VK_NULL_HANDLE || frag != VK_NULL_HANDLE)
    {
        builder.set_shaders(vert, frag);
    }

    if (rec.spec.configure) rec.spec.configure(builder);
    builder._pipelineLayout = rec.layout;

    rec.pipeline = builder.build_pipeline(_context->getDevice()->device());

    if (vert != VK_NULL_HANDLE)
        vkDestroyShaderModule(_context->getDevice()->device(), vert, nullptr);
    if (frag != VK_NULL_HANDLE)
        vkDestroyShaderModule(_context->getDevice()->device(), frag, nullptr);

    if (rec.pipeline == VK_NULL_HANDLE)
    {
        vkDestroyPipelineLayout(_context->getDevice()->device(), rec.layout, nullptr);
        rec.layout = VK_NULL_HANDLE;
        return false;
    }

    // Record timestamps for hot reload
    try
    {
        if (!rec.spec.vertexShaderPath.empty())
            rec.vertTime = std::filesystem::last_write_time(rec.spec.vertexShaderPath);
        if (!rec.spec.fragmentShaderPath.empty())
            rec.fragTime = std::filesystem::last_write_time(rec.spec.fragmentShaderPath);
    }
    catch (const std::exception &)
    {
        // ignore timestamp errors
    }

    return true;
}

void PipelineManager::destroyGraphics(GraphicsPipelineRecord &rec)
{
    if (!_context || !_context->getDevice()) return;
    if (rec.pipeline != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(_context->getDevice()->device(), rec.pipeline, nullptr);
        rec.pipeline = VK_NULL_HANDLE;
    }
    if (rec.layout != VK_NULL_HANDLE)
    {
        vkDestroyPipelineLayout(_context->getDevice()->device(), rec.layout, nullptr);
        rec.layout = VK_NULL_HANDLE;
    }
}

void PipelineManager::start_worker()
{
    bool expected = false;
    if (!_running.compare_exchange_strong(expected, true))
    {
        // Already running.
        return;
    }

    _worker = std::thread(&PipelineManager::worker_loop, this);
}

void PipelineManager::stop_worker()
{
    bool expected = true;
    if (!_running.compare_exchange_strong(expected, false))
    {
        // Was not running.
        return;
    }

    {
        std::lock_guard<std::mutex> lock(_jobs_mutex);
        _pending_jobs.clear();
        _completed_jobs.clear();
        _inflight.clear();
    }
    _jobs_cv.notify_all();

    if (_worker.joinable())
    {
        _worker.join();
    }
}

void PipelineManager::worker_loop()
{
    while (true)
    {
        ReloadJob job;
        {
            std::unique_lock<std::mutex> lock(_jobs_mutex);
            _jobs_cv.wait(lock, [this]() {
                return !_running.load(std::memory_order_acquire) || !_pending_jobs.empty();
            });

            if (!_running.load(std::memory_order_acquire) && _pending_jobs.empty())
            {
                return;
            }

            if (_pending_jobs.empty())
            {
                continue;
            }

            job = std::move(_pending_jobs.front());
            _pending_jobs.pop_front();
        }

        if (!_context || !_context->getDevice())
        {
            // Context/device went away; drop job and exit.
            std::lock_guard<std::mutex> lock(_jobs_mutex);
            _inflight.erase(job.name);
            return;
        }

        GraphicsPipelineRecord rec = job.record;
        bool ok = buildGraphics(rec);

        {
            std::lock_guard<std::mutex> lock(_jobs_mutex);
            if (ok)
            {
                ReloadJob completed;
                completed.name = job.name;
                completed.record = std::move(rec);
                _completed_jobs.push_back(std::move(completed));
            }
            else
            {
                // Allow future hotReloadChanged calls to enqueue another attempt.
                _inflight.erase(job.name);
            }
        }
    }
}

// --- Compute forwarding API ---
bool PipelineManager::createComputePipeline(const std::string &name, const ComputePipelineCreateInfo &info)
{
    if (!_context || !_context->compute) return false;
    return _context->compute->registerPipeline(name, info);
}

void PipelineManager::destroyComputePipeline(const std::string &name)
{
    if (!_context || !_context->compute) return;
    _context->compute->unregisterPipeline(name);
}

bool PipelineManager::hasComputePipeline(const std::string &name) const
{
    if (!_context || !_context->compute) return false;
    return _context->compute->hasPipeline(name);
}

void PipelineManager::dispatchCompute(VkCommandBuffer cmd, const std::string &name, const ComputeDispatchInfo &info)
{
    if (!_context || !_context->compute) return;
    _context->compute->dispatch(cmd, name, info);
}

void PipelineManager::dispatchComputeImmediate(const std::string &name, const ComputeDispatchInfo &info)
{
    if (!_context || !_context->compute) return;
    _context->compute->dispatchImmediate(name, info);
}

bool PipelineManager::createComputeInstance(const std::string &instanceName, const std::string &pipelineName)
{
    if (!_context || !_context->compute) return false;
    return _context->compute->createInstance(instanceName, pipelineName);
}

void PipelineManager::destroyComputeInstance(const std::string &instanceName)
{
    if (!_context || !_context->compute) return;
    _context->compute->destroyInstance(instanceName);
}

bool PipelineManager::setComputeInstanceStorageImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                                     VkImageLayout layout)
{
    if (!_context || !_context->compute) return false;
    return _context->compute->setInstanceStorageImage(instanceName, binding, view, layout);
}

bool PipelineManager::setComputeInstanceSampledImage(const std::string &instanceName, uint32_t binding, VkImageView view,
                                                     VkSampler sampler, VkImageLayout layout)
{
    if (!_context || !_context->compute) return false;
    return _context->compute->setInstanceSampledImage(instanceName, binding, view, sampler, layout);
}

bool PipelineManager::setComputeInstanceBuffer(const std::string &instanceName, uint32_t binding, VkBuffer buffer,
                                               VkDeviceSize size, VkDescriptorType type, VkDeviceSize offset)
{
    if (!_context || !_context->compute) return false;
    return _context->compute->setInstanceBuffer(instanceName, binding, buffer, size, type, offset);
}

AllocatedImage PipelineManager::createAndBindComputeStorageImage(const std::string &instanceName, uint32_t binding,
                                                                VkExtent3D extent, VkFormat format,
                                                                VkImageLayout layout, VkImageUsageFlags usage)
{
    if (!_context || !_context->compute) return {};
    return _context->compute->createAndBindStorageImage(instanceName, binding, extent, format, layout, usage);
}

AllocatedBuffer PipelineManager::createAndBindComputeStorageBuffer(const std::string &instanceName, uint32_t binding,
                                                                  VkDeviceSize size, VkBufferUsageFlags usage,
                                                                  VmaMemoryUsage memUsage)
{
    if (!_context || !_context->compute) return {};
    return _context->compute->createAndBindStorageBuffer(instanceName, binding, size, usage, memUsage);
}

void PipelineManager::dispatchComputeInstance(VkCommandBuffer cmd, const std::string &instanceName,
                                              const ComputeDispatchInfo &info)
{
    if (!_context || !_context->compute) return;
    _context->compute->dispatchInstance(cmd, instanceName, info);
}
