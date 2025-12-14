#include "async_loader.h"

#include <utility>

#include "manager.h"
#include "core/engine.h"
#include "scene/vk_scene.h"

AsyncAssetLoader::AsyncAssetLoader() = default;

AsyncAssetLoader::~AsyncAssetLoader()
{
    shutdown();
}

void AsyncAssetLoader::init(VulkanEngine *engine, AssetManager *assets, TextureCache *textures, uint32_t worker_count)
{
    _engine = engine;
    _assets = assets;
    _textures = textures;

    if (worker_count == 0)
    {
        worker_count = 1;
    }
    start_workers(worker_count);
}

void AsyncAssetLoader::shutdown()
{
    stop_workers();

    std::lock_guard<std::mutex> lock(_jobs_mutex);
    _jobs.clear();
    _queue.clear();
}

void AsyncAssetLoader::start_workers(uint32_t count)
{
    if (_running.load(std::memory_order_acquire))
    {
        return;
    }

    _running.store(true, std::memory_order_release);
    _workers.reserve(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        _workers.emplace_back([this]() { worker_loop(); });
    }
}

void AsyncAssetLoader::stop_workers()
{
    if (!_running.exchange(false, std::memory_order_acq_rel))
    {
        return;
    }

    _jobs_cv.notify_all();
    for (auto &t : _workers)
    {
        if (t.joinable())
        {
            t.join();
        }
    }
    _workers.clear();
}

AsyncAssetLoader::JobID AsyncAssetLoader::load_gltf_async(const std::string &scene_name,
                                                          const std::string &model_relative_path,
                                                          const glm::mat4 &transform,
                                                          bool preload_textures)
{
    if (!_assets)
    {
        return 0;
    }

    JobID id = _next_id.fetch_add(1, std::memory_order_relaxed);
    std::unique_ptr<Job> job = std::make_unique<Job>(/*args...*/);
    job->id = id;
    job->scene_name = scene_name;
    job->model_relative_path = model_relative_path;
    job->transform = transform;
    job->preload_textures = preload_textures;
    job->progress.store(0.0f, std::memory_order_relaxed);
    job->state.store(JobState::Pending, std::memory_order_relaxed);

    // Prefetch textures on the main thread and remember handles for progress.
    if (_textures)
    {
        AssetManager::GLTFTexturePrefetchResult pref = _assets->prefetchGLTFTexturesWithHandles(model_relative_path);
        job->texture_handles = std::move(pref.handles);
    }

    {
        std::lock_guard<std::mutex> lock(_jobs_mutex);
        _jobs.emplace(id, std::move(job));
        _queue.push_back(id);
    }
    _jobs_cv.notify_one();
    return id;
}

AsyncAssetLoader::JobID AsyncAssetLoader::load_gltf_async(const std::string &scene_name,
                                                          const std::string &model_relative_path,
                                                          const WorldVec3 &translation_world,
                                                          const glm::quat &rotation,
                                                          const glm::vec3 &scale,
                                                          bool preload_textures)
{
    JobID id = load_gltf_async(scene_name, model_relative_path, glm::mat4(1.0f), preload_textures);
    if (id == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(_jobs_mutex);
    auto it = _jobs.find(id);
    if (it != _jobs.end() && it->second)
    {
        Job &job = *it->second;
        job.has_world_trs = true;
        job.translation_world = translation_world;
        job.rotation = rotation;
        job.scale = scale;
    }
    return id;
}

bool AsyncAssetLoader::get_job_status(JobID id, JobState &out_state, float &out_progress, std::string *out_error)
{
    std::lock_guard<std::mutex> lock(_jobs_mutex);
    auto it = _jobs.find(id);
    if (it == _jobs.end())
    {
        return false;
    }

    Job &job = *it->second;
    JobState state = job.state.load(std::memory_order_acquire);
    float gltf_progress = job.progress.load(std::memory_order_relaxed);

    float tex_fraction = 0.0f;
    if (_textures && !job.texture_handles.empty())
    {
        size_t total = job.texture_handles.size();
        size_t resident = 0;
        for (TextureCache::TextureHandle h : job.texture_handles)
        {
            if (h == TextureCache::InvalidHandle)
            {
                continue;
            }
            auto st = _textures->state(h);
            if (st == TextureCache::EntryState::Resident)
            {
                resident++;
            }
        }
        if (total > 0)
        {
            tex_fraction = static_cast<float>(resident) / static_cast<float>(total);
        }
    }

    float combined = gltf_progress;
    if (tex_fraction > 0.0f)
    {
        combined = 0.7f * gltf_progress + 0.3f * tex_fraction;
    }
    if (state == JobState::Completed || state == JobState::Failed)
    {
        combined = 1.0f;
    }

    out_state = state;
    out_progress = combined;
    if (out_error)
    {
        *out_error = job.error;
    }
    return true;
}

void AsyncAssetLoader::debug_snapshot(std::vector<DebugJob> &out_jobs)
{
    out_jobs.clear();

    std::lock_guard<std::mutex> lock(_jobs_mutex);
    out_jobs.reserve(_jobs.size());

    for (auto &[id, jobPtr] : _jobs)
    {
        const Job &job = *jobPtr;

        float gltf_progress = job.progress.load(std::memory_order_relaxed);
        JobState state = job.state.load(std::memory_order_acquire);

        float tex_fraction = 0.0f;
        size_t tex_total = job.texture_handles.size();
        size_t tex_resident = 0;
        if (_textures && tex_total > 0)
        {
            for (TextureCache::TextureHandle h : job.texture_handles)
            {
                if (h == TextureCache::InvalidHandle)
                {
                    continue;
                }
                if (_textures->state(h) == TextureCache::EntryState::Resident)
                {
                    tex_resident++;
                }
            }
            if (tex_total > 0)
            {
                tex_fraction = static_cast<float>(tex_resident) / static_cast<float>(tex_total);
            }
        }

        float combined = gltf_progress;
        if (tex_fraction > 0.0f)
        {
            combined = 0.7f * gltf_progress + 0.3f * tex_fraction;
        }
        if (state == JobState::Completed || state == JobState::Failed)
        {
            combined = 1.0f;
        }

        DebugJob dbg{};
        dbg.id = job.id;
        dbg.state = state;
        dbg.progress = combined;
        dbg.scene_name = job.scene_name;
        dbg.model_relative_path = job.model_relative_path;
        dbg.texture_count = tex_total;
        dbg.textures_resident = tex_resident;

        out_jobs.push_back(std::move(dbg));
    }
}

void AsyncAssetLoader::pump_main_thread(SceneManager &scene)
{
    std::lock_guard<std::mutex> lock(_jobs_mutex);

    for (auto &[id, job] : _jobs)
    {
        JobState state = job->state.load(std::memory_order_acquire);
        if (state == JobState::Completed && !job->committed_to_scene)
        {
            if (job->scene)
            {
                if (job->scene->debugName.empty())
                {
                    job->scene->debugName = job->model_relative_path;
                }
                scene.addGLTFInstance(job->scene_name, job->scene, job->transform);
                if (job->has_world_trs)
                {
                    scene.setGLTFInstanceTRSWorld(job->scene_name,
                                                  job->translation_world,
                                                  job->rotation,
                                                  job->scale);
                }

                // Optionally preload textures (same logic as addGLTFInstance)
                if (job->preload_textures && _textures && _engine && _engine->_resourceManager)
                {
                    uint32_t frame = static_cast<uint32_t>(_engine->_frameNumber);
                    uint32_t count = 0;

                    for (const auto &[name, material] : job->scene->materials)
                    {
                        if (material && material->data.materialSet)
                        {
                            _textures->markSetUsed(material->data.materialSet, frame);
                            ++count;
                        }
                    }

                    if (count > 0)
                    {
                        fmt::println("[AsyncLoader] Marked {} materials for preloading in '{}'",
                                     count, job->scene_name);

                        // Trigger immediate texture loading pump to start upload
                        _textures->pumpLoads(*_engine->_resourceManager, _engine->get_current_frame());
                    }
                }
            }
            job->committed_to_scene = true;
        }
    }
}

void AsyncAssetLoader::worker_loop()
{
    while (true)
    {
        JobID id = 0;
        {
            std::unique_lock<std::mutex> lock(_jobs_mutex);
            _jobs_cv.wait(lock, [this]() { return !_queue.empty() || !_running.load(std::memory_order_acquire); });
            if (!_running.load(std::memory_order_acquire) && _queue.empty())
            {
                return;
            }
            if (_queue.empty())
            {
                continue;
            }
            id = _queue.front();
            _queue.pop_front();
        }

        Job *job = nullptr;
        {
            std::lock_guard<std::mutex> lock(_jobs_mutex);
            auto it = _jobs.find(id);
            if (it == _jobs.end())
            {
                continue;
            }
            job = it->second.get();
        }

        job->state.store(JobState::Running, std::memory_order_release);
        job->progress.store(0.01f, std::memory_order_relaxed);

        GLTFLoadCallbacks cb{};
        cb.on_progress = [job](float v)
        {
            job->progress.store(v, std::memory_order_relaxed);
        };
        cb.is_cancelled = [job]() -> bool
        {
            return job->state.load(std::memory_order_acquire) == JobState::Cancelled;
        };

        std::optional<std::shared_ptr<LoadedGLTF> > loaded = _assets->loadGLTF(job->model_relative_path, &cb);
        if (!loaded.has_value() || !loaded.value())
        {
            job->error = "loadGLTF failed or returned empty scene";
            job->state.store(JobState::Failed, std::memory_order_release);
            job->progress.store(1.0f, std::memory_order_relaxed);
            continue;
        }

        job->scene = loaded.value();
        job->progress.store(1.0f, std::memory_order_relaxed);
        job->state.store(JobState::Completed, std::memory_order_release);
    }
}
