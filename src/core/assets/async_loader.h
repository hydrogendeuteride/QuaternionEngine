#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>

#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "scene/vk_loader.h"
#include "core/world.h"
#include "core/assets/texture_cache.h"

class VulkanEngine;
class AssetManager;
class SceneManager;

// Small orchestrator for asynchronous glTF asset jobs.
// - CPU work (file I/O, fastgltf parsing, mesh/BVH build) runs on worker threads.
// - GPU uploads are still deferred through ResourceManager and the Render Graph.
// - Texture streaming and residency are tracked via TextureCache for progress.
class AsyncAssetLoader
{
public:
    using JobID = uint32_t;

    enum class JobState { Pending, Running, Completed, Failed, Cancelled };

    AsyncAssetLoader();
    ~AsyncAssetLoader();

    void init(VulkanEngine *engine, AssetManager *assets, TextureCache *textures, uint32_t worker_count = 1);
    void shutdown();

    JobID load_gltf_async(const std::string &scene_name,
                          const std::string &model_relative_path,
                          const glm::mat4 &transform,
                          bool preload_textures = false);

    JobID load_gltf_async(const std::string &scene_name,
                          const std::string &model_relative_path,
                          const WorldVec3 &translation_world,
                          const glm::quat &rotation,
                          const glm::vec3 &scale,
                          bool preload_textures = false);

    bool get_job_status(JobID id, JobState &out_state, float &out_progress, std::string *out_error = nullptr);

    // Main-thread integration: commit completed jobs into the SceneManager.
    void pump_main_thread(SceneManager &scene);

    struct DebugJob
    {
        JobID id{0};
        JobState state{JobState::Pending};
        float progress{0.0f};
        std::string scene_name;
        std::string model_relative_path;
        size_t texture_count{0};
        size_t textures_resident{0};
    };
    // Debug-only snapshot of current jobs for UI/tools (main-thread only).
    void debug_snapshot(std::vector<DebugJob> &out_jobs);

private:
    struct Job
    {
        JobID id{0};
        std::string scene_name;
        std::string model_relative_path;
        glm::mat4 transform{1.0f};
        bool has_world_trs{false};
        WorldVec3 translation_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f};
        bool preload_textures{false};

        std::shared_ptr<LoadedGLTF> scene;

        std::atomic<float> progress{0.0f};
        std::atomic<JobState> state{JobState::Pending};

        std::string error;
        bool committed_to_scene{false};

        // Texture handles associated with this glTF (prefetched via TextureCache).
        std::vector<TextureCache::TextureHandle> texture_handles;
    };

    void start_workers(uint32_t count);
    void stop_workers();
    void worker_loop();

    VulkanEngine *_engine{nullptr};
    AssetManager *_assets{nullptr};
    TextureCache *_textures{nullptr};

    std::atomic<bool> _running{false};
    std::vector<std::thread> _workers;

    std::mutex _jobs_mutex;
    std::condition_variable _jobs_cv;
    std::unordered_map<JobID, std::unique_ptr<Job>> _jobs;
    std::deque<JobID> _queue;
    std::atomic<JobID> _next_id{1};
};
