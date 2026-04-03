#include "gameplay_preload_cache.h"

#include "scene/vk_loader.h"

#include <mutex>
#include <utility>

namespace Game
{
    namespace
    {
        std::mutex g_preloaded_gltf_mutex;
        std::vector<std::shared_ptr<LoadedGLTF>> g_preloaded_gltf_scenes;
    } // namespace

    void retain_preloaded_gltf_scenes(std::vector<std::shared_ptr<LoadedGLTF>> scenes)
    {
        std::lock_guard<std::mutex> lock(g_preloaded_gltf_mutex);
        g_preloaded_gltf_scenes = std::move(scenes);
    }

    void clear_preloaded_gltf_scenes()
    {
        std::lock_guard<std::mutex> lock(g_preloaded_gltf_mutex);
        g_preloaded_gltf_scenes.clear();
    }
} // namespace Game
