#pragma once

#include <memory>
#include <vector>

struct LoadedGLTF;

namespace Game
{
    void retain_preloaded_gltf_scenes(std::vector<std::shared_ptr<LoadedGLTF>> scenes);
    void clear_preloaded_gltf_scenes();
} // namespace Game
