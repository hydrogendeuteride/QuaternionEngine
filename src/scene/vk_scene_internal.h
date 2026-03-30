#pragma once

#include "vk_scene.h"

#include <memory>

namespace SceneInternal
{
    inline glm::mat4 build_node_world_with_overrides(
        const Node *node,
        const std::unordered_map<const Node *, glm::mat4> &overrides)
    {
        if (!node)
        {
            return glm::mat4(1.0f);
        }

        std::vector<const Node *> chain;
        const Node *cur = node;
        while (cur)
        {
            chain.push_back(cur);
            std::shared_ptr<Node> parent = cur->parent.lock();
            cur = parent ? parent.get() : nullptr;
        }

        glm::mat4 world(1.0f);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        {
            const Node *n = *it;
            glm::mat4 local = n->localTransform;
            auto ov_it = overrides.find(n);
            if (ov_it != overrides.end())
            {
                local = local * ov_it->second;
            }
            world = world * local;
        }

        return world;
    }
}
