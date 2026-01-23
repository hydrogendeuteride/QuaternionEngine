#pragma once

#include "collision_shape.h"

#include <glm/mat4x4.hpp>

#include <memory>

namespace Physics
{
    // A mesh collider attached to a glTF node.
    // The mesh itself is in its local space; relative_transform positions it relative to the owning node.
    struct ColliderMeshInstance
    {
        std::shared_ptr<const TriangleMeshData> mesh;
        glm::mat4 relative_transform{1.0f};
    };
} // namespace Physics
