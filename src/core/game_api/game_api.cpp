#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"

#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

namespace GameAPI
{

// ============================================================================
// Transform helpers
// ============================================================================

glm::mat4 Transform::to_matrix() const
{
    glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 R = glm::mat4_cast(rotation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
    return T * R * S;
}

Transform Transform::from_matrix(const glm::mat4& m)
{
    Transform t;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(m, t.scale, t.rotation, t.position, skew, perspective);
    return t;
}

glm::mat4 TransformD::to_matrix() const
{
    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(position));
    glm::mat4 R = glm::mat4_cast(rotation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
    return T * R * S;
}

TransformD TransformD::from_matrix(const glm::mat4& m)
{
    TransformD t;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::vec3 pos{};
    glm::decompose(m, t.scale, t.rotation, pos, skew, perspective);
    t.position = glm::dvec3(pos);
    return t;
}

// ============================================================================
// Engine Implementation
// ============================================================================

Engine::Engine(VulkanEngine* engine)
    : _engine(engine)
{
}

} // namespace GameAPI
