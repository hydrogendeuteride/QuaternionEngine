#pragma once

#include <core/types.h>

class Camera {
public:
    glm::dvec3 position_world{0.0, 0.0, 0.0};
    // Orientation stored as a quaternion (local -> world).
    glm::quat orientation { 1.0f, 0.0f, 0.0f, 0.0f };

    // Field of view in degrees for projection
    float fovDegrees { 50.f };

    glm::mat4 getViewMatrix(const glm::vec3 &position_local) const;
    glm::mat4 getRotationMatrix() const;
};
