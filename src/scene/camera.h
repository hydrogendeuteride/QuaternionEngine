#pragma once

#include <core/types.h>
#include <SDL_events.h>

#include "glm/vec3.hpp"

class Camera {
public:
    glm::vec3 velocity{0.0f, 0.0f, 0.0f};
    glm::dvec3 position_world{0.0, 0.0, 0.0};
    // Orientation stored as a quaternion (local -> world).
    glm::quat orientation { 1.0f, 0.0f, 0.0f, 0.0f };

    // Movement/look tuning
    float moveSpeed { 0.03f };
    float lookSensitivity { 0.0020f };
    bool rmbDown { false };

    // Field of view in degrees for projection
    float fovDegrees { 50.f };

    glm::mat4 getViewMatrix(const glm::vec3 &position_local) const;
    glm::mat4 getRotationMatrix() const;

    void processSDLEvent(SDL_Event& e);

    void update();
};
