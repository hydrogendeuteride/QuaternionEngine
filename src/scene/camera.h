#pragma once

#include <core/types.h>
#include <SDL_events.h>

#include "glm/vec3.hpp"

class Camera {
public:
    glm::vec3 velocity;
    glm::vec3 position;
    // vertical rotation
    float pitch { 0.f };
    // horizontal rotation
    float yaw { 0.f };

    // Movement/look tuning
    float moveSpeed { 0.03f };
    float lookSensitivity { 0.0020f };
    bool rmbDown { false };

    // Field of view in degrees for projection
    float fovDegrees { 50.f };

    glm::mat4 getViewMatrix();
    glm::mat4 getRotationMatrix();

    void processSDLEvent(SDL_Event& e);

    void update();
};
