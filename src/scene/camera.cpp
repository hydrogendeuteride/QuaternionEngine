#include "camera.h"
#include <glm/gtx/transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <SDL2/SDL.h>
#include <algorithm>
#include <cmath>

void Camera::update()
{
    glm::mat4 cameraRotation = getRotationMatrix();
    glm::vec3 delta = glm::vec3(cameraRotation * glm::vec4(velocity * moveSpeed, 0.f));
    position_world += glm::dvec3(delta);
}

void Camera::processSDLEvent(SDL_Event& e)
{
    if (e.type == SDL_KEYDOWN) {
        // Camera uses -Z forward convention (right-handed)
        if (e.key.keysym.sym == SDLK_w) { velocity.z = -1; }
        if (e.key.keysym.sym == SDLK_s) { velocity.z = 1; }
        if (e.key.keysym.sym == SDLK_a) { velocity.x = -1; }
        if (e.key.keysym.sym == SDLK_d) { velocity.x = 1; }
    }

    if (e.type == SDL_KEYUP) {
        if (e.key.keysym.sym == SDLK_w) { velocity.z = 0; }
        if (e.key.keysym.sym == SDLK_s) { velocity.z = 0; }
        if (e.key.keysym.sym == SDLK_a) { velocity.x = 0; }
        if (e.key.keysym.sym == SDLK_d) { velocity.x = 0; }
    }

    if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_RIGHT) {
        rmbDown = true;
        SDL_SetRelativeMouseMode(SDL_TRUE);
    }
    if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_RIGHT) {
        rmbDown = false;
        SDL_SetRelativeMouseMode(SDL_FALSE);
    }

    if (e.type == SDL_MOUSEMOTION && rmbDown) {
        // Convert mouse motion to incremental yaw/pitch angles.
        float dx = static_cast<float>(e.motion.xrel) * lookSensitivity;
        float dy = static_cast<float>(e.motion.yrel) * lookSensitivity;

        // Mouse right (xrel > 0) turns view right with -Z-forward: yaw around +Y.
        glm::quat yawRotation = glm::angleAxis(dx, glm::vec3 { 0.f, 1.f, 0.f });

        // Mouse up (yrel < 0) looks up with -Z-forward: negative dy.
        float pitchDelta = -dy;
        // Pitch around the camera's local X (right) axis in world space.
        glm::vec3 right = glm::rotate(orientation, glm::vec3 { 1.f, 0.f, 0.f });
        glm::quat pitchRotation = glm::angleAxis(pitchDelta, glm::vec3(right));

        // Apply yaw, then pitch, to the current orientation.
        orientation = glm::normalize(pitchRotation * yawRotation * orientation);
    }

    if (e.type == SDL_MOUSEWHEEL) {
        // Ctrl modifies FOV, otherwise adjust move speed
        const bool ctrl = (SDL_GetModState() & KMOD_CTRL) != 0;
        const int steps = e.wheel.y; // positive = wheel up
        if (ctrl) {
            // Wheel up -> zoom in (smaller FOV)
            fovDegrees -= steps * 2.0f;
            fovDegrees = std::clamp(fovDegrees, 30.0f, 110.0f);
        } else {
            // Exponential scale for pleasant feel
            float factor = std::pow(1.15f, (float)steps);
            moveSpeed = std::clamp(moveSpeed * factor, 0.001f, 5.0f);
        }
    }
}

glm::mat4 Camera::getViewMatrix(const glm::vec3 &position_local) const
{
    // to create a correct model view, we need to move the world in opposite
    // direction to the camera
    //  so we will create the camera model matrix and invert
    glm::mat4 cameraTranslation = glm::translate(glm::mat4(1.f), position_local);
    glm::mat4 cameraRotation = getRotationMatrix();
    return glm::inverse(cameraTranslation * cameraRotation);
}

glm::mat4 Camera::getRotationMatrix() const
{
    // Use the stored quaternion orientation directly.
    return glm::toMat4(orientation);
}
