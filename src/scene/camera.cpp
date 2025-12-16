#include "camera.h"
#include <glm/gtx/transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <cmath>

void Camera::update()
{
    glm::mat4 cameraRotation = getRotationMatrix();
    glm::vec3 delta = glm::vec3(cameraRotation * glm::vec4(velocity * moveSpeed, 0.f));
    position_world += glm::dvec3(delta);
}

void Camera::process_input(InputSystem &input, bool ui_capture_keyboard, bool ui_capture_mouse)
{
    const InputState &st = input.state();

    // Movement is state-based so simultaneous keys work naturally.
    if (ui_capture_keyboard)
    {
        velocity = glm::vec3(0.0f);
    }
    else
    {
        glm::vec3 v(0.0f);
        if (st.key_down(Key::W)) { v.z -= 1.0f; }
        if (st.key_down(Key::S)) { v.z += 1.0f; }
        if (st.key_down(Key::A)) { v.x -= 1.0f; }
        if (st.key_down(Key::D)) { v.x += 1.0f; }
        velocity = v;
    }

    // Event-based mouse handling so we don't apply motion that happened before RMB was pressed in the same frame.
    for (const InputEvent &e : input.events())
    {
        if (ui_capture_mouse)
        {
            continue;
        }

        if (e.type == InputEvent::Type::MouseButtonDown && e.mouse_button == MouseButton::Right)
        {
            rmbDown = true;
            input.set_cursor_mode(CursorMode::Relative);
        }
        else if (e.type == InputEvent::Type::MouseButtonUp && e.mouse_button == MouseButton::Right)
        {
            rmbDown = false;
            input.set_cursor_mode(CursorMode::Normal);
        }
        else if (e.type == InputEvent::Type::MouseMove && rmbDown)
        {
            // Convert mouse motion to incremental yaw/pitch angles.
            float dx = e.mouse_delta.x * lookSensitivity;
            float dy = e.mouse_delta.y * lookSensitivity;

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
        else if (e.type == InputEvent::Type::MouseWheel)
        {
            const float steps = e.wheel_delta.y; // positive = wheel up
            if (std::abs(steps) < 0.001f)
            {
                continue;
            }

            // Ctrl modifies FOV, otherwise adjust move speed
            if (e.mods.ctrl)
            {
                // Wheel up -> zoom in (smaller FOV)
                fovDegrees -= steps * 2.0f;
                fovDegrees = std::clamp(fovDegrees, 30.0f, 110.0f);
            }
            else
            {
                // Exponential scale for pleasant feel
                float factor = std::pow(1.15f, steps);
                moveSpeed = std::clamp(moveSpeed * factor, 0.001f, 5.0f);
            }
        }
    }

    // Safety: if mouse state shows RMB is no longer down, release relative mode.
    if (rmbDown && !st.mouse_down(MouseButton::Right))
    {
        rmbDown = false;
        input.set_cursor_mode(CursorMode::Normal);
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
