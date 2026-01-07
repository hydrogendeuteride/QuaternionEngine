#include "mode_free.h"

#include <scene/camera/camera_rig.h>
#include <scene/camera.h>

#include <core/input/input_system.h>

#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

FreeCameraMode::FreeCameraMode(FreeCameraSettings &settings)
    : _settings(settings)
{
}

void FreeCameraMode::on_activate(SceneManager & /*scene*/, Camera & /*camera*/)
{
    _velocity = glm::vec3(0.0f);
    _roll_dir = 0.0f;
    _rmb_down = false;
}

void FreeCameraMode::process_input(SceneManager & /*scene*/,
                                   Camera &camera,
                                   InputSystem &input,
                                   bool ui_capture_keyboard,
                                   bool ui_capture_mouse)
{
    const InputState &st = input.state();

    // Movement is state-based so simultaneous keys work naturally.
    if (ui_capture_keyboard)
    {
        _velocity = glm::vec3(0.0f);
        _roll_dir = 0.0f;
    }
    else
    {
        glm::vec3 v(0.0f);
        if (st.key_down(Key::W)) { v.z -= 1.0f; }
        if (st.key_down(Key::S)) { v.z += 1.0f; }
        if (st.key_down(Key::A)) { v.x -= 1.0f; }
        if (st.key_down(Key::D)) { v.x += 1.0f; }
        _velocity = v;

        float roll = 0.0f;
        if (st.key_down(Key::Q)) { roll -= 1.0f; }
        if (st.key_down(Key::E)) { roll += 1.0f; }
        _roll_dir = roll;
    }

    // Event-based mouse handling so we don't apply motion that happened
    // before RMB was pressed in the same frame.
    for (const InputEvent &e : input.events())
    {
        if (ui_capture_mouse)
        {
            continue;
        }

        if (e.type == InputEvent::Type::MouseButtonDown && e.mouse_button == MouseButton::Right)
        {
            _rmb_down = true;
            input.set_cursor_mode(CursorMode::Relative);
        }
        else if (e.type == InputEvent::Type::MouseButtonUp && e.mouse_button == MouseButton::Right)
        {
            _rmb_down = false;
            input.set_cursor_mode(CursorMode::Normal);
        }
        else if (e.type == InputEvent::Type::MouseMove && _rmb_down)
        {
            float dx = e.mouse_delta.x * _settings.look_sensitivity;
            float dy = e.mouse_delta.y * _settings.look_sensitivity;

            // 6DOF: yaw around camera's local up (in world space)
            glm::vec3 up = glm::rotate(camera.orientation, glm::vec3{0.f, 1.f, 0.f});
            glm::quat qYaw = glm::angleAxis(dx, glm::normalize(up));
            camera.orientation = glm::normalize(qYaw * camera.orientation);

            // pitch around camera's local right (after yaw applied)
            glm::vec3 right = glm::rotate(camera.orientation, glm::vec3{1.f, 0.f, 0.f});
            glm::quat qPitch = glm::angleAxis(-dy, glm::normalize(right));
            camera.orientation = glm::normalize(qPitch * camera.orientation);
        }

        else if (e.type == InputEvent::Type::MouseWheel)
        {
            const float steps = e.wheel_delta.y; // positive = wheel up
            if (std::abs(steps) < 0.001f)
            {
                continue;
            }

            // Ctrl modifies FOV, otherwise adjust move speed.
            if (e.mods.ctrl)
            {
                // Wheel up -> zoom in (smaller FOV)
                camera.fovDegrees -= steps * 2.0f;
                camera.fovDegrees = std::clamp(camera.fovDegrees, 30.0f, 110.0f);
            }
            else
            {
                // Exponential scale for pleasant feel
                float factor = std::pow(1.15f, steps);
                _settings.move_speed = std::clamp(_settings.move_speed * factor, 0.06f, 300000000.0f);
            }
        }
    }

    // Safety: if mouse state shows RMB is no longer down, release relative mode.
    if (_rmb_down && !st.mouse_down(MouseButton::Right))
    {
        _rmb_down = false;
        input.set_cursor_mode(CursorMode::Normal);
    }
}

void FreeCameraMode::update(SceneManager & /*scene*/, Camera &camera, float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    // Roll around the camera's forward axis (world-space axis).
    if (_roll_dir != 0.0f && _settings.roll_speed > 0.0f)
    {
        glm::vec3 forward = glm::rotate(camera.orientation, glm::vec3{0.0f, 0.0f, -1.0f});
        float angle = _roll_dir * _settings.roll_speed * dt;
        glm::quat roll_rotation = glm::angleAxis(angle, glm::normalize(forward));
        camera.orientation = glm::normalize(roll_rotation * camera.orientation);
    }

    // Move in camera-local space.
    if (_velocity.x != 0.0f || _velocity.y != 0.0f || _velocity.z != 0.0f)
    {
        glm::vec3 local_delta = _velocity * (_settings.move_speed * dt);
        glm::mat4 camera_rotation = camera.getRotationMatrix();
        glm::vec3 world_delta = glm::rotate(camera.orientation, local_delta);
        camera.position_world += glm::dvec3(world_delta);
    }
}
