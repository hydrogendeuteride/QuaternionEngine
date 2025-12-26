#include "mode_orbit.h"

#include <scene/camera/camera_rig.h>
#include <scene/camera.h>
#include <scene/vk_scene.h>

#include <core/input/input_system.h>

#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

OrbitCameraMode::OrbitCameraMode(OrbitCameraSettings &settings)
    : _settings(settings)
{
}

void OrbitCameraMode::on_activate(SceneManager &scene, Camera &camera)
{
    _rmb_down = false;

    // If no target set, orbit around a point in front of the camera.
    if (_settings.target.type == CameraTargetType::None)
    {
        glm::vec3 forward = glm::rotate(camera.orientation, glm::vec3(0.0f, 0.0f, -1.0f));
        _settings.target.type = CameraTargetType::WorldPoint;
        _settings.target.world_point = camera.position_world + WorldVec3(forward) * _settings.distance;
    }

    // Derive yaw/pitch/distance from current camera pose to avoid snapping.
    WorldVec3 target_pos{};
    glm::quat target_rot{};
    if (!scene.getCameraRig().resolve_target(scene, _settings.target, target_pos, target_rot))
    {
        return;
    }

    WorldVec3 to_cam = camera.position_world - target_pos;
    double dist = glm::length(to_cam);
    if (!std::isfinite(dist) || dist < 0.001)
    {
        dist = _settings.distance;
    }
    _settings.distance = dist;

    glm::vec3 dir = glm::normalize(glm::vec3(to_cam / dist)); // target -> camera
    _settings.yaw = std::atan2(dir.x, dir.z);
    _settings.pitch = std::asin(std::clamp(-dir.y, -1.0f, 1.0f));
}

void OrbitCameraMode::process_input(SceneManager & /*scene*/,
                                    Camera &camera,
                                    InputSystem &input,
                                    bool /*ui_capture_keyboard*/,
                                    bool ui_capture_mouse)
{
    const InputState &st = input.state();

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

            _settings.yaw += dx;
            _settings.pitch += dy;

            const float limit = glm::half_pi<float>() - 0.01f;
            _settings.pitch = std::clamp(_settings.pitch, -limit, limit);
        }
        else if (e.type == InputEvent::Type::MouseWheel)
        {
            const float steps = e.wheel_delta.y; // positive = wheel up
            if (std::abs(steps) < 0.001f)
            {
                continue;
            }

            if (e.mods.ctrl)
            {
                camera.fovDegrees -= steps * 2.0f;
                camera.fovDegrees = std::clamp(camera.fovDegrees, 30.0f, 110.0f);
            }
            else
            {
                const double factor = std::pow(1.15, -static_cast<double>(steps));
                _settings.distance = std::clamp(_settings.distance * factor, 0.2, 100000.0);
            }
        }
    }

    if (_rmb_down && !st.mouse_down(MouseButton::Right))
    {
        _rmb_down = false;
        input.set_cursor_mode(CursorMode::Normal);
    }
}

void OrbitCameraMode::update(SceneManager &scene, Camera &camera, float /*dt*/)
{
    WorldVec3 target_pos{};
    glm::quat target_rot{};
    if (!scene.getCameraRig().resolve_target(scene, _settings.target, target_pos, target_rot))
    {
        return;
    }

    auto wrap_pi = [](float a) -> float {
        // Wrap angle to [-pi, pi] to avoid precision issues over long play sessions.
        constexpr float two_pi = glm::two_pi<float>();
        a = std::fmod(a + glm::pi<float>(), two_pi);
        if (a < 0.0f) a += two_pi;
        return a - glm::pi<float>();
    };

    float yaw = wrap_pi(_settings.yaw);
    float pitch = std::clamp(_settings.pitch,
                             -glm::half_pi<float>() + 0.01f,
                             glm::half_pi<float>() - 0.01f);
    _settings.yaw = yaw;
    _settings.pitch = pitch;
    double dist = std::max(0.2, _settings.distance);

    glm::quat yaw_q = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 right = glm::rotate(yaw_q, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::quat pitch_q = glm::angleAxis(pitch, right);
    glm::quat orbit_q = glm::normalize(pitch_q * yaw_q);

    // Place the camera on its local +Z axis relative to the target so the camera's
    // -Z forward axis points toward the target.
    const double yaw_d = static_cast<double>(yaw);
    const double pitch_d = static_cast<double>(pitch);
    const double cos_pitch = std::cos(pitch_d);
    const double sin_pitch = std::sin(pitch_d);
    const double sin_yaw = std::sin(yaw_d);
    const double cos_yaw = std::cos(yaw_d);

    const glm::dvec3 dir_target_to_camera(
        sin_yaw * cos_pitch,
        -sin_pitch,
        cos_yaw * cos_pitch);

    camera.position_world = target_pos + dir_target_to_camera * dist;
    camera.orientation = orbit_q;
}
