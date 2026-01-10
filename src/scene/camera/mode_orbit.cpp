#include "mode_orbit.h"

#include <scene/camera/camera_rig.h>
#include <scene/camera.h>
#include <scene/vk_scene.h>

#include <core/input/input_system.h>

#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace
{
    bool is_finite_vec3(const glm::vec3 &v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    glm::vec3 safe_normalize(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        if (!is_finite_vec3(v))
        {
            return glm::normalize(fallback);
        }

        const float len2 = glm::dot(v, v);
        if (!std::isfinite(len2) || len2 < 1.0e-12f)
        {
            return glm::normalize(fallback);
        }

        return v * (1.0f / std::sqrt(len2));
    }

    void build_orbit_plane_basis(const glm::vec3 &reference_up, glm::vec3 &out_forward_base, glm::vec3 &out_right_base)
    {
        const glm::vec3 up = safe_normalize(reference_up, glm::vec3(0.0f, 1.0f, 0.0f));

        const glm::vec3 forward_ref =
            (std::abs(glm::dot(up, glm::vec3(0.0f, 0.0f, 1.0f))) < 0.99f) ? glm::vec3(0.0f, 0.0f, 1.0f)
                                                                         : glm::vec3(1.0f, 0.0f, 0.0f);

        // Project the reference forward onto the plane perpendicular to up.
        glm::vec3 forward = forward_ref - glm::dot(forward_ref, up) * up;
        forward = safe_normalize(forward, glm::vec3(0.0f, 0.0f, 1.0f));

        glm::vec3 right = glm::cross(up, forward);
        right = safe_normalize(right, glm::vec3(1.0f, 0.0f, 0.0f));

        // Re-orthonormalize to remove drift from fallback paths.
        forward = glm::normalize(glm::cross(right, up));

        out_forward_base = forward;
        out_right_base = right;
    }

    glm::quat orientation_from_backward_and_up(const glm::vec3 &backward_world, const glm::vec3 &reference_up)
    {
        const glm::vec3 z = safe_normalize(backward_world, glm::vec3(0.0f, 0.0f, 1.0f)); // camera local +Z in world
        glm::vec3 up = safe_normalize(reference_up, glm::vec3(0.0f, 1.0f, 0.0f));

        glm::vec3 x = glm::cross(up, z);
        float x_len2 = glm::dot(x, x);
        if (!std::isfinite(x_len2) || x_len2 < 1.0e-10f)
        {
            // Fallback if reference_up is nearly parallel to backward.
            const glm::vec3 alt_up = (std::abs(z.y) < 0.99f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
            x = glm::cross(alt_up, z);
            x_len2 = glm::dot(x, x);
        }

        if (!std::isfinite(x_len2) || x_len2 < 1.0e-10f)
        {
            x = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        else
        {
            x *= (1.0f / std::sqrt(x_len2));
        }

        const glm::vec3 y = glm::cross(z, x);
        const glm::mat3 R(x, y, z);
        return glm::normalize(glm::quat_cast(R));
    }
} // namespace

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

    // Compute yaw and pitch relative to the reference up vector.
    const glm::vec3 up = safe_normalize(_settings.reference_up, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 forward_base{};
    glm::vec3 right_base{};
    build_orbit_plane_basis(up, forward_base, right_base);

    // Project dir onto the plane perpendicular to up to get yaw
    glm::vec3 dir_proj = dir - glm::dot(dir, up) * up;
    float proj_len = glm::length(dir_proj);
    if (proj_len > 0.001f)
    {
        dir_proj = dir_proj / proj_len;
        _settings.yaw = std::atan2(glm::dot(dir_proj, right_base), glm::dot(dir_proj, forward_base));
    }
    else
    {
        _settings.yaw = 0.0f;
    }

    // Pitch is the angle from the horizontal plane (perpendicular to up)
    _settings.pitch = std::asin(std::clamp(-glm::dot(dir, up), -1.0f, 1.0f));
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
                _settings.distance = std::clamp(_settings.distance * factor,
                                                OrbitCameraSettings::kMinDistance,
                                                OrbitCameraSettings::kMaxDistance);
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
    double dist = std::clamp(_settings.distance, OrbitCameraSettings::kMinDistance, OrbitCameraSettings::kMaxDistance);

    // Build orbit frame based on reference up vector.
    const glm::vec3 up = safe_normalize(_settings.reference_up, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 forward_base{};
    glm::vec3 right_base{};
    build_orbit_plane_basis(up, forward_base, right_base);

    // Yaw rotates around reference up, pitch rotates around the yawed right axis.
    const glm::quat yaw_q = glm::angleAxis(yaw, up);
    const glm::vec3 forward_yawed = glm::rotate(yaw_q, forward_base);
    const glm::vec3 right = glm::rotate(yaw_q, right_base);
    const glm::quat pitch_q = glm::angleAxis(pitch, right);

    const glm::vec3 backward_world = glm::normalize(glm::rotate(pitch_q, forward_yawed)); // target -> camera

    camera.position_world = target_pos + glm::dvec3(backward_world) * dist;
    camera.orientation = orientation_from_backward_and_up(backward_world, up);
}
