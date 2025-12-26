#include "mode_chase.h"

#include <scene/camera/camera_rig.h>
#include <scene/camera.h>
#include <scene/vk_scene.h>

#include <glm/gtx/quaternion.hpp>

#include <cmath>

ChaseCameraMode::ChaseCameraMode(ChaseCameraSettings &settings)
    : _settings(settings)
{
}

void ChaseCameraMode::on_activate(SceneManager &scene, Camera &camera)
{
    // If no target set, chase a point in front of the camera.
    if (_settings.target.type == CameraTargetType::None)
    {
        glm::vec3 forward = glm::rotate(camera.orientation, glm::vec3(0.0f, 0.0f, -1.0f));
        _settings.target.type = CameraTargetType::WorldPoint;
        _settings.target.world_point = camera.position_world + WorldVec3(forward) * 10.0;
    }

    // Preserve current relative transform to the target when possible.
    WorldVec3 target_pos{};
    glm::quat target_rot{};
    if (!scene.getCameraRig().resolve_target(scene, _settings.target, target_pos, target_rot))
    {
        return;
    }

    glm::quat inv_target = glm::inverse(target_rot);
    glm::vec3 rel_pos = glm::vec3(camera.position_world - target_pos);
    _settings.position_offset_local = glm::rotate(inv_target, rel_pos);
    _settings.rotation_offset = glm::normalize(inv_target * camera.orientation);
}

void ChaseCameraMode::process_input(SceneManager & /*scene*/,
                                    Camera & /*camera*/,
                                    InputSystem & /*input*/,
                                    bool /*ui_capture_keyboard*/,
                                    bool /*ui_capture_mouse*/)
{
}

void ChaseCameraMode::update(SceneManager &scene, Camera &camera, float dt)
{
    WorldVec3 target_pos{};
    glm::quat target_rot{};
    if (!scene.getCameraRig().resolve_target(scene, _settings.target, target_pos, target_rot))
    {
        return;
    }

    glm::vec3 offset_world = glm::rotate(target_rot, _settings.position_offset_local);
    WorldVec3 desired_pos = target_pos + WorldVec3(offset_world);
    glm::quat desired_rot = glm::normalize(target_rot * _settings.rotation_offset);

    if (dt > 0.0f)
    {
        float pos_alpha = 1.0f - std::exp(-_settings.position_lag * dt);
        float rot_alpha = 1.0f - std::exp(-_settings.rotation_lag * dt);

        camera.position_world += (desired_pos - camera.position_world) * static_cast<double>(pos_alpha);
        camera.orientation = glm::normalize(glm::slerp(camera.orientation, desired_rot, rot_alpha));
    }
    else
    {
        camera.position_world = desired_pos;
        camera.orientation = desired_rot;
    }
}
