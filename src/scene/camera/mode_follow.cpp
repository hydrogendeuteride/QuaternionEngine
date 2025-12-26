#include "mode_follow.h"

#include <scene/camera/camera_rig.h>
#include <scene/camera.h>
#include <scene/vk_scene.h>

#include <glm/gtx/quaternion.hpp>

FollowCameraMode::FollowCameraMode(FollowCameraSettings &settings)
    : _settings(settings)
{
}

void FollowCameraMode::on_activate(SceneManager &scene, Camera &camera)
{
    // If no target set, follow a point in front of the camera.
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

void FollowCameraMode::process_input(SceneManager & /*scene*/,
                                     Camera & /*camera*/,
                                     InputSystem & /*input*/,
                                     bool /*ui_capture_keyboard*/,
                                     bool /*ui_capture_mouse*/)
{
}

void FollowCameraMode::update(SceneManager &scene, Camera &camera, float /*dt*/)
{
    WorldVec3 target_pos{};
    glm::quat target_rot{};
    if (!scene.getCameraRig().resolve_target(scene, _settings.target, target_pos, target_rot))
    {
        return;
    }

    glm::vec3 offset_world = glm::rotate(target_rot, _settings.position_offset_local);
    camera.position_world = target_pos + WorldVec3(offset_world);
    camera.orientation = glm::normalize(target_rot * _settings.rotation_offset);
}
