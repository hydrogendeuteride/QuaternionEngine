#include "core/game_api.h"
#include "core/engine.h"
#include "core/context.h"
#include "core/input/input_system.h"
#include "core/picking/picking_system.h"
#include "scene/vk_scene.h"
#include "scene/camera.h"
#include "scene/camera/camera_rig.h"
#include "scene/planet/planet_system.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <cmath>

namespace GameAPI
{

// ----------------------------------------------------------------------------
// Camera - Position and Orientation
// ----------------------------------------------------------------------------

void Engine::set_camera_position(const glm::vec3& position)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->getMainCamera().position_world = WorldVec3(position);
    }
}

glm::vec3 Engine::get_camera_position() const
{
    if (_engine->_sceneManager)
    {
        return glm::vec3(_engine->_sceneManager->getMainCamera().position_world);
    }
    return glm::vec3(0.0f);
}

void Engine::set_camera_position(const glm::dvec3& position)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->getMainCamera().position_world = position;
    }
}

glm::dvec3 Engine::get_camera_position_d() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getMainCamera().position_world;
    }
    return glm::dvec3(0.0);
}

void Engine::set_camera_rotation(float pitch, float yaw)
{
    if (_engine->_sceneManager)
    {
        Camera& cam = _engine->_sceneManager->getMainCamera();

        // Convert degrees to radians.
        float pitchRad = glm::radians(pitch);
        float yawRad = glm::radians(yaw);

        // -Z forward convention: yaw around +Y, then pitch around local +X.
        glm::quat yawQ = glm::angleAxis(yawRad, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::quat pitchQ = glm::angleAxis(pitchRad, glm::vec3(1.0f, 0.0f, 0.0f));

        cam.orientation = glm::normalize(yawQ * pitchQ);
    }
}

void Engine::get_camera_rotation(float& pitch, float& yaw) const
{
    if (_engine->_sceneManager)
    {
        const Camera& cam = _engine->_sceneManager->getMainCamera();

        // Derive forward from orientation and convert to pitch/yaw (degrees).
        glm::vec3 forward = glm::rotate(cam.orientation, glm::vec3(0.0f, 0.0f, -1.0f));
        forward = glm::normalize(forward);

        pitch = glm::degrees(asinf(-forward.y));
        yaw = glm::degrees(atan2f(forward.x, forward.z));
    }
    else
    {
        pitch = 0.0f;
        yaw = 0.0f;
    }
}

void Engine::set_camera_fov(float fovDegrees)
{
    if (_engine->_sceneManager)
    {
        _engine->_sceneManager->getMainCamera().fovDegrees = fovDegrees;
    }
}

float Engine::get_camera_fov() const
{
    if (_engine->_sceneManager)
    {
        return _engine->_sceneManager->getMainCamera().fovDegrees;
    }
    return 70.0f;
}

void Engine::camera_look_at(const glm::vec3& target)
{
    if (!_engine->_sceneManager) return;

    Camera& cam = _engine->_sceneManager->getMainCamera();
    glm::vec3 to_target = target - glm::vec3(cam.position_world);
    if (!std::isfinite(to_target.x) || !std::isfinite(to_target.y) || !std::isfinite(to_target.z) || glm::length2(to_target) < 1.0e-12f)
    {
        return;
    }

    glm::vec3 dir = glm::normalize(to_target);

    glm::vec3 up(0.0f, 1.0f, 0.0f);
    if (glm::length2(glm::cross(dir, up)) < 1e-6f)
    {
        up = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    const glm::vec3 forward = dir;
    const glm::vec3 backward = -forward; // camera local +Z in world

    glm::vec3 right = glm::cross(up, backward);
    if (glm::length2(right) < 1.0e-12f)
    {
        // Fallback if up is still near-parallel (degenerate); pick an alternate up.
        const glm::vec3 alt_up = (std::abs(backward.y) < 0.99f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        right = glm::cross(alt_up, backward);
    }
    right = glm::normalize(right);

    const glm::vec3 true_up = glm::cross(backward, right);

    glm::mat3 rot;
    rot[0] = right;
    rot[1] = true_up;
    rot[2] = backward;

    cam.orientation = glm::normalize(glm::quat_cast(rot));
}

void Engine::camera_look_at(const glm::dvec3& target)
{
    if (!_engine->_sceneManager) return;

    Camera& cam = _engine->_sceneManager->getMainCamera();
    glm::dvec3 to_target_d = target - cam.position_world;
    if (!std::isfinite(to_target_d.x) || !std::isfinite(to_target_d.y) || !std::isfinite(to_target_d.z) || glm::dot(to_target_d, to_target_d) < 1.0e-24)
    {
        return;
    }

    glm::dvec3 dirD = glm::normalize(to_target_d);
    glm::vec3 dir = glm::normalize(glm::vec3(dirD));

    glm::vec3 up(0.0f, 1.0f, 0.0f);
    if (glm::length2(glm::cross(dir, up)) < 1e-6f)
    {
        up = glm::vec3(0.0f, 0.0f, 1.0f);
    }

    const glm::vec3 forward = dir;
    const glm::vec3 backward = -forward;

    glm::vec3 right = glm::cross(up, backward);
    if (glm::length2(right) < 1.0e-12f)
    {
        const glm::vec3 alt_up = (std::abs(backward.y) < 0.99f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        right = glm::cross(alt_up, backward);
    }
    right = glm::normalize(right);

    const glm::vec3 true_up = glm::cross(backward, right);

    glm::mat3 rot;
    rot[0] = right;
    rot[1] = true_up;
    rot[2] = backward;

    cam.orientation = glm::normalize(glm::quat_cast(rot));
}

// ----------------------------------------------------------------------------
// Camera Mode Conversion Helpers
// ----------------------------------------------------------------------------

namespace
{
    ::CameraMode to_internal_camera_mode(GameAPI::CameraMode mode)
    {
        switch (mode)
        {
            case GameAPI::CameraMode::Free: return ::CameraMode::Free;
            case GameAPI::CameraMode::Orbit: return ::CameraMode::Orbit;
            case GameAPI::CameraMode::Follow: return ::CameraMode::Follow;
            case GameAPI::CameraMode::Chase: return ::CameraMode::Chase;
            case GameAPI::CameraMode::Fixed: return ::CameraMode::Fixed;
            default: return ::CameraMode::Free;
        }
    }

    GameAPI::CameraMode to_api_camera_mode(::CameraMode mode)
    {
        switch (mode)
        {
            case ::CameraMode::Free: return GameAPI::CameraMode::Free;
            case ::CameraMode::Orbit: return GameAPI::CameraMode::Orbit;
            case ::CameraMode::Follow: return GameAPI::CameraMode::Follow;
            case ::CameraMode::Chase: return GameAPI::CameraMode::Chase;
            case ::CameraMode::Fixed: return GameAPI::CameraMode::Fixed;
            default: return GameAPI::CameraMode::Free;
        }
    }

    ::CameraTargetType to_internal_target_type(GameAPI::CameraTargetType type)
    {
        switch (type)
        {
            case GameAPI::CameraTargetType::None: return ::CameraTargetType::None;
            case GameAPI::CameraTargetType::WorldPoint: return ::CameraTargetType::WorldPoint;
            case GameAPI::CameraTargetType::MeshInstance: return ::CameraTargetType::MeshInstance;
            case GameAPI::CameraTargetType::GLTFInstance: return ::CameraTargetType::GLTFInstance;
            default: return ::CameraTargetType::None;
        }
    }

    GameAPI::CameraTargetType to_api_target_type(::CameraTargetType type)
    {
        switch (type)
        {
            case ::CameraTargetType::None: return GameAPI::CameraTargetType::None;
            case ::CameraTargetType::WorldPoint: return GameAPI::CameraTargetType::WorldPoint;
            case ::CameraTargetType::MeshInstance: return GameAPI::CameraTargetType::MeshInstance;
            case ::CameraTargetType::GLTFInstance: return GameAPI::CameraTargetType::GLTFInstance;
            default: return GameAPI::CameraTargetType::None;
        }
    }

    ::CameraTarget to_internal_target(const GameAPI::CameraTarget &target)
    {
        ::CameraTarget t;
        t.type = to_internal_target_type(target.type);
        t.name = target.name;
        t.world_point = WorldVec3(target.worldPoint);
        return t;
    }

    GameAPI::CameraTarget to_api_target(const ::CameraTarget &target)
    {
        GameAPI::CameraTarget t;
        t.type = to_api_target_type(target.type);
        t.name = target.name;
        t.worldPoint = glm::dvec3(target.world_point);
        return t;
    }
} // namespace

// ----------------------------------------------------------------------------
// Camera Mode
// ----------------------------------------------------------------------------

void Engine::set_camera_mode(CameraMode mode)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    SceneManager *scene = _engine->_sceneManager.get();
    Camera &cam = scene->getMainCamera();
    CameraRig &rig = scene->getCameraRig();

    if (_engine->_input)
    {
        _engine->_input->set_cursor_mode(CursorMode::Normal);
    }

    rig.set_mode(to_internal_camera_mode(mode), *scene, cam);
}

CameraMode Engine::get_camera_mode() const
{
    if (!_engine || !_engine->_sceneManager)
    {
        return CameraMode::Free;
    }
    return to_api_camera_mode(_engine->_sceneManager->getCameraRig().mode());
}

// ----------------------------------------------------------------------------
// Free Camera Settings
// ----------------------------------------------------------------------------

void Engine::set_free_camera_settings(const FreeCameraSettings& settings)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    ::FreeCameraSettings &s = _engine->_sceneManager->getCameraRig().free_settings();
    s.move_speed = settings.moveSpeed;
    s.look_sensitivity = settings.lookSensitivity;
    s.roll_speed = settings.rollSpeed;
}

FreeCameraSettings Engine::get_free_camera_settings() const
{
    FreeCameraSettings out{};
    if (!_engine || !_engine->_sceneManager)
    {
        return out;
    }

    const ::FreeCameraSettings &s = _engine->_sceneManager->getCameraRig().free_settings();
    out.moveSpeed = s.move_speed;
    out.lookSensitivity = s.look_sensitivity;
    out.rollSpeed = s.roll_speed;
    return out;
}

// ----------------------------------------------------------------------------
// Orbit Camera Settings
// ----------------------------------------------------------------------------

void Engine::set_orbit_camera_settings(const OrbitCameraSettings& settings)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    ::OrbitCameraSettings &s = _engine->_sceneManager->getCameraRig().orbit_settings();
    s.target = to_internal_target(settings.target);
    s.distance = settings.distance;
    s.yaw = settings.yaw;
    s.pitch = settings.pitch;
    s.look_sensitivity = settings.lookSensitivity;
}

OrbitCameraSettings Engine::get_orbit_camera_settings() const
{
    OrbitCameraSettings out{};
    if (!_engine || !_engine->_sceneManager)
    {
        return out;
    }

    const ::OrbitCameraSettings &s = _engine->_sceneManager->getCameraRig().orbit_settings();
    out.target = to_api_target(s.target);
    out.distance = s.distance;
    out.yaw = s.yaw;
    out.pitch = s.pitch;
    out.lookSensitivity = s.look_sensitivity;
    return out;
}

// ----------------------------------------------------------------------------
// Follow Camera Settings
// ----------------------------------------------------------------------------

void Engine::set_follow_camera_settings(const FollowCameraSettings& settings)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    ::FollowCameraSettings &s = _engine->_sceneManager->getCameraRig().follow_settings();
    s.target = to_internal_target(settings.target);
    s.position_offset_local = settings.positionOffsetLocal;
    s.rotation_offset = settings.rotationOffset;
}

FollowCameraSettings Engine::get_follow_camera_settings() const
{
    FollowCameraSettings out{};
    if (!_engine || !_engine->_sceneManager)
    {
        return out;
    }

    const ::FollowCameraSettings &s = _engine->_sceneManager->getCameraRig().follow_settings();
    out.target = to_api_target(s.target);
    out.positionOffsetLocal = s.position_offset_local;
    out.rotationOffset = s.rotation_offset;
    return out;
}

// ----------------------------------------------------------------------------
// Chase Camera Settings
// ----------------------------------------------------------------------------

void Engine::set_chase_camera_settings(const ChaseCameraSettings& settings)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    ::ChaseCameraSettings &s = _engine->_sceneManager->getCameraRig().chase_settings();
    s.target = to_internal_target(settings.target);
    s.position_offset_local = settings.positionOffsetLocal;
    s.rotation_offset = settings.rotationOffset;
    s.position_lag = settings.positionLag;
    s.rotation_lag = settings.rotationLag;
}

ChaseCameraSettings Engine::get_chase_camera_settings() const
{
    ChaseCameraSettings out{};
    if (!_engine || !_engine->_sceneManager)
    {
        return out;
    }

    const ::ChaseCameraSettings &s = _engine->_sceneManager->getCameraRig().chase_settings();
    out.target = to_api_target(s.target);
    out.positionOffsetLocal = s.position_offset_local;
    out.rotationOffset = s.rotation_offset;
    out.positionLag = s.position_lag;
    out.rotationLag = s.rotation_lag;
    return out;
}

// ----------------------------------------------------------------------------
// Camera Target from Pick
// ----------------------------------------------------------------------------

bool Engine::set_camera_target_from_last_pick()
{
    if (!_engine || !_engine->_sceneManager)
    {
        return false;
    }

    const PickingSystem *picking = _engine->picking();
    if (!picking)
    {
        return false;
    }

    const auto &pick = picking->last_pick();
    if (!pick.valid)
    {
        return false;
    }

    ::CameraTarget t;
    if (pick.ownerType == RenderObject::OwnerType::MeshInstance)
    {
        // RenderObject::OwnerType::MeshInstance is also used for some procedural objects
        // (planets etc.) which don't exist in SceneManager::dynamicMeshInstances.
        WorldVec3 inst_t{};
        glm::quat inst_r{};
        glm::vec3 inst_s{};
        if (_engine->_sceneManager->getMeshInstanceTRSWorld(pick.ownerName, inst_t, inst_r, inst_s))
        {
            t.type = ::CameraTargetType::MeshInstance;
            t.name = pick.ownerName;
        }
        else if (PlanetSystem *planets = _engine->_sceneManager->get_planet_system())
        {
            if (PlanetSystem::PlanetBody *body = planets->find_body_by_name(pick.ownerName))
            {
                t.type = ::CameraTargetType::MeshInstance;
                t.name = body->name;
            }
            else
            {
                t.type = ::CameraTargetType::WorldPoint;
                t.world_point = pick.worldPos;
            }
        }
        else
        {
            t.type = ::CameraTargetType::WorldPoint;
            t.world_point = pick.worldPos;
        }
    }
    else if (pick.ownerType == RenderObject::OwnerType::GLTFInstance)
    {
        t.type = ::CameraTargetType::GLTFInstance;
        t.name = pick.ownerName;
    }
    else
    {
        t.type = ::CameraTargetType::WorldPoint;
        t.world_point = pick.worldPos;
    }

    CameraRig &rig = _engine->_sceneManager->getCameraRig();
    rig.orbit_settings().target = t;
    rig.follow_settings().target = t;
    rig.chase_settings().target = t;
    return true;
}

// ----------------------------------------------------------------------------
// Orbit Camera Reference Up Vector
// ----------------------------------------------------------------------------

void Engine::align_orbit_camera_up_to_target()
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    _engine->_sceneManager->getCameraRig().align_orbit_up_to_target();
}

void Engine::set_orbit_camera_reference_up(const glm::vec3& up)
{
    if (!_engine || !_engine->_sceneManager)
    {
        return;
    }

    _engine->_sceneManager->getCameraRig().set_orbit_reference_up(up);
}

} // namespace GameAPI
