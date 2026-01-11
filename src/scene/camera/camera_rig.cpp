#include "camera_rig.h"

#include <scene/camera/mode_chase.h>
#include <scene/camera/mode_fixed.h>
#include <scene/camera/mode_follow.h>
#include <scene/camera/mode_free.h>
#include <scene/camera/mode_orbit.h>
#include <scene/camera.h>
#include <scene/planet/planet_system.h>
#include <scene/vk_scene.h>

#include <core/input/input_system.h>

#include <algorithm>
#include <cmath>

namespace
{
    std::unique_ptr<ICameraMode> make_mode(CameraMode mode,
                                           FreeCameraSettings &free_settings,
                                           OrbitCameraSettings &orbit_settings,
                                           FollowCameraSettings &follow_settings,
                                           ChaseCameraSettings &chase_settings,
                                           FixedCameraSettings &fixed_settings)
    {
        switch (mode)
        {
            case CameraMode::Free: return std::make_unique<FreeCameraMode>(free_settings);
            case CameraMode::Orbit: return std::make_unique<OrbitCameraMode>(orbit_settings);
            case CameraMode::Follow: return std::make_unique<FollowCameraMode>(follow_settings);
            case CameraMode::Chase: return std::make_unique<ChaseCameraMode>(chase_settings);
            case CameraMode::Fixed: return std::make_unique<FixedCameraMode>(fixed_settings);
            default: return std::make_unique<FreeCameraMode>(free_settings);
        }
    }

    void clamp_camera_above_terrain(SceneManager &scene, Camera &camera, double surface_clearance_m)
    {
        PlanetSystem *planets = scene.get_planet_system();
        if (!planets || !planets->enabled())
        {
            return;
        }

        const double clearance_m = std::max(0.0, surface_clearance_m);

        const PlanetSystem::PlanetBody *best_body = nullptr;
        WorldVec3 best_dir{0.0, 0.0, 1.0};
        double best_min_dist = 0.0;
        double best_penetration = 0.0;

        for (const PlanetSystem::PlanetBody &body : planets->bodies())
        {
            if (!body.visible || !body.terrain || !(body.radius_m > 0.0))
            {
                continue;
            }

            const WorldVec3 to_cam = camera.position_world - body.center_world;
            const double dist2 = glm::dot(to_cam, to_cam);
            const double dist = (dist2 > 0.0) ? std::sqrt(dist2) : 0.0;
            const WorldVec3 dir = (dist > 1.0e-12) ? (to_cam * (1.0 / dist)) : WorldVec3(0.0, 0.0, 1.0);

            const double displacement_m = planets->sample_terrain_displacement_m(body, dir);
            const double min_dist = body.radius_m + displacement_m + clearance_m;
            const double penetration = min_dist - dist;
            if (penetration > best_penetration)
            {
                best_penetration = penetration;
                best_body = &body;
                best_dir = dir;
                best_min_dist = min_dist;
            }
        }

        if (best_body && best_penetration > 0.0)
        {
            camera.position_world = best_body->center_world + best_dir * best_min_dist;
        }
    }
} // namespace

CameraRig::CameraRig() = default;
CameraRig::~CameraRig() = default;

void CameraRig::init(SceneManager &scene, Camera &camera)
{
    _scene = &scene;
    _camera = &camera;
    recreate_mode(scene, camera);
}

void CameraRig::set_mode(CameraMode mode, SceneManager &scene, Camera &camera)
{
    if (_mode == mode)
    {
        return;
    }
    _mode = mode;
    recreate_mode(scene, camera);
}

const char *CameraRig::mode_name() const
{
    if (_mode_impl)
    {
        return _mode_impl->name();
    }
    return "None";
}

void CameraRig::process_input(InputSystem &input, bool ui_capture_keyboard, bool ui_capture_mouse)
{
    if (_mode_impl && _scene && _camera)
    {
        _mode_impl->process_input(*_scene, *_camera, input, ui_capture_keyboard, ui_capture_mouse);
    }
}

void CameraRig::update(SceneManager &scene, Camera &camera, float dt)
{
    if (_mode_impl)
    {
        _mode_impl->update(scene, camera, dt);
    }

    if (_terrain_surface_clamp_enabled)
    {
        clamp_camera_above_terrain(scene, camera, _terrain_surface_clearance_m);
    }
}

bool CameraRig::resolve_target(SceneManager &scene,
                               const CameraTarget &target,
                               WorldVec3 &out_position_world,
                               glm::quat &out_rotation) const
{
    out_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    switch (target.type)
    {
        case CameraTargetType::WorldPoint:
            out_position_world = target.world_point;
            return true;
        case CameraTargetType::MeshInstance:
        {
            WorldVec3 t{};
            glm::quat r{};
            glm::vec3 s{};
            if (!scene.getMeshInstanceTRSWorld(target.name, t, r, s))
            {
                if (PlanetSystem *planets = scene.get_planet_system())
                {
                    if (PlanetSystem::PlanetBody *body = planets->find_body_by_name(target.name))
                    {
                        out_position_world = body->center_world;
                        return true;
                    }
                }
                return false;
            }
            out_position_world = t;
            out_rotation = r;
            return true;
        }
        case CameraTargetType::GLTFInstance:
        {
            WorldVec3 t{};
            glm::quat r{};
            glm::vec3 s{};
            if (!scene.getGLTFInstanceTRSWorld(target.name, t, r, s))
            {
                return false;
            }
            out_position_world = t;
            out_rotation = r;
            return true;
        }
        default:
            return false;
    }
}

void CameraRig::recreate_mode(SceneManager &scene, Camera &camera)
{
    _mode_impl = make_mode(_mode, _free, _orbit, _follow, _chase, _fixed);
    if (_mode_impl)
    {
        _mode_impl->on_activate(scene, camera);
    }
}

void CameraRig::align_orbit_up_to_target()
{
    if (!_scene)
    {
        return;
    }

    WorldVec3 target_pos{};
    glm::quat target_rot{};
    if (!resolve_target(*_scene, _orbit.target, target_pos, target_rot))
    {
        return;
    }

    // Extract the local up vector (Y axis) from the target's rotation
    const glm::vec3 up = target_rot * glm::vec3(0.0f, 1.0f, 0.0f);
    const float len2 = glm::dot(up, up);
    if (std::isfinite(len2) && len2 > 1.0e-12f)
    {
        _orbit.reference_up = up * (1.0f / std::sqrt(len2));
    }
    else
    {
        _orbit.reference_up = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    if (_mode == CameraMode::Orbit && _mode_impl && _camera)
    {
        _mode_impl->on_activate(*_scene, *_camera);
    }
}

void CameraRig::set_orbit_reference_up(const glm::vec3 &up)
{
    const float len2 = glm::dot(up, up);
    if (std::isfinite(len2) && len2 > 1.0e-12f)
    {
        _orbit.reference_up = up * (1.0f / std::sqrt(len2));
    }
    else
    {
        _orbit.reference_up = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    if (_mode == CameraMode::Orbit && _mode_impl && _scene && _camera)
    {
        _mode_impl->on_activate(*_scene, *_camera);
    }
}
