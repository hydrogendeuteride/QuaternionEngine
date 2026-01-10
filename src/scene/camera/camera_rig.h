#pragma once

#include <core/world.h>

#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <memory>
#include <string>

class Camera;
class InputSystem;
class SceneManager;

enum class CameraMode : uint8_t
{
    Free = 0,
    Orbit = 1,
    Follow = 2,
    Chase = 3,
    Fixed = 4
};

enum class CameraTargetType : uint8_t
{
    None = 0,
    WorldPoint = 1,
    MeshInstance = 2,
    GLTFInstance = 3
};

struct CameraTarget
{
    CameraTargetType type{CameraTargetType::None};
    std::string name{};
    WorldVec3 world_point{0.0, 0.0, 0.0};
};

struct FreeCameraSettings
{
    float move_speed{1.8f};      // world units / second
    float look_sensitivity{0.0020f};
    float roll_speed{1.0f};      // radians / second
};

struct OrbitCameraSettings
{
    static constexpr double kMinDistance = 0.2;
    static constexpr double kMaxDistance = 1.0e16;

    CameraTarget target{};
    double distance{10.0};
    float yaw{0.0f};             // radians
    float pitch{0.0f};           // radians
    float look_sensitivity{0.0020f};
    glm::vec3 reference_up{0.0f, 1.0f, 0.0f}; // up vector for orbit frame
};

struct FollowCameraSettings
{
    CameraTarget target{};
    glm::vec3 position_offset_local{0.0f, 2.0f, 6.0f};
    glm::quat rotation_offset{1.0f, 0.0f, 0.0f, 0.0f};
};

struct ChaseCameraSettings
{
    CameraTarget target{};
    glm::vec3 position_offset_local{0.0f, 2.0f, 6.0f};
    glm::quat rotation_offset{1.0f, 0.0f, 0.0f, 0.0f};
    float position_lag{8.0f};    // smoothing rate (1/sec), higher = snappier
    float rotation_lag{10.0f};   // smoothing rate (1/sec)
};

struct FixedCameraSettings
{
};

class CameraRig
{
public:
    CameraRig();
    ~CameraRig();

    CameraRig(const CameraRig &) = delete;
    CameraRig &operator=(const CameraRig &) = delete;

    void init(SceneManager &scene, Camera &camera);

    CameraMode mode() const { return _mode; }
    void set_mode(CameraMode mode, SceneManager &scene, Camera &camera);

    const char *mode_name() const;

    FreeCameraSettings &free_settings() { return _free; }
    OrbitCameraSettings &orbit_settings() { return _orbit; }
    FollowCameraSettings &follow_settings() { return _follow; }
    ChaseCameraSettings &chase_settings() { return _chase; }
    FixedCameraSettings &fixed_settings() { return _fixed; }

    const FreeCameraSettings &free_settings() const { return _free; }
    const OrbitCameraSettings &orbit_settings() const { return _orbit; }
    const FollowCameraSettings &follow_settings() const { return _follow; }
    const ChaseCameraSettings &chase_settings() const { return _chase; }
    const FixedCameraSettings &fixed_settings() const { return _fixed; }

    void process_input(InputSystem &input, bool ui_capture_keyboard, bool ui_capture_mouse);
    void update(SceneManager &scene, Camera &camera, float dt);

    bool terrain_surface_clamp_enabled() const { return _terrain_surface_clamp_enabled; }
    void set_terrain_surface_clamp_enabled(bool enabled) { _terrain_surface_clamp_enabled = enabled; }

    double terrain_surface_clearance_m() const { return _terrain_surface_clearance_m; }
    void set_terrain_surface_clearance_m(double clearance_m) { _terrain_surface_clearance_m = std::max(0.0, clearance_m); }

    bool resolve_target(SceneManager &scene,
                        const CameraTarget &target,
                        WorldVec3 &out_position_world,
                        glm::quat &out_rotation) const;

    void align_orbit_up_to_target();
    void set_orbit_reference_up(const glm::vec3 &up);

private:
    void recreate_mode(SceneManager &scene, Camera &camera);

    CameraMode _mode{CameraMode::Free};
    std::unique_ptr<class ICameraMode> _mode_impl;
    SceneManager *_scene = nullptr;
    Camera *_camera = nullptr;

    FreeCameraSettings _free{};
    OrbitCameraSettings _orbit{};
    FollowCameraSettings _follow{};
    ChaseCameraSettings _chase{};
    FixedCameraSettings _fixed{};

    // Prevent the camera from going below terrain when terrain planets are active.
    // Clearance is measured along the radial direction (meters) and can be set to 0.
    bool _terrain_surface_clamp_enabled = false;
    double _terrain_surface_clearance_m = 0.1;
};
