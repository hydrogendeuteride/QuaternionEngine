#pragma once

#include <scene/camera/icamera_mode.h>

struct OrbitCameraSettings;

class OrbitCameraMode : public ICameraMode
{
public:
    explicit OrbitCameraMode(OrbitCameraSettings &settings);
    ~OrbitCameraMode() override = default;

    const char *name() const override { return "Orbit"; }
    void on_activate(SceneManager &scene, Camera &camera) override;
    void process_input(SceneManager &scene,
                       Camera &camera,
                       InputSystem &input,
                       bool ui_capture_keyboard,
                       bool ui_capture_mouse) override;
    void update(SceneManager &scene, Camera &camera, float dt) override;

private:
    OrbitCameraSettings &_settings;
    bool _rmb_down = false;
};
