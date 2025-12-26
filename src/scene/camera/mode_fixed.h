#pragma once

#include <scene/camera/icamera_mode.h>

struct FixedCameraSettings;

class FixedCameraMode : public ICameraMode
{
public:
    explicit FixedCameraMode(FixedCameraSettings &settings);
    ~FixedCameraMode() override = default;

    const char *name() const override { return "Fixed"; }
    void on_activate(SceneManager &scene, Camera &camera) override;
    void process_input(SceneManager &scene,
                       Camera &camera,
                       InputSystem &input,
                       bool ui_capture_keyboard,
                       bool ui_capture_mouse) override;
    void update(SceneManager &scene, Camera &camera, float dt) override;

private:
    FixedCameraSettings &_settings;
};
