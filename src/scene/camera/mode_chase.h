#pragma once

#include <scene/camera/icamera_mode.h>

struct ChaseCameraSettings;

class ChaseCameraMode : public ICameraMode
{
public:
    explicit ChaseCameraMode(ChaseCameraSettings &settings);
    ~ChaseCameraMode() override = default;

    const char *name() const override { return "Chase"; }
    void on_activate(SceneManager &scene, Camera &camera) override;
    void process_input(SceneManager &scene,
                       Camera &camera,
                       InputSystem &input,
                       bool ui_capture_keyboard,
                       bool ui_capture_mouse) override;
    void update(SceneManager &scene, Camera &camera, float dt) override;

private:
    ChaseCameraSettings &_settings;
};
