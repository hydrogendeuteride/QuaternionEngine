#pragma once

#include <scene/camera/icamera_mode.h>

struct FollowCameraSettings;

class FollowCameraMode : public ICameraMode
{
public:
    explicit FollowCameraMode(FollowCameraSettings &settings);
    ~FollowCameraMode() override = default;

    const char *name() const override { return "Follow"; }
    void on_activate(SceneManager &scene, Camera &camera) override;
    void process_input(SceneManager &scene,
                       Camera &camera,
                       InputSystem &input,
                       bool ui_capture_keyboard,
                       bool ui_capture_mouse) override;
    void update(SceneManager &scene, Camera &camera, float dt) override;

private:
    FollowCameraSettings &_settings;
};
