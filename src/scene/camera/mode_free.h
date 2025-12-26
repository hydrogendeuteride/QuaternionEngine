#pragma once

#include <scene/camera/icamera_mode.h>

struct FreeCameraSettings;

class FreeCameraMode : public ICameraMode
{
public:
    explicit FreeCameraMode(FreeCameraSettings &settings);
    ~FreeCameraMode() override = default;

    const char *name() const override { return "Free"; }
    void on_activate(SceneManager &scene, Camera &camera) override;
    void process_input(SceneManager &scene,
                       Camera &camera,
                       InputSystem &input,
                       bool ui_capture_keyboard,
                       bool ui_capture_mouse) override;
    void update(SceneManager &scene, Camera &camera, float dt) override;

private:
    FreeCameraSettings &_settings;
    glm::vec3 _velocity{0.0f, 0.0f, 0.0f};
    float _roll_dir = 0.0f;
    bool _rmb_down = false;
};
