#include "mode_fixed.h"

#include <scene/camera/camera_rig.h>

FixedCameraMode::FixedCameraMode(FixedCameraSettings &settings)
    : _settings(settings)
{
}

void FixedCameraMode::on_activate(SceneManager & /*scene*/, Camera & /*camera*/)
{
}

void FixedCameraMode::process_input(SceneManager & /*scene*/,
                                    Camera & /*camera*/,
                                    InputSystem & /*input*/,
                                    bool /*ui_capture_keyboard*/,
                                    bool /*ui_capture_mouse*/)
{
}

void FixedCameraMode::update(SceneManager & /*scene*/, Camera & /*camera*/, float /*dt*/)
{
    (void)_settings;
}
