#pragma once

class Camera;
class InputSystem;
class SceneManager;

class ICameraMode
{
public:
    virtual ~ICameraMode() = default;

    virtual const char *name() const = 0;
    virtual void on_activate(SceneManager &scene, Camera &camera) = 0;
    virtual void process_input(SceneManager &scene,
                               Camera &camera,
                               InputSystem &input,
                               bool ui_capture_keyboard,
                               bool ui_capture_mouse) = 0;
    virtual void update(SceneManager &scene, Camera &camera, float dt) = 0;
};
