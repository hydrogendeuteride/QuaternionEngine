#pragma once

// IGameCallbacks: Interface for game logic callbacks
// Implement this interface and pass to GameRuntime::run() to receive game loop events.

namespace GameRuntime
{

class Runtime;

class IGameCallbacks
{
public:
    virtual ~IGameCallbacks() = default;

    // Called once after runtime initialization, before the first update.
    // Use this to load initial assets, spawn entities, set up the camera, etc.
    virtual void on_init(Runtime& runtime) = 0;

    // Called every frame with variable delta time.
    // Use for rendering-dependent logic, input handling, camera control, etc.
    // @param dt: Frame delta time in seconds (clamped to 0.0-0.1)
    virtual void on_update(float dt) = 0;

    // Called at fixed intervals for physics/simulation.
    // Use for physics updates, AI tick, game state simulation, etc.
    // @param fixed_dt: Fixed delta time in seconds (typically 1/60)
    virtual void on_fixed_update(float fixed_dt) = 0;

    // Called once before shutdown.
    // Use for cleanup, saving state, etc.
    virtual void on_shutdown() = 0;
};

} // namespace GameRuntime