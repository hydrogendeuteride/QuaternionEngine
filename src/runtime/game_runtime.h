#pragma once

// GameRuntime: game loop manager
#include "i_game_callbacks.h"
#include "time_manager.h"
#include "core/game_api.h"

#include <memory>

class VulkanEngine;

namespace GameRuntime
{

class IPhysicsWorld;
class IAudioSystem;

class Runtime
{
public:
    explicit Runtime(VulkanEngine* renderer);
    ~Runtime();

    // singleton
    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    // ------------------------------------------------------------------------
    // physics, audio, etcs
    // ------------------------------------------------------------------------

    void set_physics_world(IPhysicsWorld* physics);
    IPhysicsWorld* physics() const { return _physics; }

    void set_audio_system(IAudioSystem* audio);
    IAudioSystem* audio() const { return _audio; }

    // ------------------------------------------------------------------------
    // Time Management
    // ------------------------------------------------------------------------

    TimeManager& time() { return _time; }
    const TimeManager& time() const { return _time; }

    float delta_time() const { return _time.delta_time(); }
    float fixed_delta_time() const { return _time.fixed_delta_time(); }
    float time_scale() const { return _time.time_scale(); }
    void set_time_scale(float scale) { _time.set_time_scale(scale); }
    void set_fixed_delta_time(float dt) { _time.set_fixed_delta_time(dt); }
    float interpolation_alpha() const { return _time.interpolation_alpha(); }

    // ------------------------------------------------------------------------
    // Game API Access
    // ------------------------------------------------------------------------

    GameAPI::Engine& api() { return *_api; }
    const GameAPI::Engine& api() const { return *_api; }

    VulkanEngine* renderer() const { return _renderer; }

    // ------------------------------------------------------------------------
    // Main Loop
    // ------------------------------------------------------------------------

    void run(IGameCallbacks* game);

    void request_quit() { _quit_requested = true; }

    bool quit_requested() const { return _quit_requested; }

private:
    VulkanEngine* _renderer{nullptr};
    std::unique_ptr<GameAPI::Engine> _api;
    TimeManager _time;

    IPhysicsWorld* _physics{nullptr};
    IAudioSystem* _audio{nullptr};

    bool _quit_requested{false};

    // Internal helpers
    void update_audio_listener();
};

// ============================================================================
// physics
// ============================================================================

class IPhysicsWorld
{
public:
    virtual ~IPhysicsWorld() = default;

    virtual void step(float dt) = 0;

    virtual void get_body_transform(uint32_t id, glm::mat4& out) = 0;

    struct RayHit
    {
        bool hit{false};
        glm::vec3 position{0.0f};
        glm::vec3 normal{0.0f};
        float distance{0.0f};
        uint32_t bodyId{0};
    };
    virtual RayHit raycast(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) = 0;
};

// ============================================================================
// audio for future
// ============================================================================

class IAudioSystem
{
public:
    virtual ~IAudioSystem() = default;

    virtual void set_listener(const glm::vec3& position, const glm::vec3& forward, const glm::vec3& up) = 0;

    virtual void play_3d(const std::string& event, const glm::vec3& position) = 0;

    virtual void update() = 0;
};

} // namespace GameRuntime
