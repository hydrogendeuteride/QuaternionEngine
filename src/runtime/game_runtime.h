#pragma once

// GameRuntime: High-level game loop manager
// Provides a clean separation between engine and game logic with proper
// time management, fixed timestep for physics, and game callbacks.

#include "i_game_callbacks.h"
#include "time_manager.h"
#include "core/game_api.h"

#include <memory>

class VulkanEngine;

namespace GameRuntime
{

// Forward declarations for future integrations
class IPhysicsWorld;
class IAudioSystem;

class Runtime
{
public:
    explicit Runtime(VulkanEngine* renderer);
    ~Runtime();

    // Non-copyable
    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    // ------------------------------------------------------------------------
    // External System Integration (optional)
    // ------------------------------------------------------------------------

    // Set physics world (e.g., Jolt, Bullet, PhysX wrapper)
    void set_physics_world(IPhysicsWorld* physics);
    IPhysicsWorld* physics() const { return _physics; }

    // Set audio system (e.g., FMOD, OpenAL wrapper)
    void set_audio_system(IAudioSystem* audio);
    IAudioSystem* audio() const { return _audio; }

    // ------------------------------------------------------------------------
    // Time Management
    // ------------------------------------------------------------------------

    // Get time manager for direct access
    TimeManager& time() { return _time; }
    const TimeManager& time() const { return _time; }

    // Convenience accessors
    float delta_time() const { return _time.delta_time(); }
    float fixed_delta_time() const { return _time.fixed_delta_time(); }
    float time_scale() const { return _time.time_scale(); }
    void set_time_scale(float scale) { _time.set_time_scale(scale); }
    void set_fixed_delta_time(float dt) { _time.set_fixed_delta_time(dt); }

    // ------------------------------------------------------------------------
    // Game API Access
    // ------------------------------------------------------------------------

    // Get the high-level game API for engine interaction
    GameAPI::Engine& api() { return *_api; }
    const GameAPI::Engine& api() const { return *_api; }

    // Get the underlying Vulkan engine (for advanced use)
    VulkanEngine* renderer() const { return _renderer; }

    // ------------------------------------------------------------------------
    // Main Loop
    // ------------------------------------------------------------------------

    // Run the game loop with the given callback handler.
    // Blocks until the game exits.
    void run(IGameCallbacks* game);

    // Request quit (sets quit flag, loop will exit next frame)
    void request_quit() { _quit_requested = true; }

    // Check if quit was requested
    bool quit_requested() const { return _quit_requested; }

private:
    VulkanEngine* _renderer{nullptr};
    std::unique_ptr<GameAPI::Engine> _api;
    TimeManager _time;

    IPhysicsWorld* _physics{nullptr};
    IAudioSystem* _audio{nullptr};

    bool _quit_requested{false};

    // Internal helpers
    void sync_physics_to_render();
    void update_audio_listener();
};

// ============================================================================
// Physics Interface (for future integration)
// ============================================================================

class IPhysicsWorld
{
public:
    virtual ~IPhysicsWorld() = default;

    // Step the physics simulation by dt seconds
    virtual void step(float dt) = 0;

    // Get transform of a physics body by ID
    virtual void get_body_transform(uint32_t id, glm::mat4& out) = 0;

    // Raycast into the physics world
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
// Audio Interface (for future integration)
// ============================================================================

class IAudioSystem
{
public:
    virtual ~IAudioSystem() = default;

    // Set listener position and orientation
    virtual void set_listener(const glm::vec3& position, const glm::vec3& forward, const glm::vec3& up) = 0;

    // Play a 3D sound at a position
    virtual void play_3d(const std::string& event, const glm::vec3& position) = 0;

    // Update audio system (call once per frame)
    virtual void update() = 0;
};

} // namespace GameRuntime