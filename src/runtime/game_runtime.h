#pragma once

// GameRuntime: game loop manager
#include "i_game_callbacks.h"
#include "time_manager.h"
#include "core/game_api.h"

#include <cstdint>
#include <memory>
#include <string>

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
// audio
// ============================================================================

/**
 * @brief Abstract audio system interface.
 *
 * Provides backend-agnostic API for sound playback, 3D spatialization,
 * bus-based volume mixing, and resource preloading.
 *
 * Volume hierarchy (multiplicative): master_volume * bus_volume * base_volume.
 * Master mute overrides all output to silence.
 *
 * Implementations must call cleanup logic in update() every frame to
 * release finished one-shot sounds.
 */
class IAudioSystem
{
public:
    /** @brief Opaque handle identifying an active sound instance. */
    using SoundHandle = uint64_t;
    static constexpr SoundHandle INVALID_SOUND_HANDLE = 0;

    /**
     * @brief Audio mix bus categories.
     *
     * Each bus has an independent volume level. Sounds are routed to
     * exactly one bus at play time.
     */
    enum class Bus : uint8_t
    {
        Sfx = 0,  ///< Sound effects (footsteps, impacts, etc.)
        Ui,       ///< UI feedback (clicks, hovers)
        Voice,    ///< Voice / dialogue
        Bgm,      ///< Background music
        Ambience, ///< Ambient loops (wind, rain)
        Count
    };

    virtual ~IAudioSystem() = default;

    /**
     * @brief Update the 3D listener transform (typically matches the camera).
     * @param position  World-space position.
     * @param forward   Normalized forward direction.
     * @param up        Normalized world-up vector.
     */
    virtual void set_listener(const glm::vec3& position, const glm::vec3& forward, const glm::vec3& up) = 0;

    /**
     * @brief Play a spatialized 3D sound at a world position.
     * @param event    File path or asset name of the sound.
     * @param position World-space emission position.
     * @return Handle to the playing sound, or INVALID_SOUND_HANDLE on failure.
     */
    virtual SoundHandle play_3d(const std::string& event,
                                const glm::vec3& position,
                                Bus bus = Bus::Sfx,
                                float volume = 1.0f,
                                float pitch = 1.0f) = 0;

    /**
     * @brief Play a non-spatialized 2D sound (UI, ambient loops, etc.).
     * @param event File path or asset name of the sound.
     * @param loop  If true, the sound loops until explicitly stopped.
     * @return Handle to the playing sound, or INVALID_SOUND_HANDLE on failure.
     */
    virtual SoundHandle play_2d(const std::string& event,
                                Bus bus = Bus::Sfx,
                                float volume = 1.0f,
                                float pitch = 1.0f,
                                bool loop = false) = 0;

    /**
     * @brief Play background music, replacing any currently playing BGM.
     *
     * Only one music track is active at a time. Calling this while music
     * is already playing will stop the previous track (with optional crossfade).
     *
     * @param event          File path or asset name.
     * @param fade_in_seconds Fade-in duration; 0 for instant start.
     * @return Handle to the music sound, or INVALID_SOUND_HANDLE on failure.
     */
    virtual SoundHandle play_music(const std::string& event,
                                   float volume = 1.0f,
                                   bool loop = true,
                                   float fade_in_seconds = 0.0f) = 0;

    /**
     * @brief Stop the currently playing music track.
     * @param fade_out_seconds Fade-out duration; 0 for immediate stop.
     */
    virtual void stop_music(float fade_out_seconds = 0.0f) = 0;

    /**
     * @brief Decode and cache a sound in memory for low-latency playback.
     * @param event File path or asset name.
     * @return true if preloaded (or already cached), false on error.
     */
    virtual bool preload(const std::string& event) = 0;
    virtual void unload(const std::string& event) = 0;
    virtual void clear_preloaded() = 0;

    virtual void stop(SoundHandle sound) = 0;
    virtual void stop_all() = 0;
    virtual void pause(SoundHandle sound, bool paused) = 0;
    virtual bool is_playing(SoundHandle sound) const = 0;

    /// @name Per-sound properties
    /// @{
    virtual void set_sound_volume(SoundHandle sound, float volume) = 0;
    virtual void set_sound_pitch(SoundHandle sound, float pitch) = 0;
    virtual void set_sound_position(SoundHandle sound, const glm::vec3& position) = 0;
    /// @}

    /// @name Bus volume (0.0 ~ 1.0)
    /// @{
    virtual void set_bus_volume(Bus bus, float volume) = 0;
    virtual float get_bus_volume(Bus bus) const = 0;
    /// @}

    /// @name Master volume and mute
    /// @{
    virtual void set_master_volume(float volume) = 0;
    virtual float master_volume() const = 0;
    virtual void set_master_mute(bool muted) = 0;
    virtual bool master_mute() const = 0;
    /// @}

    /** @brief Per-frame tick. Cleans up finished sounds and processes fades. */
    virtual void update() = 0;
};

} // namespace GameRuntime
