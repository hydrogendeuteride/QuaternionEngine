#pragma once

#include "runtime/game_runtime.h"

#include <miniaudio.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Audio
{
    /**
     * @brief IAudioSystem implementation backed by miniaudio.
     *
     * Manages an ma_engine with a single 3D listener, optional custom
     * decoders (libvorbis / libopus), and a resource manager for async
     * file decoding.
     *
     * Lifecycle: construct -> init() -> [use] -> shutdown() (or destructor).
     * Not copyable; intended as a single long-lived instance.
     */
    class MiniAudioSystem : public GameRuntime::IAudioSystem
    {
    public:
        MiniAudioSystem() = default;
        ~MiniAudioSystem() override;
        MiniAudioSystem(const MiniAudioSystem &) = delete;

        MiniAudioSystem &operator=(const MiniAudioSystem &) = delete;

        /**
         * @brief Initialize miniaudio resource manager and engine.
         *
         * Sets up custom decoding backends (libvorbis/libopus) if compiled in,
         * then creates the ma_engine with one 3D listener.
         *
         * @return true on success, false if engine init fails.
         */
        bool init();

        /** @brief Tear down all sounds, preloaded cache, and the engine. */
        void shutdown();

        // -- IAudioSystem overrides ------------------------------------------
        void set_listener(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &up) override;
        SoundHandle play_3d(const std::string &event,
                            const glm::vec3 &position,
                            Bus bus,
                            float volume,
                            float pitch) override;
        SoundHandle play_2d(const std::string &event, Bus bus, float volume, float pitch, bool loop) override;
        SoundHandle play_music(const std::string &event, float volume, bool loop, float fade_in_seconds) override;
        void stop_music(float fade_out_seconds) override;
        bool preload(const std::string &event) override;
        void unload(const std::string &event) override;
        void clear_preloaded() override;
        void stop(SoundHandle sound) override;
        void stop_all() override;
        void pause(SoundHandle sound, bool paused) override;
        bool is_playing(SoundHandle sound) const override;
        void set_sound_volume(SoundHandle sound, float volume) override;
        void set_sound_pitch(SoundHandle sound, float pitch) override;
        void set_sound_position(SoundHandle sound, const glm::vec3 &position) override;
        void set_bus_volume(Bus bus, float volume) override;
        float get_bus_volume(Bus bus) const override;
        void set_master_volume(float volume) override;
        float master_volume() const override;
        void set_master_mute(bool muted) override;
        bool master_mute() const override;
        void update() override;

    private:
        struct ActiveSound;
        struct CachedSound;

        /** @brief Remove finished one-shot and fade-out-completed sounds. Called every frame from update(). */
        void cleanup_finished_sounds();
        ActiveSound *find_sound(SoundHandle sound);
        const ActiveSound *find_sound(SoundHandle sound) const;
        void remove_sound(SoundHandle sound);

        /**
         * @brief Common path for all play_*() methods.
         *
         * Loads (or copies from preloaded cache), configures spatialization/
         * looping/pitch/volume, starts playback, and registers the sound.
         *
         * @param event         File path or asset name.
         * @param position      World position (nullptr for 2D sounds).
         * @param spatialized   Enable 3D attenuation.
         * @param from_music_call If true, stores the handle as the current music track.
         * @return New SoundHandle, or INVALID_SOUND_HANDLE on failure.
         */
        SoundHandle play_internal(const std::string &event,
                                  const glm::vec3 *position,
                                  bool spatialized,
                                  bool loop,
                                  Bus bus,
                                  float volume,
                                  float pitch,
                                  bool from_music_call);

        /**
         * @brief Create an ma_sound from a preloaded cache entry or from file.
         *
         * Tries ma_sound_init_copy from the preloaded cache first (zero-copy
         * decoded data), falls back to ma_sound_init_from_file on cache miss.
         *
         * @param[out] out_sound Initialized ma_sound on success.
         * @return true on success.
         */
        bool init_sound_from_source(const std::string &event, ma_sound &out_sound);

        /** @brief Write effective_volume() result to the underlying ma_sound. */
        void apply_effective_volume(ActiveSound &sound);

        /** @brief Compute final volume: base_volume * master * bus (0 if muted). */
        float effective_volume(const ActiveSound &sound) const;

        static float clamp01(float value);
        static ma_uint64 fade_seconds_to_ms(float seconds);

        ma_engine _engine{};
        ma_resource_manager _resource_manager{};
        bool _initialized{false};
        bool _resource_manager_initialized{false};

        /** @brief Runtime state for a single playing sound instance. */
        struct ActiveSound
        {
            SoundHandle handle{INVALID_SOUND_HANDLE};
            ma_sound sound{};                  ///< Underlying miniaudio sound object.
            Bus bus{Bus::Sfx};                 ///< Mix bus this sound is routed to.
            float base_volume{1.0f};           ///< Per-sound volume before bus/master scaling.
            float pitch{1.0f};
            bool looping{false};
            bool paused{false};
            bool release_when_stopped{false};  ///< If true, auto-remove when playback ends (used for fade-out).
        };

        /** @brief Preloaded (decoded) sound kept in memory for fast copy-play. */
        struct CachedSound
        {
            ma_sound sound{};
        };

        std::vector<std::unique_ptr<ActiveSound> > _active_sounds;             ///< Owns all active sounds.
        std::unordered_map<SoundHandle, ActiveSound *> _active_by_handle;      ///< O(1) handle lookup.
        std::unordered_map<std::string, std::unique_ptr<CachedSound> > _preloaded_sounds; ///< Preloaded cache keyed by event name.

        std::array<float, static_cast<size_t>(Bus::Count)> _bus_volumes{1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        float _master_volume{1.0f};
        bool _master_muted{false};

        SoundHandle _next_sound_handle{1};                    ///< Monotonically increasing handle counter.
        SoundHandle _music_handle{INVALID_SOUND_HANDLE};      ///< Currently playing BGM handle (at most one).
    };
} // namespace Audio
