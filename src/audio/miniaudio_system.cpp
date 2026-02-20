#include "miniaudio_system.h"

#include <fmt/core.h>
#include "core/util/logger.h"
#include <algorithm>
#include <cmath>

#if defined(VULKAN_ENGINE_AUDIO_LIBVORBIS)
#include <extras/decoders/libvorbis/miniaudio_libvorbis.h>
#endif
#if defined(VULKAN_ENGINE_AUDIO_LIBOPUS)
#include <extras/decoders/libopus/miniaudio_libopus.h>
#endif

namespace Audio
{
    namespace
    {
        constexpr glm::vec3 ZERO_VECTOR{0.0f, 0.0f, 0.0f};
    }

    MiniAudioSystem::~MiniAudioSystem()
    {
        shutdown();
    }

    bool MiniAudioSystem::init()
    {
        if (_initialized)
        {
            return true;
        }

        ma_decoding_backend_vtable *custom_backends[2]{};
        ma_uint32 custom_backend_count = 0;
#if defined(VULKAN_ENGINE_AUDIO_LIBVORBIS)
        custom_backends[custom_backend_count++] = ma_decoding_backend_libvorbis;
#endif
#if defined(VULKAN_ENGINE_AUDIO_LIBOPUS)
        custom_backends[custom_backend_count++] = ma_decoding_backend_libopus;
#endif

        ma_resource_manager_config resource_manager_config = ma_resource_manager_config_init();
        if (custom_backend_count > 0)
        {
            resource_manager_config.ppCustomDecodingBackendVTables = custom_backends;
            resource_manager_config.customDecodingBackendCount = custom_backend_count;
        }

        ma_result result = ma_resource_manager_init(&resource_manager_config, &_resource_manager);
        if (result != MA_SUCCESS)
        {
            Logger::error("[Audio] Failed to initialize miniaudio resource manager (error {})",
                         static_cast<int>(result));
            return false;
        }
        _resource_manager_initialized = true;

        ma_engine_config config = ma_engine_config_init();
        config.listenerCount = 1;
        config.pResourceManager = &_resource_manager;

        result = ma_engine_init(&config, &_engine);
        if (result != MA_SUCCESS)
        {
            Logger::error("[Audio] Failed to initialize miniaudio engine (error {})", static_cast<int>(result));
            ma_resource_manager_uninit(&_resource_manager);
            _resource_manager_initialized = false;
            return false;
        }

        _initialized = true;
        Logger::info("[Audio] miniaudio engine initialized");
        return true;
    }

    void MiniAudioSystem::shutdown()
    {
        if (!_initialized)
        {
            return;
        }

        stop_all();
        clear_preloaded();

        _music_handle = INVALID_SOUND_HANDLE;
        _active_by_handle.clear();
        _active_sounds.clear();

        ma_engine_uninit(&_engine);
        if (_resource_manager_initialized)
        {
            ma_resource_manager_uninit(&_resource_manager);
            _resource_manager_initialized = false;
        }
        _initialized = false;
        Logger::info("[Audio] miniaudio engine shut down");
    }

    void MiniAudioSystem::set_listener(const glm::vec3 &position, const glm::vec3 &forward, const glm::vec3 &up)
    {
        if (!_initialized)
        {
            return;
        }

        ma_engine_listener_set_position(&_engine, 0, position.x, position.y, position.z);
        ma_engine_listener_set_direction(&_engine, 0, forward.x, forward.y, forward.z);
        ma_engine_listener_set_world_up(&_engine, 0, up.x, up.y, up.z);
    }

    GameRuntime::IAudioSystem::SoundHandle MiniAudioSystem::play_3d(const std::string &event,
                                                                    const glm::vec3 &position,
                                                                    Bus bus,
                                                                    float volume,
                                                                    float pitch)
    {
        return play_internal(event, &position, true, false, bus, volume, pitch, false);
    }

    GameRuntime::IAudioSystem::SoundHandle MiniAudioSystem::play_2d(const std::string &event,
                                                                    Bus bus,
                                                                    float volume,
                                                                    float pitch,
                                                                    bool loop)
    {
        return play_internal(event, nullptr, false, loop, bus, volume, pitch, false);
    }

    GameRuntime::IAudioSystem::SoundHandle MiniAudioSystem::play_music(const std::string &event,
                                                                       float volume,
                                                                       bool loop,
                                                                       float fade_in_seconds)
    {
        stop_music(fade_in_seconds > 0.0f ? fade_in_seconds : 0.0f);

        const SoundHandle handle = play_internal(event, nullptr, false, loop, Bus::Bgm, volume, 1.0f, true);
        if (handle == INVALID_SOUND_HANDLE)
        {
            return INVALID_SOUND_HANDLE;
        }

        _music_handle = handle;

        ActiveSound *music = find_sound(handle);
        if (!music)
        {
            return INVALID_SOUND_HANDLE;
        }

        if (fade_in_seconds > 0.0f)
        {
            const float target_volume = effective_volume(*music);
            ma_sound_set_fade_in_milliseconds(&music->sound, 0.0f, target_volume, fade_seconds_to_ms(fade_in_seconds));
        }

        return handle;
    }

    void MiniAudioSystem::stop_music(float fade_out_seconds)
    {
        if (_music_handle == INVALID_SOUND_HANDLE)
        {
            return;
        }

        ActiveSound *music = find_sound(_music_handle);
        if (!music)
        {
            _music_handle = INVALID_SOUND_HANDLE;
            return;
        }

        if (fade_out_seconds > 0.0f)
        {
            ma_sound_stop_with_fade_in_milliseconds(&music->sound, fade_seconds_to_ms(fade_out_seconds));
            music->release_when_stopped = true;
            return;
        }

        remove_sound(_music_handle);
        _music_handle = INVALID_SOUND_HANDLE;
    }

    bool MiniAudioSystem::preload(const std::string &event)
    {
        if (!_initialized || event.empty())
        {
            return false;
        }

        if (_preloaded_sounds.contains(event))
        {
            return true;
        }

        auto cached = std::make_unique<CachedSound>();
        const ma_uint32 flags = MA_SOUND_FLAG_DECODE | MA_SOUND_FLAG_NO_SPATIALIZATION;
        const ma_result result = ma_sound_init_from_file(&_engine, event.c_str(), flags, nullptr, nullptr,
                                                         &cached->sound);
        if (result != MA_SUCCESS)
        {
            Logger::error("[Audio] Failed to preload sound '{}' (error {})", event, static_cast<int>(result));
            return false;
        }

        _preloaded_sounds[event] = std::move(cached);
        return true;
    }

    void MiniAudioSystem::unload(const std::string &event)
    {
        auto it = _preloaded_sounds.find(event);
        if (it == _preloaded_sounds.end())
        {
            return;
        }

        ma_sound_uninit(&it->second->sound);
        _preloaded_sounds.erase(it);
    }

    void MiniAudioSystem::clear_preloaded()
    {
        for (auto &[_, cached]: _preloaded_sounds)
        {
            ma_sound_uninit(&cached->sound);
        }
        _preloaded_sounds.clear();
    }

    void MiniAudioSystem::stop(SoundHandle sound)
    {
        remove_sound(sound);
    }

    void MiniAudioSystem::stop_all()
    {
        for (auto &active: _active_sounds)
        {
            ma_sound_uninit(&active->sound);
        }
        _active_sounds.clear();
        _active_by_handle.clear();
        _music_handle = INVALID_SOUND_HANDLE;
    }

    void MiniAudioSystem::pause(SoundHandle sound, bool paused)
    {
        ActiveSound *active = find_sound(sound);
        if (!active || active->paused == paused)
        {
            return;
        }

        if (paused)
        {
            ma_sound_stop(&active->sound);
            active->paused = true;
            return;
        }

        ma_sound_start(&active->sound);
        active->paused = false;
    }

    bool MiniAudioSystem::is_playing(SoundHandle sound) const
    {
        const ActiveSound *active = find_sound(sound);
        if (!active || active->paused)
        {
            return false;
        }

        return ma_sound_is_playing(&active->sound) == MA_TRUE;
    }

    void MiniAudioSystem::set_sound_volume(SoundHandle sound, float volume)
    {
        ActiveSound *active = find_sound(sound);
        if (!active)
        {
            return;
        }

        active->base_volume = clamp01(volume);
        apply_effective_volume(*active);
    }

    void MiniAudioSystem::set_sound_pitch(SoundHandle sound, float pitch)
    {
        ActiveSound *active = find_sound(sound);
        if (!active)
        {
            return;
        }

        active->pitch = std::max(0.01f, pitch);
        ma_sound_set_pitch(&active->sound, active->pitch);
    }

    void MiniAudioSystem::set_sound_position(SoundHandle sound, const glm::vec3 &position)
    {
        ActiveSound *active = find_sound(sound);
        if (!active)
        {
            return;
        }

        ma_sound_set_position(&active->sound, position.x, position.y, position.z);
    }

    void MiniAudioSystem::set_bus_volume(Bus bus, float volume)
    {
        const size_t index = static_cast<size_t>(bus);
        if (index >= _bus_volumes.size())
        {
            return;
        }

        _bus_volumes[index] = clamp01(volume);
        for (auto &active: _active_sounds)
        {
            if (active->bus == bus)
            {
                apply_effective_volume(*active);
            }
        }
    }

    float MiniAudioSystem::get_bus_volume(Bus bus) const
    {
        const size_t index = static_cast<size_t>(bus);
        if (index >= _bus_volumes.size())
        {
            return 1.0f;
        }

        return _bus_volumes[index];
    }

    void MiniAudioSystem::set_master_volume(float volume)
    {
        _master_volume = clamp01(volume);
        for (auto &active: _active_sounds)
        {
            apply_effective_volume(*active);
        }
    }

    float MiniAudioSystem::master_volume() const
    {
        return _master_volume;
    }

    void MiniAudioSystem::set_master_mute(bool muted)
    {
        if (_master_muted == muted)
        {
            return;
        }

        _master_muted = muted;
        for (auto &active: _active_sounds)
        {
            apply_effective_volume(*active);
        }
    }

    bool MiniAudioSystem::master_mute() const
    {
        return _master_muted;
    }

    GameRuntime::IAudioSystem::SoundHandle MiniAudioSystem::play_internal(const std::string &event,
                                                                          const glm::vec3 *position,
                                                                          bool spatialized,
                                                                          bool loop,
                                                                          Bus bus,
                                                                          float volume,
                                                                          float pitch,
                                                                          bool from_music_call)
    {
        if (!_initialized || event.empty())
        {
            return INVALID_SOUND_HANDLE;
        }

        auto active = std::make_unique<ActiveSound>();
        if (!init_sound_from_source(event, active->sound))
        {
            return INVALID_SOUND_HANDLE;
        }

        active->handle = _next_sound_handle++;
        active->bus = bus;
        active->base_volume = clamp01(volume);
        active->pitch = std::max(0.01f, pitch);
        active->looping = loop;

        ma_sound_set_spatialization_enabled(&active->sound, spatialized ? MA_TRUE : MA_FALSE);
        ma_sound_set_looping(&active->sound, loop ? MA_TRUE : MA_FALSE);
        ma_sound_set_pitch(&active->sound, active->pitch);

        if (spatialized && position)
        {
            ma_sound_set_position(&active->sound, position->x, position->y, position->z);
        }
        else
        {
            ma_sound_set_position(&active->sound, ZERO_VECTOR.x, ZERO_VECTOR.y, ZERO_VECTOR.z);
        }

        apply_effective_volume(*active);

        if (ma_sound_start(&active->sound) != MA_SUCCESS)
        {
            ma_sound_uninit(&active->sound);
            return INVALID_SOUND_HANDLE;
        }

        const SoundHandle handle = active->handle;
        _active_by_handle[handle] = active.get();
        _active_sounds.push_back(std::move(active));

        if (from_music_call)
        {
            _music_handle = handle;
        }

        return handle;
    }

    bool MiniAudioSystem::init_sound_from_source(const std::string &event, ma_sound &out_sound)
    {
        auto cached = _preloaded_sounds.find(event);
        if (cached != _preloaded_sounds.end())
        {
            const ma_result copy_result = ma_sound_init_copy(&_engine, &cached->second->sound, 0, nullptr, &out_sound);
            if (copy_result == MA_SUCCESS)
            {
                ma_sound_seek_to_pcm_frame(&out_sound, 0);
                return true;
            }
        }

        const ma_result result = ma_sound_init_from_file(&_engine, event.c_str(), 0, nullptr, nullptr, &out_sound);
        if (result != MA_SUCCESS)
        {
            Logger::error("[Audio] Failed to load sound '{}' (error {})", event, static_cast<int>(result));
            return false;
        }

        return true;
    }

    MiniAudioSystem::ActiveSound *MiniAudioSystem::find_sound(SoundHandle sound)
    {
        auto it = _active_by_handle.find(sound);
        return it != _active_by_handle.end() ? it->second : nullptr;
    }

    const MiniAudioSystem::ActiveSound *MiniAudioSystem::find_sound(SoundHandle sound) const
    {
        auto it = _active_by_handle.find(sound);
        return it != _active_by_handle.end() ? it->second : nullptr;
    }

    void MiniAudioSystem::remove_sound(SoundHandle sound)
    {
        if (sound == INVALID_SOUND_HANDLE)
        {
            return;
        }

        auto it = std::find_if(_active_sounds.begin(), _active_sounds.end(),
                               [sound](const std::unique_ptr<ActiveSound> &s) {
                                   return s->handle == sound;
                               });
        if (it == _active_sounds.end())
        {
            return;
        }

        ma_sound_uninit(&(*it)->sound);
        _active_by_handle.erase(sound);
        _active_sounds.erase(it);

        if (_music_handle == sound)
        {
            _music_handle = INVALID_SOUND_HANDLE;
        }
    }

    void MiniAudioSystem::apply_effective_volume(ActiveSound &sound)
    {
        ma_sound_set_volume(&sound.sound, effective_volume(sound));
    }

    float MiniAudioSystem::effective_volume(const ActiveSound &sound) const
    {
        const size_t bus_index = static_cast<size_t>(sound.bus);
        const float bus_volume = bus_index < _bus_volumes.size() ? _bus_volumes[bus_index] : 1.0f;
        return _master_muted ? 0.0f : (sound.base_volume * _master_volume * bus_volume);
    }

    float MiniAudioSystem::clamp01(float value)
    {
        return std::clamp(value, 0.0f, 1.0f);
    }

    ma_uint64 MiniAudioSystem::fade_seconds_to_ms(float seconds)
    {
        if (seconds <= 0.0f)
        {
            return 0;
        }

        return static_cast<ma_uint64>(std::round(static_cast<double>(seconds) * 1000.0));
    }

    void MiniAudioSystem::cleanup_finished_sounds()
    {
        std::vector<SoundHandle> pending_remove;
        pending_remove.reserve(_active_sounds.size());

        for (const auto &active: _active_sounds)
        {
            if (active->paused)
            {
                continue;
            }

            if (active->release_when_stopped && ma_sound_is_playing(&active->sound) == MA_FALSE)
            {
                pending_remove.push_back(active->handle);
                continue;
            }

            if (!active->looping && ma_sound_at_end(&active->sound) == MA_TRUE)
            {
                pending_remove.push_back(active->handle);
            }
        }

        for (SoundHandle handle: pending_remove)
        {
            remove_sound(handle);
        }
    }

    void MiniAudioSystem::update()
    {
        if (!_initialized)
        {
            return;
        }

        cleanup_finished_sounds();
    }
} // namespace Audio
