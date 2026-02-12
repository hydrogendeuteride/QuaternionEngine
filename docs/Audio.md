## Audio System: Spatial Sound, Bus Mixing, and Music Management

Audio playback system built on miniaudio with 3D spatialization, preloading, bus-based mixing, and music management. Lives under `src/audio` and is exposed via `Runtime::getAudioSystem()`.

### Concepts

- **Spatial Audio**: 3D positioned sounds with attenuation based on listener position/orientation.
- **Bus Mixing**: Five independent mix buses (Sfx, Ui, Voice, Bgm, Ambience) with per-bus volume control.
- **Preloading**: Decode audio files to memory for instant, zero-copy playback.
- **Music Management**: Single-track BGM playback with fade in/out transitions.
- **Resource Manager**: Async file decoding via miniaudio's resource manager with custom decoders (libvorbis/libopus).

### Key Types

#### `SoundHandle` — Opaque Sound Instance Identifier

```c++
using SoundHandle = uint64_t;
static constexpr SoundHandle INVALID_SOUND_HANDLE = 0;
```

Monotonically increasing handle returned by all `play_*()` methods. Used to control runtime properties (volume, pitch, position, pause, stop).

#### `Bus` — Mix Bus Categories

```c++
enum class Bus : uint8_t {
    Sfx = 0,   // Sound effects (footsteps, impacts, weapon fire)
    Ui,        // UI feedback (clicks, hovers)
    Voice,     // Voice / dialogue
    Bgm,       // Background music
    Ambience,  // Ambient loops (wind, rain, machinery)
    Count
};
```

Each bus has an independent volume multiplier. Sounds are routed to exactly one bus at play time. Final volume is computed as:

```
effective_volume = base_volume * bus_volume * master_volume * (master_muted ? 0 : 1)
```

#### `ActiveSound` — Runtime Sound State

Internal struct holding:
- `SoundHandle handle` — unique identifier
- `ma_sound sound` — underlying miniaudio sound object
- `Bus bus` — assigned mix bus
- `float base_volume` — per-sound volume before bus/master scaling
- `float pitch` — playback rate multiplier
- `bool looping` — loop flag
- `bool paused` — pause state
- `bool release_when_stopped` — auto-cleanup flag (used for fade-out)

#### `CachedSound` — Preloaded Sound

Holds a decoded `ma_sound` in memory. When a preloaded sound is played, the system uses `ma_sound_init_copy()` to create a zero-copy instance that shares the decoded data, avoiding disk I/O and decoding overhead.

### API Surface

#### Initialization

| Method | Description |
|--------|-------------|
| `bool init()` | Initialize resource manager and engine with custom decoders |
| `void shutdown()` | Stop all sounds, free preloaded cache, uninit engine |

#### Listener (3D Audio)

| Method | Description |
|--------|-------------|
| `void set_listener(position, forward, up)` | Update 3D listener position and orientation (typically camera) |

Listener state is used for all spatialized sounds (3D). 2D sounds ignore listener position.

#### Playback

| Method | Description |
|--------|-------------|
| `SoundHandle play_3d(event, position, bus, volume, pitch)` | Play spatially positioned sound with attenuation |
| `SoundHandle play_2d(event, bus, volume, pitch, loop)` | Play non-spatial sound (UI, etc.) |
| `SoundHandle play_music(event, volume, loop, fade_in_sec)` | Play BGM track (auto-stops previous music) |
| `void stop_music(fade_out_sec)` | Stop current music with optional fade out |

**Parameters:**
- `event`: File path (e.g., `"assets/sounds/explosion.wav"`) or asset name
- `position`: World-space 3D position (3D sounds only)
- `bus`: Target mix bus
- `volume`: Base volume [0.0, 1.0]
- `pitch`: Playback rate multiplier (1.0 = normal speed)
- `loop`: Loop playback flag
- `fade_in_sec` / `fade_out_sec`: Fade duration in seconds

#### Preloading

| Method | Description |
|--------|-------------|
| `bool preload(event)` | Decode and cache sound in memory |
| `void unload(event)` | Remove sound from cache |
| `void clear_preloaded()` | Clear entire preload cache |

Preloading is useful for frequently played sounds (e.g., footsteps, weapon fire) to eliminate disk I/O and decoding latency at play time. The system automatically uses the preloaded version if available, falling back to on-demand file loading on cache miss.

#### Runtime Control

| Method | Description |
|--------|-------------|
| `void stop(SoundHandle)` | Immediately stop and remove sound |
| `void stop_all()` | Stop and remove all active sounds |
| `void pause(SoundHandle, bool paused)` | Pause or resume sound |
| `bool is_playing(SoundHandle)` | Check if sound is currently playing |
| `void set_sound_volume(SoundHandle, volume)` | Adjust per-sound base volume |
| `void set_sound_pitch(SoundHandle, pitch)` | Adjust playback rate |
| `void set_sound_position(SoundHandle, position)` | Update 3D position (spatialized sounds only) |

#### Mix Bus Control

| Method | Description |
|--------|-------------|
| `void set_bus_volume(Bus, volume)` | Set bus volume multiplier [0.0, 1.0] |
| `float get_bus_volume(Bus)` | Get current bus volume |

Changing bus volume immediately updates all active sounds on that bus.

#### Master Control

| Method | Description |
|--------|-------------|
| `void set_master_volume(float)` | Set global volume multiplier [0.0, 1.0] |
| `float master_volume()` | Get current master volume |
| `void set_master_mute(bool)` | Instant mute/unmute all audio |
| `bool master_mute()` | Get current mute state |

#### Frame Update

| Method | Description |
|--------|-------------|
| `void update()` | Cleanup finished one-shot sounds (call every frame) |

Automatically removes:
- Non-looping sounds that have reached end of file
- Sounds marked `release_when_stopped` that have stopped (used for fade-out)

### Implementation Details

#### Internal Architecture

```
MiniAudioSystem
├── ma_resource_manager    — async file decoding, custom decoder backends
├── ma_engine              — audio engine with one 3D listener
├── _active_sounds         — vector of unique_ptr<ActiveSound>
├── _active_by_handle      — unordered_map for O(1) handle lookup
├── _preloaded_sounds      — unordered_map of decoded CachedSound
├── _bus_volumes[5]        — per-bus volume multipliers
├── _master_volume         — global volume multiplier
├── _master_muted          — global mute flag
└── _music_handle          — currently playing BGM handle (at most one)
```

#### Custom Decoders

The system supports miniaudio's custom decoding backend API for efficient format support:

- **libvorbis** (`VULKAN_ENGINE_AUDIO_LIBVORBIS`): OGG Vorbis decoder
- **libopus** (`VULKAN_ENGINE_AUDIO_LIBOPUS`): Opus decoder

Custom backends are registered with the resource manager at init time if compiled in.

#### Sound Lifecycle

```
play_internal(event, ...)
  ├─ init_sound_from_source(event)
  │   ├─ check _preloaded_sounds cache
  │   │   └─ if hit: ma_sound_init_copy() (zero-copy)
  │   └─ if miss: ma_sound_init_from_file() (load and decode)
  ├─ configure spatialization, looping, pitch
  ├─ apply_effective_volume()
  │   └─ ma_sound_set_volume(base_volume * bus_volume * master_volume)
  ├─ ma_sound_start()
  └─ register in _active_sounds + _active_by_handle

update() (every frame)
  └─ cleanup_finished_sounds()
      ├─ check ma_sound_at_end() for non-looping sounds
      ├─ check ma_sound_is_playing() for release_when_stopped sounds
      └─ remove_sound() for finished instances
```

#### Music Management

Only one music track can play at a time. Calling `play_music()` automatically stops the previous track:

```c++
play_music(new_event, ...)
  ├─ stop_music(fade_out_if_fade_in_requested)
  ├─ play_internal(..., from_music_call=true)
  ├─ store handle in _music_handle
  └─ ma_sound_set_fade_in_milliseconds(0.0 → target_volume)
```

Fade-out is implemented via `ma_sound_stop_with_fade_in_milliseconds()` and sets `release_when_stopped = true` so the sound is auto-removed when the fade completes.

#### Volume Computation

Final volume is computed every time base volume, bus volume, or master volume changes:

```c++
float effective_volume(const ActiveSound& sound) const {
    const float bus_volume = _bus_volumes[sound.bus];
    return _master_muted ? 0.0f : (sound.base_volume * _master_volume * bus_volume);
}

void apply_effective_volume(ActiveSound& sound) {
    ma_sound_set_volume(&sound.sound, effective_volume(sound));
}
```

Changing master volume or bus volume triggers `apply_effective_volume()` on all affected sounds.

### Integration

#### Runtime Ownership

`MiniAudioSystem` is owned by `Runtime` and initialized during `Runtime::Runtime()`:

```c++
// src/runtime/game_runtime.cpp
Runtime::Runtime(VulkanEngine* renderer) {
    _audio_system = std::make_unique<MiniAudioSystem>();
    if (!_audio_system->init()) {
        fmt::println("[Runtime] Audio system init failed");
    }
}
```

#### Game Loop Integration

```c++
// src/core/engine.cpp
void VulkanEngine::run() {
    while (!glfwWindowShouldClose(_window)) {
        // ... frame setup ...

        runtime->getAudioSystem()->update();  // cleanup finished sounds

        // ... render ...
    }
}
```

#### Listener Sync

Typically updated in camera update or game tick:

```c++
void Camera::update_audio_listener() {
    auto* audio = Runtime::get()->getAudioSystem();
    audio->set_listener(position, forward, up);
}
```

### Usage Examples

#### Basic 3D Sound Effect

```c++
void Player::playFootstep() {
    auto* audio = Runtime::get()->getAudioSystem();
    SoundHandle h = audio->play_3d(
        "assets/sounds/footstep_concrete.ogg",
        position,
        Bus::Sfx,
        0.8f,   // volume
        1.0f    // pitch
    );
    // handle can be ignored for fire-and-forget one-shots
}
```

#### Looping Engine Sound with Pitch Control

```c++
class Vehicle {
    SoundHandle engine_sound = INVALID_SOUND_HANDLE;

    void start_engine() {
        auto* audio = Runtime::get()->getAudioSystem();
        engine_sound = audio->play_2d(
            "assets/sounds/engine_loop.wav",
            Bus::Sfx,
            1.0f,   // volume
            1.0f,   // pitch
            true    // loop
        );
    }

    void update(float speed) {
        auto* audio = Runtime::get()->getAudioSystem();
        if (engine_sound != INVALID_SOUND_HANDLE) {
            // Pitch based on speed: 0.5x at idle, 2.0x at max speed
            float pitch = 0.5f + (speed / max_speed) * 1.5f;
            audio->set_sound_pitch(engine_sound, pitch);
            audio->set_sound_position(engine_sound, position);
        }
    }

    void stop_engine() {
        auto* audio = Runtime::get()->getAudioSystem();
        audio->stop(engine_sound);
        engine_sound = INVALID_SOUND_HANDLE;
    }
};
```

#### Music Crossfade

```c++
class MusicController {
    void transition_to_combat() {
        auto* audio = Runtime::get()->getAudioSystem();
        audio->play_music(
            "assets/music/combat_theme.ogg",
            0.6f,   // volume
            true,   // loop
            3.0f    // fade in over 3 seconds
        );
        // Previous exploration music auto-fades out
    }

    void return_to_exploration() {
        auto* audio = Runtime::get()->getAudioSystem();
        audio->play_music(
            "assets/music/exploration_ambient.ogg",
            0.5f,
            true,
            2.0f
        );
    }
};
```

#### Preload Level Audio

```c++
class Level {
    void load_audio() {
        auto* audio = Runtime::get()->getAudioSystem();

        // Preload frequently used sounds
        audio->preload("assets/sounds/explosion.wav");
        audio->preload("assets/sounds/laser_shot.ogg");
        audio->preload("assets/sounds/enemy_death.wav");

        // Play BGM (not preloaded — streamed from disk)
        audio->play_music("assets/music/level1_theme.ogg", 0.7f, true, 1.5f);
    }

    void unload_audio() {
        auto* audio = Runtime::get()->getAudioSystem();
        audio->stop_music(1.0f);  // fade out
        audio->clear_preloaded();
    }
};
```

#### Settings Menu Integration

```c++
class SettingsMenu {
    void apply_audio_settings(const AudioSettings& settings) {
        auto* audio = Runtime::get()->getAudioSystem();

        audio->set_master_volume(settings.master_volume);
        audio->set_bus_volume(Bus::Sfx, settings.sfx_volume);
        audio->set_bus_volume(Bus::Bgm, settings.music_volume);
        audio->set_bus_volume(Bus::Voice, settings.voice_volume);
        audio->set_bus_volume(Bus::Ui, settings.ui_volume);
        audio->set_bus_volume(Bus::Ambience, settings.ambient_volume);

        audio->set_master_mute(settings.muted);
    }
};
```

### Thread Safety

**Not thread-safe.** All methods must be called from the main thread. Miniaudio's resource manager handles async file I/O internally, but the `MiniAudioSystem` API is not designed for concurrent access.

### Performance Considerations

1. **Preloading**: Decode commonly used sounds (< 1s duration) to eliminate disk I/O and decoding overhead at play time. For long tracks (music, ambient loops), stream from disk instead.

2. **Spatialization Cost**: 3D sounds incur additional CPU cost for distance attenuation and panning. Use 2D sounds for UI and non-positional effects.

3. **Active Sound Limit**: No hard limit, but hundreds of simultaneous sounds will increase CPU usage. Consider culling distant/inaudible sounds or using a priority system.

4. **Fade Performance**: Fades are handled by miniaudio internally and are very cheap.

5. **Cleanup**: Call `update()` every frame to promptly remove finished one-shots. Stale sounds consume memory and lookup overhead.

### File Format Support

Miniaudio natively supports:
- WAV (PCM, ADPCM, IEEE float)
- FLAC
- MP3 (via dr_mp3)

With custom decoders (compile flags):
- OGG Vorbis (via libvorbis, `VULKAN_ENGINE_AUDIO_LIBVORBIS`)
- Opus (via libopus, `VULKAN_ENGINE_AUDIO_LIBOPUS`)

Recommended formats:
- **Sound effects**: OGG Vorbis or WAV (preloaded)
- **Music**: OGG Vorbis (streamed)
- **Voice**: Opus (best compression for voice)

### Related Docs

- [src/audio/README.md](../src/audio/README.md) — quick reference and usage examples
- [src/runtime/game_runtime.h](../src/runtime/game_runtime.h) — `IAudioSystem` interface definition
