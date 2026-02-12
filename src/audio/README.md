# Audio

> Audio playback system backed by miniaudio, with 3D spatialization, bus mixing, and music management.

## Purpose

Provides a complete audio solution for game engines — 2D/3D sound playback, background music with fade in/out, preloading, bus-based mixing (Sfx, Ui, Voice, Bgm, Ambience), and master volume control. Supports custom decoders (libvorbis/libopus) for efficient OGG/Opus playback.

## Directory Layout

```
audio/
├── miniaudio_system.h    — IAudioSystem implementation and MiniAudioSystem class
└── miniaudio_system.cpp  — implementation
```

## Key Types

| Type | Role |
|------|------|
| `MiniAudioSystem` | Main audio engine — implements `IAudioSystem` interface |
| `SoundHandle` | Opaque uint64_t handle identifying an active sound instance |
| `Bus` | Mix bus categories (Sfx, Ui, Voice, Bgm, Ambience) for independent volume control |
| `ActiveSound` | Internal runtime state for a playing sound (volume, pitch, bus, looping) |
| `CachedSound` | Preloaded decoded sound kept in memory for fast copy-play |

## Lifecycle

```
init()
  ├─ initializes ma_resource_manager with custom decoders (libvorbis/libopus)
  └─ creates ma_engine with 1 listener

set_listener(position, forward, up)  ← update camera/player position for 3D audio

play_3d(event, position, bus, volume, pitch)    → SoundHandle
play_2d(event, bus, volume, pitch, loop)        → SoundHandle
play_music(event, volume, loop, fade_in_sec)    → SoundHandle  (auto-stops previous music)

preload(event)                    ← decode to memory for zero-copy playback
init_sound_from_source(event)     ← uses preloaded cache if available, else loads from file

set_sound_volume/pitch/position(handle, ...)
set_bus_volume(bus, volume)       ← affects all sounds on that bus
set_master_volume(volume)         ← global volume multiplier
set_master_mute(muted)            ← instant mute/unmute

update()                          ← call every frame to cleanup finished one-shots
  └─ cleanup_finished_sounds()    ← removes stopped non-looping and fade-out-completed sounds

shutdown()
  ├─ stop_all()
  ├─ clear_preloaded()
  └─ uninit engine and resource manager
```

## Usage

### Basic 3D sound

```cpp
auto* audio = runtime->getAudioSystem();
audio->set_listener(camera.position, camera.forward, camera.up);

SoundHandle footstep = audio->play_3d(
    "assets/sounds/footstep.wav",
    playerPosition,
    IAudioSystem::Bus::Sfx,
    1.0f,   // volume
    1.0f    // pitch
);
```

### 2D UI sound

```cpp
SoundHandle click = audio->play_2d(
    "assets/sounds/ui_click.ogg",
    IAudioSystem::Bus::Ui,
    0.8f,   // volume
    1.0f,   // pitch
    false   // loop
);
```

### Background music with fade

```cpp
// Fade out old music and fade in new music
audio->play_music(
    "assets/music/level_theme.ogg",
    0.7f,   // volume
    true,   // loop
    2.0f    // fade in 2 seconds
);

// Later: fade out over 1.5 seconds
audio->stop_music(1.5f);
```

### Preloading

```cpp
// Preload during level load
audio->preload("assets/sounds/explosion.wav");
audio->preload("assets/sounds/laser.ogg");

// Fast zero-copy playback (no disk I/O at play time)
audio->play_2d("assets/sounds/explosion.wav", Bus::Sfx, 1.0f, 1.0f, false);

// Cleanup when no longer needed
audio->unload("assets/sounds/explosion.wav");
```

### Bus mixing

```cpp
// Set individual bus volumes (e.g., from settings menu)
audio->set_bus_volume(IAudioSystem::Bus::Sfx, 0.8f);
audio->set_bus_volume(IAudioSystem::Bus::Bgm, 0.5f);

// All sounds on Bus::Sfx are now 80% of their base volume
// All sounds on Bus::Bgm are now 50% of their base volume

// Master controls
audio->set_master_volume(0.9f);   // global volume multiplier
audio->set_master_mute(true);     // instant mute
```

### Runtime control

```cpp
SoundHandle engine_loop = audio->play_2d("car_engine.ogg", Bus::Sfx, 1.0f, 1.0f, true);

// Adjust pitch based on car speed
audio->set_sound_pitch(engine_loop, 0.8f + speed * 0.02f);

// Update position for 3D sound
audio->set_sound_position(engine_loop, car.position);

// Pause/resume
audio->pause(engine_loop, true);   // pause
audio->pause(engine_loop, false);  // resume

// Check if still playing
if (audio->is_playing(engine_loop)) {
    // ...
}

// Stop immediately
audio->stop(engine_loop);
```

## Integration

`MiniAudioSystem` is owned by `Runtime` and exposed via `Runtime::getAudioSystem()`.
The system requires `update()` to be called every frame (typically from `VulkanEngine::run()`) to cleanup finished sounds.

Custom decoders are enabled via CMake options:
- `VULKAN_ENGINE_AUDIO_LIBVORBIS` — use libvorbis for OGG files
- `VULKAN_ENGINE_AUDIO_LIBOPUS` — use libopus for Opus files

## Related Docs

- [docs/Audio.md](../../docs/Audio.md) — detailed audio system architecture
