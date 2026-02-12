# Runtime

> Game loop orchestration with time management, physics integration, audio system, and lifecycle callbacks.

## Purpose

Provides a clean separation between engine rendering and game logic. The Runtime system manages the main loop, fixed timestep physics updates, variable framerate rendering updates, and integrates physics/audio backends through abstract interfaces.

## Directory Layout

```
runtime/
├── game_runtime.h      — Runtime class and IPhysicsWorld / IAudioSystem interfaces
├── game_runtime.cpp    — Main loop implementation
├── i_game_callbacks.h  — IGameCallbacks interface for game logic hooks
├── time_manager.h      — TimeManager class declaration
└── time_manager.cpp    — Time management implementation
```

## Key Types

| Type | Role |
|------|------|
| `Runtime` | Central game loop manager — integrates time, physics, audio, and delegates to game callbacks |
| `IGameCallbacks` | Interface for game logic (init, update, fixed_update, shutdown) |
| `TimeManager` | Time management — delta time, time scale, fixed timestep accumulation, interpolation alpha |
| `IPhysicsWorld` | Abstract physics backend (step, raycast, body transforms) |
| `IAudioSystem` | Abstract audio backend (3D/2D playback, buses, master volume, listener) |

## Lifecycle

```
Runtime::Runtime(VulkanEngine*)
  └─ creates GameAPI wrapper, initializes TimeManager

set_physics_world(IPhysicsWorld*)   ← inject physics backend
set_audio_system(IAudioSystem*)     ← inject audio backend

run(IGameCallbacks*)                ← main loop entry point
  ├─ game->on_init(runtime)
  │
  ├─ [Main Loop]
  │   ├─ time().begin_frame()
  │   ├─ input->pump_events()
  │   ├─ handle window resize / minimize
  │   │
  │   ├─ [Fixed Update Loop]
  │   │   ├─ while time().consume_fixed_step()
  │   │   ├─ game->on_fixed_update(fixed_dt)
  │   │   └─ physics->step(fixed_dt)
  │   │
  │   ├─ game->on_update(delta_time)
  │   ├─ audio->update() + update_audio_listener()
  │   ├─ renderer->draw()
  │   └─ repeat until quit_requested()
  │
  └─ game->on_shutdown()

~Runtime()
  └─ destroys GameAPI wrapper
```

## Usage

### Basic game loop

```cpp
class MyGame : public GameRuntime::IGameCallbacks {
public:
    void on_init(GameRuntime::Runtime& runtime) override {
        // Load assets, spawn entities, setup camera
        runtime.api().load_gltf("assets/level.gltf");
    }

    void on_fixed_update(float fixed_dt) override {
        // Physics, AI, deterministic simulation (60 Hz)
        // fixed_dt is typically 1/60 (16.67ms)
    }

    void on_update(float dt) override {
        // Input handling, camera control, rendering-dependent logic
        // dt is variable frame time (scaled by time_scale, clamped to 100ms)
    }

    void on_shutdown() override {
        // Cleanup game resources
    }
};

// In main.cpp
VulkanEngine engine;
engine.init();

GameRuntime::Runtime runtime(&engine);
MyGame game;
runtime.run(&game);  // blocks until quit_requested()

engine.cleanup();
```

### Time management

```cpp
// Access via runtime
runtime.delta_time();            // scaled dt (affected by time_scale)
runtime.time().unscaled_delta_time();  // raw frame time
runtime.time().total_time();     // elapsed time since start
runtime.set_time_scale(0.5f);    // slow-mo effect

// Fixed timestep interpolation (for smooth rendering)
float alpha = runtime.interpolation_alpha();
// interpolate between previous_state and current_state using alpha
```

### Physics integration

```cpp
class MyPhysics : public GameRuntime::IPhysicsWorld {
public:
    void step(float dt) override {
        // Jolt/PhysX/Bullet integration here
    }

    void get_body_transform(uint32_t id, glm::mat4& out) override {
        // Query body world transform
    }

    RayHit raycast(const glm::vec3& origin, const glm::vec3& dir, float maxDist) override {
        // Physics raycast
    }
};

MyPhysics physics;
runtime.set_physics_world(&physics);
// Runtime will call physics->step(fixed_dt) every fixed update
```

### Audio integration

```cpp
class MyAudio : public GameRuntime::IAudioSystem {
public:
    void set_listener(const glm::vec3& pos, const glm::vec3& fwd, const glm::vec3& up) override {
        // FMOD/Wwise/OpenAL listener update
    }

    SoundHandle play_3d(const std::string& event, const glm::vec3& pos,
                        Bus bus, float volume, float pitch) override {
        // 3D spatialized sound
    }

    void update() override {
        // Per-frame cleanup (finished one-shots, fades)
    }
};

MyAudio audio;
runtime.set_audio_system(&audio);
// Runtime auto-updates listener from main camera and calls audio->update()
```

## Integration

`Runtime` is typically instantiated in `main.cpp` and wraps a `VulkanEngine*`.
The `GameAPI::Engine` wrapper is constructed internally and accessible via `runtime.api()`.

Input handling is delegated to `VulkanEngine::input()` (SDL2 backend), with events routed to:
- ImGui (UI capture check)
- Picking system (mouse-based ray picking)
- Camera rig (WASD/mouse look)

Physics and audio backends are optional — `Runtime` checks for null before calling their methods.

## Related Docs

- [docs/GameRuntime.md](../../docs/GameRuntime.md) — detailed runtime system architecture (to be created)
- [src/core/game_api.h](../core/game_api.h) — high-level engine API exposed to game logic
- [src/core/input/](../core/input/) — input system integration
