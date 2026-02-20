## Game Runtime System: Loop, Time, and Subsystem Integration

The runtime layer orchestrates the main game loop, manages frame timing with fixed timestep physics, and binds physics/audio backends. It lives under `src/runtime` and serves as the bridge between engine rendering (`VulkanEngine`) and game logic (`IGameCallbacks`).

### Concepts

- **Runtime**: The main loop coordinator that owns time management, invokes game callbacks, and integrates physics/audio subsystems.
- **TimeManager**: Handles delta time calculation, time scaling, fixed timestep accumulation, and interpolation alpha for smooth rendering.
- **IGameCallbacks**: Interface for game code to receive lifecycle events (init, update, fixed_update, shutdown).
- **Physics::PhysicsWorld**: Physics backend interface from `src/physics` used for stepping and world queries.
- **IAudioSystem**: Abstract audio backend with 3D spatialization, bus mixing, and listener management.

### Key Types

#### `Runtime` — Main Loop Manager

Central coordinator that wraps a `VulkanEngine*` and provides:

```c++
class Runtime {
public:
    explicit Runtime(VulkanEngine* renderer);

    // Subsystem injection
    void set_physics_world(Physics::PhysicsWorld* physics);
    void set_audio_system(IAudioSystem* audio);

    // Time management
    TimeManager& time();
    float delta_time() const;              // scaled by time_scale, clamped
    float fixed_delta_time() const;        // physics timestep (default 1/60)
    void set_time_scale(float scale);      // slow-mo / pause (0.0 to N)
    float interpolation_alpha() const;     // for rendering interpolation

    // GameAPI access
    GameAPI::Engine& api();

    // Main loop
    void run(IGameCallbacks* game);        // blocks until quit
    void request_quit();
};
```

**Responsibilities:**
- Frame time tracking and fixed timestep accumulation
- Input event pumping and routing (SDL2 → ImGui / Picking / Camera)
- Window resize and minimize handling
- Fixed update loop (physics, deterministic simulation)
- Variable update loop (rendering, input, camera)
- Audio listener sync (matches main camera)
- GPU fence waiting and per-frame resource cleanup
- ImGui frame lifecycle

#### `TimeManager` — Time Management

```c++
class TimeManager {
public:
    void begin_frame();                     // call at start of each frame

    float delta_time() const;               // scaled dt (affected by time_scale)
    float unscaled_delta_time() const;      // raw dt (clamped to 100ms max)
    float fixed_delta_time() const;         // physics timestep (default 1/60)
    void set_fixed_delta_time(float dt);

    float time_scale() const;               // default 1.0
    void set_time_scale(float scale);

    float total_time() const;               // scaled elapsed time
    float unscaled_total_time() const;      // raw elapsed time

    bool consume_fixed_step();              // consume accumulator, returns true if step needed
    float interpolation_alpha() const;      // 0.0 to 1.0 for rendering interpolation
    uint64_t frame_count() const;
};
```

**Fixed Timestep Pattern:**
```c++
while (time.consume_fixed_step()) {
    // deterministic update at fixed_delta_time intervals
    physics->step(time.fixed_delta_time());
    game->on_fixed_update(time.fixed_delta_time());
}
```

**Interpolation:**
```c++
float alpha = time.interpolation_alpha();
// lerp between previous_physics_state and current_physics_state using alpha
// smooths rendering when display framerate != physics rate
```

#### `IGameCallbacks` — Game Logic Interface

```c++
class IGameCallbacks {
public:
    virtual void on_init(Runtime& runtime) = 0;
    virtual void on_update(float dt) = 0;
    virtual void on_fixed_update(float fixed_dt) = 0;
    virtual void on_shutdown() = 0;
};
```

| Callback | Timing | Purpose |
|----------|--------|---------|
| `on_init` | Once before first update | Load assets, spawn entities, configure camera |
| `on_update` | Every frame (variable dt) | Input handling, camera control, rendering logic |
| `on_fixed_update` | Fixed intervals (1/60 default) | Physics, AI, deterministic simulation |
| `on_shutdown` | Once before exit | Cleanup game resources |

#### `Physics::PhysicsWorld` — Physics Backend Interface

```c++
class PhysicsWorld {
public:
    virtual void step(float dt) = 0;
    virtual BodyId create_body(const BodySettings& settings) = 0;
    virtual void destroy_body(BodyId id) = 0;
    virtual RayHit raycast(const glm::dvec3& origin, const glm::vec3& direction, float max_distance) const = 0;
    // ... many more APIs in src/physics/physics_world.h
};
```

**Integration:**
- Runtime calls `physics->step(fixed_dt)` every fixed update
- Physics backend (Jolt/PhysX/Bullet) implements `Physics::PhysicsWorld`
- Game-side systems (e.g. `GameWorld`) use the same interface directly

#### `IAudioSystem` — Audio Backend Interface

```c++
class IAudioSystem {
public:
    using SoundHandle = uint64_t;

    enum class Bus : uint8_t {
        Sfx, Ui, Voice, Bgm, Ambience, Count
    };

    // Listener (synced to camera by Runtime)
    virtual void set_listener(const glm::vec3& position, const glm::vec3& forward, const glm::vec3& up) = 0;

    // Playback
    virtual SoundHandle play_3d(const std::string& event, const glm::vec3& position,
                                Bus bus, float volume, float pitch) = 0;
    virtual SoundHandle play_2d(const std::string& event, Bus bus, float volume, float pitch, bool loop) = 0;
    virtual SoundHandle play_music(const std::string& event, float volume, bool loop, float fade_in_seconds) = 0;
    virtual void stop_music(float fade_out_seconds) = 0;

    // Resource management
    virtual bool preload(const std::string& event) = 0;
    virtual void unload(const std::string& event) = 0;

    // Control
    virtual void stop(SoundHandle sound) = 0;
    virtual void pause(SoundHandle sound, bool paused) = 0;
    virtual void set_sound_volume(SoundHandle sound, float volume) = 0;
    virtual void set_sound_pitch(SoundHandle sound, float pitch) = 0;
    virtual void set_sound_position(SoundHandle sound, const glm::vec3& position) = 0;

    // Mixing
    virtual void set_bus_volume(Bus bus, float volume) = 0;
    virtual void set_master_volume(float volume) = 0;
    virtual void set_master_mute(bool muted) = 0;

    // Per-frame tick
    virtual void update() = 0;
};
```

**Volume Hierarchy (multiplicative):**
```
final_volume = master_volume * bus_volume * sound_volume
master_mute overrides all to silence
```

**Integration:**
- Runtime auto-syncs listener to main camera every frame
- Runtime calls `audio->update()` after game update
- Implementations should clean up finished one-shots in `update()`

### Main Loop Structure

```
Runtime::run(IGameCallbacks* game)
  │
  ├─ game->on_init(runtime)
  │
  └─ [Main Loop]
      │
      ├─ time.begin_frame()
      │
      ├─ Input & Event Handling
      │   ├─ input->pump_events()
      │   ├─ SDL events → ImGui
      │   ├─ SDL events → picking system
      │   └─ camera rig input (if not UI-captured)
      │
      ├─ Window Management
      │   ├─ handle minimize (sleep 100ms, skip frame)
      │   └─ handle resize (recreate swapchain)
      │
      ├─ Fixed Update Loop (deterministic)
      │   └─ while time.consume_fixed_step()
      │       ├─ game->on_fixed_update(fixed_dt)
      │       └─ physics->step(fixed_dt)
      │
      ├─ Variable Update (rendering-dependent)
      │   └─ game->on_update(delta_time)
      │
      ├─ Audio
      │   ├─ update_audio_listener() [sync to camera]
      │   └─ audio->update()
      │
      ├─ GPU Synchronization
      │   ├─ vkWaitForFences (current frame fence)
      │   ├─ ray manager: pump BLAS builds
      │   └─ IBL async load: pump and commit
      │
      ├─ Per-Frame Cleanup
      │   ├─ deletionQueue.flush()
      │   ├─ renderGraph->resolve_timings()
      │   └─ frameDescriptors.clear_pools()
      │
      ├─ ImGui Frame
      │   ├─ ui->beginFrame()
      │   └─ ui->endFrame()
      │
      ├─ Rendering
      │   └─ renderer->draw()
      │
      └─ [repeat until quit_requested()]
```

### Usage Patterns

#### Basic Integration

```c++
// main.cpp
class MyGame : public GameRuntime::IGameCallbacks {
    void on_init(GameRuntime::Runtime& runtime) override {
        runtime.api().load_gltf("assets/scene.gltf");
        runtime.api().set_sun_direction(glm::vec3(0.3f, -1.0f, 0.2f));
    }

    void on_update(float dt) override {
        // Input, camera, UI
    }

    void on_fixed_update(float fixed_dt) override {
        // Physics, AI (60 Hz)
    }

    void on_shutdown() override {
        // Cleanup
    }
};

int main() {
    VulkanEngine engine;
    engine.init();

    GameRuntime::Runtime runtime(&engine);
    MyGame game;
    runtime.run(&game);  // blocks until quit

    engine.cleanup();
}
```

#### Physics Integration (Jolt Example)

```c++
#include "physics/jolt/jolt_physics_world.h"

auto physics = std::make_unique<Physics::JoltPhysicsWorld>();
runtime.set_physics_world(physics.get());
```

#### Audio Integration (FMOD Example)

```c++
class FMODAudioSystem : public GameRuntime::IAudioSystem {
    FMOD::Studio::System* _system;
    FMOD::System* _coreSystem;

    void set_listener(const glm::vec3& pos, const glm::vec3& fwd, const glm::vec3& up) override {
        FMOD_3D_ATTRIBUTES attr;
        attr.position = {pos.x, pos.y, pos.z};
        attr.forward = {fwd.x, fwd.y, fwd.z};
        attr.up = {up.x, up.y, up.z};
        _system->setListenerAttributes(0, &attr);
    }

    SoundHandle play_3d(const std::string& event, const glm::vec3& position, Bus bus, float volume, float pitch) override {
        FMOD::Studio::EventDescription* desc;
        _system->getEvent(event.c_str(), &desc);

        FMOD::Studio::EventInstance* instance;
        desc->createInstance(&instance);
        instance->set3DAttributes(&attr);
        instance->start();

        return reinterpret_cast<SoundHandle>(instance);
    }

    void update() override {
        _system->update();
    }
};

// Wire up
FMODAudioSystem audio;
runtime.set_audio_system(&audio);
```

#### Time Scale Effects

```c++
// Slow-motion
runtime.set_time_scale(0.3f);  // 30% speed

// Pause
runtime.set_time_scale(0.0f);  // freeze

// Speed up (debugging)
runtime.set_time_scale(2.0f);  // 2x speed

// Unscaled timer (for UI/menus that ignore time_scale)
float realDt = runtime.time().unscaled_delta_time();
```

#### Fixed Timestep Interpolation

```c++
// Store previous state in on_fixed_update
struct RigidBody {
    glm::vec3 prevPosition;
    glm::quat prevRotation;
    glm::vec3 position;
    glm::quat rotation;
};

void on_fixed_update(float fixed_dt) {
    for (auto& body : bodies) {
        body.prevPosition = body.position;
        body.prevRotation = body.rotation;
        // step physics
        physics->step(fixed_dt);
        // update from physics
        glm::mat4 xform;
        physics->get_body_transform(body.id, xform);
        body.position = glm::vec3(xform[3]);
        body.rotation = glm::quat_cast(xform);
    }
}

void on_update(float dt) {
    float alpha = runtime.interpolation_alpha();
    for (auto& body : bodies) {
        glm::vec3 renderPos = glm::mix(body.prevPosition, body.position, alpha);
        glm::quat renderRot = glm::slerp(body.prevRotation, body.rotation, alpha);
        // update visual representation
    }
}
```

### Integration with Engine

| Component | Interaction |
|-----------|-------------|
| `VulkanEngine` | Wrapped by Runtime, provides rendering and resource management |
| `GameAPI::Engine` | High-level API wrapper created by Runtime, accessible via `runtime.api()` |
| `InputSystem` | Polled by Runtime for events, routed to ImGui/Picking/Camera |
| `ImGuiSystem` | Begin/end frame called by Runtime, receives SDL events |
| `PickingSystem` | Receives input events filtered by UI capture |
| `SceneManager` | Camera rig receives input if not UI-captured |
| `RenderGraph` | Timing resolution triggered by Runtime after fence wait |
| `ResourceManager` | Per-frame deletion queue flushed by Runtime |

### Best Practices

**Do:**
- Use `on_fixed_update` for physics, AI, and deterministic simulation (constant timestep)
- Use `on_update` for input handling, camera control, and rendering-dependent logic (variable timestep)
- Store previous state in `on_fixed_update` and interpolate in `on_update` for smooth rendering
- Preload audio assets during `on_init` to avoid hitches
- Use unscaled delta time for UI/menu systems that shouldn't be affected by time_scale

**Don't:**
- Don't mix rendering logic into `on_fixed_update` (it's called 0-N times per frame)
- Don't perform heavy CPU work in `on_update` every frame (amortize or move to background thread)
- Don't bypass `Runtime::run()` and manually call callbacks (breaks time management)
- Don't call `vkWaitForFences` or GPU sync from game code (Runtime handles synchronization)

### Related Docs

- [src/runtime/README.md](../src/runtime/README.md) — module overview and quick reference
- [src/core/game_api.h](../src/core/game_api.h) — high-level engine API for game code
- [docs/PhysicsSystem.md](PhysicsSystem.md) — Jolt physics backend implementation
- [docs/Audio.md](Audio.md) — audio system architecture
