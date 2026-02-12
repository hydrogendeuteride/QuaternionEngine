# Particle Systems

GPU-accelerated particle systems with flipbook animation, soft depth fading, and flexible spawning.

## Creating Particle Systems

```cpp
// Create system with 4096 particles
uint32_t systemId = api.create_particle_system(4096);

if (systemId == 0) {
    // Failed to create (out of particle budget)
}
```

Returns a unique system ID (0 on failure).

## Particle System Structure

```cpp
struct ParticleSystem
{
    uint32_t id{0};
    uint32_t particleCount{0};
    bool enabled{true};
    bool reset{true};                    // Trigger particle reset/burst
    ParticleBlendMode blendMode{Additive};
    ParticleParams params{};

    // Asset-relative texture paths (empty = disable)
    std::string flipbookTexture{"vfx/flame.ktx2"};
    std::string noiseTexture{"vfx/simplex.ktx2"};
};

enum class ParticleBlendMode : uint32_t
{
    Additive = 0,  // Additive blending (fire, sparks, etc.)
    Alpha = 1      // Alpha blending with depth sorting
};
```

## Particle Parameters

```cpp
struct ParticleParams
{
    // Emitter
    glm::vec3 emitterPosLocal{0.0f, 0.0f, 0.0f};
    float spawnRadius{0.1f};
    glm::vec3 emitterDirLocal{0.0f, 1.0f, 0.0f};
    float coneAngleDegrees{20.0f};

    // Velocity
    float minSpeed{2.0f};
    float maxSpeed{8.0f};

    // Lifetime
    float minLife{0.5f};
    float maxLife{1.5f};

    // Size
    float minSize{0.05f};
    float maxSize{0.15f};

    // Physics
    float drag{1.0f};         // Velocity damping
    float gravity{0.0f};      // Positive pulls down -Y in local space

    // Appearance
    glm::vec4 color{1.0f, 0.5f, 0.1f, 1.0f};

    // Soft particles (fade near opaque geometry)
    float softDepthDistance{0.15f};  // 0 = disable

    // Flipbook animation
    uint32_t flipbookCols{16};
    uint32_t flipbookRows{4};
    float flipbookFps{30.0f};
    float flipbookIntensity{1.0f};

    // Noise UV distortion
    float noiseScale{6.0f};
    float noiseStrength{0.05f};
    glm::vec2 noiseScroll{0.0f, 0.0f};
};
```

## Managing Particle Systems

### Get System Settings

```cpp
GameAPI::ParticleSystem system;
if (api.get_particle_system(systemId, system)) {
    fmt::println("Particle count: {}", system.particleCount);
}
```

### Update System Settings

```cpp
GameAPI::ParticleSystem system;
api.get_particle_system(systemId, system);

// Change emitter position
system.params.emitterPosLocal = player_position;

// Update
api.set_particle_system(systemId, system);
```

### Resize System

```cpp
bool success = api.resize_particle_system(systemId, 8192);
```

Reallocates particle count (may fail if exceeding budget).

### Destroy System

```cpp
bool destroyed = api.destroy_particle_system(systemId);
```

## Query Particle Pool

```cpp
uint32_t allocated = api.get_allocated_particles();
uint32_t free = api.get_free_particles();
uint32_t maxParticles = api.get_max_particles();

fmt::println("Particles: {}/{} (free: {})", allocated, maxParticles, free);
```

## Get All System IDs

```cpp
std::vector<uint32_t> ids = api.get_particle_system_ids();
for (uint32_t id : ids) {
    // ...
}
```

## Preload Textures

```cpp
// Preload flipbook and noise textures
api.preload_particle_texture("vfx/flame.ktx2");
api.preload_particle_texture("vfx/smoke.ktx2");
api.preload_particle_texture("vfx/simplex.ktx2");
```

Useful for avoiding frame hitches when creating particle systems.

## Complete Examples

### Fire Effect

```cpp
GameAPI::Engine api(&engine);

// Create fire system
uint32_t fireId = api.create_particle_system(4096);

GameAPI::ParticleSystem fire;
fire.id = fireId;
fire.particleCount = 4096;
fire.enabled = true;
fire.blendMode = GameAPI::ParticleBlendMode::Additive;
fire.flipbookTexture = "vfx/flame.ktx2";
fire.noiseTexture = "vfx/simplex.ktx2";

fire.params.emitterPosLocal = glm::vec3(0.0f, 0.0f, -5.0f);
fire.params.spawnRadius = 0.3f;
fire.params.emitterDirLocal = glm::vec3(0.0f, 1.0f, 0.0f);
fire.params.coneAngleDegrees = 15.0f;

fire.params.minSpeed = 2.0f;
fire.params.maxSpeed = 4.0f;
fire.params.minLife = 0.8f;
fire.params.maxLife = 1.5f;
fire.params.minSize = 0.3f;
fire.params.maxSize = 0.6f;

fire.params.drag = 1.0f;
fire.params.gravity = -2.0f;  // Upward buoyancy
fire.params.color = glm::vec4(1.0f, 0.7f, 0.3f, 1.0f);

fire.params.flipbookCols = 16;
fire.params.flipbookRows = 4;
fire.params.flipbookFps = 24.0f;
fire.params.flipbookIntensity = 1.5f;

fire.params.noiseScale = 8.0f;
fire.params.noiseStrength = 0.08f;
fire.params.noiseScroll = glm::vec2(0.0f, 0.5f);

api.set_particle_system(fireId, fire);
```

### Smoke Trail

```cpp
uint32_t smokeId = api.create_particle_system(2048);

GameAPI::ParticleSystem smoke;
smoke.id = smokeId;
smoke.particleCount = 2048;
smoke.enabled = true;
smoke.blendMode = GameAPI::ParticleBlendMode::Alpha;
smoke.flipbookTexture = "vfx/smoke.ktx2";
smoke.noiseTexture = "vfx/simplex.ktx2";

smoke.params.emitterPosLocal = vehicle_exhaust_position;
smoke.params.spawnRadius = 0.15f;
smoke.params.emitterDirLocal = -vehicle_forward;  // Behind vehicle
smoke.params.coneAngleDegrees = 30.0f;

smoke.params.minSpeed = 1.0f;
smoke.params.maxSpeed = 3.0f;
smoke.params.minLife = 2.0f;
smoke.params.maxLife = 4.0f;
smoke.params.minSize = 0.5f;
smoke.params.maxSize = 1.5f;

smoke.params.drag = 0.5f;  // Slow down over time
smoke.params.gravity = 0.5f;  // Slight upward drift
smoke.params.color = glm::vec4(0.3f, 0.3f, 0.3f, 0.5f);  // Gray, semi-transparent

smoke.params.softDepthDistance = 0.2f;

api.set_particle_system(smokeId, smoke);
```

### Sparks

```cpp
uint32_t sparksId = api.create_particle_system(1024);

GameAPI::ParticleSystem sparks;
sparks.id = sparksId;
sparks.particleCount = 1024;
sparks.enabled = true;
sparks.blendMode = GameAPI::ParticleBlendMode::Additive;
sparks.flipbookTexture = "";  // No texture, just colored quads
sparks.noiseTexture = "";

sparks.params.emitterPosLocal = impact_position;
sparks.params.spawnRadius = 0.05f;
sparks.params.emitterDirLocal = surface_normal;
sparks.params.coneAngleDegrees = 45.0f;

sparks.params.minSpeed = 5.0f;
sparks.params.maxSpeed = 15.0f;
sparks.params.minLife = 0.3f;
sparks.params.maxLife = 0.8f;
sparks.params.minSize = 0.02f;
sparks.params.maxSize = 0.05f;

sparks.params.drag = 2.0f;
sparks.params.gravity = 9.8f;  // Fall down
sparks.params.color = glm::vec4(1.0f, 0.8f, 0.3f, 1.0f);  // Bright yellow-orange

api.set_particle_system(sparksId, sparks);

// Trigger one-shot burst
sparks.reset = true;
api.set_particle_system(sparksId, sparks);
```

## Dynamic Updates

### Follow Moving Object

```cpp
// Update emitter position each frame
GameAPI::ParticleSystem system;
api.get_particle_system(engineFireId, system);
system.params.emitterPosLocal = rocket_engine_position;
api.set_particle_system(engineFireId, system);
```

### Enable/Disable

```cpp
GameAPI::ParticleSystem system;
api.get_particle_system(systemId, system);
system.enabled = should_emit;
api.set_particle_system(systemId, system);
```

### Trigger Burst

```cpp
GameAPI::ParticleSystem system;
api.get_particle_system(systemId, system);
system.reset = true;  // Reset all particles (creates burst)
api.set_particle_system(systemId, system);
```

## Performance Considerations

- Particles are simulated and rendered on GPU (compute + draw indirect)
- Total particle budget is shared across all systems
- Additive blending is cheaper than alpha blending (no depth sorting)
- Flipbook textures should be compressed (KTX2 with BC7/ASTC)
- Soft particles require depth buffer reads (small cost)

## See Also

- [Particle System](../ParticleSystem.md) — Low-level implementation details
- [Textures](Textures.md) — Texture loading for flipbooks
- [Volumetrics](Volumetrics.md) — Voxel-based volumetric effects
