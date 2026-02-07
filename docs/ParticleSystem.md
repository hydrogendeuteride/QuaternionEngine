# Particle System

The particle system provides GPU-accelerated particle simulation and rendering with support for flipbook animation, soft particles, and alpha/additive blending.

## Architecture Overview

The system is implemented across multiple components:

- **ParticlePass** (`src/render/passes/particles.h/.cpp`) — Render pass managing particle pools, compute pipelines, and graphics pipelines
- **GameAPI** (`src/core/game_api.h`) — High-level API for creating and controlling particle systems
- **Shaders** — Compute and graphics shaders for simulation and rendering
  - `shaders/particles_update.comp` — Per-particle physics simulation
  - `shaders/particles_sort_blocks.comp` — Block-level depth sorting for alpha blending
  - `shaders/particles_build_indices.comp` — Build draw indices from sorted blocks
  - `shaders/particles.vert/.frag` — Vertex/fragment shaders for rendering

## Key Features

- **Global particle pool**: Up to 128K particles (`k_max_particles = 128 * 1024`) shared across all systems
- **GPU simulation**: Fully GPU-driven via compute shaders (no CPU readback)
- **Flipbook animation**: Supports sprite sheet animation with configurable atlas layout and FPS
- **Soft particles**: Depth-aware fading near opaque geometry
- **Blend modes**: Additive (fire, sparks) and Alpha (smoke, debris) with automatic depth sorting
- **Noise distortion**: Optional UV distortion for organic motion
- **Floating-origin stable**: Automatically adjusts particle positions when world origin shifts

## Particle Data Layout

Each particle is represented as 64 bytes (4 × vec4) on the GPU:

```glsl
struct Particle
{
    vec4 pos_age;   // xyz = local position, w = remaining life (seconds)
    vec4 vel_life;  // xyz = local velocity, w = total lifetime (seconds)
    vec4 color;     // rgba
    vec4 misc;      // x=size, y=random seed, z/w=unused
};
```

## Creating Particle Systems

### Via GameAPI

```cpp
#include "core/game_api.h"

GameAPI::Engine api(&engine);

// Create a particle system with 1024 particles
uint32_t systemId = api.create_particle_system(1024);

// Configure parameters
GameAPI::ParticleSystem sys = api.get_particle_system(systemId);
sys.enabled = true;
sys.reset = true; // Respawn all particles immediately
sys.blendMode = GameAPI::ParticleBlendMode::Additive;

// Emitter settings
sys.params.emitterPosLocal = glm::vec3(0.0f, 0.0f, 0.0f);
sys.params.spawnRadius = 0.1f;
sys.params.emitterDirLocal = glm::vec3(0.0f, 1.0f, 0.0f); // Upward
sys.params.coneAngleDegrees = 20.0f;

// Particle properties
sys.params.minSpeed = 2.0f;
sys.params.maxSpeed = 8.0f;
sys.params.minLife = 0.5f;
sys.params.maxLife = 1.5f;
sys.params.minSize = 0.05f;
sys.params.maxSize = 0.15f;

// Physics
sys.params.drag = 1.0f;
sys.params.gravity = 0.0f; // Positive pulls down -Y in local space

// Appearance
sys.params.color = glm::vec4(1.0f, 0.5f, 0.1f, 1.0f); // Orange

// Flipbook animation (16×4 atlas, 30 FPS)
sys.flipbookTexture = "vfx/flame.ktx2";
sys.params.flipbookCols = 16;
sys.params.flipbookRows = 4;
sys.params.flipbookFps = 30.0f;
sys.params.flipbookIntensity = 1.0f;

// Noise distortion
sys.noiseTexture = "vfx/simplex.ktx2";
sys.params.noiseScale = 6.0f;
sys.params.noiseStrength = 0.05f;
sys.params.noiseScroll = glm::vec2(0.0f, 0.0f);

// Soft particles
sys.params.softDepthDistance = 0.15f; // Fade particles within 0.15 units of geometry

api.set_particle_system(systemId, sys);
```

### Direct API

```cpp
ParticlePass* particlePass = /* obtain from RenderPassManager */;

// Create system
uint32_t systemId = particlePass->create_system(1024);

// Access and modify
auto& systems = particlePass->systems();
for (auto& sys : systems)
{
    if (sys.id == systemId)
    {
        sys.enabled = true;
        sys.params.color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        break;
    }
}
```

## Simulation Details

### Update Pipeline (Compute)

The `particles_update.comp` shader runs once per frame for each active system:

1. **Floating-origin correction**: `p.pos_age.xyz -= origin_delta` keeps particles stable when the world origin shifts
2. **Respawn check**: Dead particles (`age <= 0`) or reset flag respawns particles with randomized properties
3. **Physics integration**:
   - Apply gravity: `vel += vec3(0, -gravity, 0) * dt`
   - Apply drag: `vel *= exp(-drag * dt)`
   - Integrate position: `pos += vel * dt`
4. **Age decrement**: `age -= dt`

Random number generation uses a per-particle seed (`misc.y`) combined with system time to ensure deterministic but varied behavior.

### Cone Emission

When `coneAngleDegrees > 0`, particles are emitted within a cone:
- Cone axis is `emitterDirLocal`
- Particles are randomly distributed within the cone solid angle
- `coneAngleDegrees = 0` emits in a single direction
- `coneAngleDegrees < 0` emits in all directions (sphere)

### Spawn Radius

Particles spawn at `emitterPosLocal ± random_in_sphere(spawnRadius)`.

## Rendering Pipeline

### Blend Modes

**Additive** (`BlendMode::Additive`):
- Source: `VK_BLEND_FACTOR_SRC_ALPHA`
- Dest: `VK_BLEND_FACTOR_ONE`
- No depth sorting required
- Ideal for fire, sparks, energy effects

**Alpha** (`BlendMode::Alpha`):
- Source: `VK_BLEND_FACTOR_SRC_ALPHA`
- Dest: `VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA`
- Block-level depth sorting (256 particles per block)
- Better for smoke, debris, leaves

### Alpha Sorting

For alpha-blended systems:

1. **Block sorting** (`particles_sort_blocks.comp`): Divides particles into 256-particle blocks, computes average depth per block, sorts blocks back-to-front
2. **Index building** (`particles_build_indices.comp`): Writes sorted particle indices into `_draw_indices` buffer
3. **Rendering**: Vertex shader reads particles via indirection: `Particle p = pool.particles[indices[gl_InstanceIndex]]`

This provides coarse-grained sorting with minimal compute overhead (512 blocks max).

### Soft Particles

Fragment shader samples G-buffer depth (`gbufferPosition.w`) and fades particle alpha near intersections:

```glsl
float sceneDepth = texture(posTex, screenUV).w;
float particleDepth = /* compute from world pos */;
float depthDiff = sceneDepth - particleDepth;
float softFactor = smoothstep(0.0, softDepthDistance, depthDiff);
outColor.a *= softFactor;
```

Set `softDepthDistance = 0` to disable.

### Flipbook Animation

The fragment shader samples an animated sprite sheet:

1. Compute frame index: `frameIndex = int(time_sec * flipbookFps) % (flipbookCols * flipbookRows)`
2. Map frame to UV rect: `(col, row) = (frameIndex % cols, frameIndex / cols)`
3. Sample texture: `color = texture(flipbookTex, baseUV * cellSize + cellOffset)`

### Noise Distortion

Optional UV distortion using a noise texture:

```glsl
vec2 noiseUV = uv * noiseScale + noiseScroll * time_sec;
vec2 distortion = (texture(noiseTex, noiseUV).rg - 0.5) * 2.0 * noiseStrength;
vec2 finalUV = uv + distortion;
```

## Memory Management

### Particle Pool Allocation

The global pool is pre-allocated (128K particles × 64 bytes = 8 MB) and subdivided into ranges:

- `create_system(count)`: Allocates a contiguous range from `_free_ranges`
- `destroy_system(id)`: Returns range to free list and merges adjacent ranges
- `resize_system(id, new_count)`: Reallocates (may move particles)

Allocation uses a simple first-fit strategy with automatic coalescing.

### Texture Caching

VFX textures (flipbook/noise) are loaded on-demand and cached in `_vfx_textures`:
- `preload_vfx_texture(assetName)`: Explicitly load texture (safe to call from UI)
- `preload_needed_textures()`: Load all textures referenced by active systems (call before ResourceUploads pass)
- Fallback 1×1 textures (`_fallback_flipbook`, `_fallback_noise`) are used when load fails

## Render Graph Integration

The particle pass registers into the render graph after lighting and SSR:

```cpp
void ParticlePass::register_graph(RenderGraph* graph,
                                   RGImageHandle hdrTarget,
                                   RGImageHandle depthHandle,
                                   RGImageHandle gbufferPosition)
{
    graph->add_pass("Particles", RGPassType::Graphics,
        [=](RGPassBuilder& b, EngineContext*) {
            b.write_color(hdrTarget); // Composite onto HDR
            b.read_depth(depthHandle); // Depth test
            b.sample_image(gbufferPosition); // Soft particles
        },
        [this](VkCommandBuffer cmd, const RGPassResources& res, EngineContext* ctx) {
            // 1. Run compute update for each system
            // 2. For alpha systems: sort blocks + build indices
            // 3. Render all systems (additive first, then alpha)
        }
    );
}
```

## Performance Considerations

- **Particle count**: 128K global limit; budget carefully across systems
- **Overdraw**: Additive blending is fill-rate intensive; keep particle size and count moderate
- **Sorting cost**: Alpha systems incur compute overhead for block sorting (~512 blocks × 256 particles)
- **Texture bandwidth**: Flipbook textures should be compressed (KTX2) and atlased (16×4 common)
- **Soft particles**: G-buffer read adds bandwidth; disable if depth fading isn't visible

## Common Presets

### Fire

```cpp
sys.blendMode = Additive;
sys.params.color = glm::vec4(1.0f, 0.5f, 0.1f, 1.0f); // Orange
sys.params.gravity = 0.0f;
sys.params.minSpeed = 1.0f; sys.params.maxSpeed = 3.0f;
sys.params.drag = 0.5f;
sys.flipbookTexture = "vfx/flame.ktx2";
```

### Smoke

```cpp
sys.blendMode = Alpha;
sys.params.color = glm::vec4(0.3f, 0.3f, 0.3f, 0.5f); // Gray, semi-transparent
sys.params.gravity = -2.0f; // Rise upward (negative gravity)
sys.params.drag = 1.5f; // Slow down quickly
sys.params.minSpeed = 0.5f; sys.params.maxSpeed = 2.0f;
sys.noiseTexture = "vfx/simplex.ktx2";
sys.params.noiseStrength = 0.2f; // Strong distortion
```

### Sparks

```cpp
sys.blendMode = Additive;
sys.params.color = glm::vec4(1.0f, 0.8f, 0.2f, 1.0f); // Bright yellow
sys.params.gravity = 9.8f; // Fall downward
sys.params.drag = 0.1f;
sys.params.minSpeed = 5.0f; sys.params.maxSpeed = 15.0f;
sys.params.minSize = 0.01f; sys.params.maxSize = 0.03f; // Small
sys.flipbookTexture = ""; // Disable flipbook (procedural sprite)
```

## Troubleshooting

**Particles not visible**:
- Ensure `enabled = true` and `particleCount > 0`
- Check `color.a > 0` (fully transparent particles are invisible)
- Verify system is allocated: `api.get_particle_systems()` should list the ID

**Particles flickering or popping**:
- Set `reset = false` after first frame (reset respawns all particles immediately)
- Increase `minLife`/`maxLife` to prevent frequent respawning

**Performance issues**:
- Reduce total particle count (check `allocated_particles()`)
- Use additive blend for most systems (cheaper than alpha)
- Reduce flipbook texture resolution or mip levels

**Textures missing**:
- Call `preload_vfx_texture("vfx/texture.ktx2")` before first frame
- Or call `preload_needed_textures()` in engine setup
- Check AssetManager can resolve path: `assetPath("vfx/texture.ktx2")`

## API Reference

### ParticlePass

```cpp
class ParticlePass : public IRenderPass
{
    // System management
    uint32_t create_system(uint32_t count);
    bool destroy_system(uint32_t id);
    bool resize_system(uint32_t id, uint32_t new_count);

    std::vector<System>& systems();
    const std::vector<System>& systems() const;

    // Pool stats
    uint32_t allocated_particles() const;
    uint32_t free_particles() const;

    // Texture preloading
    void preload_vfx_texture(const std::string& assetName);
    void preload_needed_textures();
};
```

### GameAPI::Engine Particle Methods

```cpp
// System creation/destruction
uint32_t create_particle_system(uint32_t particle_count);
bool destroy_particle_system(uint32_t system_id);

// System control
void set_particle_system(uint32_t system_id, const ParticleSystem& sys);
ParticleSystem get_particle_system(uint32_t system_id) const;
std::vector<ParticleSystem> get_particle_systems() const;

// Pool stats
uint32_t get_particle_pool_allocated() const;
uint32_t get_particle_pool_free() const;

// Texture preloading
void preload_particle_texture(const std::string& asset_path);
```

## See Also

- `docs/RenderGraph.md` — Render graph integration details
- `docs/RenderPasses.md` — Pass execution and pipeline management
- `docs/GameAPI.md` — High-level game API
- `docs/TextureLoading.md` — Asset loading and streaming
