# Shadow System

Control shadow rendering modes, quality, and resolution for directional (sun) lighting.

## Shadow Modes

```cpp
enum class ShadowMode : uint32_t
{
    ClipmapOnly = 0,   // Raster shadow maps with PCF
    ClipmapPlusRT = 1, // Shadow maps + ray-traced assist at low N.L angles
    RTOnly = 2         // Pure ray-traced shadows (no shadow maps)
};
```

### Setting Shadow Mode

```cpp
// Raster-only (fastest, good for most scenes)
api.set_shadow_mode(GameAPI::ShadowMode::ClipmapOnly);

// Hybrid (shadow maps + RT assist for soft shadows)
api.set_shadow_mode(GameAPI::ShadowMode::ClipmapPlusRT);

// Pure ray-traced (best quality, requires RT support)
api.set_shadow_mode(GameAPI::ShadowMode::RTOnly);

// Query current mode
GameAPI::ShadowMode mode = api.get_shadow_mode();
```

## Enable/Disable Shadows

```cpp
api.set_shadows_enabled(true);
bool enabled = api.get_shadows_enabled();
```

## Shadow Map Resolution

Shadow map resolution is configured via `kShadowMapResolution` in `src/core/config.h` (default `2048`).

```cpp
// Runtime query only (set via config constant)
uint32_t resolution = api.get_shadow_map_resolution();
```

**VRAM Impact:**
- Each cascade uses depth format D32F (4 bytes per pixel)
- With 4 cascades:
  - 2048×2048: ~64 MB total
  - 4096×4096: ~256 MB total
  - 8192×8192: ~1 GB total

**Performance Notes:**
- Higher resolution = better quality but higher VRAM usage
- On VRAM-constrained systems, reduce `kTextureBudgetFraction` if using large shadow maps
- Allocation failures can prevent shadows from rendering

**Recommended Values:**
- Consumer GPUs: `2048` or `3072`
- High-end GPUs: `4096`
- Integrated GPUs: `1024` or `2048`

## Shadow Quality Settings

### Minimum Visibility

Control the brightness floor for shadowed areas (prevents pitch-black shadows):

```cpp
// 0.0 = fully dark shadows (default)
api.set_shadow_min_visibility(0.0f);

// 0.2 = shadows retain 20% ambient light
api.set_shadow_min_visibility(0.2f);

// Query current value
float minVis = api.get_shadow_min_visibility();
```

Useful for artistic control and ensuring detail remains visible in shadow.

## Hybrid Ray-Traced Settings

When using `ShadowMode::ClipmapPlusRT`, control which cascades use ray-traced assist:

### Cascade Mask

```cpp
// Enable RT assist for cascade 0 only (bitmask: bit 0)
api.set_hybrid_ray_cascade_mask(0b0001);

// Enable RT for cascades 0 and 1 (bits 0-1)
api.set_hybrid_ray_cascade_mask(0b0011);

// Enable RT for all 4 cascades (bits 0-3)
api.set_hybrid_ray_cascade_mask(0b1111);

// Query current mask
uint32_t mask = api.get_hybrid_ray_cascade_mask();
```

**Performance Tip:** Use RT only for the first cascade (closest to camera) to balance quality and cost.

### N·L Threshold

Control when RT assist kicks in based on surface-to-light angle:

```cpp
// Only use RT when N·L < 0.3 (grazing angles)
api.set_hybrid_ray_threshold(0.3f);

// More aggressive (use RT when N·L < 0.5)
api.set_hybrid_ray_threshold(0.5f);

// Query current threshold
float threshold = api.get_hybrid_ray_threshold();
```

Lower thresholds = RT used less often (better performance). Higher thresholds = RT used more (better quality at grazing angles).

## Additional Quality Constants

These are set in `src/core/config.h` and require recompilation:

- `kShadowCascadeCount` — Number of cascades (default `4`)
- `kShadowCSMFar` — Far plane for cascade distribution
- `kShadowCascadeRadiusScale` / `kShadowCascadeRadiusMargin` — Cascade sizing
- `kShadowBorderSmoothNDC` — Border softness
- `kShadowPCFBaseRadius` / `kShadowPCFCascadeGain` — PCF filter kernel size
- `kShadowDepthBiasConstant` / `kShadowDepthBiasSlope` — Depth bias to prevent acne

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Setup shadows
api.set_shadows_enabled(true);
api.set_shadow_mode(GameAPI::ShadowMode::ClipmapPlusRT);

// Use RT assist only for first cascade
api.set_hybrid_ray_cascade_mask(0b0001);
api.set_hybrid_ray_threshold(0.3f);

// Soften shadows slightly
api.set_shadow_min_visibility(0.15f);

// Setup sun
api.set_sunlight_direction(glm::vec3(0.3f, -0.8f, 0.5f));
api.set_sunlight_color(glm::vec3(1.0f, 0.95f, 0.85f), 1.0f);
```

## Troubleshooting

### Shadows Not Appearing

1. Check `api.get_shadows_enabled()` returns `true`
2. Verify sun direction is set: `api.get_sunlight_direction()`
3. Ensure VRAM budget allows shadow map allocation
4. Check console for VMA allocation errors

### Shadow Acne / Peter Panning

Adjust bias constants in `src/core/config.h`:
- Increase `kShadowDepthBiasConstant` to reduce acne
- Increase `kShadowDepthBiasSlope` for steep surfaces
- Balance to avoid excessive "peter panning" (detached shadows)

### Flickering Shadows

Caused by texel snapping during cascade stabilization. Solutions:
- Increase shadow map resolution
- Adjust `kShadowCascadeRadiusMargin` for more stable cascades
- Reduce camera movement speed

### Performance Issues

- Lower shadow map resolution in config
- Use `ShadowMode::ClipmapOnly` instead of hybrid/RT modes
- Reduce hybrid RT cascade mask (e.g., only first cascade)
- Increase hybrid RT threshold to use RT less often

## See Also

- [Lighting](Lighting.md) — Directional light setup
- [Planets](Planets.md) — Sun shadow penumbra settings for planet shadows
- [Ray Tracing](../RayTracing.md) — RT backend details
- [Config](../../src/core/config.h) — Shadow quality constants
