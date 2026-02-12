# Post-Processing

Control tonemapping, bloom, FXAA anti-aliasing, and screen-space reflections (SSR).

## Tonemapping

### Tone Map Operators

```cpp
enum class TonemapOperator : int
{
    Reinhard = 0,
    ACES = 1
};
```

### Exposure

```cpp
// Set exposure (higher = brighter)
api.set_exposure(1.0f);   // Default
api.set_exposure(1.5f);   // Brighter
api.set_exposure(0.7f);   // Darker

float exposure = api.get_exposure();
```

### Operator Selection

```cpp
// Use Reinhard (simple, fast)
api.set_tonemap_operator(GameAPI::TonemapOperator::Reinhard);

// Use ACES (filmic, more accurate)
api.set_tonemap_operator(GameAPI::TonemapOperator::ACES);

GameAPI::TonemapOperator op = api.get_tonemap_operator();
```

**Comparison:**
- **Reinhard** — Simple, fast, good for most cases
- **ACES** — Industry-standard filmic curve, better color preservation, slightly more expensive

## Bloom

Bright areas glow and bleed.

### Enable/Disable

```cpp
api.set_bloom_enabled(true);
bool enabled = api.get_bloom_enabled();
```

### Threshold

Controls which pixels contribute to bloom (luminance threshold).

```cpp
// Only very bright pixels bloom (strict)
api.set_bloom_threshold(1.5f);

// More pixels bloom (generous)
api.set_bloom_threshold(0.8f);

// Default
api.set_bloom_threshold(1.0f);

float threshold = api.get_bloom_threshold();
```

Higher threshold = only very bright areas bloom.

### Intensity

Controls bloom strength.

```cpp
// Subtle bloom
api.set_bloom_intensity(0.3f);

// Strong bloom
api.set_bloom_intensity(1.0f);

// Default
api.set_bloom_intensity(0.5f);

float intensity = api.get_bloom_intensity();
```

## FXAA (Fast Approximate Anti-Aliasing)

Post-process anti-aliasing to smooth edges.

### Enable/Disable

```cpp
api.set_fxaa_enabled(true);
bool enabled = api.get_fxaa_enabled();
```

### Edge Detection Thresholds

```cpp
// Edge threshold (higher = less sensitive, faster)
api.set_fxaa_edge_threshold(0.166f);  // Default
api.set_fxaa_edge_threshold(0.125f);  // More sensitive (slower)
api.set_fxaa_edge_threshold(0.25f);   // Less sensitive (faster)

float threshold = api.get_fxaa_edge_threshold();

// Minimum edge threshold (for very dark areas)
api.set_fxaa_edge_threshold_min(0.0833f);  // Default
float thresholdMin = api.get_fxaa_edge_threshold_min();
```

**Recommended Presets:**
- **Quality** — `edge_threshold: 0.063`, `edge_threshold_min: 0.0312`
- **Default** — `edge_threshold: 0.166`, `edge_threshold_min: 0.0833`
- **Performance** — `edge_threshold: 0.333`, `edge_threshold_min: 0.1667`

## Screen-Space Reflections (SSR)

### Reflection Modes

```cpp
enum class ReflectionMode : uint32_t
{
    SSROnly = 0,       // Screen-space reflections only
    SSRPlusRT = 1,     // SSR with ray-traced fallback
    RTOnly = 2         // Pure ray-traced reflections
};
```

### Enable/Disable SSR

```cpp
api.set_ssr_enabled(true);
bool enabled = api.get_ssr_enabled();
```

### Set Reflection Mode

```cpp
// Screen-space only (fastest)
api.set_reflection_mode(GameAPI::ReflectionMode::SSROnly);

// SSR with RT fallback (hybrid)
api.set_reflection_mode(GameAPI::ReflectionMode::SSRPlusRT);

// Pure ray-traced (best quality, expensive)
api.set_reflection_mode(GameAPI::ReflectionMode::RTOnly);

GameAPI::ReflectionMode mode = api.get_reflection_mode();
```

**Notes:**
- SSR only works for on-screen geometry
- RT modes require ray-tracing support
- Hybrid mode uses RT for off-screen reflections

## Complete Examples

### Cinematic Look

```cpp
GameAPI::Engine api(&engine);

// ACES tonemapping for filmic look
api.set_tonemap_operator(GameAPI::TonemapOperator::ACES);
api.set_exposure(1.2f);

// Strong bloom
api.set_bloom_enabled(true);
api.set_bloom_threshold(0.9f);
api.set_bloom_intensity(0.8f);

// High-quality FXAA
api.set_fxaa_enabled(true);
api.set_fxaa_edge_threshold(0.063f);
api.set_fxaa_edge_threshold_min(0.0312f);

// SSR for wet surfaces
api.set_ssr_enabled(true);
api.set_reflection_mode(GameAPI::ReflectionMode::SSROnly);
```

### Performance Mode

```cpp
// Simple Reinhard tonemapping
api.set_tonemap_operator(GameAPI::TonemapOperator::Reinhard);
api.set_exposure(1.0f);

// Minimal bloom
api.set_bloom_enabled(true);
api.set_bloom_threshold(1.5f);
api.set_bloom_intensity(0.3f);

// Fast FXAA
api.set_fxaa_enabled(true);
api.set_fxaa_edge_threshold(0.333f);
api.set_fxaa_edge_threshold_min(0.1667f);

// Disable SSR
api.set_ssr_enabled(false);
```

### Stylized/Anime Look

```cpp
// Reinhard with low exposure (less washed out)
api.set_tonemap_operator(GameAPI::TonemapOperator::Reinhard);
api.set_exposure(0.9f);

// Moderate bloom for highlights
api.set_bloom_enabled(true);
api.set_bloom_threshold(1.2f);
api.set_bloom_intensity(0.6f);

// FXAA for clean edges
api.set_fxaa_enabled(true);
api.set_fxaa_edge_threshold(0.166f);

// No SSR (flatter look)
api.set_ssr_enabled(false);
```

### Night Scene

```cpp
// ACES with higher exposure to compensate
api.set_tonemap_operator(GameAPI::TonemapOperator::ACES);
api.set_exposure(1.5f);

// Strong bloom on lights
api.set_bloom_enabled(true);
api.set_bloom_threshold(0.7f);
api.set_bloom_intensity(1.0f);

// FXAA
api.set_fxaa_enabled(true);
api.set_fxaa_edge_threshold(0.166f);

// SSR for wet streets
api.set_ssr_enabled(true);
api.set_reflection_mode(GameAPI::ReflectionMode::SSROnly);
```

## Dynamic Adjustment

### Day/Night Cycle

```cpp
// Dawn: increase exposure
api.set_exposure(lerp(0.7f, 1.2f, dawn_progress));

// Night: reduce bloom threshold (make lights bloom more)
api.set_bloom_threshold(lerp(1.5f, 0.7f, night_progress));
```

### Camera Exposure Adaptation

```cpp
// Simulate camera auto-exposure based on average scene luminance
float target_exposure = calculate_auto_exposure(scene_luminance);
float current_exposure = api.get_exposure();
float adapted = lerp(current_exposure, target_exposure, dt * adaptation_speed);
api.set_exposure(adapted);
```

### Weather Effects

```cpp
// Foggy/overcast: reduce bloom and exposure
api.set_exposure(0.8f);
api.set_bloom_intensity(0.2f);

// Bright sunny day: increase bloom
api.set_exposure(1.3f);
api.set_bloom_intensity(0.7f);
```

## Performance Considerations

- **Tonemapping** — Negligible cost (single fullscreen pass)
- **Bloom** — Moderate cost (downsampling + blur + composite)
  - Disable if targeting low-end GPUs
- **FXAA** — Low cost (single fullscreen pass)
  - Cheaper than MSAA, good quality/performance tradeoff
- **SSR** — Moderate to high cost
  - Screen-space ray marching per pixel
  - RT modes require RT hardware

**Optimization Tips:**
- Disable bloom on low-end systems
- Use performance FXAA presets
- Prefer `SSROnly` over `RTOnly` unless quality is critical

## See Also

- [Render Passes](../RenderPasses.md) — Low-level post-processing passes
- [Shadows](Shadows.md) — Shadow quality settings
- [Render Graph](../RenderGraph.md) — Pass dependencies and resources
