# Mesh VFX

Animated procedural VFX materials rendered in a dedicated forward pass. Designed for effects like engine exhaust, energy beams, shields, and flames that need scrolling noise, color gradients, and fresnel rim lighting.

## Overview

Mesh VFX materials are created and updated at runtime via `GameAPI::Engine`. They share the standard PBR descriptor layout but repurpose texture slots and `extra[]` constants for VFX parameters. The dedicated render pass sorts surfaces far-to-near and renders after particles, before transparent geometry.

## API

```cpp
// Create or update a named mesh VFX material
bool create_or_update_mesh_vfx_material(const std::string &materialName,
                                         const MeshVfxMaterialSettings &settings);

// Read back current settings
bool get_mesh_vfx_material(const std::string &materialName,
                           MeshVfxMaterialSettings &out);
```

## MeshVfxMaterialSettings

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `opacity` | float | 1.0 | Overall opacity multiplier |
| `fresnelPower` | float | 2.0 | Fresnel exponent (higher = tighter rim) |
| `fresnelStrength` | float | 1.0 | Fresnel intensity multiplier |
| `tint` | vec3 | (1,1,1) | Global color tint (multiplied with base texture) |
| `albedoPath` | string | "" | Base color / alpha mask texture (relative to assets/) |
| `albedoSRGB` | bool | true | Load albedo as sRGB |
| `noise1Path` | string | "" | First noise texture (bound to metalRoughTex, slot 2) |
| `noise2Path` | string | "" | Second noise texture (bound to normalMap, slot 3) |
| `noise1SRGB` | bool | false | Load noise1 as sRGB |
| `noise2SRGB` | bool | false | Load noise2 as sRGB |
| `scrollVelocity1` | vec2 | (0, -1) | UV scroll speed for noise1 (units/sec) |
| `scrollVelocity2` | vec2 | (0, -0.5) | UV scroll speed for noise2 (units/sec) |
| `distortionStrength` | float | 0.1 | Cross-distortion: noise1 offsets noise2's UVs |
| `noiseBlend` | float | 0.5 | Blend ratio between noise1 and noise2 (0=noise1 only, 1=noise2 only) |
| `coreColor` | vec3 | (1,1,1) | Color at gradient start (e.g. nozzle) |
| `edgeColor` | vec3 | (1, 0.5, 0) | Color at gradient end (e.g. flame tip) |
| `gradientAxis` | float | 1.0 | UV axis for gradient: 0=U, 1=V |
| `gradientStart` | float | 0.0 | Gradient begins here (in UV space) |
| `gradientEnd` | float | 1.0 | Gradient ends here (in UV space) |
| `emissionStrength` | float | 1.0 | Emission multiplier (>1 triggers bloom) |

## Shader Pipeline

The fragment shader (`shaders/mesh_vfx.frag`) composites in this order:

1. **Base texture** — albedo sampling with alpha cutoff
2. **Noise layer** — two noise textures scroll independently; noise1 cross-distorts noise2's UVs, then both are blended
3. **UV gradient** — smoothstep along the chosen axis interpolates `coreColor` → `edgeColor`
4. **Fresnel** — rim lighting based on view angle
5. **Composite** — `color = base × gradient × noise × (1 + fresnel) × emission`; `alpha = baseAlpha × opacity × (1 - grad) × noise`

Time is supplied via `sceneData.timeParams.x` (elapsed seconds, wraps at 10000s to preserve float precision).

## Usage Examples

### Engine Exhaust

```cpp
MeshVfxMaterialSettings exhaust;
exhaust.tint             = glm::vec3(1.0f, 0.6f, 0.2f);
exhaust.opacity          = 0.8f;
exhaust.albedoPath       = "textures/vfx/exhaust_base.png";
exhaust.noise1Path       = "textures/vfx/perlin_noise.png";
exhaust.noise2Path       = "textures/vfx/worley_noise.png";
exhaust.scrollVelocity1  = glm::vec2(0.0f, -1.5f);
exhaust.scrollVelocity2  = glm::vec2(0.1f, -0.8f);
exhaust.distortionStrength = 0.15f;
exhaust.noiseBlend       = 0.4f;
exhaust.gradientAxis     = 1.0f;
exhaust.gradientStart    = 0.2f;
exhaust.gradientEnd      = 0.9f;
exhaust.coreColor        = glm::vec3(1.0f, 0.9f, 0.7f);
exhaust.edgeColor        = glm::vec3(1.0f, 0.3f, 0.0f);
exhaust.fresnelPower     = 2.0f;
exhaust.fresnelStrength  = 1.5f;
exhaust.emissionStrength = 3.0f;

api.create_or_update_mesh_vfx_material("exhaust_flame", exhaust);
```

### Energy Shield (Static, No Animation)

```cpp
MeshVfxMaterialSettings shield;
shield.tint              = glm::vec3(0.3f, 0.6f, 1.0f);
shield.opacity           = 0.3f;
shield.fresnelPower      = 3.0f;
shield.fresnelStrength   = 2.0f;
shield.emissionStrength  = 2.0f;
// Disable animation
shield.scrollVelocity1   = glm::vec2(0.0f);
shield.scrollVelocity2   = glm::vec2(0.0f);
shield.distortionStrength = 0.0f;
shield.gradientStart     = 1.0f;
shield.gradientEnd       = 1.0f;
shield.coreColor         = glm::vec3(1.0f);
shield.edgeColor         = glm::vec3(1.0f);
// No noise textures — falls back to white (1.0)

api.create_or_update_mesh_vfx_material("energy_shield", shield);
```

## Disabling Animation

To use Mesh VFX as a static material (fresnel + tint only, no scrolling or noise):

1. Set `scrollVelocity1 = scrollVelocity2 = {0, 0}` — stops UV movement
2. Set `distortionStrength = 0` — no cross-distortion
3. Set `gradientStart = gradientEnd = 1.0` — disables gradient (keeps `alpha × (1-grad) = alpha × 1`)
4. Set `coreColor = edgeColor = {1, 1, 1}` — neutral gradient color
5. Omit `noise1Path` / `noise2Path` — fallback textures are white (1.0), so noise has no effect

## Tuning Guide

| Parameter | Low | High | Visual Effect |
|-----------|-----|------|---------------|
| `scrollVelocity` | 0.1 | 3.0 | Slow drift → fast streaming |
| `distortionStrength` | 0.0 | 0.3 | Clean edges → organic/molten warping |
| `noiseBlend` | 0.0 | 1.0 | Single noise → dual noise interplay |
| `gradientStart/End` | 0.0/0.3 | 0.5/1.0 | Wide core → narrow core with long fade |
| `emissionStrength` | 1.0 | 5.0+ | No bloom → strong bloom glow |
| `fresnelPower` | 1.0 | 5.0 | Wide rim → tight rim |

## Texture Recommendations

- **Noise textures**: Tileable, grayscale, UNORM format. Perlin and Worley noise work well together for organic effects.
- **Albedo**: Can be an alpha mask (e.g., circular gradient for exhaust cone) or a color pattern. Use sRGB.
- **UV layout**: The gradient axis follows mesh UVs, so orient the mesh UV V-axis along the effect direction (e.g., nozzle → tip).

## Related

- [Materials Overview](../materials.md) — PBR material system and `extra[]` layout
- [Shaders](../SHADERS.md) — Shader file reference and hot reload
- [Particles](Particles.md) — GPU particle alternative for scattered effects
- [Asset Manager](../asset_manager.md) — Texture loading and path resolution
