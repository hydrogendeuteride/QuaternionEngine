# Blackbody Emission Materials

Object-space procedural emission based on blackbody radiation (Kelvin to RGB). Designed for hot-metal effects like rocket nozzles, gun barrels, engine manifolds, and re-entry heating where the surface temperature varies spatially and animates over time.

## Overview

Blackbody materials extend the standard PBR material by repurposing the emissive texture slot (`emissiveTex`, set=1 binding=5) as a tileable noise texture. A shader-side heat function computes a `[0,1]` temperature factor from object-space coordinates and noise, maps it to a Kelvin range, converts to linear RGB via a piecewise blackbody fit, and outputs it as emissive color.

Two usage modes are supported:

1. **Standalone blackbody material** — a full PBR material with integrated blackbody emission, created via `create_or_update_blackbody_material()`. Used for procedural/primitive meshes.
2. **glTF material patch** — applies blackbody emission to an existing glTF material at runtime via `set_gltf_material_blackbody()`. Preserves the original PBR textures and only overwrites the emissive slot and blackbody constants.

## API

### Standalone Material

```cpp
// Create or update a named blackbody material (procedural/primitive meshes)
bool create_or_update_blackbody_material(const std::string &materialName,
                                         const BlackbodyMaterialSettings &settings);

// Read back current settings
bool get_blackbody_material(const std::string &materialName,
                            BlackbodyMaterialSettings &out) const;

// Remove a blackbody material (fails if still in use by a mesh)
bool remove_blackbody_material(const std::string &materialName);

// Apply a named blackbody material to a primitive mesh instance
bool apply_blackbody_material_to_primitive(const std::string &primitiveName,
                                           const std::string &materialName);
```

### glTF Material Patch

```cpp
// Patch an existing glTF material on a loaded instance to use blackbody emission
bool set_gltf_material_blackbody(const std::string &instanceName,
                                 const std::string &materialName,
                                 const BlackbodySettings &settings);
```

## BlackbodySettings

Core emission parameters shared by both standalone and glTF-patch modes.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `noisePath` | string | "" | Tileable noise texture relative to assets/ (bound to emissiveTex as linear) |
| `intensity` | float | 1.0 | Emission intensity multiplier (>1 triggers bloom) |
| `tempMinK` | float | 1000 | Minimum temperature in Kelvin (cold end of gradient) |
| `tempMaxK` | float | 4000 | Maximum temperature in Kelvin (hot end of gradient) |
| `noiseScale` | float | 1.0 | Noise frequency in object space (higher = finer detail) |
| `noiseContrast` | float | 1.0 | Noise contrast multiplier (higher = sharper heat boundaries) |
| `noiseScroll` | vec2 | (0, 0) | Noise UV scroll velocity in units/sec |
| `noiseSpeed` | float | 1.0 | Animation speed multiplier (0 = static, 1 = normal) |
| `heatAxisLocal` | vec3 | (0, 1, 0) | Local-space axis for directional heat falloff |
| `hotEndBias` | float | 1.0 | Which end is hot: -1 = -axis end, +1 = +axis end, 0 = both ends |
| `hotRangeStart` | float | 0.68 | Normalized axial position where hot zone begins |
| `hotRangeEnd` | float | 0.98 | Normalized axial position where hot zone ends |

## BlackbodyMaterialSettings

Full standalone material settings. Wraps `BlackbodySettings` plus standard PBR properties.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `colorFactor` | vec4 | (1,1,1,1) | Base color factor |
| `metallic` | float | 0.0 | Metallic factor (0-1) |
| `roughness` | float | 1.0 | Roughness factor (0.04-1) |
| `normalScale` | float | 1.0 | Normal map strength (0 = disabled) |
| `albedoPath` | string | "" | Albedo texture path |
| `albedoSRGB` | bool | true | Load albedo as sRGB |
| `metalRoughPath` | string | "" | Metallic-roughness texture path |
| `normalPath` | string | "" | Normal map texture path |
| `occlusionPath` | string | "" | Ambient occlusion texture path |
| `occlusionStrength` | float | 1.0 | AO influence (0-1) |
| `blackbody` | BlackbodySettings | {} | Emission parameters (see above) |

## Shader Pipeline

The blackbody emission is evaluated in both `gbuffer.frag` (deferred) and `mesh.frag` (forward/transparent) via a shared function in `blackbody.glsl`.

### Heat Evaluation

`nozzle_blackbody_heat()` computes a `[0,1]` heat factor from:

1. **Axial projection** — dot product of object-space position with `heatAxisLocal`, normalized to `[0,1]` and warped by edge noise
2. **Hot-zone masking** — smoothstep between `hotRangeStart` and `hotRangeEnd`, with `hotEndBias` selecting which end is hot
3. **Radial shell/core** — concentric falloff from axis center
4. **Triplanar noise** — object-space triplanar sampling of the noise texture with scrolling UVs
5. **Streak noise** — axial/circumferential noise layer blended with triplanar for organic streaks
6. **Turbulence** — exponential contrast applied to combined noise

### Temperature to Color

```glsl
float tempK = mix(tempMinK, tempMaxK, heat);
vec3 emission = blackbody_rgb_linear(tempK) * intensity * heat;
```

`blackbody_rgb_linear()` uses the Tanner Helland piecewise fit (sRGB) with a gamma 2.2 linearization. The resulting color is written to the emissive output in the G-buffer or composited directly in the forward pass.

### Material UBO Layout (extra[] slots)

| Slot | Contents |
|------|----------|
| `extra[9]` | (enable, intensity, tempMinK, tempMaxK) |
| `extra[10]` | (noiseScale, noiseContrast, scrollU, scrollV) |
| `extra[11]` | (heatAxisLocal.xyz, hotEndBias) |
| `extra[12]` | (noiseSpeed, 0, 0, 0) |
| `extra[13]` | (hotRangeStart, hotRangeEnd, 0, 0) |

When blackbody is enabled (`extra[9].x > 0.5`), `emissiveTex` is treated as noise (linear, tileable) rather than an emissive color map. Standard `emissiveFactor` (`extra[1]`) is zeroed to avoid double emission.

## Usage Examples

### Rocket Nozzle (Standalone Material)

```cpp
GameAPI::BlackbodyMaterialSettings nozzle;
nozzle.metallic  = 0.9f;
nozzle.roughness = 0.4f;
nozzle.albedoPath = "textures/metal/dark_steel.ktx2";
nozzle.metalRoughPath = "textures/metal/dark_steel_mr.ktx2";
nozzle.normalPath = "textures/metal/dark_steel_n.ktx2";

nozzle.blackbody.noisePath     = "vfx/perlin.ktx2";
nozzle.blackbody.intensity     = 4.0f;
nozzle.blackbody.tempMinK      = 800.0f;
nozzle.blackbody.tempMaxK      = 3500.0f;
nozzle.blackbody.noiseScale    = 2.0f;
nozzle.blackbody.noiseContrast = 1.2f;
nozzle.blackbody.noiseScroll   = glm::vec2(0.0f, -0.3f);
nozzle.blackbody.noiseSpeed    = 0.8f;
nozzle.blackbody.heatAxisLocal = glm::vec3(0.0f, -1.0f, 0.0f); // hot at -Y
nozzle.blackbody.hotEndBias    = 1.0f;
nozzle.blackbody.hotRangeStart = 0.6f;
nozzle.blackbody.hotRangeEnd   = 0.95f;

api.create_or_update_blackbody_material("nozzle_hot", nozzle);
api.apply_blackbody_material_to_primitive("engine_nozzle", "nozzle_hot");
```

### Gun Barrel Heating (glTF Patch)

```cpp
GameAPI::BlackbodySettings barrel;
barrel.noisePath     = "vfx/perlin.ktx2";
barrel.intensity     = 2.0f;
barrel.tempMinK      = 600.0f;
barrel.tempMaxK      = 2000.0f;
barrel.noiseScale    = 3.0f;
barrel.noiseContrast = 0.8f;
barrel.noiseScroll   = glm::vec2(0.0f, 0.0f);
barrel.noiseSpeed    = 0.5f;
barrel.heatAxisLocal = glm::vec3(0.0f, 0.0f, 1.0f); // hot at +Z (barrel tip)
barrel.hotEndBias    = 1.0f;
barrel.hotRangeStart = 0.7f;
barrel.hotRangeEnd   = 1.0f;

api.set_gltf_material_blackbody("weapon_instance", "barrel_metal", barrel);
```

### Dual-End Heating (Both Ends Hot)

```cpp
barrel.hotEndBias = 0.0f;  // both ends glow equally
barrel.hotRangeStart = 0.75f;
barrel.hotRangeEnd = 0.95f;
```

## Tuning Guide

| Parameter | Low | High | Visual Effect |
|-----------|-----|------|---------------|
| `intensity` | 0.5 | 8.0+ | Subtle glow to bloom-heavy incandescence |
| `tempMinK` | 500 | 1500 | Dark red base to bright orange base |
| `tempMaxK` | 2000 | 6000 | Orange-white peak to blue-white peak |
| `noiseScale` | 0.5 | 5.0 | Large blotchy heat patches to fine turbulent detail |
| `noiseContrast` | 0.2 | 2.0 | Smooth gradient to sharp heat boundaries |
| `noiseSpeed` | 0.0 | 2.0 | Static heat pattern to fast turbulent animation |
| `hotRangeStart/End` | 0.5/0.7 | 0.8/1.0 | Wide heat zone to narrow tip glow |
| `hotEndBias` | -1.0 | 1.0 | -axis end hot only / both ends / +axis end hot only |

### Temperature Reference

| Kelvin | Color | Typical Use |
|--------|-------|-------------|
| 800-1000 | Dark red | Barely glowing metal |
| 1500-2000 | Bright orange-red | Hot steel, exhaust manifold |
| 3000-3500 | Yellow-white | Nozzle throat, re-entry heating |
| 5000-6000 | White-blue | Arc welding, plasma |

## Texture Recommendations

- **Noise texture**: Tileable, grayscale, UNORM/linear format. Perlin noise works well for organic heat patterns. KTX2 with BC4 compression is ideal for single-channel noise.
- **UV/Object-space**: The heat function operates in object space, so results are independent of mesh UV layout. Mesh orientation relative to `heatAxisLocal` determines where the hot zone appears.

## Related

- [Mesh VFX](MeshVFX.md) — Animated procedural VFX materials (noise scrolling, fresnel)
- [Materials Overview](../materials.md) — PBR material system and `extra[]` layout
- [Shaders](../SHADERS.md) — Shader file reference and `blackbody.glsl` include
- [Particles](Particles.md) — GPU particle alternative for scattered fire/spark effects
