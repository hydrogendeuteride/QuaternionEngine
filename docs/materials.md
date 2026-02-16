Materials and Textures Overview (PBR)

- PBR textures bound per material (set=1):
  - binding=0: GLTFMaterialData (UBO)
  - binding=1: `colorTex` (albedo/base color) — sRGB
  - binding=2: `metalRoughTex` (G=roughness, B=metallic) — UNORM
  - binding=3: `normalMap` (tangent-space normal, UNORM)
  - binding=4: `occlusionTex` (ambient occlusion, R channel) — UNORM
  - binding=5: `emissiveTex` (emissive RGB) — sRGB
- G‑Buffer writes world‑space normals. Tangent‑space normal maps are decoded with TBN using a sign‑correct bitangent (B = sign * cross(N, T)).
- Numeric fallbacks via `MaterialConstants` (CPU) / `GLTFMaterialData` (GPU):
  - `colorFactors` (RGBA). Defaults to 1 if zero.
  - `metal_rough_factors` (X=metallic, Y=roughness). Roughness is clamped to ≥ 0.04 in shaders.
  - `extra[0].x` = `normalScale` (scalar, default 1.0). Multiplies the XY of decoded normal.
  - `extra[0].y` = `aoStrength` (scalar, 0–1). Controls AO influence.
  - `extra[0].z` = `hasAO` (flag, 1 = use AO texture, 0 = skip).
  - `extra[1].rgb` = `emissiveFactor` (vec3). Multiplied with emissive texture.
  - `extra[2].x` = `alphaCutoff` (scalar). For MASK alpha mode.
- Defaults when a texture is missing:
  - Albedo → checkerboard error texture
  - MR → white (no effect)
  - Normal → 1×1 flat normal (0.5, 0.5, 1.0)
  - Occlusion → 1×1 white (AO = 1.0, no occlusion)
  - Emissive → 1×1 black (no emission)

Implications for primitive meshes
- Primitives can use:
  - Albedo + MR + Normal textures, or
  - Numeric factors only, or
  - Any mix (missing textures fall back to defaults).

Texture conventions
- Albedo/base color: sRGB.
- Metallic-Roughness: UNORM; channels: G=roughness, B=metallic.
- Normal map: UNORM; expected +Y (green up). If your maps look inverted, flip the green channel offline.

Notes on tangent space
- Tangents are loaded from glTF when present (`TANGENT` attribute, vec4 where w is handedness).
- If missing, the engine generates tangents:
  - Default: robust Gram–Schmidt with handedness.
  - Preferred: MikkTSpace (enabled by CMake option `ENABLE_MIKKTS=ON`).

Notes on MikkTSpace
- Recommended for parity with content tools. Enable at configure time: `-DENABLE_MIKKTS=ON`.
- Falls back automatically to the internal generator if disabled or if MikkTS fails.

Usage Examples
- Adjust normal strength per material: set `material.constants.extra[0].x` (CPU) or `normalTexture.scale` in glTF.
- Primitive with PBR textures:
  - Set `MeshMaterialDesc::Kind::Textured` and fill `albedoPath`, `metalRoughPath`, and `normalPath`.

Mesh VFX Materials

Mesh VFX is a dedicated forward-rendered material pass for animated procedural effects (engine exhaust, energy beams, shields, etc.). It uses the same `GLTFMaterialData` UBO but repurposes texture bindings and `extra[]` slots for VFX-specific parameters.

- Texture bindings (set=1, shared layout with PBR):
  - binding=1: `colorTex` — base color / alpha mask (albedo)
  - binding=2: `metalRoughTex` — **noise texture 1** (R channel sampled)
  - binding=3: `normalMap` — **noise texture 2** (R channel sampled, cross-distorted by noise 1)
  - Fallback: `_whiteImage` for albedo/noise1, `_flatNormalImage` for noise2

- `extra[]` layout for Mesh VFX:
  - `extra[2].x` — alpha cutoff
  - `extra[3]` — (opacity, fresnelPower, fresnelStrength, _)
  - `extra[4]` — (tint.rgb, _)
  - `extra[5]` — (scrollVelocity1.xy, scrollVelocity2.xy) — noise UV scroll speeds (units/sec)
  - `extra[6]` — (distortionStrength, noiseBlend, gradientAxis, emissionStrength)
  - `extra[7]` — (coreColor.rgb, gradientStart)
  - `extra[8]` — (edgeColor.rgb, gradientEnd)

- Time: `sceneData.timeParams.x` (elapsed seconds, wraps at 10000s) drives UV scrolling.

- Shader compositing order:
  1. Base texture sampling with alpha cutoff
  2. Dual noise with UV scrolling and cross-distortion
  3. UV-axis gradient (coreColor → edgeColor via smoothstep)
  4. Fresnel rim highlighting
  5. Final: `color = base × gradient × noise × (1 + fresnel) × emission`, `alpha = baseAlpha × opacity × (1-grad) × noise`

- Render ordering: sorted far→near by camera depth in a dedicated pass (`src/render/passes/mesh_vfx.cpp`), after particles and before transparent geometry.

- To disable animation effects, set `scrollVelocity1/2 = {0,0}`, `distortionStrength = 0`, `gradientStart = gradientEnd = 1.0`, and omit noise textures (they fall back to white = 1.0).

G-Buffer Outputs
- The geometry pass (`gbuffer.frag`) writes 4 render targets:
  - `outPos` (location 0): World position (xyz) + valid flag (w=1).
  - `outNorm` (location 1): World normal (xyz) + roughness (w).
  - `outAlbedo` (location 2): Albedo (rgb) + metallic (a).
  - `outExtra` (location 3): AO (x) + emissive (yzw).
- Deferred lighting reads these and computes:
  ```glsl
  vec3 color = direct + indirect * ao + emissive;
  ```
