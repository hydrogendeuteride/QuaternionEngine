Materials and Textures Overview (PBR)

Current state (as of Nov 1, 2025)
- PBR textures bound per material (set=1):
  - binding=0: GLTFMaterialData (UBO)
  - binding=1: `colorTex` (albedo/base color) — sRGB
  - binding=2: `metalRoughTex` (G=roughness, B=metallic) — UNORM
  - binding=3: `normalMap` (tangent-space normal, UNORM)
- G‑Buffer writes world‑space normals. Tangent‑space normal maps are decoded with TBN using a sign‑correct bitangent (B = sign * cross(N, T)).
- Numeric fallbacks via `MaterialConstants` (CPU) / `GLTFMaterialData` (GPU):
  - `colorFactors` (RGBA). Defaults to 1 if zero.
  - `metal_rough_factors` (X=metallic, Y=roughness). Roughness is clamped to ≥ 0.04 in shaders.
  - `extra[0].x` = `normalScale` (scalar, default 1.0). Multiplies the XY of decoded normal.
- Defaults when a texture is missing:
  - Albedo → checkerboard error texture
  - MR → white (no effect)
  - Normal → 1×1 flat normal (0.5, 0.5, 1.0)

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
