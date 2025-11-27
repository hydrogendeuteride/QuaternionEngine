## Multi-Light System: Punctual Point Lights

Extends the rendering pipeline to support multiple dynamic point lights alongside the existing directional sun light. All lighting calculations use a shared PBR (Cook-Torrance) BRDF implementation.

- Files: `src/scene/vk_scene.h/.cpp`, `src/core/types.h`, `shaders/lighting_common.glsl`

### Data Structures

**CPU-side (C++)**

```cpp
// src/scene/vk_scene.h
struct PointLight {
    glm::vec3 position;
    float radius;       // falloff radius
    glm::vec3 color;
    float intensity;
};
```

**GPU-side (GLSL)**

```glsl
// shaders/input_structures.glsl
#define MAX_PUNCTUAL_LIGHTS 64

struct GPUPunctualLight {
    vec4 position_radius;   // xyz: world position, w: radius
    vec4 color_intensity;   // rgb: color, a: intensity
};
```

The `GPUSceneData` uniform buffer includes:
- `punctualLights[MAX_PUNCTUAL_LIGHTS]`: array of packed light data
- `lightCounts.x`: number of active point lights

### Scene API

`SceneManager` exposes a simple API to manage point lights at runtime:

| Method | Description |
|--------|-------------|
| `addPointLight(const PointLight& light)` | Appends a point light to the scene |
| `clearPointLights()` | Removes all point lights |
| `getPointLights()` | Returns const reference to current lights |

Example usage:
```cpp
SceneManager::PointLight light{};
light.position  = glm::vec3(5.0f, 3.0f, 0.0f);
light.radius    = 15.0f;
light.color     = glm::vec3(1.0f, 0.9f, 0.7f);
light.intensity = 20.0f;
sceneManager->addPointLight(light);
```

### GPU Upload

During `SceneManager::update_scene()`, point lights are packed into `GPUSceneData`:

1. Iterate over `pointLights` vector (capped at `kMaxPunctualLights = 64`)
2. Pack each light into `GPUPunctualLight` format
3. Zero-fill unused array slots
4. Set `lightCounts.x` to active count

This data is uploaded to the per-frame UBO and bound via `DescriptorManager::gpuSceneDataLayout()` at set 0, binding 0.

### Shader Implementation

**lighting_common.glsl**

Shared PBR functions extracted for reuse across all lighting passes:

| Function | Purpose |
|----------|---------|
| `fresnelSchlick()` | Fresnel term (Schlick approximation) |
| `DistributionGGX()` | Normal distribution function (GGX/Trowbridge-Reitz) |
| `GeometrySchlickGGX()` | Geometry term (Schlick-GGX) |
| `GeometrySmith()` | Combined geometry attenuation |
| `evaluate_brdf(N, V, L, albedo, roughness, metallic)` | Full Cook-Torrance BRDF evaluation |
| `eval_point_light(light, pos, N, V, albedo, roughness, metallic)` | Point light contribution with falloff |

**Point Light Falloff**

Uses smooth inverse-square attenuation with radius-based cutoff:
```glsl
float att = 1.0 / max(dist * dist, 0.0001);
float x   = clamp(dist / radius, 0.0, 1.0);
float smth = (1.0 - x * x);
smth *= smth;
float falloff = att * smth;
```

This provides physically-based falloff that smoothly reaches zero at the specified radius.

### Render Path Integration

**Deferred Lighting** (`deferred_lighting.frag`, `deferred_lighting_nort.frag`)

```glsl
// Directional sun
vec3 Lsun = normalize(-sceneData.sunlightDirection.xyz);
float sunVis = calcShadowVisibility(pos, N, Lsun);
vec3 sunBRDF = evaluate_brdf(N, V, Lsun, albedo, roughness, metallic);
vec3 direct = sunBRDF * sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * sunVis;

// Accumulate point lights
uint pointCount = sceneData.lightCounts.x;
for (uint i = 0u; i < pointCount; ++i) {
    direct += eval_point_light(sceneData.punctualLights[i], pos, N, V, albedo, roughness, metallic);
}
```

**Forward Pass** (`mesh.frag`)

Same accumulation loop, but without shadow visibility (used for transparent objects).

### Ray-Traced Point Light Shadows

When ray tracing is enabled (hybrid mode), the first 4 point lights receive RT shadows:

```glsl
if (sceneData.rtOptions.x == 1u && i < 4u) {
    // Cast shadow ray from surface toward light
    vec3 toL = light.position_radius.xyz - pos;
    // ... rayQueryInitializeEXT / rayQueryProceedEXT ...
    if (hit) contrib = vec3(0.0);
}
```

This provides accurate point light shadows without additional shadow maps.

### Performance Considerations

- **Light Count**: Up to 64 point lights supported; practical limit depends on scene complexity
- **Loop Unrolling**: Shader compilers typically unroll small loops; consider dynamic branching for very high light counts
- **Shadow Maps**: Point light shadows use RT only; traditional cubemap shadows are not implemented
- **Bandwidth**: Each light adds 32 bytes to the UBO; full array is 2KB

### Default Lights

`SceneManager::init()` creates two default point lights for testing:

| Light | Position | Radius | Color | Intensity |
|-------|----------|--------|-------|-----------|
| Warm Key | (0, 0, 0) | 25 | (1.0, 0.95, 0.8) | 15 |
| Cool Fill | (-10, 4, 10) | 20 | (0.6, 0.7, 1.0) | 10 |

Call `clearPointLights()` before adding your own lights to remove these defaults.

### Future Extensions

- Spot lights (add cone angle to `GPUPunctualLight`)
