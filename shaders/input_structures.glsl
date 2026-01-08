// Maximum number of shadow cascades supported in shaders
#define MAX_CASCADES 4
// Maximum number of analytic planet occluders for directional sun shadows
#define MAX_PLANET_OCCLUDERS 4
// Maximum number of punctual (point) lights
#define MAX_PUNCTUAL_LIGHTS 64
// Maximum number of spot lights
#define MAX_SPOT_LIGHTS 32

struct GPUPunctualLight {
    vec4 position_radius;
    vec4 color_intensity;
};

struct GPUSpotLight {
    vec4 position_radius;      // xyz: position, w: radius
    vec4 direction_cos_outer;  // xyz: direction (unit), w: cos(outer_angle)
    vec4 color_intensity;      // rgb: color, a: intensity
    vec4 cone;                 // x: cos(inner_angle), yzw: unused
};

layout(set = 0, binding = 0) uniform  SceneData{

    mat4 view;
    mat4 proj;
    mat4 viewproj;
    // Legacy single shadow matrix (used for near range in mixed mode)
    mat4 lightViewProj;
    vec4 ambientColor;
    vec4 sunlightDirection; //w for sun power
    vec4 sunlightColor;

    // Cascaded shadow matrices (0 = near/simple map, 1..N-1 = CSM)
    mat4 lightViewProjCascades[4];
    // View-space split distances for selecting cascades (x,y,z,w)
    vec4 cascadeSplitsView;
    // Ray-query & reflection settings (packed)
    // rtOptions.x = RT shadows enabled (1/0)
    // rtOptions.y = cascade bitmask (bit i => cascade i assisted)
    uvec4 rtOptions;
    // rtParams.x = N·L threshold for hybrid shadows
    // rtParams.y = shadows enabled flag (1.0 = on, 0.0 = off)
    // rtParams.z = planet receiver clipmap shadow maps enabled flag (RT-only mode)
    // rtParams.w = sun angular radius (radians) for analytic planet shadow penumbra
    vec4  rtParams;

    GPUPunctualLight punctualLights[MAX_PUNCTUAL_LIGHTS];
    GPUSpotLight spotLights[MAX_SPOT_LIGHTS];
    // lightCounts.x = point light count
    // lightCounts.y = spot light count
    // lightCounts.z = planet occluder count (analytic directional sun shadow)
    uvec4 lightCounts;

    // Analytic planet shadow occluders (max 4):
    // planetOccluders[i].xyz = center in render-local space, w = radius in meters.
    vec4 planetOccluders[MAX_PLANET_OCCLUDERS];
} sceneData;

layout(set = 1, binding = 0) uniform GLTFMaterialData{

    vec4 colorFactors;
    vec4 metal_rough_factors;
    vec4 extra[14];

} materialData;
// Convention (selected fields used by engine shaders):
// - extra[0].x: normalScale (0 disables normal mapping)
// - extra[0].y: occlusionStrength (0..1), extra[0].z: hasAO (1/0)
// - extra[1].rgb: emissiveFactor
// - extra[2].x: alphaCutoff (>0 enables alpha test for MASK materials)
// - extra[2].y: gbuffer flags (planet-style: >0 => force clipmap receiver shadows in RT-only)

layout(set = 1, binding = 1) uniform sampler2D colorTex;
layout(set = 1, binding = 2) uniform sampler2D metalRoughTex;
layout(set = 1, binding = 3) uniform sampler2D normalMap;   // tangent-space normal, UNORM
layout(set = 1, binding = 4) uniform sampler2D occlusionTex; // occlusion (R channel)
layout(set = 1, binding = 5) uniform sampler2D emissiveTex;  // emissive (RGB, sRGB)
