// Maximum number of shadow cascades supported in shaders
#define MAX_CASCADES 4
// Maximum number of punctual (point) lights
#define MAX_PUNCTUAL_LIGHTS 64

struct GPUPunctualLight {
    vec4 position_radius;
    vec4 color_intensity;
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
    // Ray-query settings (packed)
    // rtOptions.x = enabled (1/0)
    // rtOptions.y = cascade bitmask (bit i => cascade i assisted)
    uvec4 rtOptions;
    // rtParams.x = N·L threshold; others reserved
    vec4  rtParams;

    GPUPunctualLight punctualLights[MAX_PUNCTUAL_LIGHTS];
    uvec4 lightCounts;
} sceneData;

layout(set = 1, binding = 0) uniform GLTFMaterialData{

    vec4 colorFactors;
    vec4 metal_rough_factors;
    vec4 extra[14];

} materialData;

layout(set = 1, binding = 1) uniform sampler2D colorTex;
layout(set = 1, binding = 2) uniform sampler2D metalRoughTex;
layout(set = 1, binding = 3) uniform sampler2D normalMap; // tangent-space normal, UNORM
