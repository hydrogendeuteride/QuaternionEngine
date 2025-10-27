#version 450
#extension GL_GOOGLE_include_directive : require
#include "shadow_config.glsl"
#include "input_structures.glsl"

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

layout(set=1, binding=0) uniform sampler2D posTex;
layout(set=1, binding=1) uniform sampler2D normalTex;
layout(set=1, binding=2) uniform sampler2D albedoTex;
// Mixed near + CSM: shadowTex[0] is the near/simple map, 1..N-1 are cascades
layout(set=2, binding=0) uniform sampler2D shadowTex[SHADOW_CASCADE_COUNT];

const float PI = 3.14159265359;

float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z);
}

const vec2 POISSON_16[16] = vec2[16](
    vec2(0.2852, -0.1883), vec2(-0.1464, 0.2591),
    vec2(-0.3651, -0.0974), vec2(0.0901, 0.3807),
    vec2(0.4740, 0.0679), vec2(-0.0512, -0.4466),
    vec2(-0.4497, 0.1673), vec2(0.3347, 0.3211),
    vec2(0.1948, -0.4196), vec2(-0.2919, -0.3291),
    vec2(-0.0763, 0.4661), vec2(0.4421, -0.2217),
    vec2(0.0281, -0.2468), vec2(-0.2104, 0.0573),
    vec2(0.1197, 0.0779), vec2(-0.0905, -0.1203)
);

uint selectCascadeIndex(vec3 worldPos)
{
    // Compute view-space positive depth
    vec4 vpos = sceneData.view * vec4(worldPos, 1.0);
    float depthVS = -vpos.z;
    // 2-cascade split: [0, x) -> 0, [x, far] -> 1
    return depthVS < sceneData.cascadeSplitsView.x ? 0u : 1u;
}

float calcShadowVisibility(vec3 worldPos, vec3 N, vec3 L)
{
    uint ci = selectCascadeIndex(worldPos);
    mat4 lightMat = sceneData.lightViewProjCascades[ci];

    vec4 lclip = lightMat * vec4(worldPos, 1.0);
    vec3 ndc  = lclip.xyz / lclip.w;
    vec2 suv  = ndc.xy * 0.5 + 0.5;

    if (any(lessThan(suv, vec2(0.0))) || any(greaterThan(suv, vec2(1.0))))
        return 1.0;

    float current = clamp(ndc.z, 0.0, 1.0);

    float NoL       = max(dot(N, L), 0.0);
    float slopeBias = max(0.0006 * (1.0 - NoL), 0.0001);

    float dzdx = dFdx(current);
    float dzdy = dFdy(current);
    float ddz  = max(abs(dzdx), abs(dzdy));
    float bias = slopeBias + ddz * 0.75;

    ivec2 dim       = textureSize(shadowTex[ci], 0);
    vec2  texelSize = 1.0 / vec2(dim);

    float baseRadius = SHADOW_PCF_BASE_RADIUS;
    // Increase filter for farther cascade
    float radius     = mix(baseRadius, baseRadius * SHADOW_PCF_FAR_MULT, float(ci));

    float ang = hash12(suv * 4096.0) * 6.2831853;
    vec2  r   = vec2(cos(ang), sin(ang));
    mat2  rot = mat2(r.x, -r.y, r.y, r.x);

    const int TAP_COUNT = 16;
    float visible = 0.0;
    float wsum    = 0.0;

    for (int i = 0; i < TAP_COUNT; ++i)
    {
        vec2  pu   = rot * POISSON_16[i];
        vec2  off  = pu * radius * texelSize;

        float pr   = length(pu);
        float w    = 1.0 - smoothstep(0.0, 0.65, pr);

        float mapD = texture(shadowTex[ci], suv + off).r;
        float vis  = step(mapD, current + bias);

        visible += vis * w;
        wsum    += w;
    }

    float visibility = (wsum > 0.0) ? (visible / wsum) : 1.0;
    return visibility;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a      = roughness * roughness;
    float a2     = a * a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / max(denom, 0.001);
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;

    float denom = NdotV * (1.0 - k) + k;
    return NdotV / max(denom, 0.001);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float ggx2 = GeometrySchlickGGX(max(dot(N, V), 0.0), roughness);
    float ggx1 = GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
    return ggx1 * ggx2;
}

void main(){
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w == 0.0)
    {
        outColor = vec4(0.0);
        return;
    }

    vec3 pos = posSample.xyz;
    vec4 normalSample = texture(normalTex, inUV);
    vec3 N = normalize(normalSample.xyz);
    float roughness = clamp(normalSample.w, 0.04, 1.0);

    vec4 albedoSample = texture(albedoTex, inUV);
    vec3 albedo = albedoSample.rgb;
    float metallic = clamp(albedoSample.a, 0.0, 1.0);

    vec3 camPos = vec3(inverse(sceneData.view)[3]);
    vec3 V = normalize(camPos - pos);
    vec3 L = normalize(-sceneData.sunlightDirection.xyz);
    vec3 H = normalize(V + L);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 F  = fresnelSchlick(max(dot(H, V), 0.0), F0);
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);

    vec3 numerator    = NDF * G * F;
    float denom       = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
    vec3 specular     = numerator / max(denom, 0.001);

    vec3 kS = F;
    vec3 kD = (1.0 - kS) * (1.0 - metallic);

    float NdotL = max(dot(N, L), 0.0);
    // Shadowing (directional, reversed-Z shadow map)
    float visibility = calcShadowVisibility(pos, N, L);

    vec3 irradiance = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * NdotL * visibility;

    vec3 color = (kD * albedo / PI + specular) * irradiance;
    color += albedo * sceneData.ambientColor.rgb;

    outColor = vec4(color, 1.0);
}
