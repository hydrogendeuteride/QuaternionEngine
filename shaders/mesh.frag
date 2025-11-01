#version 450

#extension GL_GOOGLE_include_directive : require
#include "input_structures.glsl"

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec2 inUV;
layout (location = 3) in vec3 inWorldPos;
layout (location = 4) in vec4 inTangent;

layout (location = 0) out vec4 outFragColor;

const float PI = 3.14159265359;

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
    return NdotV / denom;
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float ggx2 = GeometrySchlickGGX(max(dot(N, V), 0.0), roughness);
    float ggx1 = GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
    return ggx1 * ggx2;
}

void main()
{
    // Base color with material factor and texture
    vec4 baseTex = texture(colorTex, inUV);
    vec3 albedo = inColor * baseTex.rgb * materialData.colorFactors.rgb;
    // glTF: metallicRoughnessTexture uses G=roughness, B=metallic
    vec2 mrTex = texture(metalRoughTex, inUV).gb;
    float roughness = clamp(mrTex.x * materialData.metal_rough_factors.y, 0.04, 1.0);
    float metallic  = clamp(mrTex.y * materialData.metal_rough_factors.x, 0.0, 1.0);

    // Normal mapping path for forward/transparent pipeline
    vec3 Nm = texture(normalMap, inUV).xyz * 2.0 - 1.0;
    float normalScale = max(materialData.extra[0].x, 0.0);
    Nm.xy *= normalScale;
    vec3 Nn = normalize(inNormal);
    vec3 T = normalize(inTangent.xyz);
    vec3 B = normalize(cross(Nn, T)) * inTangent.w;
    vec3 N = normalize(T * Nm.x + B * Nm.y + Nn * Nm.z);
    vec3 camPos = vec3(inverse(sceneData.view)[3]);
    vec3 V = normalize(camPos - inWorldPos);
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
    vec3 kD = vec3(1.0) - kS;
    kD *= 1.0 - metallic;

    float NdotL = max(dot(N, L), 0.0);
    vec3 irradiance = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * NdotL;

    vec3 color = (kD * albedo / PI + specular) * irradiance;
    color += albedo * sceneData.ambientColor.rgb;

    // Alpha from baseColor texture and factor (glTF spec)
    float alpha = clamp(baseTex.a * materialData.colorFactors.a, 0.0, 1.0);
    outFragColor = vec4(color, alpha);
}
