#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_buffer_reference : require

#include "input_structures.glsl"
#include "ibl_common.glsl"
#include "lighting_common.glsl"
#include "planet_shadow.glsl"

layout(location = 0) in vec3 inBaseNormal;
layout(location = 1) in vec2 inUV;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in float inSeaRadius;

layout(location = 0) out vec4 outColor;

layout(set = 2, binding = 0) uniform sampler2D transmittanceLut;

struct Vertex
{
    vec3 position;
    float uv_x;
    vec3 normal;
    float uv_y;
    vec4 color;
    vec4 tangent;
};

layout(buffer_reference, std430) readonly buffer VertexBuffer
{
    Vertex vertices[];
};

layout(push_constant) uniform OceanPush
{
    mat4 render_matrix;
    vec4 body_center_radius;
    vec4 shell_params;
    vec4 atmosphere_center_radius;
    vec4 atmosphere_params;
    vec4 beta_rayleigh;
    vec4 beta_mie;
    vec4 beta_absorption;
    VertexBuffer vertexBuffer;
} pc;

vec3 getCameraLocalPosition()
{
    mat3 rotT = mat3(sceneData.view);
    mat3 rot = transpose(rotT);
    vec3 t = sceneData.view[3].xyz;
    return -rot * t;
}

float sample_wave_height(vec3 unitDir, float radius, float timeSeconds)
{
    const vec3 dirA = normalize(vec3(0.82, 0.0, 0.57));
    const vec3 dirB = normalize(vec3(-0.47, 0.0, 0.88));
    const vec3 dirC = normalize(vec3(0.21, 0.0, -0.98));
    const float wavelengthA = 2500.0;
    const float wavelengthB = 350.0;
    const float wavelengthC = 90.0;
    const float amplitudeA = 0.55;
    const float amplitudeB = 0.18;
    const float amplitudeC = 0.05;
    const float weightA = 0.55;
    const float weightB = 0.30;
    const float weightC = 0.15;
    const float speedA = 18.0;
    const float speedB = 5.0;
    const float speedC = 2.2;

    float phaseA = dot(unitDir, dirA) * radius * (2.0 * PI / wavelengthA) + timeSeconds * speedA * (2.0 * PI / wavelengthA);
    float phaseB = dot(unitDir, dirB) * radius * (2.0 * PI / wavelengthB) + timeSeconds * speedB * (2.0 * PI / wavelengthB);
    float phaseC = dot(unitDir, dirC) * radius * (2.0 * PI / wavelengthC) + timeSeconds * speedC * (2.0 * PI / wavelengthC);

    return weightA * amplitudeA * sin(phaseA) +
           weightB * amplitudeB * sin(phaseB) +
           weightC * amplitudeC * sin(phaseC);
}

vec2 sample_wave_slopes(vec3 unitDir, vec3 east, vec3 north, float radius, float timeSeconds)
{
    const float sampleStepMeters = 24.0;

    vec3 dirE = normalize(unitDir * radius + east * sampleStepMeters);
    vec3 dirN = normalize(unitDir * radius + north * sampleStepMeters);

    float h0 = sample_wave_height(unitDir, radius, timeSeconds);
    float hE = sample_wave_height(dirE, radius, timeSeconds);
    float hN = sample_wave_height(dirN, radius, timeSeconds);

    return vec2((hE - h0) / sampleStepMeters, (hN - h0) / sampleStepMeters);
}

vec3 compute_wave_normal(vec3 unitDir, float radius, float timeSeconds, float coverage)
{
    vec3 anchor = (abs(unitDir.y) < 0.98) ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 east = normalize(cross(anchor, unitDir));
    vec3 north = normalize(cross(unitDir, east));
    vec2 slopes = sample_wave_slopes(unitDir, east, north, radius, timeSeconds);

    float slopeScale = mix(0.0, 4.2, coverage);
    vec3 n = unitDir - east * slopes.x * slopeScale - north * slopes.y * slopeScale;
    return normalize(n);
}

vec2 sample_air_mass(vec3 positionLocal, vec3 rayDir, vec3 center, float planetRadius, float atmRadius)
{
    float r = length(positionLocal - center);
    if (planetRadius <= 0.0 || atmRadius <= planetRadius || r <= 0.0)
    {
        return vec2(0.0);
    }

    vec3 up = (positionLocal - center) / r;
    float mu = dot(up, normalize(rayDir));
    float u = clamp(mu * 0.5 + 0.5, 0.0, 1.0);
    float v = clamp((r - planetRadius) / max(atmRadius - planetRadius, 1.0e-4), 0.0, 1.0);
    return textureLod(transmittanceLut, vec2(u, v), 0.0).rg;
}

vec3 sample_atmosphere_transmittance(vec3 startLocal,
                                     vec3 rayDir,
                                     vec3 cameraLocal,
                                     vec3 center,
                                     float planetRadius,
                                     float atmRadius,
                                     float Hr,
                                     float Hm,
                                     vec3 betaR,
                                     vec3 betaM,
                                     vec3 betaA,
                                     bool segmentToCamera)
{
    if (planetRadius <= 0.0 || atmRadius <= planetRadius)
    {
        return vec3(1.0);
    }

    vec2 airMass = sample_air_mass(startLocal, rayDir, center, planetRadius, atmRadius);
    if (segmentToCamera)
    {
        float cameraRadius = length(cameraLocal - center);
        if (cameraRadius > planetRadius && cameraRadius < atmRadius)
        {
            airMass -= sample_air_mass(cameraLocal, rayDir, center, planetRadius, atmRadius);
        }
    }
    airMass = max(airMass, vec2(0.0));

    vec2 opticalDepth = airMass * vec2(Hr, Hm);
    return exp(-(betaR * opticalDepth.x + betaM * opticalDepth.y + betaA * opticalDepth.x));
}

vec3 evaluate_sky_reflection(vec3 reflectDir,
                             vec3 localUp,
                             vec3 sunDir,
                             vec3 sunColor,
                             vec3 ambientColor,
                             vec3 skyTransmittance,
                             bool atmosphereActive)
{
    float upDot = dot(reflectDir, localUp);
    float skyVisibility = smoothstep(-0.10, 0.03, upDot);
    if (skyVisibility <= 0.0)
    {
        return vec3(0.0);
    }

    float up01 = clamp(upDot * 0.5 + 0.5, 0.0, 1.0);
    float horizon = pow(1.0 - abs(upDot), 1.6);
    float sunAlign = max(dot(reflectDir, sunDir), 0.0);

    vec3 ambientBase = max(ambientColor, vec3(0.02));
    vec3 zenithColor = ambientBase * vec3(0.45, 0.70, 1.25) + sunColor * 0.015;
    vec3 horizonColor = ambientBase * vec3(0.95, 1.00, 1.05) + sunColor * 0.055;
    vec3 sky = mix(horizonColor, zenithColor, pow(up01, 0.65));

    vec3 horizonWarm = sunColor * pow(sunAlign, 10.0) * (0.22 + 0.18 * horizon);
    vec3 sunHalo = sunColor * (pow(sunAlign, 160.0) * 0.22 + pow(sunAlign, 24.0) * 0.05);
    sky += horizonWarm + sunHalo;

    if (atmosphereActive)
    {
        sky *= skyTransmittance;
    }

    return sky * skyVisibility;
}

vec3 evaluate_ocean_specular(vec3 N, vec3 V, vec3 L, float roughness, vec3 F0)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    if (NdotV <= 0.0 || NdotL <= 0.0)
    {
        return vec3(0.0);
    }

    vec3 H = normalize(V + L);
    vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);
    float D = DistributionGGX(N, H, roughness);
    float G = GeometrySmith(N, V, L, roughness);

    return (D * G * F / max(4.0 * NdotV * NdotL, 0.001)) * NdotL;
}

void main()
{
    float rawMask = texture(planetSpecularTex, inUV).r;
    float strength = clamp(materialData.extra[2].w, 0.0, 1.0);
    float waterMask = rawMask * strength;
    if (waterMask <= 0.001)
    {
        discard;
    }

    float coverage = smoothstep(0.005, 0.08, waterMask);
    float baseRoughness = clamp(materialData.extra[3].w, 0.04, 1.0);

    vec3 baseNormal = normalize(inBaseNormal);
    vec3 N = compute_wave_normal(baseNormal, inSeaRadius, sceneData.timeParams.x, coverage);
    vec3 cameraLocal = getCameraLocalPosition();
    vec3 V = normalize(cameraLocal - inWorldPos);
    vec3 Lsun = normalize(-sceneData.sunlightDirection.xyz);

    float sunVis = 1.0;
    if (sceneData.rtParams.y > 0.0)
    {
        sunVis = planet_analytic_shadow_visibility(inWorldPos + N * 0.01, Lsun);
        sunVis = max(sunVis, clamp(sceneData.shadowTuning.x, 0.0, 1.0));
    }

    const vec3 F0 = vec3(0.02);
    float lobe0 = baseRoughness;
    float lobe1 = clamp(baseRoughness * 0.18, 0.012, 0.06);
    vec3 directSpec = evaluate_ocean_specular(N, V, Lsun, lobe0, F0);
    directSpec += evaluate_ocean_specular(N, V, Lsun, lobe1, F0) * (1.75 * strength);

    vec3 directTransmittance = vec3(1.0);
    vec3 viewTransmittance = vec3(1.0);
    vec3 skyTransmittance = vec3(1.0);
    bool atmosphereActive = (pc.shell_params.y > 0.5) &&
                            (pc.atmosphere_center_radius.w > 0.0) &&
                            (pc.atmosphere_params.x > pc.atmosphere_center_radius.w) &&
                            (pc.atmosphere_params.w > 0.0);
    if (atmosphereActive)
    {
        vec3 betaR = max(pc.beta_rayleigh.rgb, vec3(0.0));
        vec3 betaM = max(pc.beta_mie.rgb, vec3(0.0));
        vec3 betaA = max(pc.beta_absorption.rgb, vec3(0.0));
        if (any(greaterThan(betaR + betaM + betaA, vec3(0.0))))
        {
            vec3 center = pc.atmosphere_center_radius.xyz;
            float planetRadius = pc.atmosphere_center_radius.w;
            float atmRadius = pc.atmosphere_params.x;
            float Hr = max(pc.atmosphere_params.y, 1.0);
            float Hm = max(pc.atmosphere_params.z, 1.0);
            directTransmittance = sample_atmosphere_transmittance(
                inWorldPos,
                Lsun,
                cameraLocal,
                center,
                planetRadius,
                atmRadius,
                Hr,
                Hm,
                betaR,
                betaM,
                betaA,
                false);
            vec3 Rsky = reflect(-V, N);
            skyTransmittance = sample_atmosphere_transmittance(
                inWorldPos,
                Rsky,
                cameraLocal,
                center,
                planetRadius,
                atmRadius,
                Hr,
                Hm,
                betaR,
                betaM,
                betaA,
                false);
            viewTransmittance = sample_atmosphere_transmittance(
                inWorldPos,
                V,
                cameraLocal,
                center,
                planetRadius,
                atmRadius,
                Hr,
                Hm,
                betaR,
                betaM,
                betaA,
                true);
        }
    }

    vec3 direct = directSpec * sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * sunVis *
                  directTransmittance * viewTransmittance;

    vec3 R = reflect(-V, N);
    float NdotV = max(dot(N, V), 0.0);
    vec3 localUp = atmosphereActive
        ? normalize(inWorldPos - pc.atmosphere_center_radius.xyz)
        : normalize(baseNormal);
    vec3 skyColor = evaluate_sky_reflection(
        R,
        localUp,
        Lsun,
        sceneData.sunlightColor.rgb * sceneData.sunlightColor.a,
        sceneData.ambientColor.rgb,
        skyTransmittance,
        atmosphereActive);
    vec3 fresnel = fresnelSchlick(NdotV, F0);
    vec3 skyReflection = skyColor * fresnel * viewTransmittance;

    vec3 waterTint = vec3(0.012, 0.026, 0.041);
    vec3 grazingTint = waterTint * mix(0.14, 0.03, NdotV) * viewTransmittance;

    vec3 color = (direct + skyReflection + grazingTint) * coverage;
    outColor = vec4(color, 1.0);
}
