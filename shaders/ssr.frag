#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Set 0: scene data (see input_structures.glsl)

// Set 1: SSR inputs
layout(set = 1, binding = 0) uniform sampler2D hdrColor;
layout(set = 1, binding = 1) uniform sampler2D posTex;
layout(set = 1, binding = 2) uniform sampler2D normalTex;
layout(set = 1, binding = 3) uniform sampler2D albedoTex;

vec3 getCameraWorldPosition()
{
    mat4 invView = inverse(sceneData.view);
    return vec3(invView[3]);
}

vec3 projectToScreen(vec3 worldPos)
{
    vec4 clip = sceneData.viewproj * vec4(worldPos, 1.0);
    if (clip.w <= 0.0)
    {
        return vec3(0.0, 0.0, -1.0);
    }
    vec3 ndc = clip.xyz / clip.w;
    if (ndc.x < -1.0 || ndc.x > 1.0 ||
        ndc.y < -1.0 || ndc.y > 1.0 ||
        ndc.z < 0.0  || ndc.z > 1.0)
    {
        return vec3(0.0, 0.0, -1.0);
    }

    vec2 uv = ndc.xy * 0.5 + 0.5;
    return vec3(uv, ndc.z);
}

void main()
{
    vec3 baseColor = texture(hdrColor, inUV).rgb;

    vec4 posSample = texture(posTex, inUV);
    if (posSample.w == 0.0)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    vec3 worldPos = posSample.xyz;

    vec4 normSample = texture(normalTex, inUV);
    vec3 N = normalize(normSample.xyz);
    float roughness = clamp(normSample.w, 0.04, 1.0);

    vec4 albSample = texture(albedoTex, inUV);
    vec3 albedo = albSample.rgb;
    float metallic = clamp(albSample.a, 0.0, 1.0);

    vec3 camPos = getCameraWorldPosition();
    vec3 V = normalize(camPos - worldPos);
    vec3 R = reflect(-V, N);

    float gloss = 1.0 - roughness;
    float reflectivity = gloss * mix(0.04, 1.0, metallic);

    if (reflectivity <= 0.05)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    if (dot(R, V) <= 0.0)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    const int   MAX_STEPS    = 64;
    const float STEP_LENGTH  = 0.5;   // world units per step
    const float MAX_DISTANCE = 50.0;  // clamp ray length
    const float THICKNESS    = 3.0;   // world-space thickness tolerance

    int maxSteps = int(mix(8.0, float(MAX_STEPS), reflectivity));

    bool  hit = false;
    vec2  hitUV = vec2(0.0);

    float t = STEP_LENGTH;
    for (int i = 0; i < maxSteps; ++i)
    {
        if (t > MAX_DISTANCE) break;

        vec3 samplePos = worldPos + R * t;
        vec3 proj = projectToScreen(samplePos);
        if (proj.z < 0.0)
        {
            break;
        }

        vec2 uv = proj.xy;
        vec4 scenePosSample = texture(posTex, uv);
        if (scenePosSample.w == 0.0)
        {
            t += STEP_LENGTH;
            continue;
        }

        // Compare distances along view direction as a simple intersection test.
        vec3 viewSample = (sceneData.view * vec4(samplePos, 1.0)).xyz;
        vec3 viewScene  = (sceneData.view * vec4(scenePosSample.xyz, 1.0)).xyz;

        float depthRay   = -viewSample.z;
        float depthScene = -viewScene.z;

        float depthDiff = depthRay - depthScene;

        if (depthRay > 0.0 && depthScene > 0.0 &&
        depthDiff > 0.0 && depthDiff < THICKNESS)
        {
            hit = true;
            hitUV = uv;
            break;
        }

        t += STEP_LENGTH;
    }

    vec3 result = baseColor;
    if (hit)
    {
        vec3 reflColor = texture(hdrColor, hitUV).rgb;

        float NoV = clamp(dot(N, V), 0.0, 1.0);
        float F0  = mix(0.04, 1.0, metallic);
        float F   = F0 + (1.0 - F0) * pow(1.0 - NoV, 5.0); // Schlick
        float gloss = 1.0 - roughness;

        float ssrVisibility = gloss;
        float weight = clamp(F * ssrVisibility, 0.0, 1.0);

        result = mix(baseColor, reflColor, weight);
    }

    outColor = vec4(result, 1.0);
}

