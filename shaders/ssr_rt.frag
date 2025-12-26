#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_ray_query : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Set 0: SceneData UBO (binding=0, declared in input_structures.glsl)
//         + optional TLAS for ray queries (binding=1).
layout(set = 0, binding = 1) uniform accelerationStructureEXT topLevelAS;

// Set 1: SSR inputs
layout(set = 1, binding = 0) uniform sampler2D hdrColor;
layout(set = 1, binding = 1) uniform sampler2D posTex;
layout(set = 1, binding = 2) uniform sampler2D normalTex;
layout(set = 1, binding = 3) uniform sampler2D albedoTex;

vec3 getCameraWorldPosition()
{
    mat3 rotT = mat3(sceneData.view);      // R^T
    mat3 rot  = transpose(rotT);           // R
    vec3 T    = sceneData.view[3].xyz;     // -R^T * C
    return -rot * T;                       // C = -R * T
}

float pow5(float x)
{
    float x2 = x * x;
    return x2 * x2 * x;
}

vec3 projectToScreenFromView(vec3 viewPos)
{
    vec4 clip = sceneData.proj * vec4(viewPos, 1.0);

    if (clip.w <= 0.0)
    {
        return vec3(0.0, 0.0, -1.0);
    }

    float invW = 1.0 / clip.w;
    vec3 ndc = clip.xyz * invW;

    if (ndc.x < -1.0 || ndc.x > 1.0 ||
        ndc.y < -1.0 || ndc.y > 1.0 ||
        ndc.z <  0.0 || ndc.z > 1.0)
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
    vec3 N          = normalize(normSample.xyz);
    float roughness = clamp(normSample.w, 0.04, 1.0);

    vec4 albSample  = texture(albedoTex, inUV);
    float metallic  = clamp(albSample.a, 0.0, 1.0);

    vec3 camPos = getCameraWorldPosition();
    vec3 V      = normalize(camPos - worldPos);
    vec3 R      = reflect(-V, N);
    vec3 viewPos = (sceneData.view * vec4(worldPos, 1.0)).xyz;
    vec3 viewDir = (sceneData.view * vec4(R, 0.0)).xyz;

    float gloss        = 1.0 - roughness;
    float F0           = mix(0.04, 1.0, metallic);
    float reflectivity = gloss * F0;

    if (reflectivity <= 0.05 || dot(R, V) <= 0.0)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    // Reflection mode (encoded in rtOptions.w):
    // 0 = SSR only, 1 = SSR + RT fallback, 2 = RT only
    uint reflMode = sceneData.rtOptions.w;
    bool useSSR = (reflMode == 0u || reflMode == 1u);
    bool useRT  = (reflMode >= 1u); // hybrid or RT-only

    vec3 result = baseColor;

    // -------------------------------------------------------------------------
    // 1) Screen-space reflections (SSR) via depth ray-march (optional).
    // -------------------------------------------------------------------------
    bool ssrHit = false;
    vec2 ssrUV  = vec2(0.0);

    if (useSSR)
    {
        const int   MAX_STEPS_SSR    = 64;
        const float STEP_LENGTH_SSR  = 0.5;   // world units per step
        const float MAX_DISTANCE_SSR = 50.0;  // clamp ray length
        const float THICKNESS_SSR    = 3.0;   // world-space thickness tolerance

        int maxSteps = int(mix(8.0, float(MAX_STEPS_SSR), reflectivity));
        float t = STEP_LENGTH_SSR;
        for (int i = 0; i < maxSteps && t <= MAX_DISTANCE_SSR; ++i, t += STEP_LENGTH_SSR)
        {
            vec3 sampleViewPos = viewPos + viewDir * t;

            vec3 proj = projectToScreenFromView(sampleViewPos);
            if (proj.z < 0.0)
            {
                break;
            }

            vec2 uv = proj.xy;
            vec4 scenePosSample = texture(posTex, uv);
            if (scenePosSample.w == 0.0)
            {
                continue;
            }

            vec3 viewScene  = (sceneData.view * vec4(scenePosSample.xyz, 1.0)).xyz;

            float depthRay   = -sampleViewPos.z;
            float depthScene = -viewScene.z;
            float depthDiff  = depthRay - depthScene;

            if (depthRay > 0.0 && depthScene > 0.0 &&
                depthDiff > 0.0 && depthDiff < THICKNESS_SSR)
            {
                ssrHit = true;
                ssrUV  = uv;
                break;
            }
        }
    }

    // If SSR hits and we are not in RT-only mode, use it as the primary reflection.
    if (ssrHit && reflMode != 2u)
    {
        vec3 reflColor = texture(hdrColor, ssrUV).rgb;

        float NoV = clamp(dot(N, V), 0.0, 1.0);
        float F   = F0 + (1.0 - F0) * pow5(1.0 - NoV); // Schlick
        float ssrVisibility = gloss;

        float weight = clamp(F * ssrVisibility, 0.0, 1.0);
        result = mix(baseColor, reflColor, weight);
        outColor = vec4(result, 1.0);
        return;
    }

    // If RT is disabled for reflections, we are done.
    if (!useRT)
    {
        outColor = vec4(result, 1.0);
        return;
    }

    // -------------------------------------------------------------------------
    // 2) Ray-traced reflections using TLAS and ray queries (1 ray / pixel).
    // -------------------------------------------------------------------------
    const float RT_TMIN        = 0.05;
    const float RT_TMAX        = 50.0;
    const float RT_ORIGIN_BIAS = 0.05;

    vec3 origin = worldPos + N * RT_ORIGIN_BIAS;

    rayQueryEXT rq;
    rayQueryInitializeEXT(
        rq,
        topLevelAS,
        gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
        0xFF,
        origin,
        RT_TMIN,
        R,
        RT_TMAX
    );

    while (rayQueryProceedEXT(rq)) { }
    bool rtHit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);

    if (rtHit)
    {
        float tHit = rayQueryGetIntersectionTEXT(rq, true);
        vec3 hitPos = origin + R * tHit;

        vec3 proj = projectToScreenFromView((sceneData.view * vec4(hitPos, 1.0)).xyz);
        if (proj.z >= 0.0)
        {
            vec2 hitUV = proj.xy;
            vec3 reflColor = texture(hdrColor, hitUV).rgb;

            float NoV = clamp(dot(N, V), 0.0, 1.0);
            float F   = F0 + (1.0 - F0) * pow5(1.0 - NoV); // Schlick
            float rtVisibility = gloss;

            float weight = clamp(F * rtVisibility, 0.0, 1.0);
            result = mix(baseColor, reflColor, weight);
        }
    }

    outColor = vec4(result, 1.0);
}
