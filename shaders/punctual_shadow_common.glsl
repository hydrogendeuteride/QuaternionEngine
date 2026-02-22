#ifndef PUNCTUAL_SHADOW_COMMON_GLSL
#define PUNCTUAL_SHADOW_COMMON_GLSL

// Shared punctual shadow sampling functions for point and spot lights.
// Requires: SHADOW_NORMAL_OFFSET (const float), spotShadowTex[], pointShadowTex[],
//           sceneData (GPUSceneData UBO) to be declared before inclusion.

uint select_point_shadow_face(vec3 d)
{
    vec3 ad = abs(d);
    if (ad.x >= ad.y && ad.x >= ad.z) return d.x >= 0.0 ? 0u : 1u;
    if (ad.y >= ad.x && ad.y >= ad.z) return d.y >= 0.0 ? 2u : 3u;
    return d.z >= 0.0 ? 4u : 5u;
}

float sample_spot_shadow_visibility(uint shadowIndex, vec3 localPos, vec3 N)
{
    if (shadowIndex >= sceneData.punctualShadowConfig.y)
    {
        return 1.0;
    }

    vec3 wp = localPos + N * SHADOW_NORMAL_OFFSET;
    vec4 clip = sceneData.spotLightShadowViewProj[shadowIndex] * vec4(wp, 1.0);
    vec3 ndc = clip.xyz / max(clip.w, 1e-6);
    vec2 uv = ndc.xy * 0.5 + 0.5;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))) || ndc.z < 0.0 || ndc.z > 1.0)
    {
        return 1.0;
    }

    ivec2 dim = textureSize(spotShadowTex[shadowIndex], 0);
    vec2 texel = 1.0 / vec2(max(dim, ivec2(1)));
    float bias = max(sceneData.punctualShadowParams.y, 1e-5);
    float sum = 0.0;
    const int radius = 1;
    const float taps = float((radius * 2 + 1) * (radius * 2 + 1));
    for (int y = -radius; y <= radius; ++y)
    {
        for (int x = -radius; x <= radius; ++x)
        {
            float mapD = texture(spotShadowTex[shadowIndex], uv + vec2(float(x), float(y)) * texel).r;
            sum += (ndc.z <= mapD + bias) ? 1.0 : 0.0;
        }
    }
    return sum / taps;
}

float sample_point_shadow_visibility(uint pointShadowIndex, vec3 lightPos, vec3 localPos, vec3 N)
{
    if (pointShadowIndex >= sceneData.punctualShadowConfig.z)
    {
        return 1.0;
    }

    vec3 wp = localPos + N * SHADOW_NORMAL_OFFSET;
    vec3 fromLight = wp - lightPos;
    uint face = select_point_shadow_face(fromLight);
    uint texIndex = pointShadowIndex * POINT_SHADOW_FACE_COUNT + face;
    if (texIndex >= MAX_POINT_SHADOW_FACES)
    {
        return 1.0;
    }

    vec4 clip = sceneData.pointLightShadowViewProj[texIndex] * vec4(wp, 1.0);
    vec3 ndc = clip.xyz / max(clip.w, 1e-6);
    vec2 uv = ndc.xy * 0.5 + 0.5;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))) || ndc.z < 0.0 || ndc.z > 1.0)
    {
        return 1.0;
    }

    ivec2 dim = textureSize(pointShadowTex[texIndex], 0);
    vec2 texel = 1.0 / vec2(max(dim, ivec2(1)));
    float bias = max(sceneData.punctualShadowParams.z, 1e-5);
    float sum = 0.0;
    const int radius = 1;
    const float taps = float((radius * 2 + 1) * (radius * 2 + 1));
    for (int y = -radius; y <= radius; ++y)
    {
        for (int x = -radius; x <= radius; ++x)
        {
            float mapD = texture(pointShadowTex[texIndex], uv + vec2(float(x), float(y)) * texel).r;
            sum += (ndc.z <= mapD + bias) ? 1.0 : 0.0;
        }
    }
    return sum / taps;
}

#endif // PUNCTUAL_SHADOW_COMMON_GLSL
