#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;

layout(location = 0) out vec4 outCloudLighting;
layout(location = 1) out vec2 outCloudSegment;

layout(set = 1, binding = 0) uniform sampler2D cloudLightingCurrentTex;
layout(set = 1, binding = 1) uniform sampler2D cloudSegmentCurrentTex;
layout(set = 1, binding = 2) uniform sampler2D cloudLightingHistoryTex;
layout(set = 1, binding = 3) uniform sampler2D cloudSegmentHistoryTex;

layout(push_constant) uniform CloudTemporalPush
{
    mat4 previous_view_proj;
    vec4 origin_delta_blend;
    vec4 viewport_params;
    ivec4 misc;
} pc;

bool segment_valid(vec2 segment)
{
    return segment.x < segment.y;
}

void main()
{
    vec4 currentLighting = texture(cloudLightingCurrentTex, inUV);
    vec2 currentSegment = texture(cloudSegmentCurrentTex, inUV).rg;

    outCloudLighting = currentLighting;
    outCloudSegment = currentSegment;

    if (pc.misc.x == 0 || !segment_valid(currentSegment))
    {
        return;
    }

    vec3 rd = normalize(inWorldRay);
    float midT = 0.5 * (currentSegment.x + currentSegment.y);
    vec3 currentMidpoint = inCamLocal + rd * midT;
    vec3 previousMidpoint = currentMidpoint + pc.origin_delta_blend.xyz;

    vec4 prevClip = pc.previous_view_proj * vec4(previousMidpoint, 1.0);
    if (prevClip.w <= 1e-5)
    {
        return;
    }

    vec2 prevUV = prevClip.xy / prevClip.w * 0.5 + 0.5;
    if (any(lessThan(prevUV, vec2(0.0))) || any(greaterThanEqual(prevUV, vec2(1.0))))
    {
        return;
    }

    vec4 historyLighting = texture(cloudLightingHistoryTex, prevUV);
    vec2 historySegment = texture(cloudSegmentHistoryTex, prevUV).rg;
    if (!segment_valid(historySegment))
    {
        return;
    }

    float currentLen = max(currentSegment.y - currentSegment.x, 1e-3);
    float segmentError = abs(historySegment.x - currentSegment.x) + abs(historySegment.y - currentSegment.y);
    if (segmentError > max(2500.0, currentLen * 0.35))
    {
        return;
    }

    ivec2 texel = ivec2(gl_FragCoord.xy) - ivec2(1);
    ivec2 size = textureSize(cloudLightingCurrentTex, 0);
    vec4 neighborhoodMin = vec4(1e20);
    vec4 neighborhoodMax = vec4(-1e20);
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            ivec2 sampleCoord = clamp(texel + ivec2(x, y), ivec2(0), size - ivec2(1));
            vec4 sampleLighting = texelFetch(cloudLightingCurrentTex, sampleCoord, 0);
            neighborhoodMin = min(neighborhoodMin, sampleLighting);
            neighborhoodMax = max(neighborhoodMax, sampleLighting);
        }
    }

    vec4 clampedHistory = clamp(historyLighting, neighborhoodMin, neighborhoodMax);
    float blend = clamp(pc.origin_delta_blend.w, 0.0, 0.98);
    outCloudLighting = mix(currentLighting, clampedHistory, blend);
    outCloudSegment = currentSegment;
}
