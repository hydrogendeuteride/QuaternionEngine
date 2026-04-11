#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;

layout(location = 0) out vec4 outCloudLighting;
layout(location = 1) out vec4 outCloudSegment;

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

int segment_count(vec4 segments)
{
    int count = 0;
    if (segment_valid(segments.xy)) count++;
    if (segment_valid(segments.zw)) count++;
    return count;
}

float segment_total_length(vec4 segments)
{
    float total = 0.0;
    if (segment_valid(segments.xy)) total += max(segments.y - segments.x, 0.0);
    if (segment_valid(segments.zw)) total += max(segments.w - segments.z, 0.0);
    return total;
}

float segment_error(vec4 lhs, vec4 rhs)
{
    int lhsCount = segment_count(lhs);
    int rhsCount = segment_count(rhs);
    if (lhsCount != rhsCount) return 1e20;

    float error = 0.0;
    if (lhsCount >= 1)
    {
        error += abs(lhs.x - rhs.x) + abs(lhs.y - rhs.y);
    }
    if (lhsCount == 2)
    {
        error += abs(lhs.z - rhs.z) + abs(lhs.w - rhs.w);
    }
    return error;
}

float segment_weighted_midpoint(vec4 segments)
{
    float len0 = segment_valid(segments.xy) ? max(segments.y - segments.x, 0.0) : 0.0;
    float len1 = segment_valid(segments.zw) ? max(segments.w - segments.z, 0.0) : 0.0;
    float totalLen = max(len0 + len1, 1e-3);

    float weightedMid = 0.0;
    if (len0 > 0.0) weightedMid += (0.5 * (segments.x + segments.y)) * len0;
    if (len1 > 0.0) weightedMid += (0.5 * (segments.z + segments.w)) * len1;
    return weightedMid / totalLen;
}

void main()
{
    vec4 currentLighting = texture(cloudLightingCurrentTex, inUV);
    vec4 currentSegment = texture(cloudSegmentCurrentTex, inUV);

    outCloudLighting = currentLighting;
    outCloudSegment = currentSegment;

    int currentCount = segment_count(currentSegment);
    if (pc.misc.x == 0 || currentCount == 0)
    {
        return;
    }

    vec3 rd = normalize(inWorldRay);
    float midT = segment_weighted_midpoint(currentSegment);
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
    vec4 historySegment = texture(cloudSegmentHistoryTex, prevUV);
    if (segment_count(historySegment) == 0)
    {
        return;
    }

    float currentLen = max(segment_total_length(currentSegment), 1e-3);
    float segmentMismatch = segment_error(historySegment, currentSegment);
    if (segmentMismatch > max(2500.0, currentLen * 0.35 * float(currentCount)))
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
