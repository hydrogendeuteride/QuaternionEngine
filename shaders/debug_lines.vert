#version 450

#extension GL_EXT_buffer_reference : require

layout(location = 0) out vec4 outColor;
layout(location = 1) out float outSide;
layout(location = 2) flat out float outEdgeSoftness;

struct DebugVertex
{
    vec3 position;
    float _pad0;
    vec4 color;
};

layout(buffer_reference, std430) readonly buffer DebugVertexBuffer
{
    DebugVertex vertices[];
};

layout(push_constant) uniform DebugPush
{
    layout(offset = 0) mat4 viewproj;
    layout(offset = 64) DebugVertexBuffer vertexBuffer;
    layout(offset = 72) vec2 invViewportSizeNdc;
    layout(offset = 80) float halfLineWidthPx;
    layout(offset = 84) float aaPx;
} pc;

const int kEndpointIndex[6] = int[6](0, 0, 1, 1, 0, 1);
const float kSideSign[6] = float[6](-1.0, 1.0, -1.0, -1.0, 1.0, 1.0);
const float kClipPlaneEpsilon = 1.0e-4;
const float kDirectionEpsilon = 1.0e-6;

float near_plane_margin(vec4 clipPos)
{
    return clipPos.w - clipPos.z;
}

bool clip_segment_to_near_plane(inout vec4 clipA, inout vec4 clipB)
{
    const float marginA = near_plane_margin(clipA);
    const float marginB = near_plane_margin(clipB);
    const bool aVisible = marginA > kClipPlaneEpsilon;
    const bool bVisible = marginB > kClipPlaneEpsilon;
    if (!aVisible && !bVisible)
    {
        return false;
    }
    if (aVisible && bVisible)
    {
        return true;
    }

    const float denom = marginB - marginA;
    const float t = abs(denom) > kDirectionEpsilon
                            ? clamp((kClipPlaneEpsilon - marginA) / denom, 0.0, 1.0)
                            : (aVisible ? 0.0 : 1.0);
    vec4 clipped = mix(clipA, clipB, t);
    clipped.z = min(clipped.z, clipped.w - kClipPlaneEpsilon);

    if (aVisible)
    {
        clipB = clipped;
    }
    else
    {
        clipA = clipped;
    }
    return true;
}

vec2 safe_ndc_xy(vec4 clipPos)
{
    return clipPos.xy / max(clipPos.w, kClipPlaneEpsilon);
}

vec2 stable_line_normal(vec4 clipA, vec4 clipB)
{
    const vec2 dirNumerator = clipB.xy * clipA.w - clipA.xy * clipB.w;
    const float dirLen = length(dirNumerator);
    if (dirLen > kDirectionEpsilon)
    {
        return vec2(-dirNumerator.y, dirNumerator.x) / dirLen;
    }

    const vec2 midpointNdc = 0.5 * (safe_ndc_xy(clipA) + safe_ndc_xy(clipB));
    const float midpointLen = length(midpointNdc);
    if (midpointLen > kDirectionEpsilon)
    {
        return vec2(-midpointNdc.y, midpointNdc.x) / midpointLen;
    }

    return vec2(0.0, 1.0);
}

void main()
{
    const uint segmentIndex = uint(gl_InstanceIndex);
    const uint segmentBase = segmentIndex * 2u;

    DebugVertex a = pc.vertexBuffer.vertices[segmentBase + 0u];
    DebugVertex b = pc.vertexBuffer.vertices[segmentBase + 1u];

    vec4 clipA = pc.viewproj * vec4(a.position, 1.0);
    vec4 clipB = pc.viewproj * vec4(b.position, 1.0);
    if (!clip_segment_to_near_plane(clipA, clipB))
    {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        outColor = vec4(0.0);
        outSide = 0.0;
        outEdgeSoftness = 1.0;
        return;
    }

    const vec2 normal = stable_line_normal(clipA, clipB);

    const int localVertex = gl_VertexIndex % 6;
    const int endpoint = kEndpointIndex[localVertex];
    const float side = kSideSign[localVertex];

    vec4 clipPos = endpoint == 0 ? clipA : clipB;
    DebugVertex src = endpoint == 0 ? a : b;

    const float halfWidthPx = max(pc.halfLineWidthPx, 0.5);
    vec2 offsetNdc = normal * side * pc.invViewportSizeNdc * halfWidthPx;
    clipPos.xy += offsetNdc * clipPos.w;

    gl_Position = clipPos;
    outColor = src.color;
    outSide = side;
    outEdgeSoftness = clamp(1.0 - (max(pc.aaPx, 0.0) / halfWidthPx), 0.0, 1.0);
}
