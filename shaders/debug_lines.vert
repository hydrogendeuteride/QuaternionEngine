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

void main()
{
    const uint segmentIndex = uint(gl_InstanceIndex);
    const uint segmentBase = segmentIndex * 2u;

    DebugVertex a = pc.vertexBuffer.vertices[segmentBase + 0u];
    DebugVertex b = pc.vertexBuffer.vertices[segmentBase + 1u];

    vec4 clipA = pc.viewproj * vec4(a.position, 1.0);
    vec4 clipB = pc.viewproj * vec4(b.position, 1.0);

    const float wa = max(abs(clipA.w), 1.0e-6);
    const float wb = max(abs(clipB.w), 1.0e-6);
    vec2 ndcA = clipA.xy / wa;
    vec2 ndcB = clipB.xy / wb;

    vec2 ndcDir = ndcB - ndcA;
    float ndcLen = length(ndcDir);
    vec2 normal = vec2(1.0, 0.0);
    if (ndcLen > 1.0e-6)
    {
        normal = vec2(-ndcDir.y, ndcDir.x) / ndcLen;
    }

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
