#version 450

#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_buffer_reference : require

#include "input_structures.glsl"

layout (location = 0) out vec3 outNormal;
layout (location = 1) out vec3 outColor;
layout (location = 2) out vec2 outUV;
layout (location = 3) out vec3 outWorldPos;
layout (location = 4) out vec4 outTangent; // xyz: world tangent, w: sign

struct Vertex {

    vec3 position;
    float uv_x;
    vec3 normal;
    float uv_y;
    vec4 color;
    vec4 tangent;
};

layout(buffer_reference, std430) readonly buffer VertexBuffer{
    Vertex vertices[];
};

//push constants block (must match GPUDrawPushConstants layout in C++)
layout(push_constant) uniform constants
{
    mat4 render_matrix;
    VertexBuffer vertexBuffer;
    uint objectID;
} PushConstants;

void main()
{
    Vertex v = PushConstants.vertexBuffer.vertices[gl_VertexIndex];

    vec4 worldPos = PushConstants.render_matrix * vec4(v.position, 1.0f);
    gl_Position = sceneData.viewproj * worldPos;

    outNormal = (PushConstants.render_matrix * vec4(v.normal, 0.f)).xyz;
    vec3 worldTangent = (PushConstants.render_matrix * vec4(v.tangent.xyz, 0.f)).xyz;
    outTangent = vec4(normalize(worldTangent), v.tangent.w);
    // Pass pure vertex color; apply baseColorFactor only in fragment
    outColor = v.color.xyz;
    outUV.x = v.uv_x;
    outUV.y = v.uv_y;
    outWorldPos = worldPos.xyz;
}
