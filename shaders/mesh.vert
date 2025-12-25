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
    mat3 normal_matrix;
    VertexBuffer vertexBuffer;
    uint objectID;
} PushConstants;

void main()
{
    Vertex v = PushConstants.vertexBuffer.vertices[gl_VertexIndex];

    mat3 normalMatrix = PushConstants.normal_matrix;

    vec4 worldPos = PushConstants.render_matrix * vec4(v.position, 1.0);
    gl_Position = sceneData.viewproj * worldPos;

    outNormal = normalize(normalMatrix * v.normal);

    vec3 worldTangent = normalize(normalMatrix * v.tangent.xyz);
    outTangent = vec4(worldTangent, v.tangent.w);

    outColor = v.color.xyz;
    outUV    = vec2(v.uv_x, v.uv_y);
    outWorldPos = worldPos.xyz;
}
