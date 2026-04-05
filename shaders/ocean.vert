#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_buffer_reference : require

#include "input_structures.glsl"

layout(location = 0) out vec3 outBaseNormal;
layout(location = 1) out vec2 outUV;
layout(location = 2) out vec3 outWorldPos;
layout(location = 3) out float outSeaRadius;

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

layout(push_constant) uniform constants
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
} PushConstants;

void main()
{
    Vertex v = PushConstants.vertexBuffer.vertices[gl_VertexIndex];

    vec3 bodyCenter = PushConstants.body_center_radius.xyz;
    float seaRadius = PushConstants.body_center_radius.w + PushConstants.shell_params.x;

    vec3 patchLocal = (PushConstants.render_matrix * vec4(v.position, 1.0)).xyz;
    vec3 radial = patchLocal - bodyCenter;
    vec3 unitDir = normalize(radial);
    vec3 oceanPos = bodyCenter + unitDir * seaRadius;

    gl_Position = sceneData.viewproj * vec4(oceanPos, 1.0);

    outBaseNormal = unitDir;
    outUV = vec2(v.uv_x, v.uv_y);
    outWorldPos = oceanPos;
    outSeaRadius = seaRadius;
}
