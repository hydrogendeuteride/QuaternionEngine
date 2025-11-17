#version 450

#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_buffer_reference : require

#include "input_structures.glsl"

struct Vertex {
    vec3 position; float uv_x;
    vec3 normal;   float uv_y;
    vec4 color;
    vec4 tangent;
};

layout(buffer_reference, std430) readonly buffer VertexBuffer{
    Vertex vertices[];
};

layout(push_constant) uniform PushConsts {
    mat4 render_matrix;
    VertexBuffer vertexBuffer;
    uint objectID;
    uint cascadeIndex; // which cascade this pass renders
    // pad to 16-byte boundary implicitly
} PC;

void main()
{
    Vertex v = PC.vertexBuffer.vertices[gl_VertexIndex];
    vec4 worldPos = PC.render_matrix * vec4(v.position, 1.0);
    // Use cascaded matrix; cascade 0 is the legacy near/simple map
    uint ci = min(PC.cascadeIndex, uint(3));
    gl_Position = sceneData.lightViewProjCascades[ci] * worldPos;
}

