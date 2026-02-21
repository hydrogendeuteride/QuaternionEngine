#version 450

#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_buffer_reference : require

struct Vertex {
    vec3 position; float uv_x;
    vec3 normal;   float uv_y;
    vec4 color;
    vec4 tangent;
};

layout(buffer_reference, std430) readonly buffer VertexBuffer
{
    Vertex vertices[];
};

layout(push_constant) uniform PushConsts
{
    mat4 light_mvp;
    VertexBuffer vertexBuffer;
    uint objectID;
    uint _pad;
} PC;

void main()
{
    Vertex v = PC.vertexBuffer.vertices[gl_VertexIndex];
    gl_Position = PC.light_mvp * vec4(v.position, 1.0);
}
