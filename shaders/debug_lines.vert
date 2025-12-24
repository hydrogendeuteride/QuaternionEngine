#version 450

#extension GL_EXT_buffer_reference : require

layout(location = 0) out vec4 outColor;

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
    mat4 viewproj;
    DebugVertexBuffer vertexBuffer;
} pc;

void main()
{
    DebugVertex v = pc.vertexBuffer.vertices[gl_VertexIndex];
    gl_Position = pc.viewproj * vec4(v.position, 1.0);
    outColor = v.color;
}

