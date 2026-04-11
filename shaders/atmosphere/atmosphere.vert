#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) out vec2 outUV;
layout(location = 1) out vec3 outWorldRay;
layout(location = 2) flat out vec3 outCamLocal;

void main()
{
    vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    vec2 uvs[3] = vec2[3](vec2(0.0, 0.0), vec2(2.0, 0.0), vec2(0.0, 2.0));

    vec2 uv = uvs[gl_VertexIndex];
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
    outUV = uv;

    mat3 view_to_world = transpose(mat3(sceneData.view));
    vec2 ndc = uv * 2.0 - 1.0;
    vec3 viewRay = vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0);
    outWorldRay = view_to_world * viewRay;
    outCamLocal = -view_to_world * sceneData.view[3].xyz;
}
