#version 450
#extension GL_GOOGLE_include_directive : require
#include "input_structures.glsl"
#include "ibl_common.glsl"

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

void main()
{
    vec2 ndc = inUV * 2.0 - 1.0; // [-1,1]

    // Avoid per-pixel matrix inverses
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0));

    vec3 worldDir = transpose(mat3(sceneData.view)) * viewDir;

    vec2 uv = dir_to_equirect_normalized(worldDir);

    vec3 col = textureLod(iblBackground2D, uv, 0.0).rgb;
    outColor = vec4(col, 1.0);
}
