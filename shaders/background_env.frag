#version 450
#extension GL_GOOGLE_include_directive : require
#include "input_structures.glsl"
#include "ibl_common.glsl"

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

void main()
{
    // Reconstruct world-space direction from screen UV
    vec2 ndc = inUV * 2.0 - 1.0; // [-1,1]

    // Avoid per-pixel matrix inverses. With a perspective projection, a view-space ray can be
    // reconstructed directly from the projection diagonal and then rotated to world space.
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0));
    // view matrix is rigid-body => transpose(mat3(view)) preserves length for normalized vectors.
    vec3 worldDir = transpose(mat3(sceneData.view)) * viewDir;

    vec2 uv = dir_to_equirect_normalized(worldDir);
    // Sample a dedicated background environment map when available.
    // The engine binds iblBackground2D to a texture that may differ from the IBL specular map.
    vec3 col = textureLod(iblBackground2D, uv, 0.0).rgb;
    outColor = vec4(col, 1.0);
}
