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
    vec4 clip = vec4(ndc, 1.0, 1.0);
    vec4 vpos = inverse(sceneData.proj) * clip;
    vec3 viewDir = normalize(vpos.xyz / max(vpos.w, 1e-6));
    vec3 worldDir = normalize((inverse(sceneData.view) * vec4(viewDir, 0.0)).xyz);

    vec2 uv = dir_to_equirect(worldDir);
    vec3 col = textureLod(iblSpec2D, uv, 0.0).rgb;
    outColor = vec4(col, 1.0);
}
