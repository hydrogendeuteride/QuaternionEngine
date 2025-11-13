#version 450
#extension GL_GOOGLE_include_directive : require
#include "input_structures.glsl"

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

// IBL specular equirect 2D (LOD 0 for background)
layout(set=3, binding=0) uniform sampler2D iblSpec2D;

vec2 dir_to_equirect(vec3 d)
{
    d = normalize(d);
    float phi = atan(d.z, d.x);
    float theta = acos(clamp(d.y, -1.0, 1.0));
    return vec2(phi * (0.15915494309) + 0.5, theta * (0.31830988618));
}

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
