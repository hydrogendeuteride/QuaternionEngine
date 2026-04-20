#version 450

#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 2) in vec2 inUV;

layout(location = 0) out float outMask;

void main()
{
    vec4 baseTex = texture(colorTex, inUV);
    float alpha = clamp(baseTex.a * materialData.colorFactors.a, 0.0, 1.0);
    float alphaCutoff = materialData.extra[2].x;
    if (alphaCutoff > 0.0 && alpha < alphaCutoff)
    {
        discard;
    }

    outMask = 1.0;
}
