#version 450

layout(location = 0) in vec4 inColor;
layout(location = 1) in float inSide;
layout(location = 2) flat in float inEdgeSoftness;
layout(location = 0) out vec4 outColor;

void main()
{
    outColor = inColor;
    const float edgeDist = abs(inSide);
    const float coverage = 1.0 - smoothstep(inEdgeSoftness, 1.0, edgeDist);
    outColor.a *= coverage;
    if (outColor.a <= 0.0)
    {
        discard;
    }
}
