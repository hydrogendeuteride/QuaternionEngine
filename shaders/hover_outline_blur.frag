#version 450

layout(location = 0) in vec2 inUV;

layout(location = 0) out float outMask;

layout(set = 0, binding = 0) uniform sampler2D inputMask;

layout(push_constant) uniform constants
{
    vec2 inverseExtent;
    vec2 direction;
    float radiusPx;
    float _pad0;
    float _pad1;
    float _pad2;
} PushConstants;

void main()
{
    vec2 axis = PushConstants.inverseExtent * PushConstants.direction;
    float radiusPx = max(PushConstants.radiusPx, 0.5);
    float sigma = max(radiusPx * 0.5, 0.85);

    float accum = 0.0;
    float weightSum = 0.0;

    for (int i = -4; i <= 4; ++i)
    {
        float t = float(i) * 0.25;
        float offsetPx = radiusPx * t;
        float weight = exp(-0.5 * (offsetPx * offsetPx) / (sigma * sigma));
        accum += texture(inputMask, inUV + axis * offsetPx).r * weight;
        weightSum += weight;
    }

    outMask = (weightSum > 0.0) ? (accum / weightSum) : texture(inputMask, inUV).r;
}
