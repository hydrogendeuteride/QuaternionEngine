#version 450

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uColor;

layout(push_constant) uniform Push
{
    float inverse_width;
    float inverse_height;
    float edge_threshold;
    float edge_threshold_min;
} pc;

float luma(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

void main()
{
    vec2 texel = vec2(pc.inverse_width, pc.inverse_height);

    vec3 cM = texture(uColor, inUV).rgb;
    vec3 cN = texture(uColor, inUV + vec2(0.0,  texel.y)).rgb;
    vec3 cS = texture(uColor, inUV + vec2(0.0, -texel.y)).rgb;
    vec3 cE = texture(uColor, inUV + vec2( texel.x, 0.0)).rgb;
    vec3 cW = texture(uColor, inUV + vec2(-texel.x, 0.0)).rgb;

    float lM = luma(cM);
    float lN = luma(cN);
    float lS = luma(cS);
    float lE = luma(cE);
    float lW = luma(cW);

    float lMin = min(lM, min(min(lN, lS), min(lE, lW)));
    float lMax = max(lM, max(max(lN, lS), max(lE, lW)));
    float lRange = lMax - lMin;

    float threshold = max(pc.edge_threshold_min, pc.edge_threshold * lMax);
    if (lRange < threshold)
    {
        outColor = vec4(cM, 1.0);
        return;
    }

    vec3 avg = (cM + cN + cS + cE + cW) * 0.2;
    outColor = vec4(avg, 1.0);
}

