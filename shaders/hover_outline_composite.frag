#version 450

layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 outFragColor;

layout(set = 0, binding = 0) uniform sampler2D baseColorTex;
layout(set = 1, binding = 0) uniform sampler2D rawMaskTex;
layout(set = 2, binding = 0) uniform sampler2D blurredMaskTex;

layout(push_constant) uniform constants
{
    vec4 colorIntensity;
    vec4 params; // x: outlineWidthPx, y: blurRadiusPx
} PushConstants;

void main()
{
    vec4 base = texture(baseColorTex, inUV);
    float rawMask = texture(rawMaskTex, inUV).r;
    float blurredMask = texture(blurredMaskTex, inUV).r;

    float ring = max(blurredMask - rawMask, 0.0);

    float widthPx = max(PushConstants.params.x, 0.25);
    float blurPx = max(PushConstants.params.y, 0.5);
    float widthFactor = clamp(widthPx / blurPx, 0.25, 8.0);

    float outline = smoothstep(0.0, 0.25 / widthFactor, ring);
    float glow = smoothstep(0.0, 0.35 / blurPx, ring) * 0.6;
    float highlight = clamp(max(outline, glow), 0.0, 1.0);

    vec3 color = base.rgb + PushConstants.colorIntensity.rgb * PushConstants.colorIntensity.a * highlight;
    outFragColor = vec4(color, base.a);
}
