#version 450

layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 outFragColor;

layout(set = 0, binding = 0) uniform sampler2D baseColorTex;
layout(set = 1, binding = 0) uniform sampler2D hoverRawMaskTex;
layout(set = 2, binding = 0) uniform sampler2D hoverBlurredMaskTex;
layout(set = 3, binding = 0) uniform sampler2D selectionRawMaskTex;
layout(set = 4, binding = 0) uniform sampler2D selectionBlurredMaskTex;

layout(push_constant) uniform constants
{
    vec4 hoverColorIntensity;
    vec4 selectionColorIntensity;
    vec4 params; // x: hoverWidthPx, y: hoverBlurPx, z: selectionWidthPx, w: selectionBlurPx
} PushConstants;

float compute_highlight(float rawMask, float blurredMask, float widthPx, float blurPx)
{
    float ring = max(blurredMask - rawMask, 0.0);
    float safeWidthPx = max(widthPx, 0.25);
    float safeBlurPx = max(blurPx, 0.5);
    float widthFactor = clamp(safeWidthPx / safeBlurPx, 0.25, 8.0);

    float outline = smoothstep(0.0, 0.25 / widthFactor, ring);
    float glow = smoothstep(0.0, 0.35 / safeBlurPx, ring) * 0.6;
    return clamp(max(outline, glow), 0.0, 1.0);
}

void main()
{
    vec4 base = texture(baseColorTex, inUV);

    float hoverRawMask = texture(hoverRawMaskTex, inUV).r;
    float hoverBlurredMask = texture(hoverBlurredMaskTex, inUV).r;
    float selectionRawMask = texture(selectionRawMaskTex, inUV).r;
    float selectionBlurredMask = texture(selectionBlurredMaskTex, inUV).r;

    float hoverHighlight = compute_highlight(hoverRawMask, hoverBlurredMask, PushConstants.params.x, PushConstants.params.y);
    float selectionHighlight = compute_highlight(selectionRawMask,
                                                 selectionBlurredMask,
                                                 PushConstants.params.z,
                                                 PushConstants.params.w);

    vec3 color = base.rgb;
    color += PushConstants.hoverColorIntensity.rgb * PushConstants.hoverColorIntensity.a * hoverHighlight;
    color += PushConstants.selectionColorIntensity.rgb * PushConstants.selectionColorIntensity.a * selectionHighlight;
    outFragColor = vec4(color, base.a);
}
