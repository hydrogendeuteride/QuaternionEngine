#version 450

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

layout(set=0, binding=0) uniform sampler2D uSrc;

layout(push_constant) uniform Push
{
    vec2 rect_min;   // normalized (0..1) min corner in swapchain UV space
    vec2 rect_size;  // normalized size in swapchain UV space
} pc;

void main()
{
    if (pc.rect_size.x <= 0.0 || pc.rect_size.y <= 0.0)
    {
        outColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 rect_max = pc.rect_min + pc.rect_size;
    vec2 uv = inUV;

    if (uv.x < pc.rect_min.x || uv.y < pc.rect_min.y || uv.x > rect_max.x || uv.y > rect_max.y)
    {
        outColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 local = (uv - pc.rect_min) / pc.rect_size;
    local = clamp(local, vec2(0.0), vec2(1.0));

    outColor = texture(uSrc, local);
}

