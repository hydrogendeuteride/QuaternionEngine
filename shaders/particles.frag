#version 450

layout(location = 0) in vec4 v_color;
layout(location = 1) in vec2 v_uv;

layout(location = 0) out vec4 outColor;

void main()
{
    // Soft circular sprite.
    vec2 p = v_uv * 2.0 - 1.0;
    float r = length(p);
    float mask = smoothstep(1.0, 0.0, r);

    vec4 c = v_color;
    c.rgb *= mask;
    c.a *= mask;

    if (c.a <= 0.001)
    {
        discard;
    }

    outColor = c;
}

