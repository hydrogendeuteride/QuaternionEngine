#version 450

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

layout(set=0, binding=0) uniform sampler2D uHdr;

layout(push_constant) uniform Push
{
    float exposure;
    int   mode;
    int   bloomEnabled;
    float bloomThreshold;
    float bloomIntensity;
} pc;

vec3 reinhard(vec3 x)
{
    return x / (1.0 + x);
}

// Narkowicz ACES approximation
vec3 aces_tonemap(vec3 x)
{
    // https://64.github.io/tonemapping/
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0);
}

void main()
{
    vec3 hdr = texture(uHdr, inUV).rgb;

    // Simple bloom in HDR space: gather bright neighbors and add a small blurred contribution.
    if (pc.bloomEnabled != 0)
    {
        vec2 texel = 1.0 / vec2(textureSize(uHdr, 0));
        vec3 bloom = vec3(0.0);
        int radius = 2;
        int count = 0;
        for (int x = -radius; x <= radius; ++x)
        {
            for (int y = -radius; y <= radius; ++y)
            {
                vec2 offset = vec2(x, y) * texel;
                vec3 c = texture(uHdr, clamp(inUV + offset, vec2(0.0), vec2(1.0))).rgb;
                float bright = max(max(c.r, c.g), c.b) - pc.bloomThreshold;
                if (bright > 0.0)
                {
                    bloom += c * bright;
                    count++;
                }
            }
        }
        if (count > 0)
        {
            bloom /= float(count);
        }
        hdr += pc.bloomIntensity * bloom;
    }

    // Simple exposure
    float exposure = max(pc.exposure, 0.0001);
    vec3 mapped = hdr * exposure;

    if (pc.mode == 1)
        mapped = aces_tonemap(mapped);
    else
        mapped = reinhard(mapped);

    const float gamma = 2.2;
    mapped = pow(mapped, vec3(1.0 / gamma));

    outColor = vec4(mapped, 1.0);
}

