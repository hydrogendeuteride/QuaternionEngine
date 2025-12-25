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

void accum_bloom(vec3 c, float kernel_weight, inout vec3 bloom, inout float weight_sum)
{
    float bright = max(max(c.r, c.g), c.b) - pc.bloomThreshold;
    bright = max(bright, 0.0);

    // Match the old behavior: only normalize over samples that pass the threshold.
    float contribute = step(1e-5, bright);

    bloom += c * bright * kernel_weight;
    weight_sum += kernel_weight * contribute;
}

void main()
{
    vec3 hdr = texture(uHdr, inUV).rgb;

    // Simple bloom in HDR space: approximate a 5x5 Gaussian blur using 9 bilinear samples (vs. 25 taps).
    if (pc.bloomEnabled != 0 && pc.bloomIntensity > 0.0)
    {
        vec2 texel = 1.0 / vec2(textureSize(uHdr, 0));
        vec2 d = texel * 1.2; // Combines 1- and 2-texel taps via linear filtering (4:1 weight).

        vec3 bloom = vec3(0.0);
        float wsum = 0.0;

        // 1D weights [1 4 6 4 1] collapsed to 3 linear samples => weights [5 6 5]
        // 2D separable => center 36, axis 30, corners 25 (sum 256).
        accum_bloom(hdr, 36.0, bloom, wsum); // reuse center sample

        accum_bloom(texture(uHdr, clamp(inUV + vec2( d.x, 0.0), vec2(0.0), vec2(1.0))).rgb, 30.0, bloom, wsum);
        accum_bloom(texture(uHdr, clamp(inUV + vec2(-d.x, 0.0), vec2(0.0), vec2(1.0))).rgb, 30.0, bloom, wsum);
        accum_bloom(texture(uHdr, clamp(inUV + vec2(0.0,  d.y), vec2(0.0), vec2(1.0))).rgb, 30.0, bloom, wsum);
        accum_bloom(texture(uHdr, clamp(inUV + vec2(0.0, -d.y), vec2(0.0), vec2(1.0))).rgb, 30.0, bloom, wsum);

        accum_bloom(texture(uHdr, clamp(inUV + vec2( d.x,  d.y), vec2(0.0), vec2(1.0))).rgb, 25.0, bloom, wsum);
        accum_bloom(texture(uHdr, clamp(inUV + vec2(-d.x,  d.y), vec2(0.0), vec2(1.0))).rgb, 25.0, bloom, wsum);
        accum_bloom(texture(uHdr, clamp(inUV + vec2( d.x, -d.y), vec2(0.0), vec2(1.0))).rgb, 25.0, bloom, wsum);
        accum_bloom(texture(uHdr, clamp(inUV + vec2(-d.x, -d.y), vec2(0.0), vec2(1.0))).rgb, 25.0, bloom, wsum);

        if (wsum > 0.0)
        {
            bloom /= wsum;
            hdr += pc.bloomIntensity * bloom;
        }
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
