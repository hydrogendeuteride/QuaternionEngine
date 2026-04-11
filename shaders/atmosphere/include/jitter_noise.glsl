float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float sample_jitter_noise_screen()
{
    ivec3 size_i = textureSize(jitterNoiseTex, 0);
    vec3 size = max(vec3(size_i), vec3(1.0));
    uint miscPacked = uint(pc.misc.w);
    float frameIndex = float((miscPacked >> MISC_JITTER_FRAME_SHIFT) & 0xFFu);
    float slice = mod(frameIndex, size.z);
    vec3 uvw = vec3(fract(gl_FragCoord.xy / size.xy), (slice + 0.5) / size.z);
    return textureLod(jitterNoiseTex, uvw, 0.0).r;
}

float resolve_jitter_sample(vec2 screenUv)
{
    uint miscPacked = uint(pc.misc.w);
    int flags = int(miscPacked & MISC_FLAGS_MASK);
    float jitterSample = hash12(screenUv * 1024.0);
    if ((flags & FLAG_JITTER_BLUE_NOISE) != 0)
    {
        jitterSample = sample_jitter_noise_screen();
    }
    return mix(0.5, jitterSample, clamp(pc.jitter_params.x, 0.0, 1.0));
}
