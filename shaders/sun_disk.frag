#version 450
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform SunDiskPush
{
    vec4 params0; // x: disk intensity, y: halo intensity, z: starburst intensity, w: halo radius (deg)
    vec4 params1; // x: starburst radius (deg), y: spikes, z: sharpness, w: anamorphic streak enabled
} pc;

const float PI  = 3.14159265359;
const float TAU = 6.28318530718;

// ─── Noise helpers ───────────────────────────────────────────────────

float hash11(float p) { return fract(sin(p) * 43758.5453123); }
float hash21(vec2 p)  { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123); }

float noise1D(float x)
{
    float i = floor(x);
    float f = fract(x);
    float t = f * f * (3.0 - 2.0 * f);
    return mix(hash11(i), hash11(i + 1.0), t);
}

float noise2D(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);

    float a = hash21(i);
    float b = hash21(i + vec2(1.0, 0.0));
    float c = hash21(i + vec2(0.0, 1.0));
    float d = hash21(i + vec2(1.0, 1.0));

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Fractal Brownian Motion — 4 octaves
float fbm(vec2 p)
{
    float v = 0.0, a = 0.5;
    const mat2 rot = mat2(0.8, 0.6, -0.6, 0.8);
    for (int i = 0; i < 4; ++i)
    {
        v += a * noise2D(p);
        p  = rot * p * 2.0;
        a *= 0.5;
    }
    return v;
}

void main()
{
    float diskIntensity = max(pc.params0.x, 0.0);
    float haloIntensity = max(pc.params0.y, 0.0);
    float starIntensity = max(pc.params0.z, 0.0);
    float haloRadiusDeg = max(pc.params0.w, 0.0);
    float starRadiusDeg = max(pc.params1.x, 0.0);
    float starSpikes    = max(pc.params1.y, 2.0);
    float starSharpness = max(pc.params1.z, 1.0);
    bool anamorphicStreakEnabled = (pc.params1.w > 0.5);

    if (diskIntensity <= 0.0 && haloIntensity <= 0.0 && starIntensity <= 0.0)
    {
        outColor = vec4(0.0);
        return;
    }

    // ─── View / world direction ──────────────────────────────────────

    vec2 ndc = inUV * 2.0 - 1.0;
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0],
                                  ndc.y / sceneData.proj[1][1], -1.0));
    vec3 worldDir = transpose(mat3(sceneData.view)) * viewDir;

    vec3  sunDir  = -sceneData.sunlightDirection.xyz;
    float theta   = max(sceneData.rtParams.w, 0.0);

    float cosTheta = dot(worldDir, sunDir);
    float cosEdge  = cos(theta);

    // ─── Disk mask (anti-aliased) ────────────────────────────────────

    float diskMask = 0.0;
    if (theta > 0.0)
    {
        float w = max(fwidth(cosTheta) * 2.0, 1.0e-6);
        diskMask = smoothstep(cosEdge, cosEdge + w, cosTheta);
    }

    // ─── Disk with quadratic limb-darkening + COLOR SHIFT ────────────

    float disk = diskMask;
    vec3  diskColor = vec3(1.0);

    if (theta > 0.0 && diskMask > 0.0)
    {
        float mu = clamp((cosTheta - cosEdge) / max(1.0 - cosEdge, 1.0e-6), 0.0, 1.0);

        float oneMinusMu = 1.0 - mu;
        float limb = 1.0 - 0.5 * oneMinusMu - 0.1 * oneMinusMu * oneMinusMu;
        disk *= limb;

        vec3 centerCol = vec3(1.0, 1.0, 0.95);
        vec3 edgeCol   = vec3(1.0, 0.45, 0.15);
        diskColor = mix(edgeCol, centerCol, mu * mu);
    }

    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;
    vec3 result = diskIntensity * disk * diskColor * sunCol;
    if (haloIntensity <= 0.0 && starIntensity <= 0.0)
    {
        outColor = vec4(result, 1.0);
        return;
    }

    // ─── Angular distance squared (defer sqrt) ──────────────────────

    float theta2 = max(2.0 * (1.0 - cosTheta), 0.0);

    // ─── Screen-space sun position (shared by several effects) ───────

    bool needCorona = (haloIntensity > 0.0 && theta > 0.0);
    bool needStar = (starIntensity > 0.0 && starRadiusDeg > 0.0);
    bool needStreak = (anamorphicStreakEnabled && haloIntensity > 0.0);
    bool needScreenDelta = needCorona || needStar || needStreak;

    bool sunOnScreen = false;
    vec2 delta = vec2(0.0);
    if (needScreenDelta)
    {
        vec3 sunViewDir = mat3(sceneData.view) * sunDir;
        sunOnScreen = (sunViewDir.z < -1.0e-6);

        vec2 sunUV = vec2(0.5);
        if (sunOnScreen)
        {
            float invZ = 1.0 / (-sunViewDir.z);
            vec2 sunNdc = vec2(sunViewDir.x * sceneData.proj[0][0] * invZ,
                               sunViewDir.y * sceneData.proj[1][1] * invZ);
            sunUV = sunNdc * 0.5 + 0.5;
        }

        delta = inUV - sunUV;
        delta.x *= sceneData.proj[1][1] / max(sceneData.proj[0][0], 1.0e-6);
    }

    // ─── Shared precomputations ──────────────────────────────────────
    // Hoist values used across multiple effect blocks

    float s = 0.0;
    if (haloIntensity > 0.0)
    {
        float haloRadius = radians(haloRadiusDeg);
        float invHR2 = 1.0 / max(haloRadius * haloRadius, 1.0e-12);
        s = theta2 * invHR2;
    }

    bool needPolarFx = needCorona || (needStar && sunOnScreen);
    float phi = 0.0;
    float seed = 0.0;
    float thetaDiff = 0.0;
    if (needPolarFx)
    {
        phi = atan(delta.y, delta.x);
        seed = fract(sin(dot(sunDir, vec3(12.9898, 78.233, 37.719))) * 43758.5453);
        thetaDiff = sqrt(theta2);
    }

    // ─── 1. CHROMATIC HALO + MULTI-LAYER BLOOM ──────────────────────
    //    Merged: both gate on haloIntensity and share radial exp(−s·k).
    //    Coefficients: k = originalExp / channelScale²

    vec3 halo  = vec3(0.0);
    vec3 bloom = vec3(0.0);

    if (haloIntensity > 0.0 && haloRadiusDeg > 0.0)
    {
        // Chromatic halo  (R×1.08  G×1.00  B×0.92)
        //   core 16/scale²          outer 1.5/scale²
        halo.r = exp(-s * 13.717) + exp(-s * 1.286) * 0.15;   // 1.08² = 1.1664
        halo.g = exp(-s * 16.0)   + exp(-s * 1.5)   * 0.15;
        halo.b = exp(-s * 18.898) + exp(-s * 1.773)  * 0.15;  // 0.92² = 0.8464

        // Wide warm glow  (scale 2.5 → 4/6.25 = 0.64)
        float wideGlow = exp(-s * 0.64) * 0.08;
        halo += vec3(wideGlow * 1.2, wideGlow * 0.9, wideGlow * 0.5);

        // Multi-layer bloom rings (scale² denominators pre-folded)
        bloom  = exp(-s * 222.222) * vec3(0.15, 0.1425, 0.1275);   // ×0.3 → 20/0.09
        bloom += exp(-s * 2.667)   * vec3(0.06, 0.042, 0.018);     // ×1.5 → 6/2.25
        bloom += exp(-s * 0.125)   * vec3(0.02, 0.006, 0.002);     // ×4.0 → 2/16
    }

    // ─── 2. CORONA TENDRILS ──────────────────────────────────────────

    float corona = 0.0;
    if (needCorona)
    {
        float n  = fbm(vec2(phi * 3.0 + seed * 100.0, thetaDiff * 60.0));
        float n2 = fbm(vec2(phi * 7.0 - seed * 47.0,  thetaDiff * 30.0 + 5.0));

        float tendril = mix(n, n2, 0.4);
        tendril = pow(max(tendril, 0.0), 1.5);

        // Envelope: coronaRadius = haloRadius×0.8 → 3/0.64 = 4.6875
        float env = exp(-s * 4.6875) * smoothstep(0.0, theta * 1.2, thetaDiff);

        corona = tendril * env * 0.6;
    }

    // ─── 3. STARBURST ────────────────────────────────────────────────

    float star = 0.0;
    if (needStar && sunOnScreen)
    {
        float k = starSpikes * 0.5;

        float u = (phi + PI) / TAU * starSpikes;

        float rA = noise1D(u + seed * 17.0);
        float rB = noise1D(u * 2.7 + seed * 53.0);
        float rC = noise1D(u * 8.0 + seed * 91.0);

        float phaseJit = (rA - 0.5) * 0.12;
        float amp      = mix(0.75, 1.25, rB);
        float sharp    = mix(starSharpness * 0.7, starSharpness * 1.4, rC);

        float spikes = amp * pow(abs(cos((phi + phaseJit) * k)), sharp);

        float fineSpikes = 0.3 * pow(abs(cos((phi - phaseJit * 0.5) * k * 2.0)), sharp * 1.8);
        spikes = max(spikes, fineSpikes);

        float center = smoothstep(0.0, 1.5e-4, dot(delta, delta));
        spikes = mix(1.0, spikes, center);

        float starRadius = radians(starRadiusDeg);
        float lenJit     = mix(0.75, 1.35, rA);
        float t = clamp(thetaDiff / max(starRadius * lenJit, 1.0e-6), 0.0, 1.0);

        float envelope = 1.0 - t;
        envelope *= envelope;

        star = envelope * spikes;
    }

    // ─── 4. ANAMORPHIC STREAK ────────────────────────────────────────
    //    exp(a)·exp(b) → exp(a+b);  abs() unnecessary before squaring.

    float streak = 0.0;
    if (needStreak && sunOnScreen)
    {
        streak = exp(-delta.y * delta.y * 24000.0
                     - delta.x * delta.x * 32.0) * 0.35;
    }

    // ─── Composite ───────────────────────────────────────────────────

    result += haloIntensity * halo * sunCol;

    result += haloIntensity * corona * (sunCol * vec3(1.0, 0.8, 0.5));

    result += starIntensity * star * sunCol;

    result += haloIntensity * streak * (sunCol * vec3(1.0, 0.9, 0.7));

    result += haloIntensity * bloom * sunCol;

    outColor = vec4(result, 1.0);
}
