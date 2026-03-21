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
    mat2 rot = mat2(0.8, 0.6, -0.6, 0.8);
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

    vec2 ndc     = inUV * 2.0 - 1.0;
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0],
    ndc.y / sceneData.proj[1][1], -1.0));
    vec3 worldDir = transpose(mat3(sceneData.view)) * viewDir;

    vec3  sunDir  = normalize(-sceneData.sunlightDirection.xyz);
    float theta   = max(sceneData.rtParams.w, 0.0);    // angular radius

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
    vec3  diskColor = vec3(1.0);   // will tint center-to-edge

    if (theta > 0.0 && diskMask > 0.0)
    {
        float mu = clamp((cosTheta - cosEdge) / max(1.0 - cosEdge, 1.0e-6), 0.0, 1.0);

        // Standard quadratic limb-darkening
        float u1 = 0.5;
        float u2 = 0.1;
        float limb = 1.0 - u1 * (1.0 - mu) - u2 * (1.0 - mu) * (1.0 - mu);
        disk *= limb;

        // Color shift: center → white-hot,  edge → warm orange-red
        vec3 centerCol = vec3(1.0, 1.0, 0.95);          // slightly cool white
        vec3 edgeCol   = vec3(1.0, 0.45, 0.15);          // deep orange
        diskColor = mix(edgeCol, centerCol, mu * mu);
    }

    // ─── Angular distance from sun center ────────────────────────────

    float theta2    = max(2.0 * (1.0 - cosTheta), 0.0);
    float thetaDiff = sqrt(theta2);

    // ─── Screen-space sun position (shared by several effects) ───────

    vec3  sunViewDir = mat3(sceneData.view) * sunDir;
    bool  sunOnScreen = (sunViewDir.z < -1.0e-6);
    vec2  sunUV  = vec2(0.5);
    float aspect = sceneData.proj[1][1] / max(sceneData.proj[0][0], 1.0e-6);

    if (sunOnScreen)
    {
        float invZ = 1.0 / (-sunViewDir.z);
        vec2 sunNdc = vec2(sunViewDir.x * sceneData.proj[0][0] * invZ,
        sunViewDir.y * sceneData.proj[1][1] * invZ);
        sunUV = sunNdc * 0.5 + 0.5;
    }

    vec2 delta = inUV - sunUV;
    delta.x *= aspect;
    float screenDist = length(delta);

    // ─── 1. CHROMATIC HALO ───────────────────────────────────────────
    //        Three gaussian layers (R, G, B) at slightly different radii
    //        to produce a subtle rainbow fringe around the sun.

    vec3 halo = vec3(0.0);
    if (haloIntensity > 0.0 && haloRadiusDeg > 0.0)
    {
        float haloRadius = radians(haloRadiusDeg);

        // Per-channel radius offsets → chromatic fringe
        float rR = haloRadius * 1.08;   // red extends further
        float rG = haloRadius * 1.00;
        float rB = haloRadius * 0.92;   // blue is tighter

        float tR = thetaDiff / max(rR, 1.0e-6);
        float tG = thetaDiff / max(rG, 1.0e-6);
        float tB = thetaDiff / max(rB, 1.0e-6);

        // Tight core + soft outer bloom
        halo.r = exp(-tR * tR * 16.0) + exp(-tR * tR * 1.5) * 0.15;
        halo.g = exp(-tG * tG * 16.0) + exp(-tG * tG * 1.5) * 0.15;
        halo.b = exp(-tB * tB * 16.0) + exp(-tB * tB * 1.5) * 0.15;

        // Extra wide soft glow (warm-tinted)
        float tWide = thetaDiff / max(haloRadius * 2.5, 1.0e-6);
        float wideGlow = exp(-tWide * tWide * 4.0) * 0.08;
        halo += vec3(wideGlow * 1.2, wideGlow * 0.9, wideGlow * 0.5);
    }

    // ─── 2. CORONA TENDRILS ──────────────────────────────────────────
    //        Procedural wisps radiating outward from the limb, using FBM
    //        so each direction has unique length & brightness.

    float corona = 0.0;
    if (haloIntensity > 0.0 && theta > 0.0)
    {
        float phi = atan(delta.y, delta.x);

        // Seed from sun direction so pattern changes with sun position
        float seed = fract(sin(dot(sunDir, vec3(12.9898, 78.233, 37.719))) * 43758.5453);

        // FBM-based radial noise for organic tendrils
        float n  = fbm(vec2(phi * 3.0 + seed * 100.0, thetaDiff * 60.0));
        float n2 = fbm(vec2(phi * 7.0 - seed * 47.0,  thetaDiff * 30.0 + 5.0));

        float tendril = mix(n, n2, 0.4);
        tendril = pow(max(tendril, 0.0), 1.5);

        // Radial envelope: strongest near limb, fades outward
        float coronaRadius = radians(haloRadiusDeg) * 0.8;
        float rNorm = thetaDiff / max(coronaRadius, 1.0e-6);
        float env = exp(-rNorm * rNorm * 3.0) * smoothstep(0.0, theta * 1.2, thetaDiff);

        corona = tendril * env * 0.6;
    }

    // ─── 3. STARBURST (original + enhanced) ──────────────────────────

    float star = 0.0;
    if (starIntensity > 0.0 && starRadiusDeg > 0.0 && sunOnScreen)
    {
        float phi = atan(delta.y, delta.x);
        float k   = starSpikes * 0.5;

        float seed = fract(sin(dot(sunDir, vec3(12.9898, 78.233, 37.719))) * 43758.5453);

        float u = (phi + PI) / TAU * starSpikes;

        float rA = noise1D(u + seed * 17.0);
        float rB = noise1D(u * 2.7 + seed * 53.0);
        float rC = noise1D(u * 8.0 + seed * 91.0);

        float phaseJit = (rA - 0.5) * 0.12;
        float amp      = mix(0.75, 1.25, rB);
        float sharp    = mix(starSharpness * 0.7, starSharpness * 1.4, rC);

        float spikes = amp * pow(abs(cos((phi + phaseJit) * k)), sharp);

        // Secondary fine spikes layer (twice the frequency, thinner)
        float fineSpikes = 0.3 * pow(abs(cos((phi - phaseJit * 0.5) * k * 2.0)), sharp * 1.8);
        spikes = max(spikes, fineSpikes);

        float center = smoothstep(0.0, 1.5e-4, dot(delta, delta));
        spikes = mix(1.0, spikes, center);

        float starRadius = radians(starRadiusDeg);
        float lenJit     = mix(0.75, 1.35, rA);
        float t = clamp(thetaDiff / max(starRadius * lenJit, 1.0e-6), 0.0, 1.0);

        float envelope = (1.0 - t);
        envelope = envelope * envelope;

        star = envelope * spikes;
    }

    // ─── 4. ANAMORPHIC STREAK ────────────────────────────────────────
    //        Horizontal light streak (simulates anamorphic lens flare).

    float streak = 0.0;
    if (anamorphicStreakEnabled && haloIntensity > 0.0 && sunOnScreen)
    {
        // Tight vertical falloff with broad horizontal reach.
        float dy = abs(delta.y);
        float dx = abs(delta.x);

        float hStreak = exp(-dy * dy * 24000.0)
            * exp(-dx * dx * 32.0);

        streak = hStreak * 0.35;
    }

    // ─── 5. MULTI-LAYER BLOOM ────────────────────────────────────────
    //        Several concentric soft rings at different scales for richness.

    vec3 bloom = vec3(0.0);
    if (haloIntensity > 0.0)
    {
        float haloRadius = radians(haloRadiusDeg);

        // Ring 1: tight, bright
        float r1 = thetaDiff / max(haloRadius * 0.3, 1.0e-6);
        bloom += exp(-r1 * r1 * 20.0) * vec3(1.0, 0.95, 0.85) * 0.15;

        // Ring 2: medium, warm
        float r2 = thetaDiff / max(haloRadius * 1.5, 1.0e-6);
        bloom += exp(-r2 * r2 * 6.0) * vec3(1.0, 0.7, 0.3) * 0.06;

        // Ring 3: very wide, subtle crimson
        float r3 = thetaDiff / max(haloRadius * 4.0, 1.0e-6);
        bloom += exp(-r3 * r3 * 2.0) * vec3(1.0, 0.3, 0.1) * 0.02;
    }

    // ─── Composite ───────────────────────────────────────────────────

    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;

    // Disk: tinted by limb color shift
    vec3 result = diskIntensity * disk * diskColor * sunCol;

    // Chromatic halo (per-channel)
    result += haloIntensity * halo * sunCol;

    // Corona tendrils (warm-tinted)
    vec3 coronaColor = sunCol * vec3(1.0, 0.8, 0.5);
    result += haloIntensity * corona * coronaColor;

    // Starburst
    result += starIntensity * star * sunCol;

    // Anamorphic streak (slightly desaturated / warm)
    vec3 streakColor = sunCol * vec3(1.0, 0.9, 0.7);
    result += haloIntensity * streak * streakColor;

    // Multi-layer bloom
    result += haloIntensity * bloom * sunCol;

    outColor = vec4(result, 1.0);
}
