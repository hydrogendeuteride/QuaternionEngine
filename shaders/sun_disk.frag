#version 450
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform SunDiskPush
{
    vec4 params0; // x: disk intensity, y: halo intensity, z: starburst intensity, w: halo radius (deg)
    vec4 params1; // x: starburst radius (deg), y: spikes, z: sharpness, w: reserved
} pc;

const float PI = 3.14159265359;

float hash11(float p) { return fract(sin(p) * 43758.5453123); }

float noise1D(float x)
{
    float i = floor(x);
    float f = fract(x);
    float a = hash11(i);
    float b = hash11(i + 1.0);
    float t = f * f * (3.0 - 2.0 * f);
    return mix(a, b, t);
}

void main()
{
    float diskIntensity = max(pc.params0.x, 0.0);
    float haloIntensity = max(pc.params0.y, 0.0);
    float starIntensity = max(pc.params0.z, 0.0);
    float haloRadiusDeg = max(pc.params0.w, 0.0);
    float starRadiusDeg = max(pc.params1.x, 0.0);
    float starSpikes = max(pc.params1.y, 2.0);
    float starSharpness = max(pc.params1.z, 1.0);

    if (diskIntensity <= 0.0 && haloIntensity <= 0.0 && starIntensity <= 0.0)
    {
        outColor = vec4(0.0);
        return;
    }

    vec2 ndc = inUV * 2.0 - 1.0;
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0));
    vec3 worldDir = transpose(mat3(sceneData.view)) * viewDir;

    vec3 sunDir = normalize(-sceneData.sunlightDirection.xyz);
    float theta = max(sceneData.rtParams.w, 0.0);

    float cosTheta = dot(worldDir, sunDir);
    float cosEdge = cos(theta);

    float diskMask = 0.0;
    if (theta > 0.0)
    {
        float w = max(fwidth(cosTheta) * 2.0, 1.0e-6);
        diskMask = smoothstep(cosEdge, cosEdge + w, cosTheta);
    }

    ////////////////////////////////////////////
    float disk = diskMask;
    if (theta > 0.0 && diskMask > 0.0)
    {
        float mu = clamp((cosTheta - cosEdge) / max(1.0 - cosEdge, 1.0e-6), 0.0, 1.0);

        float u1 = 0.5;
        float u2 = 0.1;
        float limb = 1.0 - u1 * (1.0 - mu) - u2 * (1.0 - mu) * (1.0 - mu);

        disk *= limb;
    }
    ///////////////////////////////////////////

    // Approximate angular distance from the sun center. For small angles:
    // 1 - cos(a) ≈ a^2 / 2  =>  a ≈ sqrt(2 * (1 - cos(a))).
    float theta2 = max(2.0 * (1.0 - cosTheta), 0.0);
    float thetaDiff = sqrt(theta2);

    float halo = 0.0;
    if (haloIntensity > 0.0 && haloRadiusDeg > 0.0)
    {
        float haloRadius = radians(haloRadiusDeg);
        float t = thetaDiff / max(haloRadius, 1.0e-6);
        halo = exp(-t * t * 16.0);
        halo += exp(-t * t * 1.5) * 0.15;
    }

    float star = 0.0;
    if (starIntensity > 0.0 && starRadiusDeg > 0.0)
    {
        // Project sun direction to screen to get a stable angular coordinate for spikes.
        vec3 sunViewDir = mat3(sceneData.view) * sunDir;
        if (sunViewDir.z < -1.0e-6)
        {
            float invZ = 1.0 / (-sunViewDir.z);
            vec2 sunNdc = vec2(sunViewDir.x * sceneData.proj[0][0] * invZ,
                               sunViewDir.y * sceneData.proj[1][1] * invZ);
            vec2 sunUV = sunNdc * 0.5 + 0.5;

            float aspect = sceneData.proj[1][1] / max(sceneData.proj[0][0], 1.0e-6);
            vec2 d = inUV - sunUV;
            d.x *= aspect;

            float phi = atan(d.y, d.x);
            float k = starSpikes * 0.5;

            float seed = fract(sin(dot(sunDir, vec3(12.9898, 78.233, 37.719))) * 43758.5453);

            float u = (phi + PI) / (2.0 * PI) * starSpikes;

            float rA = noise1D(u + seed * 17.0);
            float rB = noise1D(u * 2.7 + seed * 53.0);
            float rC = noise1D(u * 8.0 + seed * 91.0);

            float phaseJit = (rA - 0.5) * 0.12;

            float amp   = mix(0.75, 1.25, rB);
            float sharp = mix(starSharpness * 0.7, starSharpness * 1.4, rC);

            float spikes = amp * pow(abs(cos((phi + phaseJit) * k)), sharp);

            float center = smoothstep(0.0, 1.5e-4, dot(d, d));
            spikes = mix(1.0, spikes, center);

            float starRadius = radians(starRadiusDeg);
            float lenJit = mix(0.75, 1.35, rA);
            float t = clamp(thetaDiff / max(starRadius * lenJit, 1.0e-6), 0.0, 1.0);

            float envelope = (1.0 - t);
            envelope = envelope * envelope;

            star = envelope * spikes;
        }
    }

    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;

    float term = diskIntensity * diskMask
                 + haloIntensity * halo
                 + starIntensity * star;

    outColor = vec4(sunCol * term, 1.0);
}
