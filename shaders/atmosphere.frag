#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Set 1: inputs
layout(set = 1, binding = 0) uniform sampler2D hdrInput;
layout(set = 1, binding = 1) uniform sampler2D posTex;

layout(push_constant) uniform AtmospherePush
{
    vec4 planet_center_radius; // xyz: planet center (local), w: planet radius (m)
    vec4 atmosphere_params;    // x: atmosphere radius (m), y: rayleigh H (m), z: mie H (m), w: mie g
    vec4 beta_rayleigh;        // rgb: betaR (1/m), w: intensity
    vec4 beta_mie;             // rgb: betaM (1/m), w: sun disk intensity
    vec4 jitter_params;        // x: jitter strength (0..1), y: planet snap (m), zw: reserved
    ivec4 misc;                // x: view steps, y: light steps
} pc;

const float PI = 3.14159265359;

vec3 getCameraWorldPosition()
{
    mat3 rotT = mat3(sceneData.view); // R^T
    mat3 rot  = transpose(rotT);      // R
    vec3 T    = sceneData.view[3].xyz;
    return -rot * T;
}

float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

bool ray_sphere_intersect(vec3 ro, vec3 rd, vec3 center, float radius, out float t0, out float t1)
{
    vec3 oc = ro - center;
    float b = dot(oc, rd);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - c;
    if (h < 0.0)
    {
        return false;
    }
    h = sqrt(h);
    t0 = -b - h;
    t1 = -b + h;
    return true;
}

float phase_rayleigh(float cosTheta)
{
    return (3.0 / (16.0 * PI)) * (1.0 + cosTheta * cosTheta);
}

float phase_mie_hg(float cosTheta, float g)
{
    float g2 = g * g;
    float denom = pow(max(1.0 + g2 - 2.0 * g * cosTheta, 1.0e-4), 1.5);
    return (1.0 / (4.0 * PI)) * (1.0 - g2) / denom;
}

void main()
{
    vec3 baseColor = texture(hdrInput, inUV).rgb;

    float planetRadius = pc.planet_center_radius.w;
    float atmRadius = pc.atmosphere_params.x;
    vec3 betaR = max(pc.beta_rayleigh.rgb, vec3(0.0));
    vec3 betaM = max(pc.beta_mie.rgb, vec3(0.0));
    float intensity = max(pc.beta_rayleigh.w, 0.0);

    if (planetRadius <= 0.0 || atmRadius <= 0.0 || intensity <= 0.0 || all(equal(betaR + betaM, vec3(0.0))))
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    vec3 camPos = getCameraWorldPosition();

    // Reconstruct a world-space ray for this pixel (Vulkan depth range 0..1).
    vec2 ndc = inUV * 2.0 - 1.0;
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0));
    vec3 rd = transpose(mat3(sceneData.view)) * viewDir;

    vec3 center = pc.planet_center_radius.xyz;

    // Intersect view ray with atmosphere. If it never enters the atmosphere, keep the background.
    float tAtm0, tAtm1;
    if (!ray_sphere_intersect(camPos, rd, center, atmRadius, tAtm0, tAtm1))
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    // Clamp integration to the nearest surface point if available (aerial perspective on opaque).
    float tEnd = tAtm1;
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w > 0.0)
    {
        float tSurf = dot(posSample.xyz - camPos, rd);
        if (tSurf > 0.0)
        {
            bool isPlanet = (posSample.w > 1.5);
            if (isPlanet)
            {
                float snapM = max(pc.jitter_params.y, 0.0);
                if (snapM > 0.0)
                {
                    float tP0, tP1;
                    if (ray_sphere_intersect(camPos, rd, center, planetRadius, tP0, tP1))
                    {
                        float tSphere = (tP0 > 0.0) ? tP0 : tP1;
                        if (tSphere > 0.0 && tSurf <= tSphere && tSurf > tSphere - snapM)
                        {
                            tSurf = tSphere;
                        }
                    }
                }
            }
            tEnd = min(tEnd, tSurf);
        }
    }

    float tStart = max(tAtm0, 0.0);
    if (tEnd <= tStart)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    int viewSteps = clamp(pc.misc.x, 4, 64);
    int lightSteps = clamp(pc.misc.y, 2, 32);

    float dt = (tEnd - tStart) / float(viewSteps);
    float jitterStrength = clamp(pc.jitter_params.x, 0.0, 1.0);
    float jitter = mix(0.5, hash12(inUV * 1024.0), jitterStrength);

    float Hr = max(pc.atmosphere_params.y, 1.0);
    float Hm = max(pc.atmosphere_params.z, 1.0);
    float mieG = clamp(pc.atmosphere_params.w, -0.99, 0.99);

    vec3 sunDir = normalize(-sceneData.sunlightDirection.xyz);
    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;

    float cosTheta = dot(rd, sunDir);
    float phaseR = phase_rayleigh(cosTheta);
    float phaseM = phase_mie_hg(cosTheta, mieG);

    float odR = 0.0;
    float odM = 0.0;
    vec3 scatterSum = vec3(0.0);

    for (int i = 0; i < viewSteps; ++i)
    {
        float ts = tStart + (float(i) + jitter) * dt;
        vec3 p = camPos + rd * ts;
        float height = length(p - center) - planetRadius;
        height = max(height, 0.0);

        float densR = exp(-height / Hr);
        float densM = exp(-height / Hm);

        odR += densR * dt;
        odM += densM * dt;

        // Sun ray integration from this sample point to top of atmosphere.
        float tL0, tL1;
        if (!ray_sphere_intersect(p, sunDir, center, atmRadius, tL0, tL1) || tL1 <= 0.0)
        {
            continue;
        }
        float dtL = tL1 / float(lightSteps);

        float odRL = 0.0;
        float odML = 0.0;

        // Shadow: if the sun ray hits the planet, this sample is in shadow (no direct sun).
        float tp0, tp1;
        bool shadowed = false;
        if (ray_sphere_intersect(p, sunDir, center, planetRadius, tp0, tp1))
        {
            shadowed = (tp1 > 0.0);
        }

        if (!shadowed)
        {
            float tl = 0.0;
            for (int j = 0; j < lightSteps; ++j)
            {
                vec3 lp = p + sunDir * (tl + 0.5 * dtL);
                float lh = length(lp - center) - planetRadius;
                lh = max(lh, 0.0);

                odRL += exp(-lh / Hr) * dtL;
                odML += exp(-lh / Hm) * dtL;

                tl += dtL;
            }

            vec3 tau = betaR * (odR + odRL) + betaM * (odM + odML);
            vec3 atten = exp(-tau);
            vec3 scatter = atten * (densR * betaR * phaseR + densM * betaM * phaseM) * dt;
            scatterSum += scatter;
        }
    }

    vec3 transmittance = exp(-(betaR * odR + betaM * odM));
    vec3 outRgb = baseColor * transmittance + scatterSum * (sunCol * intensity);

    // Simple sun disk.
    float sunDisk = max(pc.beta_mie.w, 0.0);
    if (sunDisk > 0.0)
    {
        float sunTerm = pow(max(cosTheta, 0.0), 2048.0);
        outRgb += sunCol * (sunDisk * sunTerm) * transmittance;
    }

    outColor = vec4(outRgb, 1.0);
}
