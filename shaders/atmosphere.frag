#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D hdrInput;
layout(set = 1, binding = 1) uniform sampler2D posTex;
layout(set = 1, binding = 2) uniform sampler2D transmittanceLut;

layout(push_constant) uniform AtmospherePush
{
    vec4 planet_center_radius;// xyz: planet center (local), w: planet radius (m)
    vec4 atmosphere_params;// x: atmosphere radius (m), y: rayleigh H (m), z: mie H (m), w: mie g
    vec4 beta_rayleigh;// rgb: betaR (1/m), w: intensity
    vec4 beta_mie;// rgb: betaM (1/m), w: sun disk intensity
    vec4 jitter_params;// x: jitter strength (0..1), y: planet snap (m), zw: reserved
    ivec4 misc;// x: view steps, y: light steps
} pc;

const float PI = 3.14159265359;

vec3 getCameraWorldPosition()
{
    mat3 rotT = mat3(sceneData.view);// R^T
    mat3 rot  = transpose(rotT);// R
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

    if (planetRadius <= 0.0 || atmRadius <= planetRadius || intensity <= 0.0 || all(equal(betaR + betaM, vec3(0.0))))
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    vec3 camPos = getCameraWorldPosition();

    // Reconstruct a world-space ray for this pixel [0~1].
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
                        if (tSphere > 0.0)
                        {
                            // Compare in radial space (more stable than t-space at grazing angles).
                            float rSurf = length(posSample.xyz - center);

                            // Cube-sphere patches (and LOD skirts) are planar and sit inside the analytic sphere.
                            // When just using planet patch to make it: grid/ring shaped atmosphere appears
                            // Because of cracks between each patches and  skirts.
                            if (rSurf < planetRadius)
                            {
                                tSurf = min(tSurf, tSphere);
                            }

                            // LOD stepping. Large deviations remain unaffected.
                            float radialErr = abs(rSurf - planetRadius);
                            if (radialErr <= snapM)
                            {
                                tSurf = tSphere;
                            }
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

        // Planet shadow: if the sun ray hits the planet, this sample is in shadow.
        float tp0, tp1;
        if (ray_sphere_intersect(p, sunDir, center, planetRadius, tp0, tp1))
        {
            if (tp1 > 0.0)
            {
                continue;
            }
        }

        // Sun transmittance (optical depth) lookup: removes per-pixel sun raymarch.
        vec3 radial = p - center;
        float r = max(length(radial), planetRadius);
        float muS = dot(radial / r, sunDir);
        float u = clamp(muS * 0.5 + 0.5, 0.0, 1.0);
        float v = clamp((r - planetRadius) / (atmRadius - planetRadius), 0.0, 1.0);
        vec2 odSun = texture(transmittanceLut, vec2(u, v)).rg;

        vec3 tau = betaR * (odR + odSun.x) + betaM * (odM + odSun.y);
        vec3 atten = exp(-tau);
        vec3 scatter = atten * (densR * betaR * phaseR + densM * betaM * phaseM) * dt;
        scatterSum += scatter;
    }

    vec3 transmittance = exp(-(betaR * odR + betaM * odM));
    vec3 outRgb = baseColor * transmittance + scatterSum * (sunCol * intensity);

    outColor = vec4(outRgb, 1.0);
}
