#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Set 1: plume inputs
layout(set = 1, binding = 0) uniform sampler2D hdrInput;
layout(set = 1, binding = 1) uniform sampler2D posTex;
layout(set = 1, binding = 3) uniform sampler2D noiseTex;

struct PlumeData
{
    mat4 world_to_plume;

    vec4 shape;       // x length, y nozzleRadius, z expansionAngleRad, w radiusExp
    vec4 emission0;   // rgb coreColor, w intensity
    vec4 emission1;   // rgb plumeColor, w coreStrength
    vec4 params;      // x coreLength, y radialFalloff, z axialFalloff, w softAbsorption
    vec4 noise_shock; // x noiseStrength, y noiseScale, z noiseSpeed, w shockStrength
    vec4 shock_misc;  // x shockFrequency
};

layout(set = 1, binding = 2, std430) readonly buffer Plumes
{
    PlumeData plumes[];
} plume;

layout(push_constant) uniform PlumePush
{
    ivec4 misc; // x steps, y plumeCount
} pc;

vec3 getCameraLocalPosition()
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

bool intersectAABB(vec3 ro, vec3 rd, vec3 bmin, vec3 bmax, out float tmin, out float tmax)
{
    vec3 invD = 1.0 / rd;
    vec3 t0s = (bmin - ro) * invD;
    vec3 t1s = (bmax - ro) * invD;
    vec3 tsmaller = min(t0s, t1s);
    vec3 tbigger  = max(t0s, t1s);
    tmin = max(max(tsmaller.x, tsmaller.y), tsmaller.z);
    tmax = min(min(tbigger.x,  tbigger.y),  tbigger.z);
    return tmax >= max(tmin, 0.0);
}

float plume_radius(PlumeData p, float z)
{
    float len = max(p.shape.x, 0.001);
    float u = clamp(z / len, 0.0, 1.0);

    float baseR = max(p.shape.y, 0.0);
    float ang = clamp(p.shape.z, 0.0, 1.55); // clamp below ~89deg
    float rExp = max(p.shape.w, 0.0);

    // Vacuum-ish expansion: cone with optional exponent shaping along axis.
    float k = tan(ang);
    float t = (rExp > 0.0) ? pow(u, rExp) : u;
    return baseR + k * len * t;
}

void main()
{
    vec3 baseColor = texture(hdrInput, inUV).rgb;

    int plumeCount = clamp(pc.misc.y, 0, 1024);
    if (plumeCount <= 0)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    vec3 camLocal = getCameraLocalPosition();

    // Reconstruct a local-space ray for this pixel (Vulkan depth range 0..1).
    vec2 ndc = inUV * 2.0 - 1.0;
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0));
    vec3 rdWorld = transpose(mat3(sceneData.view)) * viewDir;

    // Clamp march to geometry distance (gbufferPosition.w == 1 for valid surfaces).
    float tGeom = 1e30;
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w > 0.0)
    {
        float surfT = dot(posSample.xyz - camLocal, rdWorld);
        if (surfT > 0.0)
        {
            tGeom = surfT;
        }
    }

    int steps = clamp(pc.misc.x, 8, 256);

    vec3 add = vec3(0.0);
    float trans = 1.0;
    vec3 scatter = vec3(0.0);

    float time = sceneData.timeParams.x;

    for (int pi = 0; pi < plumeCount; ++pi)
    {
        PlumeData p = plume.plumes[pi];

        float len = max(p.shape.x, 0.001);
        float baseR = max(p.shape.y, 0.0);
        float ang = clamp(p.shape.z, 0.0, 1.55);

        float rMax = baseR + tan(ang) * len;
        rMax = max(rMax, baseR);

        // Transform ray to plume-local space.
        vec3 ro = (p.world_to_plume * vec4(camLocal, 1.0)).xyz;
        vec3 rd = (p.world_to_plume * vec4(rdWorld, 0.0)).xyz;
        float rdLen = max(length(rd), 1e-6);
        rd /= rdLen;

        // Conservative local bounds (cylinder/box around the cone).
        vec3 bmin = vec3(-rMax, -rMax, 0.0);
        vec3 bmax = vec3( rMax,  rMax, len);

        float t0, t1;
        if (!intersectAABB(ro, rd, bmin, bmax, t0, t1))
        {
            continue;
        }

        // Clamp to opaque geometry distance. Assumes world_to_plume is rigid (rdLen ~= 1).
        t1 = min(t1, tGeom);
        if (t1 <= t0)
        {
            continue;
        }

        float dt = (t1 - t0) / float(steps);
        float jitter = hash12(inUV * 1024.0 + vec2(float(pi) * 13.37, float(pi) * 7.17));
        float t = max(t0, 0.0) + (jitter - 0.5) * dt;

        vec3 coreCol = max(p.emission0.rgb, vec3(0.0));
        vec3 plumeCol = max(p.emission1.rgb, vec3(0.0));
        float intensity = max(p.emission0.w, 0.0);
        float coreStrength = max(p.emission1.w, 0.0);

        float coreLen = max(p.params.x, 0.0);
        float radialPow = max(p.params.y, 0.0);
        float axialPow = max(p.params.z, 0.0);
        float absorb = max(p.params.w, 0.0);

        float noiseStrength = max(p.noise_shock.x, 0.0);
        float noiseScale = max(p.noise_shock.y, 0.001);
        float noiseSpeed = p.noise_shock.z;
        float shockStrength = max(p.noise_shock.w, 0.0);
        float shockFreq = max(p.shock_misc.x, 0.0);

        // Animate patterns outward along +Z.
        // noiseSpeed is interpreted as "plume lengths per second" (dimensionless).
        float flow = time * noiseSpeed * len;

        for (int si = 0; si < steps; ++si)
        {
            vec3 pos = ro + rd * (t + 0.5 * dt);

            float u = clamp(pos.z / len, 0.0, 1.0);
            float pr = plume_radius(p, pos.z);
            pr = max(pr, 1e-4);

            float zFlow = pos.z - flow;

            // Mix radial breakup + axial streaks from a tileable 2D noise texture.
            float ang01 = atan(pos.y, pos.x) * 0.15915494 + 0.5; // 1 / (2*pi)
            vec2 uvRad = pos.xy * (noiseScale * 0.12) + vec2(0.0, -flow * 0.015);
            vec2 uvAx  = vec2(zFlow * (noiseScale * 0.06), ang01 * (noiseScale * 1.25));

            float nRad = texture(noiseTex, uvRad).r;
            float nAx  = texture(noiseTex, uvAx).r;
            float nTex = mix(nRad, nAx, 0.65);
            float nCentered = nTex * 2.0 - 1.0;

            // Break up the cross-section to reduce uniformity (turbulence grows downstream).
            float turbStrength = clamp(noiseStrength * (0.25 + 0.75 * u) * 2.0, 0.0, 3.0);
            vec2 warpN = vec2(nRad, nAx) * 2.0 - 1.0;
            vec2 pxy = pos.xy + warpN * (pr * 0.12 * turbStrength);

            float r = length(pxy);

            // Inside cone check (soft).
            float rr = r / pr;
            float inside = clamp(1.0 - rr, 0.0, 1.0);

            if (inside > 1e-4)
            {
                float radial = (radialPow > 0.0) ? pow(inside, radialPow) : inside;
                float axial = (axialPow > 0.0) ? pow(1.0 - u, axialPow) : (1.0 - u);

                // Exponential turbulence keeps the field positive and yields stronger contrast.
                float turb = exp2(nCentered * (0.85 * turbStrength));
                float streak = exp2((nAx * 2.0 - 1.0) * (0.75 * turbStrength));

                // Shock diamonds: periodic modulation along axis (optional).
                float shock = 1.0;
                if (shockStrength > 0.0 && shockFreq > 0.0)
                {
                    float ph = 6.2831853 * (shockFreq * u - time * 0.35);
                    shock = 1.0 + shockStrength * sin(ph);
                }

                // Core near nozzle.
                float core = 0.0;
                if (coreLen > 0.0)
                {
                    core = 1.0 - smoothstep(coreLen * 0.65, coreLen, pos.z);
                }
                vec3 col = mix(plumeCol, coreCol, core);
                float str = intensity * mix(1.0, coreStrength, core);

                float density = radial * axial * turb * streak * shock;
                density = max(density, 0.0);

                vec3 emit = col * str * density;

                if (absorb > 0.0)
                {
                    float alpha = 1.0 - exp(-density * absorb * dt);
                    scatter += trans * alpha * emit;
                    trans *= (1.0 - alpha);
                    if (trans < 0.01)
                    {
                        break;
                    }
                }
                else
                {
                    add += emit * dt;
                }
            }

            t += dt;
            if (t > t1)
            {
                break;
            }
        }
    }

    vec3 outRgb = add + scatter + trans * baseColor;
    outColor = vec4(outRgb, 1.0);
}
