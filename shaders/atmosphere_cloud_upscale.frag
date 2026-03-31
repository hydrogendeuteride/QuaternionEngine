#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;

layout(location = 0) out vec4 outCloudLighting;
layout(location = 1) out vec2 outCloudSegment;

layout(set = 1, binding = 0) uniform sampler2D posTex;
layout(set = 1, binding = 1) uniform sampler2D cloudLightingLowResTex;
layout(set = 1, binding = 2) uniform sampler2D cloudSegmentLowResTex;

layout(push_constant) uniform AtmospherePush
{
    vec4 planet_center_radius;
    vec4 atmosphere_params;
    vec4 beta_rayleigh;
    vec4 beta_mie;
    vec4 jitter_params;
    vec4 cloud_layer;
    vec4 cloud_params;
    ivec4 misc;
} pc;

bool ray_sphere_intersect(vec3 ro, vec3 rd, vec3 center, float radius, out float t0, out float t1)
{
    vec3 oc = ro - center;
    float b = dot(oc, rd);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - c;
    if (h < 0.0) return false;
    h = sqrt(h);
    t0 = -b - h;
    t1 = -b + h;
    return true;
}

int compute_cloud_segments(vec3 camLocal,
                           vec3 rd,
                           vec3 center,
                           float rBase,
                           float rTop,
                           float tStart,
                           float tEnd,
                           out vec2 seg0,
                           out vec2 seg1)
{
    seg0 = vec2(0.0);
    seg1 = vec2(0.0);

    float tO0;
    float tO1;
    if (!ray_sphere_intersect(camLocal, rd, center, rTop, tO0, tO1))
    {
        return 0;
    }

    float tI0;
    float tI1;
    bool hasInner = ray_sphere_intersect(camLocal, rd, center, rBase, tI0, tI1);
    int count = 0;

    if (!hasInner)
    {
        float a = max(tO0, tStart);
        float b = min(tO1, tEnd);
        if (b > a)
        {
            seg0 = vec2(a, b);
            count = 1;
        }
    }
    else
    {
        if (tI0 > tO0)
        {
            float a = max(tO0, tStart);
            float b = min(min(tI0, tO1), tEnd);
            if (b > a)
            {
                seg0 = vec2(a, b);
                count = 1;
            }
        }
        if (tI1 < tO1)
        {
            float a = max(max(tI1, tO0), tStart);
            float b = min(tO1, tEnd);
            if (b > a)
            {
                if (count == 0) seg0 = vec2(a, b);
                else seg1 = vec2(a, b);
                count++;
            }
        }
    }

    if (count == 2 && seg1.x < seg0.x)
    {
        vec2 tmp = seg0;
        seg0 = seg1;
        seg1 = tmp;
    }

    return count;
}

bool compute_ray_bounds(vec3 camLocal,
                        vec3 rd,
                        vec3 center,
                        float planetRadius,
                        float boundRadius,
                        out float tStart,
                        out float tEnd)
{
    float tBound0;
    float tBound1;
    if (!ray_sphere_intersect(camLocal, rd, center, boundRadius, tBound0, tBound1))
    {
        return false;
    }

    tEnd = tBound1;
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w > 0.0)
    {
        float tSurf = dot(posSample.xyz - camLocal, rd);
        if (tSurf > 0.0)
        {
            bool isPlanet = (posSample.w > 1.5);
            if (isPlanet)
            {
                float snapM = max(pc.jitter_params.y, 0.0);
                if (snapM > 0.0)
                {
                    float tP0;
                    float tP1;
                    if (ray_sphere_intersect(camLocal, rd, center, planetRadius, tP0, tP1))
                    {
                        float tSphere = (tP0 > 0.0) ? tP0 : tP1;
                        if (tSphere > 0.0)
                        {
                            float rSurf = length(posSample.xyz - center);
                            if (rSurf < planetRadius) tSurf = min(tSurf, tSphere);
                            if (abs(rSurf - planetRadius) <= snapM) tSurf = tSphere;
                        }
                    }
                }
            }
            tEnd = min(tEnd, tSurf);
        }
    }

    tStart = max(tBound0, 0.0);
    return tEnd > tStart;
}

bool segment_valid(vec2 seg)
{
    return seg.x < seg.y;
}

void main()
{
    outCloudLighting = vec4(0.0, 0.0, 0.0, 1.0);
    outCloudSegment = vec2(0.0);

    float planetRadius = pc.planet_center_radius.w;
    if (planetRadius <= 0.0) return;

    uint miscPacked = uint(pc.misc.w);
    int flags = int(miscPacked & 0xFFu);
    bool wantClouds = (flags & 2) != 0;
    if (!wantClouds) return;

    float cloudBaseM = max(pc.cloud_layer.x, 0.0);
    float cloudThicknessM = max(pc.cloud_layer.y, 0.0);
    float cloudDensScale = max(pc.cloud_layer.z, 0.0);
    float rBase = planetRadius + cloudBaseM;
    float rTop = rBase + cloudThicknessM;
    if (cloudThicknessM <= 0.0 || cloudDensScale <= 0.0 || rTop <= planetRadius) return;

    bool wantAtmosphere = (flags & 1) != 0;
    float atmRadius = pc.atmosphere_params.x;
    float boundRadius = (wantAtmosphere && atmRadius > planetRadius) ? atmRadius : rTop;

    vec3 camLocal = inCamLocal;
    vec3 rd = normalize(inWorldRay);
    vec3 center = pc.planet_center_radius.xyz;

    float tStart;
    float tEnd;
    if (!compute_ray_bounds(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        return;
    }

    vec2 analyticSeg0;
    vec2 analyticSeg1;
    int analyticCount = compute_cloud_segments(camLocal, rd, center, rBase, rTop, tStart, tEnd, analyticSeg0, analyticSeg1);
    if (analyticCount != 1 || !segment_valid(analyticSeg0))
    {
        return;
    }

    ivec2 lowSize = textureSize(cloudSegmentLowResTex, 0);
    ivec2 baseCoord = clamp(ivec2(gl_FragCoord.xy) / 2, ivec2(0), lowSize - ivec2(1));

    float bestScore = 1e30;
    vec4 bestLighting = vec4(0.0, 0.0, 0.0, 1.0);
    bool found = false;

    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            ivec2 coord = clamp(baseCoord + ivec2(x, y), ivec2(0), lowSize - ivec2(1));
            vec2 candidateSeg = texelFetch(cloudSegmentLowResTex, coord, 0).rg;
            if (!segment_valid(candidateSeg))
            {
                continue;
            }

            float score = abs(candidateSeg.x - analyticSeg0.x) + abs(candidateSeg.y - analyticSeg0.y);
            if (score < bestScore)
            {
                bestScore = score;
                bestLighting = texelFetch(cloudLightingLowResTex, coord, 0);
                found = true;
            }
        }
    }

    if (!found)
    {
        return;
    }

    outCloudLighting = bestLighting;
    outCloudSegment = analyticSeg0;
}
