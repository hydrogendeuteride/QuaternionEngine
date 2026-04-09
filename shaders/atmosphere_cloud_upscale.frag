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
layout(set = 1, binding = 3) uniform sampler2D planetHeightTexPX;
layout(set = 1, binding = 4) uniform sampler2D planetHeightTexNX;
layout(set = 1, binding = 5) uniform sampler2D planetHeightTexPY;
layout(set = 1, binding = 6) uniform sampler2D planetHeightTexNY;
layout(set = 1, binding = 7) uniform sampler2D planetHeightTexPZ;
layout(set = 1, binding = 8) uniform sampler2D planetHeightTexNZ;

layout(push_constant) uniform AtmospherePush
{
    vec4 planet_center_radius;
    vec4 atmosphere_params;
    vec4 beta_rayleigh;
    vec4 beta_mie;
    vec4 jitter_params;
    vec4 terrain_params;
    vec4 cloud_layer;
    vec4 cloud_params;
    vec4 cloud_color;
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

bool terrain_height_enabled()
{
    return pc.terrain_params.x > 0.0 || pc.terrain_params.y > 0.0;
}

float sample_planet_height_face(int faceIndex, vec2 uv01)
{
    vec2 uv = clamp(uv01, vec2(0.0), vec2(1.0));
    switch (faceIndex)
    {
        case 0: return textureLod(planetHeightTexPX, uv, 0.0).r;
        case 1: return textureLod(planetHeightTexNX, uv, 0.0).r;
        case 2: return textureLod(planetHeightTexPY, uv, 0.0).r;
        case 3: return textureLod(planetHeightTexNY, uv, 0.0).r;
        case 4: return textureLod(planetHeightTexPZ, uv, 0.0).r;
        case 5: return textureLod(planetHeightTexNZ, uv, 0.0).r;
        default: return 0.0;
    }
}

bool cubesphere_direction_to_face_uv(vec3 dir, out int faceIndex, out vec2 uv01)
{
    vec3 d = normalize(dir);
    vec3 ad = abs(d);
    float ma = 1.0;
    vec2 uv = vec2(0.0);

    if (ad.x >= ad.y && ad.x >= ad.z)
    {
        ma = ad.x;
        if (d.x >= 0.0)
        {
            faceIndex = 0;
            uv = vec2(-d.z / ma, -d.y / ma);
        }
        else
        {
            faceIndex = 1;
            uv = vec2(d.z / ma, -d.y / ma);
        }
    }
    else if (ad.y >= ad.x && ad.y >= ad.z)
    {
        ma = ad.y;
        if (d.y >= 0.0)
        {
            faceIndex = 2;
            uv = vec2(d.x / ma, d.z / ma);
        }
        else
        {
            faceIndex = 3;
            uv = vec2(d.x / ma, -d.z / ma);
        }
    }
    else
    {
        ma = ad.z;
        if (d.z >= 0.0)
        {
            faceIndex = 4;
            uv = vec2(d.x / ma, -d.y / ma);
        }
        else
        {
            faceIndex = 5;
            uv = vec2(-d.x / ma, -d.y / ma);
        }
    }

    uv01 = clamp(uv * 0.5 + 0.5, vec2(0.0), vec2(1.0));
    return true;
}

float sample_planet_surface_radius(vec3 p, vec3 center, float planetRadius)
{
    float heightScale = max(pc.terrain_params.x, 0.0);
    float heightOffset = max(pc.terrain_params.y, 0.0);
    vec3 radial = p - center;
    float radialLen = length(radial);
    if (radialLen <= 1e-5)
    {
        return planetRadius + heightOffset;
    }

    int faceIndex = 0;
    vec2 uv01 = vec2(0.5);
    cubesphere_direction_to_face_uv(radial / radialLen, faceIndex, uv01);
    return planetRadius + heightOffset + sample_planet_height_face(faceIndex, uv01) * heightScale;
}

float planet_surface_signed_distance(vec3 camLocal,
                                     vec3 rd,
                                     vec3 center,
                                     float planetRadius,
                                     float t)
{
    vec3 p = camLocal + rd * t;
    return length(p - center) - sample_planet_surface_radius(p, center, planetRadius);
}

bool solve_planet_heightmap_surface_depth(vec3 camLocal,
                                          vec3 rd,
                                          vec3 center,
                                          float planetRadius,
                                          out float outTSurf)
{
    if (!terrain_height_enabled())
    {
        return false;
    }

    float heightScale = max(pc.terrain_params.x, 0.0);
    float heightOffset = max(pc.terrain_params.y, 0.0);
    float innerRadius = planetRadius + heightOffset;
    float outerRadius = innerRadius + heightScale;
    if (outerRadius <= 0.0)
    {
        return false;
    }

    float tOuter0;
    float tOuter1;
    if (!ray_sphere_intersect(camLocal, rd, center, outerRadius, tOuter0, tOuter1))
    {
        return false;
    }

    float tLo = max(tOuter0, 0.0);
    float fLo = planet_surface_signed_distance(camLocal, rd, center, planetRadius, tLo);
    if (fLo <= 0.0)
    {
        outTSurf = tLo;
        return true;
    }

    float tHi = tOuter1;
    float fHi = 1.0;
    bool haveHi = false;

    float tInner0;
    float tInner1;
    if (ray_sphere_intersect(camLocal, rd, center, innerRadius, tInner0, tInner1))
    {
        float candidate = -1.0;
        if (tInner0 > tLo)
        {
            candidate = tInner0;
        }
        else if (tInner1 > tLo)
        {
            candidate = tInner1;
        }

        if (candidate > 0.0)
        {
            tHi = candidate;
            fHi = planet_surface_signed_distance(camLocal, rd, center, planetRadius, tHi);
            haveHi = fHi <= 0.0;
        }
    }

    if (!haveHi)
    {
        float searchEnd = max(tOuter1, tLo);
        float prevT = tLo;
        for (int i = 1; i <= 32; ++i)
        {
            float t = mix(tLo, searchEnd, float(i) / 32.0);
            float f = planet_surface_signed_distance(camLocal, rd, center, planetRadius, t);
            if (f <= 0.0)
            {
                tLo = prevT;
                tHi = t;
                fHi = f;
                haveHi = true;
                break;
            }
            prevT = t;
        }
    }

    if (!haveHi)
    {
        return false;
    }

    for (int i = 0; i < 8; ++i)
    {
        float tm = 0.5 * (tLo + tHi);
        float fm = planet_surface_signed_distance(camLocal, rd, center, planetRadius, tm);
        if (fm > 0.0)
        {
            tLo = tm;
        }
        else
        {
            tHi = tm;
            fHi = fm;
        }
    }

    outTSurf = tHi;
    return fHi <= 0.0;
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
                float terrainTSurf = tSurf;
                if (solve_planet_heightmap_surface_depth(camLocal, rd, center, planetRadius, terrainTSurf))
                {
                    tSurf = terrainTSurf;
                }
                else
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

float segment_error(vec2 lhs, vec2 rhs)
{
    return abs(lhs.x - rhs.x) + abs(lhs.y - rhs.y);
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

    float analyticLen = max(analyticSeg0.y - analyticSeg0.x, 1e-3);
    float rejectThreshold = max(2500.0, analyticLen * 0.45);
    float blendThreshold = max(900.0, analyticLen * 0.14);
    vec4 accumulatedLighting = vec4(0.0);
    float accumulatedWeight = 0.0;
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

            float score = segment_error(candidateSeg, analyticSeg0);
            if (score > rejectThreshold)
            {
                continue;
            }

            float scoreNorm = score / max(blendThreshold, 1e-3);
            float segmentWeight = exp(-scoreNorm * scoreNorm);
            float spatialWeight = 1.0 / (1.0 + float(x * x + y * y));
            float weight = segmentWeight * spatialWeight;
            if (weight <= 1e-4)
            {
                continue;
            }

            accumulatedLighting += texelFetch(cloudLightingLowResTex, coord, 0) * weight;
            accumulatedWeight += weight;
            found = true;
        }
    }

    if (!found || accumulatedWeight <= 1e-4)
    {
        return;
    }

    outCloudLighting = accumulatedLighting / accumulatedWeight;
    outCloudSegment = analyticSeg0;
}
