bool terrain_height_enabled()
{
    return pc.terrain_params.x > 0.0 || pc.terrain_params.y > 0.0;
}

float compute_planet_height_sample_lod(vec3 camLocal, vec3 center, float planetRadius)
{
    float camAltitude = max(length(camLocal - center) - planetRadius, 0.0);
    ivec2 baseSize = textureSize(planetHeightTexPX, 0);
    int baseDim = max(max(baseSize.x, baseSize.y), 1);
    float baseTexelMeters = max((2.0 * planetRadius) / float(baseDim), 1.0);
    float targetMeters = max(baseTexelMeters, camAltitude * 0.05);
    int maxLevel = max(textureQueryLevels(planetHeightTexPX) - 1, 0);
    float lod = log2(max(targetMeters / baseTexelMeters, 1.0));
    return clamp(lod, 0.0, float(maxLevel));
}

float compute_planet_raster_depth_weight(vec3 camLocal, vec3 center, float planetRadius)
{
    float camAltitude = max(length(camLocal - center) - planetRadius, 0.0);
    float heightScale = max(pc.terrain_params.x, 0.0);
    float fadeStart = max(heightScale * 8.0, planetRadius * 0.0025);
    float fadeEnd = max(heightScale * 32.0, planetRadius * 0.02);
    if (fadeEnd <= fadeStart)
    {
        return 1.0;
    }
    return 1.0 - smoothstep(fadeStart, fadeEnd, camAltitude);
}

float sample_planet_height_face(int faceIndex, vec2 uv01, float lod)
{
    vec2 uv = clamp(uv01, vec2(0.0), vec2(1.0));
    switch (faceIndex)
    {
        case 0: return textureLod(planetHeightTexPX, uv, lod).r;
        case 1: return textureLod(planetHeightTexNX, uv, lod).r;
        case 2: return textureLod(planetHeightTexPY, uv, lod).r;
        case 3: return textureLod(planetHeightTexNY, uv, lod).r;
        case 4: return textureLod(planetHeightTexPZ, uv, lod).r;
        case 5: return textureLod(planetHeightTexNZ, uv, lod).r;
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
    float lod = compute_planet_height_sample_lod(inCamLocal, center, planetRadius);
    return planetRadius + heightOffset + sample_planet_height_face(faceIndex, uv01, lod) * heightScale;
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
        float prevF = fLo;
        for (int i = 1; i <= 32; ++i)
        {
            float t = mix(tLo, searchEnd, float(i) / 32.0);
            float f = planet_surface_signed_distance(camLocal, rd, center, planetRadius, t);
            if (f <= 0.0)
            {
                tLo = prevT;
                fLo = prevF;
                tHi = t;
                fHi = f;
                haveHi = true;
                break;
            }
            prevT = t;
            prevF = f;
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
