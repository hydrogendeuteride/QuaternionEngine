#ifndef PLANET_TERRAIN_COMMON_GLSL
#define PLANET_TERRAIN_COMMON_GLSL

bool terrain_material_enabled()
{
    return materialData.extra[3].x > 0.5;
}

vec3 terrain_planet_center_local()
{
    return materialData.extra[4].xyz;
}

float terrain_planet_radius_m()
{
    return max(materialData.extra[4].w, 1.0);
}

int terrain_face_index()
{
    return int(materialData.extra[5].x + 0.5);
}

float terrain_detail_normal_strength()
{
    return materialData.extra[5].y;
}

bool terrain_terminator_enabled()
{
    return (materialData.extra[5].w > 0.5) && (materialData.extra[6].x > 0.0);
}

float terrain_height_max_m()
{
    return max(materialData.extra[6].x, 0.0);
}

float terrain_height_offset_m()
{
    return max(materialData.extra[6].y, 0.0);
}

vec3 cubesphere_face_uv_to_direction(int faceIndex, vec2 uv01)
{
    vec2 uv = clamp(uv01, vec2(0.0), vec2(1.0)) * 2.0 - 1.0;
    vec3 dir = vec3(0.0, 0.0, 1.0);
    switch (faceIndex)
    {
        case 0: dir = vec3(1.0, -uv.y, -uv.x); break;
        case 1: dir = vec3(-1.0, -uv.y, uv.x); break;
        case 2: dir = vec3(uv.x, 1.0, uv.y); break;
        case 3: dir = vec3(uv.x, -1.0, -uv.y); break;
        case 4: dir = vec3(uv.x, -uv.y, 1.0); break;
        case 5: dir = vec3(-uv.x, -uv.y, -1.0); break;
        default: break;
    }
    return normalize(dir);
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

float terrain_sample_height01(vec2 uv, vec2 uv_dx, vec2 uv_dy)
{
    return textureGrad(metalRoughTex, uv, uv_dx, uv_dy).r;
}

vec3 terrain_surface_position_from_height(int faceIndex,
                                          vec2 uv,
                                          float radiusM,
                                          float heightOffset,
                                          float heightScale,
                                          vec2 uv_dx,
                                          vec2 uv_dy)
{
    vec3 dir = cubesphere_face_uv_to_direction(faceIndex, uv);
    float heightM = terrain_sample_height01(uv, uv_dx, uv_dy) * heightScale;
    return dir * (radiusM + heightOffset + heightM);
}

vec3 terrain_shading_normal_from_height(vec2 uv, vec3 worldPos, vec3 fallbackNormal)
{
    vec3 fallback = normalize(fallbackNormal);
    if (!terrain_material_enabled())
    {
        return fallback;
    }

    float heightScale = terrain_height_max_m();
    if (heightScale <= 0.0)
    {
        return fallback;
    }

    vec3 localPos = worldPos - terrain_planet_center_local();
    float localLen = length(localPos);
    if (localLen <= 1e-4)
    {
        return fallback;
    }

    vec2 texelSize = 1.0 / max(vec2(textureSize(metalRoughTex, 0)), vec2(1.0));
    float edgeDist = min(min(uv.x, uv.y), min(1.0 - uv.x, 1.0 - uv.y));
    float edgeFade = smoothstep(0.0, 2.0 * max(texelSize.x, texelSize.y), edgeDist);
    if (edgeFade <= 0.0)
    {
        return fallback;
    }

    vec2 uv_dx = dFdx(uv);
    vec2 uv_dy = dFdy(uv);
    float radiusM = terrain_planet_radius_m();
    float heightOffset = terrain_height_offset_m();
    int faceIndex = terrain_face_index();

    vec2 uvL = vec2(max(uv.x - texelSize.x, 0.0), uv.y);
    vec2 uvR = vec2(min(uv.x + texelSize.x, 1.0), uv.y);
    vec2 uvU = vec2(uv.x, max(uv.y - texelSize.y, 0.0));
    vec2 uvD = vec2(uv.x, min(uv.y + texelSize.y, 1.0));

    vec3 pL = terrain_surface_position_from_height(faceIndex, uvL, radiusM, heightOffset, heightScale, uv_dx, uv_dy);
    vec3 pR = terrain_surface_position_from_height(faceIndex, uvR, radiusM, heightOffset, heightScale, uv_dx, uv_dy);
    vec3 pU = terrain_surface_position_from_height(faceIndex, uvU, radiusM, heightOffset, heightScale, uv_dx, uv_dy);
    vec3 pD = terrain_surface_position_from_height(faceIndex, uvD, radiusM, heightOffset, heightScale, uv_dx, uv_dy);

    vec3 n = cross(pD - pU, pR - pL);
    float len2 = dot(n, n);
    if (len2 <= 1e-8)
    {
        return fallback;
    }

    n *= inversesqrt(len2);
    vec3 radialDir = localPos / localLen;
    if (dot(n, radialDir) < 0.0)
    {
        n = -n;
    }

    float agreement = clamp(dot(n, fallback), -1.0, 1.0);
    float geometryBias = smoothstep(0.15, 0.55, 1.0 - agreement);
    vec3 resolved = normalize(mix(n, fallback, geometryBias));
    return normalize(mix(fallback, resolved, edgeFade));
}

vec3 apply_terrain_detail_normal(vec3 baseNormal, vec2 uv, mat3 normalMatrix)
{
    if (!terrain_material_enabled())
    {
        return baseNormal;
    }

    float strength = clamp(terrain_detail_normal_strength(), 0.0, 1.0);
    if (abs(strength) <= 1e-4)
    {
        return baseNormal;
    }

    vec3 detail_normal_obj = texture(normalMap, uv).xyz * 2.0 - 1.0;
    float len2 = dot(detail_normal_obj, detail_normal_obj);
    if (len2 <= 1e-5)
    {
        return baseNormal;
    }

    detail_normal_obj *= inversesqrt(len2);
    vec3 detail_normal_ws = normalize(normalMatrix * detail_normal_obj);
    vec3 mixed_normal = normalize(mix(baseNormal, detail_normal_ws, strength));
    return (dot(mixed_normal, baseNormal) > 0.0) ? mixed_normal : baseNormal;
}

float terrain_terminator_visibility(vec2 uv,
                                    vec3 shadedNormal,
                                    vec3 geometryNormal,
                                    vec3 worldPos,
                                    vec3 sunLocalDir)
{
    if (!terrain_material_enabled() || !terrain_terminator_enabled())
    {
        return 1.0;
    }

    vec3 shadedN = normalize(shadedNormal);
    vec3 geomN = normalize(geometryNormal);

    float NdotL = dot(geomN, sunLocalDir);
    if (NdotL >= 0.35)
    {
        return 1.0;
    }

    vec3 localPos = worldPos - terrain_planet_center_local();
    float localLen = length(localPos);
    if (localLen <= 1e-4)
    {
        return 1.0;
    }

    vec3 radialDir = localPos / localLen;
    vec3 sunTangent = sunLocalDir - radialDir * dot(radialDir, sunLocalDir);
    float sunTanLen = length(sunTangent);
    if (sunTanLen <= 1e-4)
    {
        return 1.0;
    }
    sunTangent /= sunTanLen;

    int faceIndex = terrain_face_index();
    int probeFace = faceIndex;
    vec2 probeUv = uv;
    cubesphere_direction_to_face_uv(normalize(radialDir + sunTangent * 0.01), probeFace, probeUv);
    if (probeFace != faceIndex)
    {
        return 1.0;
    }

    vec2 uvDir = probeUv - uv;
    float uvDirLen = length(uvDir);
    if (uvDirLen <= 1e-5)
    {
        return 1.0;
    }
    uvDir /= uvDirLen;

    vec2 texelSize = 1.0 / max(vec2(textureSize(metalRoughTex, 0)), vec2(1.0));
    float edgeDist = min(min(uv.x, uv.y), min(1.0 - uv.x, 1.0 - uv.y));
    float edgeFade = smoothstep(0.0, 2.0 * max(texelSize.x, texelSize.y), edgeDist);
    if (edgeFade <= 0.0)
    {
        return 1.0;
    }

    float texelStep = max(texelSize.x, texelSize.y);
    float heightScale = terrain_height_max_m();
    float heightOffset = terrain_height_offset_m();
    float radiusM = terrain_planet_radius_m();
    float height0 = heightOffset + texture(metalRoughTex, uv).r * heightScale;
    vec3 currentPos = radialDir * (radiusM + height0);
    float sunSlope = dot(sunLocalDir, radialDir) / max(sunTanLen, 1e-4);
    float maxSlope = -1e6;

    for (int tap = 1; tap <= 8; ++tap)
    {
        vec2 sampleUv = uv + uvDir * (float(tap) * 2.0 * texelStep);
        if (any(lessThan(sampleUv, vec2(0.0))) || any(greaterThan(sampleUv, vec2(1.0))))
        {
            return 1.0;
        }

        vec3 sampleDir = cubesphere_face_uv_to_direction(faceIndex, sampleUv);
        float sampleHeight = heightOffset + texture(metalRoughTex, sampleUv).r * heightScale;
        vec3 samplePos = sampleDir * (radiusM + sampleHeight);
        vec3 delta = samplePos - currentPos;
        float vertical = dot(delta, radialDir);
        float horizontal = length(delta - radialDir * vertical);
        float slope = vertical / max(horizontal, 1.0);
        maxSlope = max(maxSlope, slope);
    }

    float visibility = 1.0 - smoothstep(sunSlope - 0.02, sunSlope + 0.02, maxSlope);
    float agreement = clamp(dot(shadedN, geomN), 0.0, 1.0);
    float mismatchFade = 1.0 - smoothstep(0.15, 0.45, 1.0 - agreement);
    return mix(1.0, clamp(visibility, 0.0, 1.0), edgeFade * mismatchFade);
}

#endif
