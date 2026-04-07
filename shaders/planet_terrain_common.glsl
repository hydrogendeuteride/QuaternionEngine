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

vec3 apply_terrain_detail_normal(vec3 baseNormal, vec2 uv)
{
    if (!terrain_material_enabled())
    {
        return baseNormal;
    }

    float strength = terrain_detail_normal_strength();
    if (abs(strength) <= 1e-4)
    {
        return baseNormal;
    }

    vec3 detail_normal = texture(normalMap, uv).xyz * 2.0 - 1.0;
    float len2 = dot(detail_normal, detail_normal);
    if (len2 <= 1e-5)
    {
        return baseNormal;
    }

    detail_normal *= inversesqrt(len2);
    return normalize(mix(baseNormal, detail_normal, strength));
}

float terrain_terminator_visibility(vec2 uv, vec3 shadedNormal, vec3 worldPos, vec3 sunLocalDir)
{
    if (!terrain_material_enabled() || !terrain_terminator_enabled())
    {
        return 1.0;
    }

    float NdotL = dot(shadedNormal, sunLocalDir);
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
    float radiusM = terrain_planet_radius_m();
    float height0 = texture(metalRoughTex, uv).r * heightScale;
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
        float sampleHeight = texture(metalRoughTex, sampleUv).r * heightScale;
        vec3 samplePos = sampleDir * (radiusM + sampleHeight);
        vec3 delta = samplePos - currentPos;
        float vertical = dot(delta, radialDir);
        float horizontal = length(delta - radialDir * vertical);
        float slope = vertical / max(horizontal, 1.0);
        maxSlope = max(maxSlope, slope);
    }

    float visibility = 1.0 - smoothstep(sunSlope - 0.02, sunSlope + 0.02, maxSlope);
    return mix(1.0, clamp(visibility, 0.0, 1.0), edgeFade);
}

#endif
