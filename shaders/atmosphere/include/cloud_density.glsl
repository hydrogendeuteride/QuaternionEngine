float sample_cloud_noise_grad(vec2 uvE, vec2 uvDx, vec2 uvDy, float scale, vec2 offset)
{
    vec2 uv = uvE * scale + offset;
    return textureGrad(cloudNoiseTex, fract(uv), uvDx * scale, uvDy * scale).r;
}

float sample_cloud_noise_grad_aniso(vec2 uvE, vec2 uvDx, vec2 uvDy, vec2 scale, vec2 offset)
{
    vec2 uv = uvE * scale + offset;
    return textureGrad(cloudNoiseTex, fract(uv), uvDx * scale, uvDy * scale).r;
}

float sample_cloud_noise_3d_grad(vec3 p, vec3 dpdx, vec3 dpdy, vec3 offset)
{
    return textureGrad(cloudNoiseTex3D, fract(p + offset), dpdx, dpdy).r;
}

void build_local_cloud_frame(vec3 dir, out vec3 east, out vec3 north)
{
    vec3 refAxis = mix(vec3(0.0, 1.0, 0.0),
                       vec3(1.0, 0.0, 0.0),
                       step(0.999, abs(dir.y)));
    east = normalize(cross(refAxis, dir));
    north = cross(dir, east);
}

vec3 differentiate_normalized(vec3 v, vec3 dv)
{
    float vLen2 = max(dot(v, v), 1e-8);
    float invLen = inversesqrt(vLen2);
    vec3 n = v * invLen;
    return (dv - n * dot(n, dv)) * invLen;
}

void dir_to_equirect_grad(vec3 d,
                          vec3 ddx,
                          vec3 ddy,
                          bool flipV,
                          out vec2 uv,
                          out vec2 uvDx,
                          out vec2 uvDy)
{
    d = normalize(d);
    uv = dir_to_equirect(d, flipV);

    float xz2 = max(dot(d.xz, d.xz), 1e-6);
    float invXZ2 = 1.0 / xz2;
    float invSqrtOneMinusY2 = inversesqrt(max(1.0 - d.y * d.y, 1e-6));

    uvDx.x = (d.x * ddx.z - d.z * ddx.x) * invXZ2 * INV_TWO_PI;
    uvDy.x = (d.x * ddy.z - d.z * ddy.x) * invXZ2 * INV_TWO_PI;
    uvDx.y = -ddx.y * invSqrtOneMinusY2 * INV_PI;
    uvDy.y = -ddy.y * invSqrtOneMinusY2 * INV_PI;

    if (flipV)
    {
        uvDx.y = -uvDx.y;
        uvDy.y = -uvDy.y;
    }
}

float cloud_density(vec3 dir,
                    vec3 dirDx,
                    vec3 dirDy,
                    float height,
                    float heightDx,
                    float heightDy,
                    bool cloudsActive,
                    bool flipV,
                    vec2 overlayRotSC,
                    vec2 windHeading,
                    vec2 windSC)
{
    float baseHeightM = max(pc.cloud_layer.x, 0.0);
    float thicknessM = max(pc.cloud_layer.y, 0.0);
    float densityScale = max(pc.cloud_layer.z, 0.0);
    float coverage = clamp(pc.cloud_layer.w, 0.0, 0.999);

    if (thicknessM <= 0.0 || densityScale <= 0.0) return 0.0;
    if (height < baseHeightM || height > baseHeightM + thicknessM) return 0.0;

    float h01 = (height - baseHeightM) / max(thicknessM, 1e-3);

    vec3 dirW = dir;
    bool windActive = cloudsActive && abs(windSC.x) > 1e-4;
    if (windActive)
    {
        vec3 east;
        vec3 north;
        build_local_cloud_frame(dir, east, north);
        vec3 windT = east * windHeading.x + north * windHeading.y;

        float s = windSC.x;
        float c = windSC.y;
        dirW = dir * c + windT * s;
    }

    vec3 dirWdx = dirDx;
    vec3 dirWdy = dirDy;
    bool rotateOverlay = abs(overlayRotSC.x) > 1e-4;
    if (rotateOverlay)
    {
        dirW = rotate_y_sc(dirW, overlayRotSC);
        dirWdx = rotate_y_sc(dirDx, overlayRotSC);
        dirWdy = rotate_y_sc(dirDy, overlayRotSC);
    }

    vec2 uvE;
    vec2 uvDx;
    vec2 uvDy;
    dir_to_equirect_grad(dirW, dirWdx, dirWdy, flipV, uvE, uvDx, uvDy);
    float localCov = clamp(textureGrad(cloudOverlayTex, uvE, uvDx, uvDy).r, 0.0, 1.0);

    uint miscPacked = uint(pc.misc.w);
    float weatherBlend = float((miscPacked >> MISC_NOISE_BLEND_SHIFT) & 0xFFu) * (1.0 / 255.0);
    float detailErode = float((miscPacked >> MISC_DETAIL_ERODE_SHIFT) & 0xFFu) * (1.0 / 255.0);
    bool useNoise3D = (int(miscPacked & MISC_FLAGS_MASK) & FLAG_CLOUD_NOISE_3D) != 0;

    float lowScale = max(pc.cloud_params.x, 0.001);
    float detailScale = max(pc.cloud_params.y, 0.001);
    float weatherField = localCov;
    if (weatherBlend > 1e-4)
    {
        float weatherNoise = sample_cloud_noise_grad(uvE, uvDx, uvDy, lowScale, vec2(0.173, 0.547));
        weatherNoise = clamp((weatherNoise - 0.5) * 1.25 + 0.5, 0.0, 1.0);
        weatherField = clamp(mix(localCov, weatherNoise, weatherBlend), 0.0, 1.0);
    }

    float centerShift = (weatherField - 0.5) * CLOUD_HEIGHT_SHIFT;
    float halfSpan = 0.5 * mix(CLOUD_THICKNESS_MIN, CLOUD_THICKNESS_MAX, weatherField);
    float localBottom01 = clamp(0.5 + centerShift - halfSpan, 0.0, 1.0);
    float localTop01 = clamp(0.5 + centerShift + halfSpan, 0.0, 1.0);
    float localSpan = max(localTop01 - localBottom01, 1e-3);

    if (h01 < localBottom01 || h01 > localTop01) return 0.0;
    float hLocal = (h01 - localBottom01) / localSpan;

    float bottomEdge = mix(0.22, 0.10, weatherField);
    float topEdgeStart = mix(0.60, 0.82, weatherField);
    float bottom = smoothstep(0.0, bottomEdge, hLocal);
    float top = 1.0 - smoothstep(topEdgeStart, 1.0, hLocal);
    float profile = bottom * top;
    if (profile <= 0.0) return 0.0;

    float weatherCov = mix(localCov, weatherField, 0.50);
    float cov = max(0.0, weatherCov - coverage) / max(1.0 - coverage, 1e-3);
    cov = pow(cov, mix(1.45, 0.75, weatherField));
    if (cov <= 0.0) return 0.0;

    float hLocalDx = heightDx / max(thicknessM * localSpan, 1e-3);
    float hLocalDy = heightDy / max(thicknessM * localSpan, 1e-3);

    vec2 shearDir = windHeading;
    float shearLen2 = dot(shearDir, shearDir);
    if (shearLen2 < 1e-6)
    {
        shearDir = normalize(vec2(0.8660254, 0.5));
    }
    else
    {
        shearDir *= inversesqrt(shearLen2);
    }

    float shearAmount = mix(CLOUD_SHEAR_MIN, CLOUD_SHEAR_MAX, weatherField);
    vec2 shearUv = shearDir * ((hLocal - 0.5) * shearAmount);

    float sliceU;
    float sliceV;
    if (useNoise3D)
    {
        float radialScaleA = detailScale * (1.0 + hLocal * CLOUD_3D_HEIGHT_FREQ);
        float radialScaleB = detailScale * (0.83 + hLocal * CLOUD_3D_HEIGHT_FREQ * 1.17);
        float radialScaleDxA = detailScale * CLOUD_3D_HEIGHT_FREQ * hLocalDx;
        float radialScaleDyA = detailScale * CLOUD_3D_HEIGHT_FREQ * hLocalDy;
        float radialScaleDxB = detailScale * CLOUD_3D_HEIGHT_FREQ * 1.17 * hLocalDx;
        float radialScaleDyB = detailScale * CLOUD_3D_HEIGHT_FREQ * 1.17 * hLocalDy;

        vec3 sampleA = dirW * radialScaleA + vec3(shearUv.x, hLocal * CLOUD_SLICE_HEIGHT_FREQ, shearUv.y);
        vec3 sampleB = vec3(dirW.z, dirW.x, -dirW.y) * radialScaleB +
                       vec3(-shearUv.y, hLocal * CLOUD_SLICE_HEIGHT_FREQ * 1.13, shearUv.x);

        vec3 sampleADx = dirWdx * radialScaleA + dirW * radialScaleDxA +
                         vec3(0.0, hLocalDx * CLOUD_SLICE_HEIGHT_FREQ, 0.0);
        vec3 sampleADy = dirWdy * radialScaleA + dirW * radialScaleDyA +
                         vec3(0.0, hLocalDy * CLOUD_SLICE_HEIGHT_FREQ, 0.0);
        vec3 sampleBDx = vec3(dirWdx.z, dirWdx.x, -dirWdx.y) * radialScaleB +
                         vec3(dirW.z, dirW.x, -dirW.y) * radialScaleDxB +
                         vec3(0.0, hLocalDx * CLOUD_SLICE_HEIGHT_FREQ * 1.13, 0.0);
        vec3 sampleBDy = vec3(dirWdy.z, dirWdy.x, -dirWdy.y) * radialScaleB +
                         vec3(dirW.z, dirW.x, -dirW.y) * radialScaleDyB +
                         vec3(0.0, hLocalDy * CLOUD_SLICE_HEIGHT_FREQ * 1.13, 0.0);

        sliceU = sample_cloud_noise_3d_grad(sampleA, sampleADx, sampleADy, vec3(0.619, 0.281, 0.113));
        sliceV = sample_cloud_noise_3d_grad(sampleB, sampleBDx, sampleBDy, vec3(0.173, 0.547, 0.763));
    }
    else
    {
        // Reuse the 2D noise texture as two orthogonal height slices to fake internal depth.
        sliceU = sample_cloud_noise_grad_aniso(vec2(uvE.x + shearUv.x, hLocal),
                                               vec2(uvDx.x, hLocalDx),
                                               vec2(uvDy.x, hLocalDy),
                                               vec2(detailScale, CLOUD_SLICE_HEIGHT_FREQ),
                                               vec2(0.619, 0.281));
        sliceV = sample_cloud_noise_grad_aniso(vec2(uvE.y + shearUv.y, hLocal),
                                               vec2(uvDx.y, hLocalDx),
                                               vec2(uvDy.y, hLocalDy),
                                               vec2(detailScale * 0.85, CLOUD_SLICE_HEIGHT_FREQ * 1.13),
                                               vec2(0.113, 0.763));
    }
    sliceU = clamp((sliceU - 0.5) * 1.55 + 0.5, 0.0, 1.0);
    sliceV = clamp((sliceV - 0.5) * 1.55 + 0.5, 0.0, 1.0);

    float column = smoothstep(0.18, 0.82, mix(sliceU, sliceV, 0.45));
    if (column <= 0.0) return 0.0;

    float erosion = smoothstep(0.14, 0.72, min(sliceU, sliceV));
    float detailMask = mix(column, column * erosion, detailErode);
    if (detailMask <= 0.0) return 0.0;

    float d = cov * profile * detailMask;
    return max(d, 0.0) * densityScale;
}
