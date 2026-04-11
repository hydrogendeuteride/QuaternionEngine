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

bool compute_primary_ray_bounds(vec3 camLocal,
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

bool compute_primary_ray_bounds_fast(vec3 camLocal,
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
                bool resolvedApprox = false;
                if (terrain_height_enabled())
                {
                    float approxRadius = sample_planet_surface_radius(posSample.xyz, center, planetRadius);
                    float tP0;
                    float tP1;
                    if (ray_sphere_intersect(camLocal, rd, center, approxRadius, tP0, tP1))
                    {
                        float tApprox = (tP0 > 0.0) ? tP0 : tP1;
                        if (tApprox > 0.0)
                        {
                            tSurf = tApprox;
                            resolvedApprox = true;
                        }
                    }
                }

                if (!resolvedApprox)
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

bool compute_primary_ray_bounds_cloud_hybrid(vec3 camLocal,
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
                bool resolvedApprox = false;
                bool requireExact = terrain_height_enabled();

                if (terrain_height_enabled())
                {
                    vec3 surfaceRadial = posSample.xyz - center;
                    float surfaceRadialLen = length(surfaceRadial);
                    if (surfaceRadialLen > 1e-5)
                    {
                        float approxRadius = sample_planet_surface_radius(posSample.xyz, center, planetRadius);
                        float tP0;
                        float tP1;
                        if (ray_sphere_intersect(camLocal, rd, center, approxRadius, tP0, tP1))
                        {
                            float tApprox = (tP0 > 0.0) ? tP0 : tP1;
                            if (tApprox > 0.0)
                            {
                                float terrainScale = max(pc.terrain_params.x, 0.0);
                                float tolerance = max(max(pc.jitter_params.y,
                                                          terrainScale * CLOUD_TERRAIN_FAST_TOLERANCE_FRAC),
                                                      CLOUD_TERRAIN_FAST_TOLERANCE_MIN_M);
                                float radialMismatch = abs(surfaceRadialLen - approxRadius);
                                float depthMismatch = abs(tApprox - tSurf);
                                float viewCos = abs(dot(surfaceRadial / surfaceRadialLen, rd));
                                if (radialMismatch <= tolerance &&
                                    depthMismatch <= tolerance &&
                                    viewCos >= CLOUD_TERRAIN_FAST_GRAZE_MIN_COS)
                                {
                                    tSurf = tApprox;
                                    resolvedApprox = true;
                                    requireExact = false;
                                }
                            }
                        }
                    }
                }

                if (requireExact)
                {
                    float terrainTSurf = tSurf;
                    if (solve_planet_heightmap_surface_depth(camLocal, rd, center, planetRadius, terrainTSurf))
                    {
                        tSurf = terrainTSurf;
                        resolvedApprox = true;
                    }
                }

                if (!resolvedApprox)
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
