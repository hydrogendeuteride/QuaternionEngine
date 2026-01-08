#ifndef PLANET_SHADOW_GLSL
#define PLANET_SHADOW_GLSL

// Analytic directional shadowing from up to MAX_PLANET_OCCLUDERS spheres.
// - Uses sceneData.planetOccluders[i].xyz as center (render-local space) and .w as radius (meters).
// - Uses sceneData.lightCounts.z as the active occluder count.
// - Uses sceneData.rtParams.w as the sun angular radius (radians) for a soft penumbra.
//
// Returns visibility in [0..1] (1 = fully lit, 0 = fully shadowed).
float planet_analytic_shadow_visibility(vec3 p, vec3 Lsun)
{
    uint count = min(sceneData.lightCounts.z, uint(MAX_PLANET_OCCLUDERS));
    if (count == 0u)
    {
        return 1.0;
    }

    // Sun angular radius in radians. 0 => hard shadow edge.
    float theta = max(sceneData.rtParams.w, 0.0);

    float vis = 1.0;
    for (uint i = 0u; i < count; ++i)
    {
        vec4 occluder = sceneData.planetOccluders[i];
        float r = occluder.w;
        if (r <= 0.0)
        {
            continue;
        }

        vec3 oc = occluder.xyz - p;

        // Project onto sun direction: only occluders in front of the point can block.
        float b = dot(oc, Lsun);
        if (b <= 0.0)
        {
            continue;
        }

        // Distance from occluder center to the ray (closest approach).
        float oc2 = dot(oc, oc);
        float h2 = max(oc2 - b * b, 0.0);
        float h = sqrt(h2);

        // Signed "miss" distance in meters relative to the occluder radius.
        // miss < 0 => the sun-center ray intersects (hard shadow)
        float miss = h - r;

        float v = 1.0;
        if (theta <= 1.0e-6)
        {
            v = (miss >= 0.0) ? 1.0 : 0.0;
        }
        else
        {
            // Small-angle approximation: a ray rotated by theta misses by ~b*theta at distance b.
            float penumbra = b * theta;
            // Soft transition around the geometric boundary (miss==0).
            v = smoothstep(-penumbra, penumbra, miss);
        }

        vis = min(vis, v);
        if (vis <= 0.0)
        {
            break;
        }
    }

    return vis;
}

#endif // PLANET_SHADOW_GLSL

