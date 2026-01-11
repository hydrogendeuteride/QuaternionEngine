//Why RT: Because CSM is shit

#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_ray_query : require
#include "input_structures.glsl"
#include "ibl_common.glsl"
#include "lighting_common.glsl"
#include "planet_shadow.glsl"

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

layout(set=1, binding=0) uniform sampler2D posTex;
layout(set=1, binding=1) uniform sampler2D normalTex;
layout(set=1, binding=2) uniform sampler2D albedoTex;
layout(set=1, binding=3) uniform sampler2D extraTex;
layout(set=2, binding=0) uniform sampler2D shadowTex[4];
// TLAS for ray query (optional, guarded by sceneData.rtOptions.x)

#ifdef GL_EXT_ray_query
layout(set=0, binding=1) uniform accelerationStructureEXT topLevelAS;
#endif

// Tunables for shadow quality and blending
// Border smoothing width in light-space NDC (0..1). Larger = wider cross-fade.
const float SHADOW_BORDER_SMOOTH_NDC = 0.08;
// Base PCF radius in texels for cascade 0; higher cascades scale this up slightly.
const float SHADOW_PCF_BASE_RADIUS = 1.15;
// Additional per-cascade radius scale for coarser cascades (0..1 factor added across levels)
const float SHADOW_PCF_CASCADE_GAIN = 2.0;// extra radius at far end
// Receiver normal-based offset to reduce acne (in world units)
const float SHADOW_NORMAL_OFFSET = 0.0025;
// Scale for receiver-plane depth bias term (tweak if over/under biased)
const float SHADOW_RPDB_SCALE = 1.0;
// Minimum clamp to keep a tiny bias even on perpendicular receivers
const float SHADOW_MIN_BIAS = 1e-5;
// Ray query safety params
const float SHADOW_RAY_TMIN = 0.02;// start a bit away from the surface
const float SHADOW_RAY_ORIGIN_BIAS = 0.01;// world units

// Estimate the float ULP scale at this world position magnitude, used to keep
// ray bias and tMin effective even when world coordinates are very large.
float world_pos_ulp(vec3 p)
{
    float m = max(max(abs(p.x), abs(p.y)), abs(p.z));
    // For IEEE-754 float, relative precision is ~2^-23 (~1.192e-7). Clamp to a small baseline to avoid tiny values near the origin.
    return max(1e-4, m * 1.1920929e-7);
}

float shadow_ray_origin_bias(vec3 p)
{
    return max(SHADOW_RAY_ORIGIN_BIAS, world_pos_ulp(p) * 8.0);
}

float shadow_ray_tmin(vec3 p)
{
    return max(SHADOW_RAY_TMIN, world_pos_ulp(p) * 16.0);
}

vec3 getCameraWorldPosition()   // Because I'm not clever enough to add cmaera position...
{
    // view = [ R^T  -R^T*C ]
    //        [ 0       1   ]
    // => C = -R * T, where T is view[3].xyz and R = transpose(mat3(view))
    mat3 rotT = mat3(sceneData.view);
    mat3 rot  = transpose(rotT);
    vec3 T    = sceneData.view[3].xyz;
    return -rot * T;
}

float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z);
}

const vec2 POISSON_16[16] = vec2[16](
vec2(0.2852, -0.1883), vec2(-0.1464, 0.2591),
vec2(-0.3651, -0.0974), vec2(0.0901, 0.3807),
vec2(0.4740, 0.0679), vec2(-0.0512, -0.4466),
vec2(-0.4497, 0.1673), vec2(0.3347, 0.3211),
vec2(0.1948, -0.4196), vec2(-0.2919, -0.3291),
vec2(-0.0763, 0.4661), vec2(0.4421, -0.2217),
vec2(0.0281, -0.2468), vec2(-0.2104, 0.0573),
vec2(0.1197, 0.0779), vec2(-0.0905, -0.1203)
);

// Precomputed per-tap weights: w = 1 - smoothstep(0, 0.65, length(POISSON_16[i])).
// (Rotation preserves length, so these are invariant.)
const float POISSON_16_WEIGHT[16] = float[16](
0.46137072, 0.56308092, 0.37907144, 0.34930667,
0.17150249, 0.22669642, 0.16976301, 0.19912809,
0.20140948, 0.24589236, 0.18334537, 0.14418702,
0.67350789, 0.73787198, 0.87638682, 0.86392944
);

// Compute primary cascade and an optional neighbor for cross-fade near borders
struct CascadeMix { uint i0; uint i1; float w1; };

CascadeMix computeCascadeMix(vec3 worldPos)
{
    uint primary = 3u;
    vec3 ndcP = vec3(0);
    for (uint i = 0u; i < 4u; ++i)
    {
        vec4 lclip = sceneData.lightViewProjCascades[i] * vec4(worldPos, 1.0);
        vec3 ndc = lclip.xyz / max(lclip.w, 1e-6);
        if (abs(ndc.x) <= 1.0 && abs(ndc.y) <= 1.0 && ndc.z >= 0.0 && ndc.z <= 1.0)
        {
            primary = i;
            ndcP = ndc;
            break;
        }
    }

    CascadeMix cm; cm.i0 = primary; cm.i1 = primary; cm.w1 = 0.0;

    if (primary < 3u)
    {
        float edge = max(abs(ndcP.x), abs(ndcP.y));// 0..1, 1 at border
        // start blending when we are within S of the border
        float t = clamp((edge - (1.0 - SHADOW_BORDER_SMOOTH_NDC)) / max(SHADOW_BORDER_SMOOTH_NDC, 1e-4), 0.0, 1.0);
        float w = smoothstep(0.0, 1.0, t);

        if (w > 0.0)
        {
            // Only blend if neighbor actually covers the point
            uint neighbor = primary + 1u;
            vec4 lclipN = sceneData.lightViewProjCascades[neighbor] * vec4(worldPos, 1.0);
            vec3 ndcN = lclipN.xyz / max(lclipN.w, 1e-6);
            bool insideN = (abs(ndcN.x) <= 1.0 && abs(ndcN.y) <= 1.0 && ndcN.z >= 0.0 && ndcN.z <= 1.0);
            if (insideN)
            {
                cm.i1 = neighbor;
                cm.w1 = w;
            }
        }
    }

    return cm;
}

// Compute receiver-plane depth gradient dz/duv using derivatives of shadow NDC
// Reference: Akenine-Möller et al., "Receiver Plane Depth Bias" (PCF-friendly)
vec2 receiverPlaneDepthGradient(vec3 ndc, vec3 dndc_dx, vec3 dndc_dy)
{
    // Convert XY to shadow map UV derivatives (ndc -> uv: u = 0.5*x + 0.5)
    vec2 duv_dx = 0.5 * dndc_dx.xy;
    vec2 duv_dy = 0.5 * dndc_dy.xy;

    // Build Jacobian J = [du/dx du/dy; dv/dx dv/dy] (column-major)
    mat2 J = mat2(duv_dx.x, duv_dy.x,
    duv_dx.y, duv_dy.y);

    // Depth derivatives w.r.t screen pixels
    vec2 dz_dxdy = vec2(dndc_dx.z, dndc_dy.z);

    // Invert J to obtain dz/du and dz/dv. Guard against near-singular Jacobian.
    float det = J[0][0] * J[1][1] - J[1][0] * J[0][1];
    if (abs(det) < 1e-8)
    {
        // Degenerate mapping; return zero gradient so only slope/const bias applies
        return vec2(0.0);
    }

    // Manual inverse for stability/perf on some drivers
    mat2 invJ = (1.0 / det) * mat2(J[1][1], -J[0][1],
    -J[1][0], J[0][0]);
    return invJ * dz_dxdy;// (dz/du, dz/dv)
}

float sampleCascadeShadow(uint ci, vec3 worldPos, vec3 N, vec3 L)
{
    mat4 lightMat = sceneData.lightViewProjCascades[ci];

    vec4 lclip = lightMat * vec4(worldPos, 1.0);
    vec3 ndc  = lclip.xyz / lclip.w;
    vec2 suv  = ndc.xy * 0.5 + 0.5;

    if (any(lessThan(suv, vec2(0.0))) || any(greaterThan(suv, vec2(1.0))))
    return 1.0;

    float current = clamp(ndc.z, 0.0, 1.0);

    // Slope-based tiny baseline bias (cheap safety net)
    float NoL       = max(dot(N, L), 0.0);
    float slopeBias = max(0.0006 * (1.0 - NoL), SHADOW_MIN_BIAS);
    float currentBias = current + slopeBias;

    // Receiver-plane depth gradient in shadow UV space
    vec3 dndc_dx = dFdx(ndc);
    vec3 dndc_dy = dFdy(ndc);
    vec2 dz_duv  = receiverPlaneDepthGradient(ndc, dndc_dx, dndc_dy);
    vec2 abs_dz_duv = abs(dz_duv) * SHADOW_RPDB_SCALE;

    ivec2 dim       = textureSize(shadowTex[ci], 0);
    vec2  texelSize = 1.0 / vec2(dim);

    float baseRadius = SHADOW_PCF_BASE_RADIUS;
    float radius     = mix(baseRadius, baseRadius + SHADOW_PCF_CASCADE_GAIN, float(ci) / 3.0);

    float ang = hash12(suv * 4096.0) * 6.2831853;
    vec2  r   = vec2(cos(ang), sin(ang));
    mat2  rot = mat2(r.x, -r.y, r.y, r.x);

    const int TAP_COUNT = 16;
    float visible = 0.0;
    float wsum    = 0.0;

    for (int i = 0; i < TAP_COUNT; ++i)
    {
        vec2  pu   = rot * POISSON_16[i];
        vec2  off  = pu * radius * texelSize;// uv-space offset of this tap

        float w    = POISSON_16_WEIGHT[i];

        float mapD = texture(shadowTex[ci], suv + off).r;

        // Receiver-plane depth bias: conservative depth delta over this tap's offset
        // Approximate |Δz| ≈ |dz/du|*|Δu| + |dz/dv|*|Δv|
        float rpdb = dot(abs_dz_duv, abs(off));

        float vis  = step(mapD, currentBias + rpdb);

        visible += vis * w;
        wsum    += w;
    }

    float visibility = (wsum > 0.0) ? (visible / wsum) : 1.0;
    return visibility;
}

float calcShadowVisibility(vec3 worldPos, vec3 N, vec3 L, bool forceClipmapShadows)
{
    // Early out: shadows are globally disabled.
    if (sceneData.rtParams.y <= 0.0)
    {
        return 1.0;
    }

    vec3 wp = worldPos + N * SHADOW_NORMAL_OFFSET * (0.5 + 0.5 * (1.0 - max(dot(N, L), 0.0)));
    float planetVis = planet_analytic_shadow_visibility(wp, L);
    if (planetVis <= 0.0)
    {
        return 0.0;
    }

    // RT-only mode: by default cast a ray and skip clipmap sampling entirely.
    // Some materials (e.g. planet terrain) opt into clipmap sampling to get stable
    // near-range shadows without requiring planet geometry in the TLAS.
    if (sceneData.rtOptions.z == 2u)
    {
        if (forceClipmapShadows)
        {
            CascadeMix cm = computeCascadeMix(wp);
            float v0 = sampleCascadeShadow(cm.i0, wp, N, L);
            if (cm.w1 <= 0.0)
            {
                return min(planetVis, v0);
            }
            float v1 = sampleCascadeShadow(cm.i1, wp, N, L);
            float vis = mix(v0, v1, clamp(cm.w1, 0.0, 1.0));
            return min(planetVis, vis);
        }

        #ifdef GL_EXT_ray_query
        float farR = max(max(sceneData.cascadeSplitsView.x, sceneData.cascadeSplitsView.y),
        max(sceneData.cascadeSplitsView.z, sceneData.cascadeSplitsView.w));
        float originBias = shadow_ray_origin_bias(wp);
        float tmin = shadow_ray_tmin(wp);
        rayQueryEXT rq;
        rayQueryInitializeEXT(rq, topLevelAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
        0xFF, wp + N * originBias, tmin, L, farR);
        while (rayQueryProceedEXT(rq)) { }
        bool hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
        return min(planetVis, hit ? 0.0 : 1.0);
        #else
        // Fallback to clipmap PCF if ray query is not available at compile time
        return min(planetVis, sampleCascadeShadow(0u, wp, N, L));
        #endif
    }

    CascadeMix cm = computeCascadeMix(wp);
    float v0 = sampleCascadeShadow(cm.i0, wp, N, L);
    if (cm.w1 <= 0.0)
    {
        // Hybrid ray query assist (terminate-on-first-hit along -L)
        #ifdef GL_EXT_ray_query
        if (sceneData.rtOptions.x == 1u)
        {
            float NoL = max(dot(N, L), 0.0);
            uint mask = sceneData.rtOptions.y;
            bool cascadeEnabled = ((mask >> cm.i0) & 1u) == 1u;
            if (cascadeEnabled && NoL < sceneData.rtParams.x)
            {
                float maxT = sceneData.cascadeSplitsView[cm.i0];
                float originBias = shadow_ray_origin_bias(wp);
                float tmin = shadow_ray_tmin(wp);
                rayQueryEXT rq;
                // tmin: small offset to avoid self-hits
                rayQueryInitializeEXT(rq, topLevelAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                0xFF, wp + N * originBias, tmin, L, maxT);
                bool hit = false;
                while (rayQueryProceedEXT(rq)) { }
                hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
                if (hit) v0 = min(v0, 0.0);
            }
        }
        #endif
        return min(planetVis, v0);
    }

    float v1 = sampleCascadeShadow(cm.i1, wp, N, L);
    float vis = mix(v0, v1, clamp(cm.w1, 0.0, 1.0));
    // Hybrid assist across blended border: take min if a ray hits in either cascade
    #ifdef GL_EXT_ray_query
    if (sceneData.rtOptions.x == 1u)
    {
        float NoL = max(dot(N, L), 0.0);
        uint mask = sceneData.rtOptions.y;
        bool e0 = ((mask >> cm.i0) & 1u) == 1u;
        bool e1 = ((mask >> cm.i1) & 1u) == 1u;
        if (NoL < sceneData.rtParams.x && (e0 || e1))
        {
            float maxT0 = sceneData.cascadeSplitsView[cm.i0];
            float maxT1 = sceneData.cascadeSplitsView[cm.i1];
            float maxT = max(maxT0, maxT1);
            float originBias = shadow_ray_origin_bias(wp);
            float tmin = shadow_ray_tmin(wp);
            rayQueryEXT rq;
            rayQueryInitializeEXT(rq, topLevelAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
            0xFF, wp + N * originBias, tmin, L, maxT);
            while (rayQueryProceedEXT(rq)) { }
            bool hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
            if (hit) vis = min(vis, 0.0);
        }
    }
    #endif
    return min(planetVis, vis);
}

void main(){
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w == 0.0)
    {
        outColor = vec4(0.0);
        return;
    }

    vec3 pos = posSample.xyz;
    bool forceClipmapShadows = (posSample.w > 1.5) && (sceneData.rtParams.z > 0.5);
    vec4 normalSample = texture(normalTex, inUV);
    vec3 N = normalize(normalSample.xyz);
    float roughness = clamp(normalSample.w, 0.04, 1.0);

    vec4 albedoSample = texture(albedoTex, inUV);
    vec3 albedo = albedoSample.rgb;
    float metallic = clamp(albedoSample.a, 0.0, 1.0);

    vec4 extraSample = texture(extraTex, inUV);
    float ao = extraSample.x;
    vec3 emissive = extraSample.yzw;

    vec3 camPos = getCameraWorldPosition();
    vec3 V = normalize(camPos - pos);

    // Directional sun term using evaluate_brdf + cascaded shadowing
    vec3 Lsun = normalize(-sceneData.sunlightDirection.xyz);

    // Planet night emission: only show emission on the dark side of planet surfaces
    bool isPlanet = (posSample.w > 1.5);
    if (isPlanet)
    {
        float NdotL = max(dot(N, Lsun), 0.0);
        float nightFactor = 1.0 - smoothstep(0.0, 0.15, NdotL);
        emissive *= nightFactor;
    }
    float sunVis = calcShadowVisibility(pos, N, Lsun, forceClipmapShadows);
    vec3 sunBRDF = evaluate_brdf(N, V, Lsun, albedo, roughness, metallic);
    vec3 direct = sunBRDF * sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * sunVis;


    // ---------------------------- //
    // point light
    uint pointCount = sceneData.lightCounts.x;
    for (uint i = 0u; i < pointCount; ++i)
    {
        vec3 contrib = eval_point_light(sceneData.punctualLights[i], pos, N, V, albedo, roughness, metallic);

        // Optional RT shadow for the first few point lights (hybrid mode)
        #ifdef GL_EXT_ray_query
        if (sceneData.rtOptions.x == 1u && sceneData.rtParams.y > 0.0 && i < 4u)
        {
            vec3 toL = sceneData.punctualLights[i].position_radius.xyz - pos;
            float maxT = length(toL);
            if (maxT > 0.01)
            {
                vec3 dir = toL / maxT;
                float originBias = shadow_ray_origin_bias(pos);
                float tmin = shadow_ray_tmin(pos);
                vec3 origin = pos + N * originBias;

                rayQueryEXT rq;
                rayQueryInitializeEXT(
                    rq,
                    topLevelAS,
                    gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                    0xFF,
                    origin,
                    tmin,
                    dir,
                    maxT
                );
                while (rayQueryProceedEXT(rq)) { }
                bool hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
                if (hit)
                {
                    contrib = vec3(0.0);
                }
            }
        }
        #endif

        direct += contrib;
    }

    // ------------------------------ //
    // spot light
    uint spotCount = sceneData.lightCounts.y;
    for (uint i = 0u; i < spotCount; ++i)
    {
        vec3 contrib = eval_spot_light(sceneData.spotLights[i], pos, N, V, albedo, roughness, metallic);

        // Optional RT shadow for the first few spot lights (hybrid mode)
        #ifdef GL_EXT_ray_query
        if (sceneData.rtOptions.x == 1u && sceneData.rtParams.y > 0.0 && i < 4u)
        {
            vec3 toL = sceneData.spotLights[i].position_radius.xyz - pos;
            float maxT = length(toL);
            if (maxT > 0.01)
            {
                vec3 L = toL / maxT;
                vec3 dir = sceneData.spotLights[i].direction_cos_outer.xyz;
                float cosTheta = dot(-L, dir);
                if (cosTheta > sceneData.spotLights[i].direction_cos_outer.w)
                {
                    float originBias = shadow_ray_origin_bias(pos);
                    float tmin = shadow_ray_tmin(pos);
                    vec3 origin = pos + N * originBias;

                    rayQueryEXT rq;
                    rayQueryInitializeEXT(
                        rq,
                        topLevelAS,
                        gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                        0xFF,
                        origin,
                        tmin,
                        L,
                        maxT
                    );
                    while (rayQueryProceedEXT(rq)) { }
                    bool hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
                    if (hit)
                    {
                        contrib = vec3(0.0);
                    }
                }
            }
        }
        #endif

        direct += contrib;
    }

    // Image-Based Lighting: split-sum approximation
    vec3 R = reflect(-V, N);
    float NdotV = max(dot(N, V), 0.0);
    float levels = float(textureQueryLevels(iblSpec2D));
    float lod = ibl_lod_from_roughness(roughness, levels);
    vec2 uv = dir_to_equirect_normalized(R);
    vec3 prefiltered = textureLod(iblSpec2D, uv, lod).rgb;
    vec2 brdf = texture(iblBRDF, vec2(NdotV, roughness)).rg;
    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 specIBL = prefiltered * (F0 * brdf.x + brdf.y);
    vec3 diffIBL = (1.0 - metallic) * albedo * sh_eval_irradiance(N);

    vec3 indirect = diffIBL + specIBL;
    vec3 color = direct + indirect * ao + emissive;

    outColor = vec4(color, 1.0);
}
