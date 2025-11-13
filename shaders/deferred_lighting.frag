#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_ray_query : require
#include "input_structures.glsl"

layout(location=0) in vec2 inUV;
layout(location=0) out vec4 outColor;

layout(set=1, binding=0) uniform sampler2D posTex;
layout(set=1, binding=1) uniform sampler2D normalTex;
layout(set=1, binding=2) uniform sampler2D albedoTex;
layout(set=2, binding=0) uniform sampler2D shadowTex[4];
// IBL (set=3): specular prefiltered cube, diffuse irradiance cube, BRDF LUT
layout(set=3, binding=0) uniform sampler2D iblSpec2D;
layout(set=3, binding=1) uniform sampler2D iblBRDF;
layout(std140, set=3, binding=2) uniform IBL_SH { vec4 sh[9]; } iblSH;

vec3 sh_eval_irradiance(vec3 n)
{
    float x=n.x, y=n.y, z=n.z;
    const float c0=0.2820947918;
    const float c1=0.4886025119;
    const float c2=1.0925484306;
    const float c3=0.3153915653;
    const float c4=0.5462742153;
    float Y[9];
    Y[0]=c0; Y[1]=c1*y; Y[2]=c1*z; Y[3]=c1*x; Y[4]=c2*x*y; Y[5]=c2*y*z; Y[6]=c3*(3.0*z*z-1.0); Y[7]=c2*x*z; Y[8]=c4*(x*x-y*y);
    vec3 r=vec3(0.0);
    for (int i=0;i<9;++i) r += iblSH.sh[i].rgb * Y[i];
    return r;
}

vec2 dir_to_equirect(vec3 d)
{
    d = normalize(d);
    float phi = atan(d.z, d.x);
    float theta = acos(clamp(d.y, -1.0, 1.0));
    return vec2(phi * (0.15915494309) + 0.5, theta * (0.31830988618));
}
// TLAS for ray query (optional, guarded by sceneData.rtOptions.x)
#ifdef GL_EXT_ray_query
layout(set=0, binding=1) uniform accelerationStructureEXT topLevelAS;
#endif

// Tunables for shadow quality and blending
// Border smoothing width in light-space NDC (0..1). Larger = wider cross-fade.
const float SHADOW_BORDER_SMOOTH_NDC = 0.08;
// Base PCF radius in texels for cascade 0; higher cascades scale this up slightly.
const float SHADOW_PCF_BASE_RADIUS = 1.35;
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

const float PI = 3.14159265359;

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

    // Receiver-plane depth gradient in shadow UV space
    vec3 dndc_dx = dFdx(ndc);
    vec3 dndc_dy = dFdy(ndc);
    vec2 dz_duv  = receiverPlaneDepthGradient(ndc, dndc_dx, dndc_dy);

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

        float pr   = length(pu);
        float w    = 1.0 - smoothstep(0.0, 0.65, pr);

        float mapD = texture(shadowTex[ci], suv + off).r;

        // Receiver-plane depth bias: conservative depth delta over this tap's offset
        // Approximate |Δz| ≈ |dz/du|*|Δu| + |dz/dv|*|Δv|
        float rpdb = dot(abs(dz_duv), abs(off)) * SHADOW_RPDB_SCALE;

        float vis  = step(mapD, current + slopeBias + rpdb);

        visible += vis * w;
        wsum    += w;
    }

    float visibility = (wsum > 0.0) ? (visible / wsum) : 1.0;
    return visibility;
}

float calcShadowVisibility(vec3 worldPos, vec3 N, vec3 L)
{
    vec3 wp = worldPos + N * SHADOW_NORMAL_OFFSET * (0.5 + 0.5 * (1.0 - max(dot(N, L), 0.0)));

    // RT-only mode: cast a ray and skip clipmap sampling entirely
    if (sceneData.rtOptions.z == 2u) {
        #ifdef GL_EXT_ray_query
        float farR = max(max(sceneData.cascadeSplitsView.x, sceneData.cascadeSplitsView.y),
        max(sceneData.cascadeSplitsView.z, sceneData.cascadeSplitsView.w));
        rayQueryEXT rq;
        rayQueryInitializeEXT(rq, topLevelAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
        0xFF, wp + N * SHADOW_RAY_ORIGIN_BIAS, SHADOW_RAY_TMIN, L, farR);
        while (rayQueryProceedEXT(rq)) { }
        bool hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
        return hit ? 0.0 : 1.0;
        #else
        // Fallback to clipmap PCF if ray query is not available at compile time
        ;
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
                rayQueryEXT rq;
                // tmin: small offset to avoid self-hits
                rayQueryInitializeEXT(rq, topLevelAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                0xFF, wp + N * SHADOW_RAY_ORIGIN_BIAS, SHADOW_RAY_TMIN, L, maxT);
                bool hit = false;
                while (rayQueryProceedEXT(rq)) { }
                hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
                if (hit) v0 = min(v0, 0.0);
            }
        }
        #endif
        return v0;
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
            rayQueryEXT rq;
            rayQueryInitializeEXT(rq, topLevelAS, gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
            0xFF, wp + N * SHADOW_RAY_ORIGIN_BIAS, SHADOW_RAY_TMIN, L, maxT);
            while (rayQueryProceedEXT(rq)) { }
            bool hit = (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT);
            if (hit) vis = min(vis, 0.0);
        }
    }
    #endif
    return vis;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a      = roughness * roughness;
    float a2     = a * a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / max(denom, 0.001);
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;

    float denom = NdotV * (1.0 - k) + k;
    return NdotV / max(denom, 0.001);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float ggx2 = GeometrySchlickGGX(max(dot(N, V), 0.0), roughness);
    float ggx1 = GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
    return ggx1 * ggx2;
}

void main(){
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w == 0.0)
    {
        outColor = vec4(0.0);
        return;
    }

    vec3 pos = posSample.xyz;
    vec4 normalSample = texture(normalTex, inUV);
    vec3 N = normalize(normalSample.xyz);
    float roughness = clamp(normalSample.w, 0.04, 1.0);

    vec4 albedoSample = texture(albedoTex, inUV);
    vec3 albedo = albedoSample.rgb;
    float metallic = clamp(albedoSample.a, 0.0, 1.0);

    vec3 camPos = vec3(inverse(sceneData.view)[3]);
    vec3 V = normalize(camPos - pos);
    vec3 L = normalize(-sceneData.sunlightDirection.xyz);
    vec3 H = normalize(V + L);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 F  = fresnelSchlick(max(dot(H, V), 0.0), F0);
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);

    vec3 numerator    = NDF * G * F;
    float denom       = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
    vec3 specular     = numerator / max(denom, 0.001);

    vec3 kS = F;
    vec3 kD = (1.0 - kS) * (1.0 - metallic);

    float NdotL = max(dot(N, L), 0.0);
    // Shadowing (directional, forward-Z shadow map)
    float visibility = calcShadowVisibility(pos, N, L);

    vec3 irradiance = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * NdotL * visibility;

    vec3 color = (kD * albedo / PI + specular) * irradiance;

    // Image-Based Lighting: split-sum approximation
    vec3 R = reflect(-V, N);
    float levels = float(textureQueryLevels(iblSpec2D));
    float lod = clamp(roughness * max(levels - 1.0, 0.0), 0.0, max(levels - 1.0, 0.0));
    vec2 uv = dir_to_equirect(R);
    vec3 prefiltered = textureLod(iblSpec2D, uv, lod).rgb;
    vec2 brdf = texture(iblBRDF, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 specIBL = prefiltered * (F0 * brdf.x + brdf.y);
    vec3 diffIBL = (1.0 - metallic) * albedo * sh_eval_irradiance(N);
    color += diffIBL + specIBL;

    outColor = vec4(color, 1.0);
}
