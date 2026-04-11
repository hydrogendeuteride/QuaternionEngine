#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;
layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D hdrInput;
layout(set = 1, binding = 1) uniform sampler2D posTex;
layout(set = 1, binding = 2) uniform sampler2D transmittanceLut;
layout(set = 1, binding = 3) uniform sampler2D cloudOverlayTex;
layout(set = 1, binding = 4) uniform sampler2D cloudNoiseTex;
layout(set = 1, binding = 5) uniform sampler2D cloudLightingResolvedTex;
layout(set = 1, binding = 6) uniform sampler2D cloudSegmentResolvedTex;
layout(set = 1, binding = 7) uniform sampler3D cloudNoiseTex3D;
layout(set = 1, binding = 8) uniform sampler3D jitterNoiseTex;
layout(set = 1, binding = 9) uniform sampler2D planetHeightTexPX;
layout(set = 1, binding = 10) uniform sampler2D planetHeightTexNX;
layout(set = 1, binding = 11) uniform sampler2D planetHeightTexPY;
layout(set = 1, binding = 12) uniform sampler2D planetHeightTexNY;
layout(set = 1, binding = 13) uniform sampler2D planetHeightTexPZ;
layout(set = 1, binding = 14) uniform sampler2D planetHeightTexNZ;

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

#include "atmosphere/include/common.glsl"

bool segment_valid(vec2 seg)
{
    return seg.x < seg.y;
}

int normalize_resolved_segments(vec4 resolvedSegments,
                                float tStart,
                                float tEnd,
                                out vec2 seg0,
                                out vec2 seg1)
{
    seg0 = vec2(clamp(resolvedSegments.x, tStart, tEnd), clamp(resolvedSegments.y, tStart, tEnd));
    seg1 = vec2(clamp(resolvedSegments.z, tStart, tEnd), clamp(resolvedSegments.w, tStart, tEnd));

    bool valid0 = segment_valid(seg0);
    bool valid1 = segment_valid(seg1);
    if (valid0 && valid1 && seg1.x < seg0.x)
    {
        vec2 tmp = seg0;
        seg0 = seg1;
        seg1 = tmp;
    }
    else if (!valid0 && valid1)
    {
        seg0 = seg1;
        seg1 = vec2(0.0);
        valid0 = true;
        valid1 = false;
    }

    return (valid0 ? 1 : 0) + (valid1 ? 1 : 0);
}

void main()
{
    vec3 baseColor = texture(hdrInput, inUV).rgb;

    float planetRadius = pc.planet_center_radius.w;
    if (planetRadius <= 0.0)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    uint miscPacked = uint(pc.misc.w);
    int flags = int(miscPacked & MISC_FLAGS_MASK);
    bool wantAtmosphere = (flags & FLAG_ATMOSPHERE) != 0;
    bool wantClouds = (flags & FLAG_CLOUDS) != 0;
    bool cloudFlipV = (flags & FLAG_CLOUD_FLIP_V) != 0;

    float atmRadius = pc.atmosphere_params.x;
    float Hr = max(pc.atmosphere_params.y, 1.0);
    float Hm = max(pc.atmosphere_params.z, 1.0);
    float mieG = clamp(pc.atmosphere_params.w, -0.99, 0.99);

    vec3 betaR = max(pc.beta_rayleigh.rgb, vec3(0.0));
    vec3 betaM = max(pc.beta_mie.rgb, vec3(0.0));
    float atmIntensity = max(pc.beta_rayleigh.w, 0.0);

    vec3 absorptionColor = clamp(unpackUnorm4x8(uint(pc.misc.y)).rgb, vec3(0.0), vec3(1.0));
    float absorptionStrength = max(pc.beta_mie.w, 0.0);
    vec3 betaA = absorptionColor * absorptionStrength;

    bool atmActive = wantAtmosphere && (atmRadius > planetRadius) && (atmIntensity > 0.0) &&
                     any(greaterThan(betaR + betaM + betaA, vec3(0.0)));

    float cloudBaseM = max(pc.cloud_layer.x, 0.0);
    float cloudThicknessM = max(pc.cloud_layer.y, 0.0);
    float cloudDensScale = max(pc.cloud_layer.z, 0.0);
    float rBase = planetRadius + cloudBaseM;
    float rTop = rBase + cloudThicknessM;
    bool cloudsActive = wantClouds && (cloudThicknessM > 0.0) && (cloudDensScale > 0.0) && (rTop > planetRadius);

    vec3 betaR_eff = atmActive ? betaR : vec3(0.0);
    vec3 betaM_eff = atmActive ? betaM : vec3(0.0);
    vec3 betaA_eff = atmActive ? betaA : vec3(0.0);
    float atmRadius_eff = atmActive ? atmRadius : 0.0;

    if (!atmActive && !cloudsActive)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    if (!cloudsActive)
    {
        outColor = vec4(render_atmosphere_monolithic(baseColor), 1.0);
        return;
    }

    vec3 camLocal = inCamLocal;
    vec3 rd = normalize(inWorldRay);
    vec3 rdDx = dFdx(rd);
    vec3 rdDy = dFdy(rd);
    vec3 center = pc.planet_center_radius.xyz;

    float boundRadius = atmActive ? atmRadius_eff : rTop;
    if (boundRadius <= planetRadius)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    float tStart;
    float tEnd;
    if (!compute_primary_ray_bounds_fast(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    vec4 resolvedLighting = texture(cloudLightingResolvedTex, inUV);
    vec4 resolvedSegment = texture(cloudSegmentResolvedTex, inUV);

    float jitter = resolve_jitter_sample(inUV);
    vec3 sunDir = -sceneData.sunlightDirection.xyz;
    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;
    float cosTheta = dot(rd, sunDir);

    MarchParams mp;
    mp.camLocal = camLocal;
    mp.rd = rd;
    mp.center = center;
    mp.planetRadius = planetRadius;
    mp.atmRadius = atmRadius_eff;
    mp.Hr = Hr;
    mp.Hm = Hm;
    mp.betaR = betaR_eff;
    mp.betaM = betaM_eff;
    mp.betaA = betaA_eff;
    mp.sunDir = sunDir;
    mp.phaseR = phase_rayleigh(cosTheta);
    mp.phaseM = phase_mie_hg(cosTheta, mieG);
    mp.phaseC = phase_mie_hg(cosTheta, CLOUD_PHASE_G);
    mp.jitter = jitter;
    mp.atmActive = atmActive;
    mp.cloudsActive = cloudsActive;
    mp.cloudFlipV = cloudFlipV;
    mp.cloudWindHeading = vec2(1.0, 0.0);
    mp.cloudWindSC = vec2(0.0, 1.0);
    mp.cloudRTop = rTop;
    mp.cloudOverlayRotSC = pc.jitter_params.zw;
    mp.rayDx = rdDx;
    mp.rayDy = rdDy;

    float cloudR = planetRadius + cloudBaseM + 0.5 * cloudThicknessM;
    cloudR = max(cloudR, planetRadius);
    float windAngle = pc.cloud_params.w;
    mp.cloudWindHeading = vec2(cos(windAngle), sin(windAngle));
    float arc = (pc.cloud_params.z * max(sceneData.timeParams.x, 0.0)) / max(cloudR, 1.0);
    mp.cloudWindSC = vec2(sin(arc), cos(arc));

    vec2 cloudSeg0;
    vec2 cloudSeg1;
    int cloudSegCount = normalize_resolved_segments(resolvedSegment, tStart, tEnd, cloudSeg0, cloudSeg1);

    vec3 betaRA_eff = betaR_eff + betaA_eff;
    MarchState state = march_state_init();

    if (cloudSegCount == 0)
    {
        if (atmActive)
        {
            integrate_segment(tStart, tEnd, clamp(pc.misc.x, 4, 64), false, mp, state);
        }

        vec3 transmittance = exp(-(betaRA_eff * state.odR + betaM_eff * state.odM));
        vec3 outRgb = baseColor * transmittance;
        if (atmActive)
        {
            outRgb += state.scatterAtm * (sunCol * atmIntensity);
        }

        outColor = vec4(outRgb, 1.0);
        return;
    }

    float gapStart[3];
    float gapEnd[3];
    int gapCount;
    if (cloudSegCount == 1)
    {
        gapStart[0] = tStart;       gapEnd[0] = cloudSeg0.x;
        gapStart[1] = cloudSeg0.y;  gapEnd[1] = tEnd;
        gapCount = 2;
    }
    else
    {
        gapStart[0] = tStart;       gapEnd[0] = cloudSeg0.x;
        gapStart[1] = cloudSeg0.y;  gapEnd[1] = cloudSeg1.x;
        gapStart[2] = cloudSeg1.y;  gapEnd[2] = tEnd;
        gapCount = 3;
    }

    vec2 gapStepSegs[MAX_SEGS];
    for (int i = 0; i < MAX_SEGS; ++i) gapStepSegs[i] = vec2(0.0);
    for (int i = 0; i < gapCount; ++i)
    {
        gapStepSegs[i].x = max(gapEnd[i] - gapStart[i], 0.0);
    }
    distribute_steps(gapStepSegs, gapCount, atmActive ? clamp(pc.misc.x, 4, 64) : 0);

    for (int i = 0; i < gapCount; ++i)
    {
        if (gapStepSegs[i].x > 0.0 && gapStepSegs[i].y > 0.0)
        {
            integrate_segment(gapStart[i], gapEnd[i], int(gapStepSegs[i].y), false, mp, state);
        }
    }

    state.odC = max(-log(max(resolvedLighting.a, 1e-4)) / CLOUD_BETA, 0.0);

    vec3 transmittance = exp(-(betaRA_eff * state.odR + betaM_eff * state.odM + vec3(CLOUD_BETA * state.odC)));
    vec3 outRgb = baseColor * transmittance + resolvedLighting.rgb;
    if (atmActive)
    {
        outRgb += state.scatterAtm * (sunCol * atmIntensity);
    }

    outColor = vec4(outRgb, 1.0);
}
