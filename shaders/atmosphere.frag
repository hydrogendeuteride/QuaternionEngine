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

#include "atmosphere_common.glsl"

bool segment_valid(vec2 seg)
{
    return seg.x < seg.y;
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

    vec4 resolvedLighting = texture(cloudLightingResolvedTex, inUV);
    vec2 resolvedSegment = texture(cloudSegmentResolvedTex, inUV).rg;

    if (!cloudsActive || !segment_valid(resolvedSegment))
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
    if (!compute_primary_ray_bounds(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    vec2 analyticSeg0;
    vec2 analyticSeg1;
    int cloudSegCount = compute_cloud_segments(camLocal, rd, center, rBase, rTop, tStart, tEnd, analyticSeg0, analyticSeg1);
    if (cloudSegCount != 1 || !segment_valid(analyticSeg0))
    {
        outColor = vec4(render_atmosphere_monolithic(baseColor), 1.0);
        return;
    }

    float analyticLen = max(analyticSeg0.y - analyticSeg0.x, 1e-3);
    float resolvedError = abs(resolvedSegment.x - analyticSeg0.x) + abs(resolvedSegment.y - analyticSeg0.y);
    float resolvedSoftReject = max(900.0, analyticLen * 0.14);
    float resolvedHardReject = max(2500.0, analyticLen * 0.35);
    if (resolvedError > resolvedHardReject)
    {
        outColor = vec4(render_atmosphere_monolithic(baseColor), 1.0);
        return;
    }

    float jitter = resolve_jitter_sample(inUV);
    vec3 sunDir = normalize(-sceneData.sunlightDirection.xyz);
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

    MarchState state = march_state_init();

    vec2 gapStepSegs[MAX_SEGS];
    for (int i = 0; i < MAX_SEGS; ++i) gapStepSegs[i] = vec2(0.0);
    gapStepSegs[0].x = max(analyticSeg0.x - tStart, 0.0);
    gapStepSegs[1].x = max(tEnd - analyticSeg0.y, 0.0);
    distribute_steps(gapStepSegs, 2, atmActive ? clamp(pc.misc.x, 4, 64) : 0);

    if (gapStepSegs[0].x > 0.0 && gapStepSegs[0].y > 0.0)
    {
        integrate_segment(tStart, analyticSeg0.x, int(gapStepSegs[0].y), false, mp, state);
    }

    state.odC = max(-log(max(resolvedLighting.a, 1e-4)) / CLOUD_BETA, 0.0);

    if (gapStepSegs[1].x > 0.0 && gapStepSegs[1].y > 0.0)
    {
        integrate_segment(analyticSeg0.y, tEnd, int(gapStepSegs[1].y), false, mp, state);
    }

    vec3 transmittance = exp(-(betaR_eff * state.odR + betaM_eff * state.odM + betaA_eff * state.odR + vec3(CLOUD_BETA * state.odC)));
    vec3 outRgb = baseColor * transmittance + resolvedLighting.rgb;
    if (atmActive)
    {
        outRgb += state.scatterAtm * (sunCol * atmIntensity);
    }

    float resolvedTrust = 1.0 - smoothstep(resolvedSoftReject, resolvedHardReject, resolvedError);
    if (resolvedTrust < 0.999)
    {
        vec3 fallbackRgb = render_atmosphere_monolithic(baseColor);
        outRgb = mix(fallbackRgb, outRgb, resolvedTrust);
    }

    outColor = vec4(outRgb, 1.0);
}
