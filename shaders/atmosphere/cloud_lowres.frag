#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;

layout(location = 0) out vec4 outCloudLighting;
layout(location = 1) out vec4 outCloudSegment;

layout(set = 1, binding = 0) uniform sampler2D posTex;
layout(set = 1, binding = 1) uniform sampler2D transmittanceLut;
layout(set = 1, binding = 2) uniform sampler2D cloudOverlayTex;
layout(set = 1, binding = 3) uniform sampler2D cloudNoiseTex;
layout(set = 1, binding = 4) uniform sampler3D cloudNoiseTex3D;
layout(set = 1, binding = 5) uniform sampler3D jitterNoiseTex;
layout(set = 1, binding = 6) uniform sampler2D planetHeightTexPX;
layout(set = 1, binding = 7) uniform sampler2D planetHeightTexNX;
layout(set = 1, binding = 8) uniform sampler2D planetHeightTexPY;
layout(set = 1, binding = 9) uniform sampler2D planetHeightTexNY;
layout(set = 1, binding = 10) uniform sampler2D planetHeightTexPZ;
layout(set = 1, binding = 11) uniform sampler2D planetHeightTexNZ;

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

const float CLOUD_LOWRES_PREGAP_STEP_SCALE = 0.35;
const int CLOUD_LOWRES_PREGAP_MAX_STEPS = 8;
const float CLOUD_LOWRES_CLOUD_STEP_SCALE = 0.50;
const int CLOUD_LOWRES_CLOUD_STEP_MAX = 16;
const int CLOUD_LOWRES_CLOUD_STEP_MIN = 3;

bool segment_valid(vec2 seg)
{
    return seg.x < seg.y;
}

void distribute_steps_scalar_2(float len0,
                               float len1,
                               int count,
                               int totalSteps,
                               out int steps0,
                               out int steps1)
{
    steps0 = 0;
    steps1 = 0;

    float totalLen = 0.0;
    int activeCount = 0;
    if (count >= 1 && len0 > 0.0)
    {
        totalLen += len0;
        activeCount++;
    }
    if (count >= 2 && len1 > 0.0)
    {
        totalLen += len1;
        activeCount++;
    }

    if (totalSteps <= 0 || activeCount == 0)
    {
        return;
    }

    totalSteps = max(totalSteps, activeCount);
    if (count >= 1 && len0 > 0.0)
    {
        if (activeCount == 1)
        {
            steps0 = max(1, totalSteps);
            return;
        }

        steps0 = int(round(float(totalSteps) * (len0 / totalLen)));
        steps0 = clamp(steps0, 1, totalSteps - (activeCount - 1));
        totalSteps -= steps0;
        activeCount--;
    }

    if (count >= 2 && len1 > 0.0)
    {
        steps1 = max(1, totalSteps);
    }
}

void main()
{
    outCloudLighting = vec4(0.0, 0.0, 0.0, 1.0);
    outCloudSegment = vec4(0.0);

    float planetRadius = pc.planet_center_radius.w;
    if (planetRadius <= 0.0) return;

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
    if (!cloudsActive) return;

    vec3 camLocal = inCamLocal;
    vec3 rd = normalize(inWorldRay);
    vec3 center = pc.planet_center_radius.xyz;

    float boundRadius = atmActive ? atmRadius : rTop;
    if (boundRadius <= planetRadius) return;

    float tStart;
    float tEnd;
    if (!compute_primary_ray_bounds_cloud_hybrid(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        return;
    }

    vec2 cloudSeg0;
    vec2 cloudSeg1;
    int cloudSegCount = compute_cloud_segments(camLocal, rd, center, rBase, rTop, tStart, tEnd, cloudSeg0, cloudSeg1);
    if (cloudSegCount == 0 || !segment_valid(cloudSeg0))
    {
        return;
    }

    vec3 rdDx = dFdx(rd);
    vec3 rdDy = dFdy(rd);
    float jitter = resolve_jitter_sample(inUV);

    vec3 sunDir = normalize(-sceneData.sunlightDirection.xyz);
    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;
    vec3 ambCol = sceneData.ambientColor.rgb;
    vec3 cloudTint = max(pc.cloud_color.rgb, vec3(0.0));

    float sunLuma = luminance(sunCol);
    vec3 cloudSunCol = mix(sunCol, vec3(max(sunLuma, 1e-3)), CLOUD_SUN_WHITEN) * cloudTint;
    float ambLuma = luminance(ambCol);
    vec3 cloudAmbNeutral = vec3(max(ambLuma, CLOUD_AMBIENT_MIN_LUMA));
    vec3 cloudAmbCol = mix(ambCol, cloudAmbNeutral, CLOUD_AMBIENT_WHITEN) * cloudTint;

    float cosTheta = dot(rd, sunDir);

    MarchParams mp;
    mp.camLocal = camLocal;
    mp.rd = rd;
    mp.center = center;
    mp.planetRadius = planetRadius;
    mp.atmRadius = atmActive ? atmRadius : 0.0;
    mp.Hr = Hr;
    mp.Hm = Hm;
    mp.betaR = atmActive ? betaR : vec3(0.0);
    mp.betaM = atmActive ? betaM : vec3(0.0);
    mp.betaA = atmActive ? betaA : vec3(0.0);
    mp.sunDir = sunDir;
    mp.phaseR = phase_rayleigh(cosTheta);
    mp.phaseM = phase_mie_hg(cosTheta, mieG);
    mp.phaseC = phase_mie_hg(cosTheta, CLOUD_PHASE_G);
    mp.jitter = jitter;
    mp.atmActive = atmActive;
    mp.cloudsActive = true;
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
    bool hasSecondCloud = (cloudSegCount == 2 && segment_valid(cloudSeg1));

    float cloud0Len = max(cloudSeg0.y - cloudSeg0.x, 0.0);
    float cloud1Len = hasSecondCloud ? max(cloudSeg1.y - cloudSeg1.x, 0.0) : 0.0;

    float preGap0Start = tStart;
    float preGap0End = cloudSeg0.x;
    float preGap0Len = max(preGap0End - preGap0Start, 0.0);

    float preGap1Start = cloudSeg0.y;
    float preGap1End = hasSecondCloud ? cloudSeg1.x : cloudSeg0.y;
    float preGap1Len = hasSecondCloud ? max(preGap1End - preGap1Start, 0.0) : 0.0;

    int preGapCount = hasSecondCloud ? 2 : 1;
    float cloudLenTotal = cloud0Len + cloud1Len;
    float preGapLenTotal = preGap0Len + preGap1Len;
    float consideredLen = max(cloudLenTotal + preGapLenTotal, 1e-3);
    int preGapStepsTotal = 0;
    if (atmActive && preGapLenTotal > 0.0)
    {
        int preGapBudget = clamp(int(round(float(pc.misc.x) * (preGapLenTotal / consideredLen))), 0, pc.misc.x);
        preGapStepsTotal = clamp(int(round(float(preGapBudget) * CLOUD_LOWRES_PREGAP_STEP_SCALE)),
                                 1,
                                 min(pc.misc.x, CLOUD_LOWRES_PREGAP_MAX_STEPS));
    }
    int cloudStepBudget = compute_adaptive_cloud_steps(cloudLenTotal, cloudThicknessM, clamp(pc.misc.z, 4, 128));
    int cloudStepsTotal = 0;
    if (cloudStepBudget > 0)
    {
        cloudStepsTotal = clamp(int(round(float(cloudStepBudget) * CLOUD_LOWRES_CLOUD_STEP_SCALE)),
                                min(CLOUD_LOWRES_CLOUD_STEP_MIN, cloudStepBudget),
                                min(clamp(pc.misc.z, 4, 128), CLOUD_LOWRES_CLOUD_STEP_MAX));
    }

    int preGap0Steps;
    int preGap1Steps;
    distribute_steps_scalar_2(preGap0Len, preGap1Len, preGapCount, preGapStepsTotal, preGap0Steps, preGap1Steps);

    int cloud0Steps;
    int cloud1Steps;
    distribute_steps_scalar_2(cloud0Len, cloud1Len, cloudSegCount, cloudStepsTotal, cloud0Steps, cloud1Steps);

    if (preGap0Len > 0.0 && preGap0Steps > 0)
    {
        integrate_optical_depth_segment(preGap0Start, preGap0End, preGap0Steps, mp, state);
    }

    if (cloud0Len > 0.0 && cloud0Steps > 0)
    {
        integrate_segment(cloudSeg0.x, cloudSeg0.y, cloud0Steps, true, mp, state);
    }

    if (hasSecondCloud)
    {
        if (preGap1Len > 0.0 && preGap1Steps > 0)
        {
            integrate_optical_depth_segment(preGap1Start, preGap1End, preGap1Steps, mp, state);
        }

        if (cloud1Len > 0.0 && cloud1Steps > 0)
        {
            integrate_segment(cloudSeg1.x, cloudSeg1.y, cloud1Steps, true, mp, state);
        }
    }

    float cloudTransmittance = exp(-CLOUD_BETA * state.odC);
    vec3 cloudIntervalAtmRadiance = state.scatterAtm * (sunCol * atmIntensity);
    vec3 cloudRadiance = state.scatterCloudSun * cloudSunCol +
                         state.scatterCloudAmb * (cloudAmbCol * CLOUD_AMBIENT_SCALE) +
                         cloudIntervalAtmRadiance;

    outCloudLighting = vec4(cloudRadiance, cloudTransmittance);
    outCloudSegment = vec4(cloudSeg0, (cloudSegCount == 2 && segment_valid(cloudSeg1)) ? cloudSeg1 : vec2(0.0));
}
