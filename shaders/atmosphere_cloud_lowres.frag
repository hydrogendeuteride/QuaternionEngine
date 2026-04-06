#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 1) in vec3 inWorldRay;
layout(location = 2) flat in vec3 inCamLocal;

layout(location = 0) out vec4 outCloudLighting;
layout(location = 1) out vec2 outCloudSegment;

layout(set = 1, binding = 0) uniform sampler2D posTex;
layout(set = 1, binding = 1) uniform sampler2D transmittanceLut;
layout(set = 1, binding = 2) uniform sampler2D cloudOverlayTex;
layout(set = 1, binding = 3) uniform sampler2D cloudNoiseTex;
layout(set = 1, binding = 4) uniform sampler3D cloudNoiseTex3D;
layout(set = 1, binding = 5) uniform sampler3D jitterNoiseTex;

layout(push_constant) uniform AtmospherePush
{
    vec4 planet_center_radius;
    vec4 atmosphere_params;
    vec4 beta_rayleigh;
    vec4 beta_mie;
    vec4 jitter_params;
    vec4 cloud_layer;
    vec4 cloud_params;
    vec4 cloud_color;
    ivec4 misc;
} pc;

#include "atmosphere_common.glsl"

void main()
{
    outCloudLighting = vec4(0.0, 0.0, 0.0, 1.0);
    outCloudSegment = vec2(0.0);

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
    if (!compute_primary_ray_bounds(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        return;
    }

    vec2 cloudSeg0;
    vec2 cloudSeg1;
    int cloudSegCount = compute_cloud_segments(camLocal, rd, center, rBase, rTop, tStart, tEnd, cloudSeg0, cloudSeg1);
    if (cloudSegCount != 1 || cloudSeg0.y <= cloudSeg0.x)
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

    float preLen = max(cloudSeg0.x - tStart, 0.0);
    float cloudLen = max(cloudSeg0.y - cloudSeg0.x, 0.0);
    float totalLen = max(preLen + cloudLen, 1e-3);
    int preSteps = atmActive ? clamp(int(round(float(pc.misc.x) * (preLen / totalLen))), 0, pc.misc.x) : 0;
    int cloudSteps = clamp(pc.misc.z, 4, 128);

    if (preSteps > 0 && preLen > 0.0)
    {
        integrate_segment(tStart, cloudSeg0.x, preSteps, false, mp, state);
    }

    vec3 preCloudScatterAtm = state.scatterAtm;
    integrate_segment(cloudSeg0.x, cloudSeg0.y, cloudSteps, true, mp, state);

    float cloudTransmittance = exp(-CLOUD_BETA * state.odC);
    vec3 cloudIntervalAtmRadiance = (state.scatterAtm - preCloudScatterAtm) * (sunCol * atmIntensity);
    vec3 cloudRadiance = state.scatterCloudSun * cloudSunCol +
                         state.scatterCloudAmb * (cloudAmbCol * CLOUD_AMBIENT_SCALE) +
                         cloudIntervalAtmRadiance;

    outCloudLighting = vec4(cloudRadiance, cloudTransmittance);
    outCloudSegment = cloudSeg0;
}
