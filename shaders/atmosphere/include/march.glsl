const float RAYLEIGH_PHASE_SCALE = 3.0 / (16.0 * PI);
const float MIE_PHASE_SCALE = 1.0 / (4.0 * PI);

float phase_rayleigh(float cosTheta)
{
    return RAYLEIGH_PHASE_SCALE * (1.0 + cosTheta * cosTheta);
}

float phase_mie_hg(float cosTheta, float g)
{
    float g2 = g * g;
    float d = max(1.0 + g2 - 2.0 * g * cosTheta, 1e-4);
    return MIE_PHASE_SCALE * (1.0 - g2) / (d * sqrt(d));
}
vec2 sun_optical_depth(float r, float muS, float planetRadius, float atmRadius)
{
    float u = clamp(muS * 0.5 + 0.5, 0.0, 1.0);
    float v = clamp((r - planetRadius) / max(atmRadius - planetRadius, 1e-4), 0.0, 1.0);
    return textureLod(transmittanceLut, vec2(u, v), 0.0).rg;
}

struct MarchParams
{
    vec3 camLocal;
    vec3 rd;
    vec3 center;
    float planetRadius;
    float atmRadius;
    float Hr;
    float Hm;
    vec3 betaR;
    vec3 betaM;
    vec3 betaA;
    vec3 sunDir;
    float phaseR;
    float phaseM;
    float phaseC;
    float jitter;
    bool atmActive;
    bool cloudsActive;
    bool cloudFlipV;
    vec2 cloudWindHeading;
    vec2 cloudWindSC;
    float cloudRTop;
    vec2 cloudOverlayRotSC;
    vec3 rayDx;
    vec3 rayDy;
};

struct MarchState
{
    float odR;
    float odM;
    float odC;
    vec3 scatterAtm;
    vec3 scatterCloudSun;
    vec3 scatterCloudAmb;
};

MarchState march_state_init()
{
    return MarchState(0.0, 0.0, 0.0, vec3(0.0), vec3(0.0), vec3(0.0));
}

int compute_adaptive_cloud_steps(float segmentLen, float layerThickness, int maxSteps)
{
    if (maxSteps <= 0 || segmentLen <= 0.0) return 0;

    int minSteps = min(4, maxSteps);
    float thickness = max(layerThickness, 1.0);
    float ratio = clamp(segmentLen / thickness, 0.0, 1.0);
    float minFrac = float(minSteps) / float(maxSteps);
    float stepScale = mix(minFrac, 1.0, ratio);
    return clamp(int(round(float(maxSteps) * stepScale)), minSteps, maxSteps);
}

void integrate_optical_depth_segment(float t0, float t1, int steps, MarchParams mp, inout MarchState s)
{
    if (!mp.atmActive || steps <= 0 || t1 <= t0) return;

    float dt = (t1 - t0) / float(steps);
    float invHr = 1.0 / max(mp.Hr, 1.0);
    float invHm = 1.0 / max(mp.Hm, 1.0);
    vec3 dp = mp.rd * dt;
    vec3 p = mp.camLocal + mp.rd * (t0 + mp.jitter * dt);

    for (int i = 0; i < steps; ++i)
    {
        float height = max(length(p - mp.center) - mp.planetRadius, 0.0);
        s.odR += exp(-height * invHr) * dt;
        s.odM += exp(-height * invHm) * dt;
        p += dp;
    }
}

void integrate_segment(float t0, float t1, int steps, bool doCloud, MarchParams mp, inout MarchState s)
{
    if (steps <= 0 || t1 <= t0) return;

    float dt = (t1 - t0) / float(steps);
    float planetRadius2 = mp.planetRadius * mp.planetRadius;
    float invHr = 1.0 / max(mp.Hr, 1.0);
    float invHm = 1.0 / max(mp.Hm, 1.0);
    vec3 betaRA = mp.betaR + mp.betaA;
    float ts = t0 + mp.jitter * dt;
    vec3 dp = mp.rd * dt;
    vec3 p = mp.camLocal + mp.rd * ts;

    for (int i = 0; i < steps; ++i)
    {
        vec3 radial = p - mp.center;
        float r = length(radial);
        float invR = (r > 0.0) ? (1.0 / r) : 0.0;
        vec3 dir = (r > 0.0) ? (radial * invR) : vec3(0.0, 1.0, 0.0);
        float height = max(r - mp.planetRadius, 0.0);

        float densR = 0.0;
        float densM = 0.0;
        if (mp.atmActive)
        {
            densR = exp(-height * invHr);
            densM = exp(-height * invHm);
            s.odR += densR * dt;
            s.odM += densM * dt;
        }

        float densC = 0.0;
        if (doCloud)
        {
            vec3 radialDx = mp.rayDx * ts;
            vec3 radialDy = mp.rayDy * ts;
            vec3 dirDx = differentiate_normalized(radial, radialDx);
            vec3 dirDy = differentiate_normalized(radial, radialDy);
            float heightDx = dot(dir, radialDx);
            float heightDy = dot(dir, radialDy);
            densC = cloud_density(dir,
                                  dirDx,
                                  dirDy,
                                  height,
                                  heightDx,
                                  heightDy,
                                  mp.cloudsActive,
                                  mp.cloudFlipV,
                                  mp.cloudOverlayRotSC,
                                  mp.cloudWindHeading,
                                  mp.cloudWindSC);
            if (densC > 0.0) s.odC += densC * dt;
        }

        vec3 tauCam = betaRA * s.odR + mp.betaM * s.odM + vec3(CLOUD_BETA * s.odC);
        vec3 attenCam = exp(-tauCam);

        float bSun = dot(radial, mp.sunDir);
        float muS = bSun * invR;
        float d2 = r * r - bSun * bSun;
        bool inShadow = (bSun < 0.0) && (d2 < planetRadius2);

        if (!inShadow)
        {
            vec3 attenSun = vec3(1.0);
            if (mp.atmActive && mp.atmRadius > mp.planetRadius)
            {
                vec2 amSun = sun_optical_depth(r, muS, mp.planetRadius, mp.atmRadius);
                vec2 odSun = amSun * vec2(mp.Hr, mp.Hm);
                attenSun = exp(-(betaRA * odSun.x + mp.betaM * odSun.y));
            }

            vec3 atten = attenCam * attenSun;
            if (mp.atmActive)
            {
                vec3 scatterCoeff = densR * mp.betaR * mp.phaseR + densM * mp.betaM * mp.phaseM;
                s.scatterAtm += atten * scatterCoeff * dt;
            }
            if (doCloud && densC > 0.0)
            {
                float cloudMuS = max(muS, 0.0);
                float pathToTop = max(mp.cloudRTop - r, 0.0) / max(cloudMuS, 0.15);
                float cloudSunTrans = exp(-CLOUD_BETA * densC * pathToTop);
                float powder = 1.0 - exp(-densC * 2.0);
                float scatterBoost = mix(0.45, 1.0, powder);
                s.scatterCloudSun += atten * cloudSunTrans * (densC * CLOUD_BETA * mp.phaseC) * dt *
                                     scatterBoost * CLOUD_SCATTER_SCALE;
            }
        }

        if (doCloud && densC > 0.0)
        {
            float powder = 1.0 - exp(-densC * 1.25);
            float scatterBoost = mix(0.25, 1.0, powder);
            float dayMask = smoothstep(-0.02, 0.12, muS);
            if (inShadow) dayMask = 0.0;
            s.scatterCloudAmb += attenCam * (densC * CLOUD_BETA) * dt * scatterBoost * CLOUD_SCATTER_SCALE * dayMask;
        }

        float transMax = max(attenCam.r, max(attenCam.g, attenCam.b));
        if (transMax < 1e-3)
        {
            break;
        }

        ts += dt;
        p += dp;
    }
}

const int MAX_SEGS = 5;

void distribute_steps(inout vec2 segments[MAX_SEGS], int count, int totalSteps)
{
    float totalLen = 0.0;
    int activeCount = 0;
    for (int i = 0; i < count; ++i)
    {
        if (segments[i].x > 0.0)
        {
            totalLen += segments[i].x;
            activeCount++;
        }
    }

    if (totalSteps <= 0 || activeCount == 0)
    {
        for (int i = 0; i < count; ++i) segments[i].y = 0.0;
        return;
    }

    totalSteps = max(totalSteps, activeCount);
    int remaining = totalSteps;
    int segsLeft = activeCount;

    for (int i = 0; i < count; ++i)
    {
        if (segments[i].x <= 0.0)
        {
            segments[i].y = 0.0;
            continue;
        }
        if (segsLeft == 1)
        {
            segments[i].y = float(max(1, remaining));
            break;
        }
        int s = int(round(float(totalSteps) * (segments[i].x / totalLen)));
        s = clamp(s, 1, remaining - (segsLeft - 1));
        segments[i].y = float(s);
        remaining -= s;
        segsLeft--;
    }
}
vec3 render_atmosphere_monolithic(vec3 baseColor)
{
    float planetRadius = pc.planet_center_radius.w;
    if (planetRadius <= 0.0) return baseColor;

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

    if (!atmActive && !cloudsActive) return baseColor;

    vec3 camLocal = inCamLocal;
    vec3 rd = normalize(inWorldRay);
    vec3 rdDx = dFdx(rd);
    vec3 rdDy = dFdy(rd);
    vec3 center = pc.planet_center_radius.xyz;

    float boundRadius = atmActive ? atmRadius_eff : rTop;
    if (boundRadius <= planetRadius) return baseColor;

    float tStart;
    float tEnd;
    if (!compute_primary_ray_bounds(camLocal, rd, center, planetRadius, boundRadius, tStart, tEnd))
    {
        return baseColor;
    }

    int viewSteps = clamp(pc.misc.x, 4, 64);
    float jitter = resolve_jitter_sample(inUV);

    vec3 sunDir = -sceneData.sunlightDirection.xyz;
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
    mp.cloudOverlayRotSC = cloudsActive ? pc.jitter_params.zw : vec2(0.0, 1.0);
    mp.rayDx = rdDx;
    mp.rayDy = rdDy;

    if (cloudsActive)
    {
        float cloudR = planetRadius + cloudBaseM + 0.5 * cloudThicknessM;
        cloudR = max(cloudR, planetRadius);
        float windSpeedMps = pc.cloud_params.z;
        float windAngle = pc.cloud_params.w;
        mp.cloudWindHeading = vec2(cos(windAngle), sin(windAngle));
        float arc = (windSpeedMps * max(sceneData.timeParams.x, 0.0)) / max(cloudR, 1.0);
        mp.cloudWindSC = vec2(sin(arc), cos(arc));
    }

    vec2 cloudSeg0;
    vec2 cloudSeg1;
    int cloudSegCount = 0;
    if (cloudsActive)
    {
        cloudSegCount = compute_cloud_segments(camLocal, rd, center, rBase, rTop, tStart, tEnd, cloudSeg0, cloudSeg1);
        if (cloudSegCount == 0) cloudsActive = false;
    }

    if (!atmActive && !cloudsActive) return baseColor;

    MarchState state = march_state_init();
    if (!cloudsActive)
    {
        integrate_segment(tStart, tEnd, atmActive ? viewSteps : 0, false, mp, state);
    }
    else
    {
        vec2 cStepSegs[MAX_SEGS];
        for (int i = 0; i < MAX_SEGS; ++i) cStepSegs[i] = vec2(0.0);
        cStepSegs[0].x = max(cloudSeg0.y - cloudSeg0.x, 0.0);
        if (cloudSegCount == 2) cStepSegs[1].x = max(cloudSeg1.y - cloudSeg1.x, 0.0);
        float cloudLenTotal = cStepSegs[0].x + cStepSegs[1].x;
        int cloudStepsTotal = compute_adaptive_cloud_steps(cloudLenTotal, cloudThicknessM, clamp(pc.misc.z, 4, 128));
        distribute_steps(cStepSegs, cloudSegCount, cloudStepsTotal);

        float atmGapStart[3];
        float atmGapEnd[3];
        int atmGapCount;
        if (cloudSegCount == 1)
        {
            atmGapStart[0] = tStart;        atmGapEnd[0] = cloudSeg0.x;
            atmGapStart[1] = cloudSeg0.y;   atmGapEnd[1] = tEnd;
            atmGapCount = 2;
        }
        else
        {
            atmGapStart[0] = tStart;        atmGapEnd[0] = cloudSeg0.x;
            atmGapStart[1] = cloudSeg0.y;   atmGapEnd[1] = cloudSeg1.x;
            atmGapStart[2] = cloudSeg1.y;   atmGapEnd[2] = tEnd;
            atmGapCount = 3;
        }

        vec2 aStepSegs[MAX_SEGS];
        for (int i = 0; i < MAX_SEGS; ++i) aStepSegs[i] = vec2(0.0);
        for (int i = 0; i < atmGapCount; ++i)
        {
            aStepSegs[i].x = max(atmGapEnd[i] - atmGapStart[i], 0.0);
        }
        distribute_steps(aStepSegs, atmGapCount, atmActive ? viewSteps : 0);

        integrate_segment(atmGapStart[0], atmGapEnd[0], int(aStepSegs[0].y), false, mp, state);
        integrate_segment(cloudSeg0.x, cloudSeg0.y, int(cStepSegs[0].y), true, mp, state);
        integrate_segment(atmGapStart[1], atmGapEnd[1], int(aStepSegs[1].y), false, mp, state);

        if (cloudSegCount == 2)
        {
            integrate_segment(cloudSeg1.x, cloudSeg1.y, int(cStepSegs[1].y), true, mp, state);
            integrate_segment(atmGapStart[2], atmGapEnd[2], int(aStepSegs[2].y), false, mp, state);
        }
    }

    vec3 betaRA_eff = betaR_eff + betaA_eff;
    vec3 transmittance = exp(-(betaRA_eff * state.odR + betaM_eff * state.odM + vec3(CLOUD_BETA * state.odC)));
    vec3 outRgb = baseColor * transmittance;
    if (atmActive) outRgb += state.scatterAtm * (sunCol * atmIntensity);
    outRgb += state.scatterCloudSun * cloudSunCol;
    outRgb += state.scatterCloudAmb * (cloudAmbCol * CLOUD_AMBIENT_SCALE);
    return outRgb;
}
