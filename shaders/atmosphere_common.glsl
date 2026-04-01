const float PI = 3.14159265359;
const float INV_TWO_PI = 0.15915494309189535;
const float INV_PI = 0.3183098861837907;

const int FLAG_ATMOSPHERE = 1;
const int FLAG_CLOUDS = 2;
const int FLAG_CLOUD_FLIP_V = 4;
const uint MISC_FLAGS_MASK = 0xFFu;
const uint MISC_NOISE_BLEND_SHIFT = 8u;
const uint MISC_DETAIL_ERODE_SHIFT = 16u;

const float CLOUD_BETA = 1.0e-3;
const float CLOUD_AMBIENT_SCALE = 0.25;
const float CLOUD_PHASE_G = 0.80;
const float CLOUD_SCATTER_SCALE = 2.0;
const float CLOUD_SUN_WHITEN = 0.65;
const float CLOUD_AMBIENT_WHITEN = 0.85;
const float CLOUD_AMBIENT_MIN_LUMA = 0.30;
const float CLOUD_HEIGHT_SHIFT = 0.30;
const float CLOUD_THICKNESS_MIN = 0.55;
const float CLOUD_THICKNESS_MAX = 1.35;

float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float luminance(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec3 rotate_y_sc(vec3 v, vec2 sc)
{
    float s = sc.x;
    float c = sc.y;
    return vec3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z);
}

vec2 dir_to_equirect(vec3 d, bool flipV)
{
    d = normalize(d);
    float u = atan(d.z, d.x) * INV_TWO_PI + 0.5;
    float v = 0.5 - asin(clamp(d.y, -1.0, 1.0)) * INV_PI;
    if (flipV) v = 1.0 - v;
    return vec2(fract(u), clamp(v, 0.001, 0.999));
}

bool ray_sphere_intersect(vec3 ro, vec3 rd, vec3 center, float radius, out float t0, out float t1)
{
    vec3 oc = ro - center;
    float b = dot(oc, rd);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - c;
    if (h < 0.0) return false;
    h = sqrt(h);
    t0 = -b - h;
    t1 = -b + h;
    return true;
}

float sample_cloud_noise_grad(vec2 uvE, vec2 uvDx, vec2 uvDy, float scale, vec2 offset)
{
    vec2 uv = uvE * scale + offset;
    return textureGrad(cloudNoiseTex, fract(uv), uvDx * scale, uvDy * scale).r;
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

float phase_rayleigh(float cosTheta)
{
    return (3.0 / (16.0 * PI)) * (1.0 + cosTheta * cosTheta);
}

float phase_mie_hg(float cosTheta, float g)
{
    float g2 = g * g;
    return (1.0 / (4.0 * PI)) * (1.0 - g2) / pow(max(1.0 + g2 - 2.0 * g * cosTheta, 1e-4), 1.5);
}

float cloud_density(vec3 dir,
                    vec3 dirDx,
                    vec3 dirDy,
                    float height,
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
    if (cloudsActive)
    {
        vec3 east;
        vec3 north;
        build_local_cloud_frame(dir, east, north);
        vec3 windT = east * windHeading.x + north * windHeading.y;

        float s = windSC.x;
        float c = windSC.y;
        dirW = dir * c + windT * s;
    }

    dirW = rotate_y_sc(dirW, overlayRotSC);
    vec3 dirWdx = rotate_y_sc(dirDx, overlayRotSC);
    vec3 dirWdy = rotate_y_sc(dirDy, overlayRotSC);

    vec2 uvE;
    vec2 uvDx;
    vec2 uvDy;
    dir_to_equirect_grad(dirW, dirWdx, dirWdy, flipV, uvE, uvDx, uvDy);
    float localCov = clamp(textureGrad(cloudOverlayTex, uvE, uvDx, uvDy).r, 0.0, 1.0);

    uint miscPacked = uint(pc.misc.w);
    float weatherBlend = float((miscPacked >> MISC_NOISE_BLEND_SHIFT) & 0xFFu) * (1.0 / 255.0);
    float detailErode = float((miscPacked >> MISC_DETAIL_ERODE_SHIFT) & 0xFFu) * (1.0 / 255.0);

    float lowScale = max(pc.cloud_params.x, 0.001);
    float detailScale = max(pc.cloud_params.y, 0.001);
    float weatherNoise = sample_cloud_noise_grad(uvE, uvDx, uvDy, lowScale, vec2(0.173, 0.547));
    weatherNoise = clamp((weatherNoise - 0.5) * 1.25 + 0.5, 0.0, 1.0);
    float weatherField = clamp(mix(localCov, weatherNoise, weatherBlend), 0.0, 1.0);

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

    float detailNoise = sample_cloud_noise_grad(uvE, uvDx, uvDy, detailScale, vec2(0.619, 0.281));
    detailNoise = clamp((detailNoise - 0.5) * 1.55 + 0.5, 0.0, 1.0);
    float erosion = smoothstep(0.25, 0.85, detailNoise);
    float erosionMask = mix(0.40, 1.00, erosion);
    float detailMask = mix(1.0, erosionMask, detailErode);

    float d = cov * profile * detailMask;
    return max(d, 0.0) * densityScale;
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

void integrate_segment(float t0, float t1, int steps, bool doCloud, MarchParams mp, inout MarchState s)
{
    if (steps <= 0 || t1 <= t0) return;

    float dt = (t1 - t0) / float(steps);
    float planetRadius2 = mp.planetRadius * mp.planetRadius;

    for (int i = 0; i < steps; ++i)
    {
        float ts = t0 + (float(i) + mp.jitter) * dt;
        vec3 p = mp.camLocal + mp.rd * ts;

        vec3 radial = p - mp.center;
        float r = length(radial);
        vec3 dir = (r > 0.0) ? (radial / r) : vec3(0.0, 1.0, 0.0);
        vec3 dirDx = differentiate_normalized(radial, mp.rayDx * ts);
        vec3 dirDy = differentiate_normalized(radial, mp.rayDy * ts);
        float height = max(r - mp.planetRadius, 0.0);

        float densR = 0.0;
        float densM = 0.0;
        if (mp.atmActive)
        {
            densR = exp(-height / max(mp.Hr, 1.0));
            densM = exp(-height / max(mp.Hm, 1.0));
            s.odR += densR * dt;
            s.odM += densM * dt;
        }

        float densC = 0.0;
        if (doCloud)
        {
            densC = cloud_density(dir,
                                  dirDx,
                                  dirDy,
                                  height,
                                  mp.cloudsActive,
                                  mp.cloudFlipV,
                                  mp.cloudOverlayRotSC,
                                  mp.cloudWindHeading,
                                  mp.cloudWindSC);
            if (densC > 0.0) s.odC += densC * dt;
        }

        vec3 tauCam = mp.betaR * s.odR + mp.betaM * s.odM + mp.betaA * s.odR + vec3(CLOUD_BETA * s.odC);
        vec3 attenCam = exp(-tauCam);

        float bSun = dot(radial, mp.sunDir);
        float d2 = r * r - bSun * bSun;
        bool inShadow = (bSun < 0.0) && (d2 < planetRadius2);

        if (!inShadow)
        {
            vec3 attenSun = vec3(1.0);
            if (mp.atmActive && mp.atmRadius > mp.planetRadius)
            {
                float muS = dot(dir, mp.sunDir);
                vec2 amSun = sun_optical_depth(r, muS, mp.planetRadius, mp.atmRadius);
                vec2 odSun = amSun * vec2(mp.Hr, mp.Hm);
                attenSun = exp(-(mp.betaR * odSun.x + mp.betaM * odSun.y + mp.betaA * odSun.x));
            }

            vec3 atten = attenCam * attenSun;
            if (mp.atmActive)
            {
                vec3 scatterCoeff = densR * mp.betaR * mp.phaseR + densM * mp.betaM * mp.phaseM;
                s.scatterAtm += atten * scatterCoeff * dt;
            }
            if (doCloud && densC > 0.0)
            {
                float muS = max(dot(dir, mp.sunDir), 0.0);
                float pathToTop = max(mp.cloudRTop - r, 0.0) / max(muS, 0.15);
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
            float muS = dot(dir, mp.sunDir);
            float dayMask = smoothstep(-0.02, 0.12, muS);
            if (inShadow) dayMask = 0.0;
            s.scatterCloudAmb += attenCam * (densC * CLOUD_BETA) * dt * scatterBoost * CLOUD_SCATTER_SCALE * dayMask;
        }

        float transMax = max(attenCam.r, max(attenCam.g, attenCam.b));
        if (transMax < 1e-3)
        {
            break;
        }
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
            tEnd = min(tEnd, tSurf);
        }
    }

    tStart = max(tBound0, 0.0);
    return tEnd > tStart;
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
    int cloudStepsTotal = clamp(pc.misc.z, 4, 128);
    float jitter = mix(0.5, hash12(inUV * 1024.0), clamp(pc.jitter_params.x, 0.0, 1.0));

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

    vec3 transmittance = exp(-(betaR_eff * state.odR + betaM_eff * state.odM + betaA_eff * state.odR + vec3(CLOUD_BETA * state.odC)));
    vec3 outRgb = baseColor * transmittance;
    if (atmActive) outRgb += state.scatterAtm * (sunCol * atmIntensity);
    outRgb += state.scatterCloudSun * cloudSunCol;
    outRgb += state.scatterCloudAmb * (cloudAmbCol * CLOUD_AMBIENT_SCALE);
    return outRgb;
}
