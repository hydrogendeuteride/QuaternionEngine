#version 460
#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Set 1: cloud inputs
layout(set = 1, binding = 0) uniform sampler2D hdrInput;
layout(set = 1, binding = 1) uniform sampler2D posTex;

layout(set = 1, binding = 2, std430) readonly buffer VoxelDensity
{
    float density[];
} voxel;

layout(push_constant) uniform VolumePush
{
    vec4 volume_center_follow; // xyz: center_local (or offset), w: followCameraXZ (0/1)
    vec4 volume_half_extents;  // xyz: half extents (local)
    vec4 density_params;       // x: densityScale, y: coverage, z: extinction, w: time_sec
    vec4 scatter_params;       // rgb: albedo/tint, w: scatterStrength
    vec4 emission_params;      // rgb: emissionColor, w: emissionStrength
    ivec4 misc;                // x: stepCount, y: gridResolution, z: volumeType (0=cloud,1=smoke,2=flame)
} pc;

vec3 getCameraWorldPosition()
{
    mat3 rotT = mat3(sceneData.view); // R^T
    mat3 rot  = transpose(rotT);      // R
    vec3 T    = sceneData.view[3].xyz;
    return -rot * T;
}

float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

bool intersectAABB(vec3 ro, vec3 rd, vec3 bmin, vec3 bmax, out float tmin, out float tmax)
{
    vec3 invD = 1.0 / rd;
    vec3 t0s = (bmin - ro) * invD;
    vec3 t1s = (bmax - ro) * invD;
    vec3 tsmaller = min(t0s, t1s);
    vec3 tbigger  = max(t0s, t1s);
    tmin = max(max(tsmaller.x, tsmaller.y), tsmaller.z);
    tmax = min(min(tbigger.x,  tbigger.y),  tbigger.z);
    return tmax >= max(tmin, 0.0);
}

float sample_voxel_density(vec3 p, vec3 bmin, vec3 bmax)
{
    vec3 uvw = (p - bmin) / (bmax - bmin);
    if (any(lessThan(uvw, vec3(0.0))) || any(greaterThan(uvw, vec3(1.0))))
    {
        return 0.0;
    }

    int res = max(pc.misc.y, 1);
    int slice = res * res;
    float fres = float(res);
    vec3 g = uvw * (fres - 1.0);

    ivec3 base = ivec3(floor(g));
    base = clamp(base, ivec3(0), ivec3(res - 1));
    vec3 f = fract(g);

    ivec3 b1 = min(base + ivec3(1), ivec3(res - 1));
    ivec3 step = b1 - base; // 0 or 1 per axis

    int baseIndex = base.x + base.y * res + base.z * slice;
    int dx = step.x;
    int dy = step.y * res;
    int dz = step.z * slice;

    float d000 = voxel.density[baseIndex];
    float d100 = voxel.density[baseIndex + dx];
    float d010 = voxel.density[baseIndex + dy];
    float d110 = voxel.density[baseIndex + dy + dx];

    int baseIndexZ = baseIndex + dz;
    float d001 = voxel.density[baseIndexZ];
    float d101 = voxel.density[baseIndexZ + dx];
    float d011 = voxel.density[baseIndexZ + dy];
    float d111 = voxel.density[baseIndexZ + dy + dx];

    float x00 = mix(d000, d100, f.x);
    float x10 = mix(d010, d110, f.x);
    float x01 = mix(d001, d101, f.x);
    float x11 = mix(d011, d111, f.x);

    float y0 = mix(x00, x10, f.y);
    float y1 = mix(x01, x11, f.y);

    return mix(y0, y1, f.z);
}

void main()
{
    vec3 baseColor = texture(hdrInput, inUV).rgb;

    vec3 camPos = getCameraWorldPosition();

    // Reconstruct a world-space ray for this pixel (Vulkan depth range 0..1).
    vec2 ndc = inUV * 2.0 - 1.0;
    vec3 viewDir = normalize(vec3(ndc.x / sceneData.proj[0][0], ndc.y / sceneData.proj[1][1], -1.0));
    vec3 rd = transpose(mat3(sceneData.view)) * viewDir;

    // Define a local-space cloud volume (optionally anchored to camera XZ).
    vec3 center = pc.volume_center_follow.xyz;
    if (pc.volume_center_follow.w > 0.5)
    {
        center.xz += camPos.xz;
    }
    vec3 halfExt = max(pc.volume_half_extents.xyz, vec3(0.01));
    vec3 bmin = center - halfExt;
    vec3 bmax = center + halfExt;

    float t0, t1;
    if (!intersectAABB(camPos, rd, bmin, bmax, t0, t1))
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    // Clamp march to geometry distance (gbufferPosition.w == 1 for valid surfaces).
    vec4 posSample = texture(posTex, inUV);
    if (posSample.w > 0.0)
    {
        float surfT = dot(posSample.xyz - camPos, rd);
        if (surfT > 0.0)
        {
            t1 = min(t1, surfT);
        }
    }

    int steps = clamp(pc.misc.x, 8, 256);
    if (t1 <= t0 || steps <= 0)
    {
        outColor = vec4(baseColor, 1.0);
        return;
    }

    float dt = (t1 - t0) / float(steps);
    float jitter = hash12(inUV * 1024.0);
    float t = max(t0, 0.0) + (jitter - 0.5) * dt;

    vec3 Lsun = normalize(-sceneData.sunlightDirection.xyz);
    vec3 sunCol = sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;
    vec3 ambCol = sceneData.ambientColor.rgb;

    float trans = 1.0;
    vec3 scatter = vec3(0.0);
    int volType = pc.misc.z;

    for (int i = 0; i < steps; ++i)
    {
        vec3 p = camPos + rd * (t + 0.5 * dt);

        float d = sample_voxel_density(p, bmin, bmax);
        d = max(0.0, d - pc.density_params.y) / max(1.0 - pc.density_params.y, 1e-3);
        d *= max(pc.density_params.x, 0.0);
        d *= max(pc.density_params.z, 0.0);

        if (d > 1e-4)
        {
            // Exponential absorption / single-scattering approximation.
            float alpha = 1.0 - exp(-d * dt);

            if (volType == 2)
            {
                // Flames: emissive contribution.
                float flicker = mix(0.65, 1.0, hash12(p.xz * 0.35 + pc.density_params.w * 0.25));
                vec3 emit = pc.emission_params.rgb * (pc.emission_params.w * flicker);
                scatter += trans * alpha * emit;
            }
            else
            {
                float cosTheta = clamp(dot(rd, Lsun), 0.0, 1.0);
                float cos2 = cosTheta * cosTheta;
                float phase = 0.30 + 0.70 * (cos2 * cos2); // cheap forward-scatter bias
                vec3 light = ambCol * 0.25 + sunCol * phase;

                vec3 albedo = clamp(pc.scatter_params.rgb, vec3(0.0), vec3(1.0));
                float s = max(pc.scatter_params.w, 0.0);
                scatter += trans * alpha * light * albedo * s;
            }
            trans *= (1.0 - alpha);

            if (trans < 0.01)
            {
                break;
            }
        }

        t += dt;
        if (t > t1)
        {
            break;
        }
    }

    vec3 outRgb = scatter + trans * baseColor;
    outColor = vec4(outRgb, 1.0);
}
