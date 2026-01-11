// For transparent. Also can be used for normal meshes

#version 450

#extension GL_GOOGLE_include_directive : require
#include "input_structures.glsl"
#include "ibl_common.glsl"
#include "lighting_common.glsl"
#include "planet_shadow.glsl"

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec2 inUV;
layout (location = 3) in vec3 inWorldPos;
layout (location = 4) in vec4 inTangent;

layout (location = 0) out vec4 outFragColor;

vec3 getCameraWorldPosition() // Because I'm not clever enough to add cmaera position...
{
    // view = [ R^T  -R^T*C ]
    //        [ 0       1   ]
    // => C = -R * T, where T is view[3].xyz and R = transpose(mat3(view))
    mat3 rotT = mat3(sceneData.view);
    mat3 rot  = transpose(rotT);
    vec3 T    = sceneData.view[3].xyz;
    return -rot * T;
}

void main()
{
    // Base color with material factor and texture
    vec4 baseTex = texture(colorTex, inUV);
    // Alpha from baseColor texture and factor (glTF spec)
    float alpha = clamp(baseTex.a * materialData.colorFactors.a, 0.0, 1.0);
    // Optional alpha-cutout support for MASK materials (alphaCutoff > 0)
    float alphaCutoff = materialData.extra[2].x;
    if (alphaCutoff > 0.0 && alpha < alphaCutoff)
    {
        discard;
    }
    vec3 albedo = inColor * baseTex.rgb * materialData.colorFactors.rgb;
    // glTF: metallicRoughnessTexture uses G=roughness, B=metallic
    vec2 mrTex = texture(metalRoughTex, inUV).gb;
    float roughness = clamp(mrTex.x * materialData.metal_rough_factors.y, 0.04, 1.0);
    float metallic  = clamp(mrTex.y * materialData.metal_rough_factors.x, 0.0, 1.0);

    // Normal mapping path for forward/transparent pipeline
    // Expect UNORM normal map; support BC5 (RG) by reconstructing Z from XY.
    vec3 Nn = normalize(inNormal);
    vec3 N = Nn;

    float normalScale = max(materialData.extra[0].x, 0.0);
    if (normalScale > 0.0)
    {
        vec2 enc = texture(normalMap, inUV).xy * 2.0 - 1.0;
        enc *= normalScale;
        float z2 = 1.0 - dot(enc, enc);
        float nz = z2 > 0.0 ? sqrt(z2) : 0.0;
        vec3 Nm = vec3(enc, nz);

        vec3 T = normalize(inTangent.xyz);
        vec3 B = normalize(cross(Nn, T)) * inTangent.w;
        N = normalize(T * Nm.x + B * Nm.y + Nn * Nm.z);
    }
    vec3 camPos = getCameraWorldPosition();
    vec3 V = normalize(camPos - inWorldPos);

    // Directional sun term (no shadows in forward path)
    vec3 Lsun = normalize(-sceneData.sunlightDirection.xyz);
    float sunVis = 1.0;
    if (sceneData.rtParams.y > 0.0)
    {
        // Use a small receiver offset to reduce precision issues near the boundary.
        vec3 wp = inWorldPos + N * 0.0025;
        sunVis = planet_analytic_shadow_visibility(wp, Lsun);
    }
    vec3 sunBRDF = evaluate_brdf(N, V, Lsun, albedo, roughness, metallic);
    vec3 direct = sunBRDF * sceneData.sunlightColor.rgb * sceneData.sunlightColor.a * sunVis;

    //point lights
    uint pointCount = sceneData.lightCounts.x;
    for (uint i = 0u; i < pointCount; ++i)
    {
        direct += eval_point_light(sceneData.punctualLights[i], inWorldPos, N, V, albedo, roughness, metallic);
    }

    // Spot lights
    uint spotCount = sceneData.lightCounts.y;
    for (uint i = 0u; i < spotCount; ++i)
    {
        direct += eval_spot_light(sceneData.spotLights[i], inWorldPos, N, V, albedo, roughness, metallic);
    }

    // IBL: specular from equirect 2D mips; diffuse from SH
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

    // Ambient occlusion from texture + strength (indirect only)
    // extra[0].y = AO strength, extra[0].z = hasAO flag (1 = use AO texture)
    float hasAO = materialData.extra[0].z;
    float aoStrength = clamp(materialData.extra[0].y, 0.0, 1.0);
    float ao = 1.0;
    if (hasAO > 0.5 && aoStrength > 0.0)
    {
        float aoTex = texture(occlusionTex, inUV).r;
        ao = 1.0 - aoStrength + aoStrength * aoTex;
    }

    // Emissive from texture and factor
    vec3 emissive = vec3(0.0);
    vec3 emissiveFactor = materialData.extra[1].rgb;
    if (any(greaterThan(emissiveFactor, vec3(0.0))))
    {
        vec3 emissiveSample = texture(emissiveTex, inUV).rgb;
        emissive = emissiveSample * emissiveFactor;

        // Planet night emission: only show emission on the dark side of planet surfaces
        // Convention: extra[2].y > 0 => planet material
        bool isPlanet = (materialData.extra[2].y > 0.0);
        if (isPlanet)
        {
            float NdotL = max(dot(N, Lsun), 0.0);
            float nightFactor = 1.0 - smoothstep(0.0, 0.15, NdotL);
            emissive *= nightFactor;
        }
    }

    vec3 indirect = diffIBL + specIBL;
    vec3 color = direct + indirect * ao + emissive;

    outFragColor = vec4(color, alpha);
}
