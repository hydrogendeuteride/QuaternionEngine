#version 450

#extension GL_GOOGLE_include_directive : require
#include "input_structures.glsl"
#include "ibl_common.glsl"
#include "lighting_common.glsl"

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec2 inUV;
layout (location = 3) in vec3 inWorldPos;
layout (location = 4) in vec4 inTangent;

layout (location = 0) out vec4 outFragColor;

void main()
{
    // Base color with material factor and texture
    vec4 baseTex = texture(colorTex, inUV);
    vec3 albedo = inColor * baseTex.rgb * materialData.colorFactors.rgb;
    // glTF: metallicRoughnessTexture uses G=roughness, B=metallic
    vec2 mrTex = texture(metalRoughTex, inUV).gb;
    float roughness = clamp(mrTex.x * materialData.metal_rough_factors.y, 0.04, 1.0);
    float metallic  = clamp(mrTex.y * materialData.metal_rough_factors.x, 0.0, 1.0);

    // Normal mapping path for forward/transparent pipeline
    // Expect UNORM normal map; support BC5 (RG) by reconstructing Z from XY.
    vec2 enc = texture(normalMap, inUV).xy * 2.0 - 1.0;
    float normalScale = max(materialData.extra[0].x, 0.0);
    enc *= normalScale;
    float z2 = 1.0 - dot(enc, enc);
    float nz = z2 > 0.0 ? sqrt(z2) : 0.0;
    vec3 Nm = vec3(enc, nz);
    vec3 Nn = normalize(inNormal);
    vec3 T = normalize(inTangent.xyz);
    vec3 B = normalize(cross(Nn, T)) * inTangent.w;
    vec3 N = normalize(T * Nm.x + B * Nm.y + Nn * Nm.z);
    vec3 camPos = vec3(inverse(sceneData.view)[3]);
    vec3 V = normalize(camPos - inWorldPos);

    // Directional sun term (no shadows in forward path)
    vec3 Lsun = normalize(-sceneData.sunlightDirection.xyz);
    vec3 sunBRDF = evaluate_brdf(N, V, Lsun, albedo, roughness, metallic);
    vec3 direct = sunBRDF * sceneData.sunlightColor.rgb * sceneData.sunlightColor.a;

    // Punctual point lights
    uint pointCount = sceneData.lightCounts.x;
    for (uint i = 0u; i < pointCount; ++i)
    {
        direct += eval_point_light(sceneData.punctualLights[i], inWorldPos, N, V, albedo, roughness, metallic);
    }

    // IBL: specular from equirect 2D mips; diffuse from SH
    vec3 R = reflect(-V, N);
    float levels = float(textureQueryLevels(iblSpec2D));
    float lod = ibl_lod_from_roughness(roughness, levels);
    vec2 uv = dir_to_equirect(R);
    vec3 prefiltered = textureLod(iblSpec2D, uv, lod).rgb;
    vec2 brdf = texture(iblBRDF, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 specIBL = prefiltered * (F0 * brdf.x + brdf.y);
    vec3 diffIBL = (1.0 - metallic) * albedo * sh_eval_irradiance(N);

    vec3 color = direct + diffIBL + specIBL;

    // Alpha from baseColor texture and factor (glTF spec)
    float alpha = clamp(baseTex.a * materialData.colorFactors.a, 0.0, 1.0);
    outFragColor = vec4(color, alpha);
}
