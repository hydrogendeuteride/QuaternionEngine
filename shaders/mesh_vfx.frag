#version 450

#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec2 inUV;
layout (location = 3) in vec3 inWorldPos;

layout (location = 0) out vec4 outFragColor;

vec3 get_camera_world_position()
{
    mat3 rotT = mat3(sceneData.view);
    mat3 rot = transpose(rotT);
    vec3 t = sceneData.view[3].xyz;
    return -rot * t;
}

void main()
{
    vec4 baseTex = texture(colorTex, inUV);
    float alphaCutoff = materialData.extra[2].x;

    float opacity = clamp(materialData.extra[3].x, 0.0, 1.0);
    float alpha = clamp(baseTex.a * materialData.colorFactors.a * opacity, 0.0, 1.0);
    if (alphaCutoff > 0.0 && alpha < alphaCutoff)
    {
        discard;
    }

    vec3 tint = materialData.extra[4].rgb;
    vec3 baseColor = inColor * baseTex.rgb * materialData.colorFactors.rgb * tint;

    vec3 N = normalize(inNormal);
    vec3 V = normalize(get_camera_world_position() - inWorldPos);
    float fresnelPower = max(materialData.extra[3].y, 0.001);
    float fresnelStrength = max(materialData.extra[3].z, 0.0);
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), fresnelPower);

    vec3 color = baseColor * (1.0 + fresnel * fresnelStrength);
    outFragColor = vec4(color, alpha);
}

