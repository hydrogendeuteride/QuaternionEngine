#version 450

#extension GL_GOOGLE_include_directive : require

#include "input_structures.glsl"

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inColor;
layout (location = 2) in vec2 inUV;
layout (location = 3) in vec3 inWorldPos;

layout (location = 0) out vec4 outFragColor;

vec3 get_camera_local_position()
{
    mat3 rotT = mat3(sceneData.view);
    mat3 rot = transpose(rotT);
    vec3 t = sceneData.view[3].xyz;
    return -rot * t;
}

void main()
{
    // --- Unpack material parameters ---
    float opacity        = clamp(materialData.extra[3].x, 0.0, 1.0);
    float fresnelPower   = max(materialData.extra[3].y, 0.001);
    float fresnelStrength = max(materialData.extra[3].z, 0.0);
    vec3  tint           = materialData.extra[4].rgb;

    vec2  scrollVelocity1    = materialData.extra[5].xy;
    vec2  scrollVelocity2    = materialData.extra[5].zw;
    float distortionStrength = materialData.extra[6].x;
    float noiseBlend         = materialData.extra[6].y;
    float gradientAxis       = materialData.extra[6].z;
    float emissionStrength   = materialData.extra[6].w;
    vec3  coreColor          = materialData.extra[7].rgb;
    float gradientStart      = materialData.extra[7].w;
    vec3  edgeColor          = materialData.extra[8].rgb;
    float gradientEnd        = materialData.extra[8].w;

    float time = sceneData.timeParams.x;

    // --- Base texture ---
    vec4 baseTex = texture(colorTex, inUV);
    float alphaCutoff = materialData.extra[2].x;
    float baseAlpha = baseTex.a * materialData.colorFactors.a;
    if (alphaCutoff > 0.0 && baseAlpha < alphaCutoff)
    {
        discard;
    }

    vec3 baseColor = inColor * baseTex.rgb * materialData.colorFactors.rgb * tint;

    // --- Noise UV scrolling ---
    vec2 uv1 = inUV + scrollVelocity1 * time;
    vec2 uv2 = inUV + scrollVelocity2 * time;

    // --- Noise sampling with cross-distortion ---
    float noise1 = texture(metalRoughTex, uv1).r;
    vec2 uv2d = uv2 + (noise1 - 0.5) * distortionStrength;
    float noise2 = texture(normalMap, uv2d).r;
    float noise = mix(noise1, noise2, noiseBlend);

    // --- UV-axis gradient ---
    float gc = mix(inUV.x, inUV.y, gradientAxis);
    float grad = smoothstep(gradientStart, gradientEnd, gc);
    vec3 gradColor = mix(coreColor, edgeColor, grad);

    // --- Fresnel ---
    vec3 N = normalize(inNormal);
    vec3 V = normalize(get_camera_local_position() - inWorldPos);
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), fresnelPower);

    // --- Composite ---
    vec3 color = baseColor * gradColor * noise * (1.0 + fresnel * fresnelStrength) * emissionStrength;
    float alpha = clamp(baseAlpha * opacity * (1.0 - grad) * noise, 0.0, 1.0);

    outFragColor = vec4(color, alpha);
}
