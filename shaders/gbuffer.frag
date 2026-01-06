#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_buffer_reference : require
#include "input_structures.glsl"

layout(location = 0) in vec3 inNormal;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inUV;
layout(location = 3) in vec3 inWorldPos;
layout(location = 4) in vec4 inTangent;

layout(location = 0) out vec4 outPos;
layout(location = 1) out vec4 outNorm;
layout(location = 2) out vec4 outAlbedo;
layout(location = 3) out uint outObjectID;
layout(location = 4) out vec4 outExtra;

// Keep push constants layout in sync with mesh.vert / GPUDrawPushConstants
struct Vertex {
    vec3 position; float uv_x;
    vec3 normal;   float uv_y;
    vec4 color;
    vec4 tangent;
};

layout(buffer_reference, std430) readonly buffer VertexBuffer{
    Vertex vertices[];
};

layout(push_constant) uniform constants
{
    mat4 render_matrix;
    mat3 normal_matrix;
    VertexBuffer vertexBuffer;
    uint objectID;
} PushConstants;

void main() {
    // Apply baseColor texture and baseColorFactor once
    vec4 baseTex = texture(colorTex, inUV);
    // Alpha from baseColor texture and factor, used for cutouts on MASK materials.
    float alpha = clamp(baseTex.a * materialData.colorFactors.a, 0.0, 1.0);
    float alphaCutoff = materialData.extra[2].x;
    if (alphaCutoff > 0.0 && alpha < alphaCutoff)
    {
        discard;
    }
    vec3 albedo = inColor * baseTex.rgb * materialData.colorFactors.rgb;

    // glTF metallic-roughness in G (roughness) and B (metallic)
    vec2 mrTex = texture(metalRoughTex, inUV).gb;
    float roughness = clamp(mrTex.x * materialData.metal_rough_factors.y, 0.04, 1.0);
    float metallic  = clamp(mrTex.y * materialData.metal_rough_factors.x, 0.0, 1.0);

    // Normal mapping: decode tangent-space normal and transform to world space
    // Expect UNORM normal map; support BC5 (RG) by reconstructing Z from XY.
    vec3 N = normalize(inNormal);
    vec3 Nw = N;

    float normalScale = max(materialData.extra[0].x, 0.0);
    if (normalScale > 0.0)
    {
        vec2 enc = texture(normalMap, inUV).xy * 2.0 - 1.0;
        enc *= normalScale;
        float z2 = 1.0 - dot(enc, enc);
        float nz = z2 > 0.0 ? sqrt(z2) : 0.0;
        vec3 Nm = vec3(enc, nz);

        vec3 T = normalize(inTangent.xyz);
        vec3 B = normalize(cross(N, T)) * inTangent.w;
        Nw = normalize(T * Nm.x + B * Nm.y + N * Nm.z);
    }

    // outPos.w is used as a "valid pixel" marker by downstream passes.
    // Keep 0.0 reserved for background/no-geometry, and encode small flags above 1.0.
    // Convention: materialData.extra[2].y > 0 => planet-style shadowing override.
    float gbuffer_flags = materialData.extra[2].y;
    outPos = vec4(inWorldPos, 1.0 + clamp(gbuffer_flags, 0.0, 1.0));
    outNorm = vec4(Nw, roughness);
    outAlbedo = vec4(albedo, metallic);
    // Extra G-buffer: x = AO, yzw = emissive
    // extra[0].y = AO strength, extra[0].z = hasAO flag (1 = use AO texture)
    float hasAO = materialData.extra[0].z;
    float aoStrength = clamp(materialData.extra[0].y, 0.0, 1.0);
    float ao = 1.0;
    if (hasAO > 0.5 && aoStrength > 0.0)
    {
        float aoTex = texture(occlusionTex, inUV).r;
        ao = 1.0 - aoStrength + aoStrength * aoTex;
    }

    vec3 emissive = vec3(0.0);
    vec3 emissiveFactor = materialData.extra[1].rgb;
    if (any(greaterThan(emissiveFactor, vec3(0.0))))
    {
        vec3 emissiveSample = texture(emissiveTex, inUV).rgb;
        emissive = emissiveSample * emissiveFactor;
    }
    outExtra = vec4(ao, emissive);
    outObjectID = PushConstants.objectID;
}
