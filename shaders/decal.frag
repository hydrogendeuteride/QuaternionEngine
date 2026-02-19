#version 450

layout(location = 0) out vec4 outNormal;
layout(location = 1) out vec4 outAlbedo;

layout(set = 1, binding = 0) uniform sampler2D gbufferPosition;
layout(set = 2, binding = 0) uniform sampler2D decalAlbedoTex;
layout(set = 2, binding = 1) uniform sampler2D decalNormalTex;

layout(push_constant) uniform Push
{
    vec4 axis_x;
    vec4 axis_y;
    vec4 axis_z;
    vec4 center_extent_x;
    vec4 extent_yz_shape_opacity;
    vec4 tint_normal;
} pc;

void main()
{
    vec2 texSize = vec2(textureSize(gbufferPosition, 0));
    vec2 uv = gl_FragCoord.xy / texSize;

    vec4 posSample = texture(gbufferPosition, uv);
    if (posSample.w == 0.0)
    {
        discard;
    }

    vec3 center = pc.center_extent_x.xyz;
    vec3 halfExtents = vec3(pc.center_extent_x.w, pc.extent_yz_shape_opacity.x, pc.extent_yz_shape_opacity.y);
    vec3 rel = posSample.xyz - center;

    vec3 local = vec3(
        dot(rel, pc.axis_x.xyz) / halfExtents.x,
        dot(rel, pc.axis_y.xyz) / halfExtents.y,
        dot(rel, pc.axis_z.xyz) / halfExtents.z
    );

    int mode = int(pc.extent_yz_shape_opacity.z + 0.5);
    bool sphere = ((mode & 1) != 0);
    if (!sphere)
    {
        if (any(greaterThan(abs(local), vec3(1.0))))
        {
            discard;
        }
    }
    else
    {
        if (dot(local, local) > 1.0)
        {
            discard;
        }
    }

    // Project along local Z axis.
    vec2 decalUV = local.xy * 0.5 + 0.5;
    if (any(lessThan(decalUV, vec2(0.0))) || any(greaterThan(decalUV, vec2(1.0))))
    {
        discard;
    }

    vec4 albedoSample = texture(decalAlbedoTex, decalUV);
    float alpha = clamp(albedoSample.a * pc.extent_yz_shape_opacity.w, 0.0, 1.0);
    if (alpha <= 0.001)
    {
        discard;
    }

    vec3 albedo = albedoSample.rgb * pc.tint_normal.xyz;

    vec2 enc = texture(decalNormalTex, decalUV).xy * 2.0 - 1.0;
    float normalStrength = max(pc.tint_normal.w, 0.0);
    enc *= normalStrength;
    float z2 = max(0.0, 1.0 - dot(enc, enc));
    vec3 Nm = vec3(enc, sqrt(z2));
    vec3 Nw = normalize(pc.axis_x.xyz * Nm.x + pc.axis_y.xyz * Nm.y + pc.axis_z.xyz * Nm.z);

    outNormal = vec4(Nw, alpha);
    outAlbedo = vec4(albedo, alpha);
}
