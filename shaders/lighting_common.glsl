#ifndef LIGHTING_COMMON_GLSL
#define LIGHTING_COMMON_GLSL

const float PI = 3.14159265359;

float pow5(float x)
{
    float x2 = x * x;
    return x2 * x2 * x;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    float m = clamp(1.0 - cosTheta, 0.0, 1.0);
    return F0 + (1.0 - F0) * pow5(m);
}

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a      = roughness * roughness;
    float a2     = a * a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / max(denom, 0.001);
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;

    float denom = NdotV * (1.0 - k) + k;
    return NdotV / max(denom, 0.001);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float ggx2 = GeometrySchlickGGX(max(dot(N, V), 0.0), roughness);
    float ggx1 = GeometrySchlickGGX(max(dot(N, L), 0.0), roughness);
    return ggx1 * ggx2;
}

vec3 evaluate_brdf(vec3 N, vec3 V, vec3 L, vec3 albedo, float roughness, float metallic)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    // Avoid undefined half-vector when V and L are opposite (normalize(0) => NaN),
    // which can manifest as single-pixel fireflies on smooth surfaces.
    if (NdotL <= 0.0 || NdotV <= 0.0)
    {
        return vec3(0.0);
    }

    vec3 H = V + L;
    float Hlen2 = dot(H, H);
    if (Hlen2 <= 1.0e-8)
    {
        return vec3(0.0);
    }
    H *= inversesqrt(Hlen2);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 F  = fresnelSchlick(max(dot(H, V), 0.0), F0);
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);

    vec3 numerator    = NDF * G * F;
    float denom       = 4.0 * NdotV * NdotL;
    vec3 specular     = numerator / max(denom, 0.001);

    vec3 kS = F;
    vec3 kD = (1.0 - kS) * (1.0 - metallic);

    return (kD * albedo / PI + specular) * NdotL;
}

vec3 eval_point_light(GPUPunctualLight light, vec3 pos, vec3 N, vec3 V, vec3 albedo, float roughness, float metallic)
{
    vec3 lightPos = light.position_radius.xyz;
    float radius  = max(light.position_radius.w, 0.0001);

    vec3 toLight = lightPos - pos;
    float dist2  = dot(toLight, toLight);
    if (dist2 <= 1.0e-8)
    {
        return vec3(0.0);
    }
    float invDist = inversesqrt(dist2);
    float dist    = dist2 * invDist;
    vec3 L        = toLight * invDist;

    // Smooth falloff: inverse-square with soft clamp at radius
    float att = 1.0 / max(dist2, 1.0e-8);
    float x   = clamp(dist / radius, 0.0, 1.0);
    float smth = (1.0 - x * x);
    smth *= smth;
    float falloff = att * smth;

    vec3 brdf = evaluate_brdf(N, V, L, albedo, roughness, metallic);
    vec3 lightColor = light.color_intensity.rgb * light.color_intensity.a;
    return brdf * lightColor * falloff;
}

vec3 eval_spot_light(GPUSpotLight light, vec3 pos, vec3 N, vec3 V, vec3 albedo, float roughness, float metallic)
{
    vec3 lightPos = light.position_radius.xyz;
    float radius  = max(light.position_radius.w, 0.0001);

    vec3 toLight = lightPos - pos;
    float dist2 = dot(toLight, toLight);
    if (dist2 <= 1.0e-8)
    {
        return vec3(0.0);
    }
    float invDist = inversesqrt(dist2);
    float dist = dist2 * invDist;
    vec3 L = toLight * invDist; // surface -> light

    // direction_cos_outer.xyz is expected to be unit length (normalized on the CPU).
    vec3 dir = light.direction_cos_outer.xyz; // light -> forward
    float cosOuter = light.direction_cos_outer.w;
    float cosInner = light.cone.x;
    float cosTheta = dot(-L, dir); // light -> surface vs light forward
    if (cosTheta <= cosOuter)
    {
        return vec3(0.0);
    }
    float denom = max(cosInner - cosOuter, 0.0001);
    float spot = clamp((cosTheta - cosOuter) / denom, 0.0, 1.0);
    spot *= spot;

    // Smooth falloff: inverse-square with soft clamp at radius
    float att = 1.0 / max(dist2, 1.0e-8);
    float x   = clamp(dist / radius, 0.0, 1.0);
    float smth = (1.0 - x * x);
    smth *= smth;
    float falloff = att * smth;

    vec3 brdf = evaluate_brdf(N, V, L, albedo, roughness, metallic);
    vec3 lightColor = light.color_intensity.rgb * light.color_intensity.a;
    return brdf * lightColor * falloff * spot;
}

#endif // LIGHTING_COMMON_GLSL
