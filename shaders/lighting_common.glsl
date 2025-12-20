#ifndef LIGHTING_COMMON_GLSL
#define LIGHTING_COMMON_GLSL

const float PI = 3.14159265359;

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
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
    vec3 H = normalize(V + L);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 F  = fresnelSchlick(max(dot(H, V), 0.0), F0);
    float NDF = DistributionGGX(N, H, roughness);
    float G   = GeometrySmith(N, V, L, roughness);

    vec3 numerator    = NDF * G * F;
    float denom       = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
    vec3 specular     = numerator / max(denom, 0.001);

    vec3 kS = F;
    vec3 kD = (1.0 - kS) * (1.0 - metallic);

    float NdotL = max(dot(N, L), 0.0);
    return (kD * albedo / PI + specular) * NdotL;
}

vec3 eval_point_light(GPUPunctualLight light, vec3 pos, vec3 N, vec3 V, vec3 albedo, float roughness, float metallic)
{
    vec3 lightPos = light.position_radius.xyz;
    float radius  = max(light.position_radius.w, 0.0001);
    vec3 L        = lightPos - pos;
    float dist    = length(L);
    if (dist <= 0.0001)
    {
        return vec3(0.0);
    }
    L /= dist;

    // Smooth falloff: inverse-square with soft clamp at radius
    float att = 1.0 / max(dist * dist, 0.0001);
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
    float dist = length(toLight);
    if (dist <= 0.0001)
    {
        return vec3(0.0);
    }
    vec3 L = toLight / dist; // surface -> light

    vec3 dir = normalize(light.direction_cos_outer.xyz); // light -> forward
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
    float att = 1.0 / max(dist * dist, 0.0001);
    float x   = clamp(dist / radius, 0.0, 1.0);
    float smth = (1.0 - x * x);
    smth *= smth;
    float falloff = att * smth;

    vec3 brdf = evaluate_brdf(N, V, L, albedo, roughness, metallic);
    vec3 lightColor = light.color_intensity.rgb * light.color_intensity.a;
    return brdf * lightColor * falloff * spot;
}

#endif // LIGHTING_COMMON_GLSL
