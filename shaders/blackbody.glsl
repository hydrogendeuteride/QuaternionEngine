// Blackbody emission helpers (Kelvin -> RGB) + object-space noise sampling.
// Intended usage:
// - Use a tileable 2D noise texture bound as emissiveTex (set=1, binding=5)
// - Compute a nozzle/barrel-like heat profile from object-space coordinates + noise
// - Convert temperature (K) to RGB and output as emissive in linear space

#ifndef VKE_BLACKBODY_GLSL
#define VKE_BLACKBODY_GLSL

float saturate(float x) { return clamp(x, 0.0, 1.0); }
vec3  saturate(vec3 v)  { return clamp(v, vec3(0.0), vec3(1.0)); }

// Approximate blackbody (Kelvin) to sRGB-like RGB.
// Based on widely used piecewise fit (Tanner Helland style).
vec3 blackbody_rgb_srgb(float temp_k)
{
    float t = clamp(temp_k, 1000.0, 40000.0) / 100.0;

    float r;
    float g;
    float b;

    if (t <= 66.0)
    {
        r = 255.0;
        g = 99.4708025861 * log(t) - 161.1195681661;
        if (t <= 19.0)
        {
            b = 0.0;
        }
        else
        {
            b = 138.5177312231 * log(t - 10.0) - 305.0447927307;
        }
    }
    else
    {
        float tt = t - 60.0;
        r = 329.698727446 * pow(tt, -0.1332047592);
        g = 288.1221695283 * pow(tt, -0.0755148492);
        b = 255.0;
    }

    vec3 rgb = vec3(r, g, b) * (1.0 / 255.0);
    return saturate(rgb);
}

// Convert to linear-ish RGB for the engine lighting pipeline.
vec3 blackbody_rgb_linear(float temp_k)
{
    vec3 srgb = blackbody_rgb_srgb(temp_k);
    return pow(srgb, vec3(2.2));
}

// Object-space triplanar sampling of a 2D noise texture.
// - obj_pos/object_normal are in mesh-local space.
// - scale controls noise frequency in object space.
// - scroll_velocity is applied as a UV offset per second.
float triplanar_noise(sampler2D noise_tex,
                      vec3 obj_pos,
                      vec3 obj_normal,
                      float scale,
                      vec2 scroll_velocity,
                      float time_seconds)
{
    vec3 p = obj_pos * scale;
    vec3 n = obj_normal;
    if (dot(n, n) < 1e-8)
    {
        n = vec3(0.0, 1.0, 0.0);
    }
    n = normalize(n);

    vec3 w = abs(n);
    float wsum = w.x + w.y + w.z;
    w = (wsum > 0.0) ? (w / wsum) : vec3(0.0, 1.0, 0.0);

    vec2 s = scroll_velocity * time_seconds;
    float nx = texture(noise_tex, p.yz + s).r;
    float ny = texture(noise_tex, p.xz + s).r;
    float nz = texture(noise_tex, p.xy + s).r;

    return nx * w.x + ny * w.y + nz * w.z;
}

vec3 safe_normalize(vec3 v, vec3 fallback)
{
    float len2 = dot(v, v);
    return (len2 > 1e-8) ? (v * inversesqrt(len2)) : fallback;
}

// Grouped parameters for nozzle_blackbody_heat() to keep call sites tidy.
struct BlackbodyHeatParams
{
    float noise_scale;
    float noise_contrast;
    vec2  scroll_velocity;
    float noise_speed;
    vec3  heat_axis_local;
    float hot_end_bias;
    float hot_range_start;
    float hot_range_end;
};

// Returns [0,1] heat factor tuned for elongated hot-metal effects (nozzle/barrel).
// - p.heat_axis_local: local-space axial direction of the nozzle/barrel.
// - p.hot_end_bias: -1 = hot near -axis end, +1 = hot near +axis end, 0 = both ends.
// - p.hot_range_start/end: normalized axial smoothstep range ([0,1], start < end).
float nozzle_blackbody_heat(sampler2D noise_tex,
                            vec3 obj_pos,
                            vec3 obj_normal,
                            BlackbodyHeatParams p,
                            float time_seconds)
{
    float noise_scale = p.noise_scale;
    float noise_contrast = p.noise_contrast;
    vec2  scroll_velocity = p.scroll_velocity;
    float noise_speed = p.noise_speed;
    vec3  heat_axis_local = p.heat_axis_local;
    float hot_end_bias = p.hot_end_bias;
    float hot_range_start = p.hot_range_start;
    float hot_range_end = p.hot_range_end;

    vec3 axis = safe_normalize(heat_axis_local, vec3(0.0, 1.0, 0.0));
    float bias = clamp(hot_end_bias, -1.0, 1.0);
    if (dot(heat_axis_local, heat_axis_local) < 1e-8 && abs(bias) < 1e-5)
    {
        // Backward-compatible fallback for materials authored before axis/bias existed.
        bias = 1.0;
    }

    float axial = dot(obj_pos, axis);
    vec3 radial_vec = obj_pos - axis * axial;
    float radial = length(radial_vec);

    float axis_extent = max(max(abs(obj_pos.x), abs(obj_pos.y)), abs(obj_pos.z));
    axis_extent = max(axis_extent, 1e-4);
    float axial_signed = clamp(axial / axis_extent, -1.0, 1.0);
    float axial01 = axial_signed * 0.5 + 0.5;

    float range_start = clamp(hot_range_start, 0.0, 1.0);
    float range_end = clamp(hot_range_end, 0.0, 1.0);
    if (range_end <= range_start + 1e-4)
    {
        // Backward-compatible fallback for materials authored before hot-range existed.
        range_start = 0.68;
        range_end = 0.98;
    }

    float norm = max(abs(axial) + radial, 1e-4);
    float radial01 = clamp(radial / norm, 0.0, 1.0);

    vec3 normal_n = safe_normalize(obj_normal, axis);
    float side = 1.0 - abs(dot(normal_n, axis));
    side = pow(clamp(side, 0.0, 1.0), 0.45);

    float anim_time = time_seconds * max(noise_speed, 0.0);

    const float INV_TWO_PI = 0.15915494309189535;
    float angle01 = atan(radial_vec.z, radial_vec.x) * INV_TWO_PI + 0.5;
    vec2 scroll = scroll_velocity * anim_time;

    // Break up hot-zone boundaries using axial/circumferential noise, focused near the transition edges.
    float edge_width = max((range_end - range_start) * 0.75, 0.03);
    float edge_band = max(1.0 - smoothstep(0.0, edge_width, abs(axial01 - range_start)),
                          1.0 - smoothstep(0.0, edge_width, abs(axial01 - range_end)));
    float edge_noise_u = axial * (noise_scale * 0.08) + scroll.x * 0.6;
    float edge_noise_v = angle01 * (noise_scale * 2.0) + scroll.y * 0.35;
    float edge_noise = texture(noise_tex, vec2(edge_noise_u, edge_noise_v)).r * 2.0 - 1.0;
    float edge_amp = (0.018 + 0.045 * clamp(noise_contrast, 0.0, 2.0)) * mix(0.5, 1.0, side);
    float axial_warped = clamp(axial01 + edge_noise * edge_amp * edge_band, 0.0, 1.0);

    float hot_plus = smoothstep(range_start, range_end, axial_warped);
    float hot_minus = smoothstep(range_start, range_end, 1.0 - axial_warped);
    float hot_dual = max(hot_plus, hot_minus);

    float end_mask = hot_dual;
    if (bias > 0.0)
    {
        end_mask = mix(hot_dual, hot_plus, bias);
    }
    else if (bias < 0.0)
    {
        end_mask = mix(hot_dual, hot_minus, -bias);
    }

    float shell = smoothstep(0.10, 0.95, radial01);
    float core = 1.0 - smoothstep(0.0, 0.55, radial01);

    float base_profile = end_mask * (0.25 + 0.75 * shell);
    base_profile = mix(base_profile * 0.85, base_profile, side);
    base_profile = clamp(base_profile + end_mask * core * 0.18, 0.0, 1.0);
    base_profile = clamp(base_profile * (1.0 + edge_noise * edge_band * 0.22), 0.0, 1.0);

    float tri_noise = triplanar_noise(noise_tex,
                                      obj_pos,
                                      obj_normal,
                                      noise_scale,
                                      scroll_velocity,
                                      anim_time);

    float axis_flow = axial * (noise_scale * 0.06) - anim_time * scroll_velocity.y * 0.35;
    vec2 streak_uv = vec2(axis_flow + scroll.x * 0.4,
                          angle01 * (noise_scale * 1.1));
    float streak_noise = texture(noise_tex, streak_uv).r;

    float n = mix(tri_noise, streak_noise, 0.68);
    float n_centered = n * 2.0 - 1.0;

    float contrast = max(noise_contrast, 0.001);
    float turbulence = exp2(n_centered * (0.45 + 0.55 * contrast));

    return clamp(base_profile * turbulence, 0.0, 1.0);
}

// Evaluate blackbody emissive from material extra[] slots.
// Shared by gbuffer.frag and mesh.frag to avoid code duplication.
// Requires: materialData.extra[], sceneData.timeParams, emissiveTex,
//           inObjectPos, inObjectNormal, inUV.
vec3 evaluate_blackbody_emissive(sampler2D emissive_tex,
                                 vec4 extra9, vec4 extra10, vec4 extra11,
                                 vec4 extra12, vec4 extra13,
                                 vec3 obj_pos, vec3 obj_normal,
                                 vec2 uv, float time)
{
    bool bb_enabled = (extra9.x > 0.5);
    if (!bb_enabled)
    {
        return vec3(0.0);
    }

    float intensity = max(extra9.y, 0.0);
    float tempMinK  = extra9.z;
    float tempMaxK  = extra9.w;

    if (intensity <= 0.0 || tempMaxK <= tempMinK)
    {
        return vec3(0.0);
    }

    BlackbodyHeatParams hp;
    hp.noise_scale    = max(extra10.x, 0.001);
    hp.noise_contrast = max(extra10.y, 0.001);
    hp.scroll_velocity = extra10.zw;
    hp.noise_speed    = max(extra12.x, 0.0);
    hp.heat_axis_local = extra11.xyz;
    hp.hot_end_bias   = extra11.w;
    hp.hot_range_start = extra13.x;
    hp.hot_range_end   = extra13.y;

    float t = nozzle_blackbody_heat(emissive_tex, obj_pos, obj_normal, hp, time);
    float tempK = mix(tempMinK, tempMaxK, t);
    vec3 bb = blackbody_rgb_linear(tempK);
    return bb * intensity * t;
}

#endif // VKE_BLACKBODY_GLSL
