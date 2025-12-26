#ifndef IBL_COMMON_GLSL
#define IBL_COMMON_GLSL

// IBL bindings (set=3): specular equirect 2D, BRDF LUT, SH UBO, optional background map.
layout(set=3, binding=0) uniform sampler2D iblSpec2D;
layout(set=3, binding=1) uniform sampler2D iblBRDF;
layout(std140, set=3, binding=2) uniform IBL_SH { vec4 sh[9]; } iblSH;
layout(set=3, binding=3) uniform sampler2D iblBackground2D;

// Evaluate diffuse irradiance from 2nd-order SH coefficients (9 coeffs).
// Coefficients are pre-convolved with the Lambert kernel on the CPU.
vec3 sh_eval_irradiance(vec3 n)
{
    float x = n.x, y = n.y, z = n.z;
    const float c0 = 0.2820947918;
    const float c1 = 0.4886025119;
    const float c2 = 1.0925484306;
    const float c3 = 0.3153915653;
    const float c4 = 0.5462742153;

    float x2 = x * x;
    float y2 = y * y;
    float z2 = z * z;

    vec3 r = vec3(0.0);
    r += iblSH.sh[0].rgb * c0;
    r += iblSH.sh[1].rgb * (c1 * y);
    r += iblSH.sh[2].rgb * (c1 * z);
    r += iblSH.sh[3].rgb * (c1 * x);
    r += iblSH.sh[4].rgb * (c2 * x * y);
    r += iblSH.sh[5].rgb * (c2 * y * z);
    r += iblSH.sh[6].rgb * (c3 * (3.0 * z2 - 1.0));
    r += iblSH.sh[7].rgb * (c2 * x * z);
    r += iblSH.sh[8].rgb * (c4 * (x2 - y2));
    return r;
}

// Map direction to equirectangular UV (same convention across shaders).
vec2 dir_to_equirect_normalized(vec3 d)
{
    float phi = atan(d.z, d.x);
    float theta = acos(clamp(d.y, -1.0, 1.0));
    // 1/(2*pi) = 0.15915494309, 1/pi = 0.31830988618
    return vec2(phi * 0.15915494309 + 0.5, theta * 0.31830988618);
}

vec2 dir_to_equirect(vec3 d)
{
    return dir_to_equirect_normalized(normalize(d));
}

// Helper for selecting mip LOD from roughness and available levels.
// Uses roughness^2 to bias towards blurrier reflections at mid roughness.
float ibl_lod_from_roughness(float roughness, float levels)
{
    float maxLevel = max(levels - 1.0, 0.0);
    float r = clamp(roughness, 0.0, 1.0);
    return r * r * maxLevel;
}

#endif // IBL_COMMON_GLSL
