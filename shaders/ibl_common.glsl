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
    float Y[9];
    Y[0] = c0;
    Y[1] = c1 * y;
    Y[2] = c1 * z;
    Y[3] = c1 * x;
    Y[4] = c2 * x * y;
    Y[5] = c2 * y * z;
    Y[6] = c3 * (3.0 * z * z - 1.0);
    Y[7] = c2 * x * z;
    Y[8] = c4 * (x * x - y * y);
    vec3 r = vec3(0.0);
    for (int i = 0; i < 9; ++i)
    {
        r += iblSH.sh[i].rgb * Y[i];
    }
    return r;
}

// Map direction to equirectangular UV (same convention across shaders).
vec2 dir_to_equirect(vec3 d)
{
    d = normalize(d);
    float phi = atan(d.z, d.x);
    float theta = acos(clamp(d.y, -1.0, 1.0));
    // 1/(2*pi) = 0.15915494309, 1/pi = 0.31830988618
    return vec2(phi * 0.15915494309 + 0.5, theta * 0.31830988618);
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

