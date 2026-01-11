#version 450

layout(set = 0, binding = 0) uniform SceneData
{
    mat4 view;
    mat4 proj;
    mat4 viewproj;
} sceneData;

struct Particle
{
    vec4 pos_age;
    vec4 vel_life;
    vec4 color;
    vec4 misc;
};

layout(std430, set = 1, binding = 0) readonly buffer ParticlePool
{
    Particle particles[];
} pool;

layout(std430, set = 1, binding = 1) readonly buffer DrawIndices
{
    uint indices[];
} drawIndices;

layout(location = 0) out vec4 v_color;
layout(location = 1) out vec2 v_uv;
layout(location = 2) out float v_view_depth;
layout(location = 3) out float v_seed;

vec2 quad_corner(uint vidx)
{
    // Two triangles (6 verts) in a unit quad centered at origin.
    const vec2 corners[6] = vec2[6](
    vec2(-0.5, -0.5),
    vec2(0.5, -0.5),
    vec2(0.5, 0.5),
    vec2(-0.5, -0.5),
    vec2(0.5, 0.5),
    vec2(-0.5, 0.5)
    );
    return corners[vidx % 6u];
}

void main()
{
    uint particle_index = drawIndices.indices[uint(gl_InstanceIndex)];
    Particle p = pool.particles[particle_index];

    float life = max(p.vel_life.w, 1e-6);
    float remaining = clamp(p.pos_age.w, 0.0, life);
    float t = remaining / life;// remaining fraction

    float fade_out = smoothstep(0.0, 0.15, t);
    float fade_in  = smoothstep(0.0, 0.05, 1.0 - t);
    float fade = fade_in * fade_out;

    vec2 corner = quad_corner(uint(gl_VertexIndex));
    v_uv = corner + vec2(0.5);

    // Camera right/up in world-local space from view matrix rows.
    vec3 cam_right = vec3(sceneData.view[0][0], sceneData.view[1][0], sceneData.view[2][0]);
    vec3 cam_up    = vec3(sceneData.view[0][1], sceneData.view[1][1], sceneData.view[2][1]);

    float size = max(p.misc.x, 0.0);
    vec3 pos = p.pos_age.xyz + (cam_right * corner.x + cam_up * corner.y) * size;

    v_color = vec4(p.color.rgb * fade, p.color.a * fade);
    v_view_depth = -(sceneData.view * vec4(pos, 1.0)).z;
    v_seed = p.misc.y;
    gl_Position = sceneData.viewproj * vec4(pos, 1.0);
}
