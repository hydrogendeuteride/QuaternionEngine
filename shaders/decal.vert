#version 450

layout(set = 0, binding = 0) uniform SceneData
{
    mat4 view;
    mat4 proj;
    mat4 viewproj;
} sceneData;

layout(push_constant) uniform Push
{
    vec4 axis_x;
    vec4 axis_y;
    vec4 axis_z;
    vec4 center_extent_x;
    vec4 extent_yz_shape_opacity;
    vec4 tint_normal;
} pc;

vec3 cube_vertex(uint index)
{
    const vec3 v[36] = vec3[36](
        // +Z
        vec3(-1.0, -1.0,  1.0), vec3( 1.0, -1.0,  1.0), vec3( 1.0,  1.0,  1.0),
        vec3(-1.0, -1.0,  1.0), vec3( 1.0,  1.0,  1.0), vec3(-1.0,  1.0,  1.0),
        // -Z
        vec3( 1.0, -1.0, -1.0), vec3(-1.0, -1.0, -1.0), vec3(-1.0,  1.0, -1.0),
        vec3( 1.0, -1.0, -1.0), vec3(-1.0,  1.0, -1.0), vec3( 1.0,  1.0, -1.0),
        // +X
        vec3( 1.0, -1.0,  1.0), vec3( 1.0, -1.0, -1.0), vec3( 1.0,  1.0, -1.0),
        vec3( 1.0, -1.0,  1.0), vec3( 1.0,  1.0, -1.0), vec3( 1.0,  1.0,  1.0),
        // -X
        vec3(-1.0, -1.0, -1.0), vec3(-1.0, -1.0,  1.0), vec3(-1.0,  1.0,  1.0),
        vec3(-1.0, -1.0, -1.0), vec3(-1.0,  1.0,  1.0), vec3(-1.0,  1.0, -1.0),
        // +Y
        vec3(-1.0,  1.0,  1.0), vec3( 1.0,  1.0,  1.0), vec3( 1.0,  1.0, -1.0),
        vec3(-1.0,  1.0,  1.0), vec3( 1.0,  1.0, -1.0), vec3(-1.0,  1.0, -1.0),
        // -Y
        vec3(-1.0, -1.0, -1.0), vec3( 1.0, -1.0, -1.0), vec3( 1.0, -1.0,  1.0),
        vec3(-1.0, -1.0, -1.0), vec3( 1.0, -1.0,  1.0), vec3(-1.0, -1.0,  1.0)
    );
    return v[index];
}

void main()
{
    int mode = int(pc.extent_yz_shape_opacity.z + 0.5);
    bool fullscreen = (mode >= 2);
    if (fullscreen)
    {
        // Fullscreen triangle fallback when camera is inside the decal volume.
        const vec2 p[3] = vec2[3](
            vec2(-1.0, -1.0),
            vec2( 3.0, -1.0),
            vec2(-1.0,  3.0)
        );
        gl_Position = vec4(p[gl_VertexIndex], 0.0, 1.0);
        return;
    }

    vec3 localPos = cube_vertex(uint(gl_VertexIndex));
    vec3 center = pc.center_extent_x.xyz;
    vec3 halfExtents = vec3(pc.center_extent_x.w, pc.extent_yz_shape_opacity.x, pc.extent_yz_shape_opacity.y);

    vec3 worldPos = center
                  + pc.axis_x.xyz * (localPos.x * halfExtents.x)
                  + pc.axis_y.xyz * (localPos.y * halfExtents.y)
                  + pc.axis_z.xyz * (localPos.z * halfExtents.z);

    gl_Position = sceneData.viewproj * vec4(worldPos, 1.0);
}
