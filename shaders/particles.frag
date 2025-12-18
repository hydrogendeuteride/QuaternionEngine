#version 450

layout(set = 0, binding = 0) uniform SceneData
{
    mat4 view;
    mat4 proj;
    mat4 viewproj;
} sceneData;

layout(location = 0) in vec4 v_color;
layout(location = 1) in vec2 v_uv;
layout(location = 2) in float v_view_depth;
layout(location = 3) in float v_seed;

layout(set = 2, binding = 0) uniform sampler2D gbufPosTex;
layout(set = 3, binding = 0) uniform sampler2D flipbookTex;
layout(set = 3, binding = 1) uniform sampler2D noiseTex;

layout(push_constant) uniform ParticlePush
{
    vec4 screen;   // x=invW, y=invH, z=softDepthDistance, w=timeSeconds
    vec4 flipbook; // x=cols, y=rows, z=fps, w=intensity
    vec4 noise;    // x=scale, y=strength, z=scrollX, w=scrollY
} pc;

layout(location = 0) out vec4 outColor;

void main()
{
    // Soft circular sprite.
    vec2 p = v_uv * 2.0 - 1.0;
    float r = length(p);
    float mask = smoothstep(1.0, 0.0, r);

    vec4 c = v_color;
    c.rgb *= mask;
    c.a *= mask;

    // Flipbook sampling + noise UV distortion (atlas).
    float cols = max(pc.flipbook.x, 1.0);
    float rows = max(pc.flipbook.y, 1.0);
    float frames = cols * rows;
    float fps = max(pc.flipbook.z, 0.0);
    float intensity = max(pc.flipbook.w, 0.0);

    vec2 uv = v_uv;
    float noiseScale = max(pc.noise.x, 0.0);
    float noiseStrength = max(pc.noise.y, 0.0);
    if (noiseScale > 0.0 && noiseStrength > 0.0)
    {
        vec2 nUV = uv * noiseScale + pc.screen.w * pc.noise.zw;
        vec2 n = texture(noiseTex, nUV).rg * 2.0 - 1.0;
        vec2 cell = vec2(1.0 / cols, 1.0 / rows);
        uv = clamp(uv + n * noiseStrength * cell, 0.0, 1.0);
    }

    uint frame = 0u;
    if (frames > 0.5 && fps > 0.0)
    {
        float ff = pc.screen.w * fps + v_seed * frames;
        frame = uint(ff) % uint(frames);
    }
    uint cols_u = uint(cols);
    uint rows_u = uint(rows);
    uint fx = frame % cols_u;
    // Flipbook sheets are usually laid out with row 0 at the top.
    uint fy = (rows_u > 0u) ? (rows_u - 1u - (frame / cols_u)) : 0u;

    vec2 cell = vec2(1.0 / cols, 1.0 / rows);
    vec2 atlas_uv = uv * cell + vec2(float(fx), float(fy)) * cell;

    vec3 flip = texture(flipbookTex, atlas_uv).rgb;
    // BC6H has no alpha; approximate mask from luminance.
    float flip_a = clamp(dot(flip, vec3(0.2126, 0.7152, 0.0722)), 0.0, 1.0);

    c.rgb *= flip * intensity;
    c.a *= flip_a;

    // Soft particles: fade out near opaque geometry intersections.
    float soft = 1.0;
    float softDist = pc.screen.z;
    if (softDist > 0.0)
    {
        vec2 suv = gl_FragCoord.xy * pc.screen.xy;
        vec4 scenePos = texture(gbufPosTex, suv);
        if (scenePos.w > 0.0)
        {
            float sceneDepth = -(sceneData.view * vec4(scenePos.xyz, 1.0)).z;
            float delta = sceneDepth - v_view_depth;
            soft = clamp(delta / softDist, 0.0, 1.0);
        }
    }
    c.rgb *= soft;
    c.a *= soft;

    if (c.a <= 0.001)
    {
        discard;
    }

    outColor = c;
}
