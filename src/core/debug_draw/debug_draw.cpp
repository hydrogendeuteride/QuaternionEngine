#include "debug_draw.h"

#include <algorithm>
#include <cmath>

namespace
{
    static float clamp_nonnegative_finite(float v, float fallback = 0.0f)
    {
        if (!std::isfinite(v)) return fallback;
        return std::max(0.0f, v);
    }

    static float ttl_from_seconds(float seconds)
    {
        if (!std::isfinite(seconds) || seconds <= 0.0f)
        {
            return -1.0f; // one-frame
        }
        return seconds;
    }

    static int clamp_segments(int segments)
    {
        if (segments < 3) return 3;
        if (segments > 256) return 256;
        return segments;
    }

    static glm::vec3 safe_normalize(const glm::vec3 &v, const glm::vec3 &fallback)
    {
        const float len2 = glm::dot(v, v);
        if (!std::isfinite(len2) || len2 <= 1.0e-12f) return fallback;
        return v * (1.0f / std::sqrt(len2));
    }

    static void basis_from_normal(const glm::vec3 &n, glm::vec3 &out_u, glm::vec3 &out_v)
    {
        const glm::vec3 nn = safe_normalize(n, glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::vec3 a = (std::abs(nn.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        out_u = safe_normalize(glm::cross(nn, a), glm::vec3(1.0f, 0.0f, 0.0f));
        out_v = safe_normalize(glm::cross(nn, out_u), glm::vec3(0.0f, 0.0f, 1.0f));
    }

    static void push_line(std::vector<DebugDrawVertex> &dst,
                          const glm::vec3 &a,
                          const glm::vec3 &b,
                          const glm::vec4 &color)
    {
        DebugDrawVertex v0{};
        v0.position = a;
        v0.color = color;
        DebugDrawVertex v1{};
        v1.position = b;
        v1.color = color;
        dst.push_back(v0);
        dst.push_back(v1);
    }

    static bool layer_enabled(uint32_t layer_mask, DebugDrawLayer layer)
    {
        return (layer_mask & static_cast<uint32_t>(layer)) != 0u;
    }

    template <typename CmdT>
    static std::vector<DebugDrawVertex> *select_bucket(const CmdT &cmd,
                                                       const DebugDrawSystem::Settings &settings,
                                                       std::vector<DebugDrawVertex> &depth_vertices,
                                                       std::vector<DebugDrawVertex> &overlay_vertices)
    {
        if (!layer_enabled(settings.layer_mask, cmd.layer))
        {
            return nullptr;
        }
        if (cmd.depth == DebugDepth::DepthTested)
        {
            if (!settings.show_depth_tested) return nullptr;
            return &depth_vertices;
        }
        if (!settings.show_overlay) return nullptr;
        return &overlay_vertices;
    }

    static void emit_aabb(std::vector<DebugDrawVertex> &dst,
                          const glm::vec3 &center_local,
                          const glm::vec3 &half_extents,
                          const glm::vec4 &color)
    {
        glm::vec3 e = half_extents;
        e.x = std::max(0.0f, e.x);
        e.y = std::max(0.0f, e.y);
        e.z = std::max(0.0f, e.z);

        const glm::vec3 c = center_local;
        const glm::vec3 corners[8] = {
            c + glm::vec3(-e.x, -e.y, -e.z),
            c + glm::vec3(+e.x, -e.y, -e.z),
            c + glm::vec3(-e.x, +e.y, -e.z),
            c + glm::vec3(+e.x, +e.y, -e.z),
            c + glm::vec3(-e.x, -e.y, +e.z),
            c + glm::vec3(+e.x, -e.y, +e.z),
            c + glm::vec3(-e.x, +e.y, +e.z),
            c + glm::vec3(+e.x, +e.y, +e.z),
        };

        constexpr uint8_t edges[12][2] = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7},
        };

        for (auto &eidx : edges)
        {
            push_line(dst, corners[eidx[0]], corners[eidx[1]], color);
        }
    }

    static void emit_obb(std::vector<DebugDrawVertex> &dst,
                         const std::array<glm::vec3, 8> &corners_local,
                         const glm::vec4 &color)
    {
        constexpr uint8_t edges[12][2] = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7},
        };

        for (auto &eidx : edges)
        {
            push_line(dst, corners_local[eidx[0]], corners_local[eidx[1]], color);
        }
    }

    static void emit_circle(std::vector<DebugDrawVertex> &dst,
                            const glm::vec3 &center_local,
                            const glm::vec3 &normal,
                            float radius,
                            int segments,
                            const glm::vec4 &color)
    {
        radius = clamp_nonnegative_finite(radius, 0.0f);
        if (radius <= 0.0f) return;

        glm::vec3 u{}, v{};
        basis_from_normal(normal, u, v);

        const int seg = clamp_segments(segments);
        const float two_pi = 6.2831853071795864769f;

        glm::vec3 prev{};
        for (int i = 0; i <= seg; ++i)
        {
            const float t = (static_cast<float>(i) / static_cast<float>(seg)) * two_pi;
            const glm::vec3 p = center_local + (u * std::cos(t) + v * std::sin(t)) * radius;
            if (i > 0)
            {
                push_line(dst, prev, p, color);
            }
            prev = p;
        }
    }

    static void emit_sphere(std::vector<DebugDrawVertex> &dst,
                            const glm::vec3 &center_local,
                            float radius,
                            int segments,
                            const glm::vec4 &color)
    {
        radius = clamp_nonnegative_finite(radius, 0.0f);
        if (radius <= 0.0f) return;

        // 3 great circles
        emit_circle(dst, center_local, glm::vec3(0.0f, 0.0f, 1.0f), radius, segments, color); // XY
        emit_circle(dst, center_local, glm::vec3(0.0f, 1.0f, 0.0f), radius, segments, color); // XZ
        emit_circle(dst, center_local, glm::vec3(1.0f, 0.0f, 0.0f), radius, segments, color); // YZ
    }

    static void emit_cone(std::vector<DebugDrawVertex> &dst,
                          const glm::vec3 &apex_local,
                          const glm::vec3 &direction_local,
                          float length,
                          float angle_degrees,
                          int segments,
                          const glm::vec4 &color)
    {
        length = clamp_nonnegative_finite(length, 0.0f);
        if (length <= 0.0f) return;

        float angle = angle_degrees;
        if (!std::isfinite(angle)) angle = 0.0f;
        angle = std::clamp(angle, 0.0f, 89.9f);
        const float radius = length * std::tan(glm::radians(angle));

        const glm::vec3 dir = safe_normalize(direction_local, glm::vec3(0.0f, -1.0f, 0.0f));
        const glm::vec3 base_center = apex_local + dir * length;

        // Axis
        push_line(dst, apex_local, base_center, color);

        // Base circle + spokes
        glm::vec3 u{}, v{};
        basis_from_normal(dir, u, v);
        const int seg = clamp_segments(segments);
        const float two_pi = 6.2831853071795864769f;

        glm::vec3 first{};
        glm::vec3 prev{};
        for (int i = 0; i < seg; ++i)
        {
            const float t = (static_cast<float>(i) / static_cast<float>(seg)) * two_pi;
            const glm::vec3 p = base_center + (u * std::cos(t) + v * std::sin(t)) * radius;
            if (i == 0) first = p;
            if (i > 0)
            {
                push_line(dst, prev, p, color);
            }
            push_line(dst, apex_local, p, color);
            prev = p;
        }
        push_line(dst, prev, first, color);
    }

    static void emit_capsule(std::vector<DebugDrawVertex> &dst,
                             const glm::vec3 &p0_local,
                             const glm::vec3 &p1_local,
                             float radius,
                             int segments,
                             const glm::vec4 &color)
    {
        radius = clamp_nonnegative_finite(radius, 0.0f);
        if (radius <= 0.0f) return;

        const glm::vec3 axis = p1_local - p0_local;
        const float axis_len2 = glm::dot(axis, axis);
        if (!std::isfinite(axis_len2) || axis_len2 <= 1.0e-10f)
        {
            emit_sphere(dst, p0_local, radius, segments, color);
            return;
        }

        const float axis_len = std::sqrt(axis_len2);
        const glm::vec3 u = axis * (1.0f / axis_len);

        // Basis around the capsule axis
        const glm::vec3 a = (std::abs(u.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 v = safe_normalize(glm::cross(u, a), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::vec3 w = safe_normalize(glm::cross(u, v), glm::vec3(0.0f, 0.0f, 1.0f));

        const int seg = clamp_segments(segments);
        const float two_pi = 6.2831853071795864769f;

        // Rings + side lines
        glm::vec3 prev0{}, prev1{}, first0{}, first1{};
        for (int i = 0; i < seg; ++i)
        {
            const float t = (static_cast<float>(i) / static_cast<float>(seg)) * two_pi;
            const glm::vec3 offset = (v * std::cos(t) + w * std::sin(t)) * radius;
            const glm::vec3 a0 = p0_local + offset;
            const glm::vec3 a1 = p1_local + offset;

            if (i == 0)
            {
                first0 = a0;
                first1 = a1;
            }
            else
            {
                push_line(dst, prev0, a0, color);
                push_line(dst, prev1, a1, color);
            }
            push_line(dst, a0, a1, color);
            prev0 = a0;
            prev1 = a1;
        }
        push_line(dst, prev0, first0, color);
        push_line(dst, prev1, first1, color);

        // Endcap arcs (2 meridians per end)
        const int half_seg = std::max(3, seg / 2);
        for (int i = 0; i < half_seg; ++i)
        {
            const float t0 = (static_cast<float>(i) / static_cast<float>(half_seg)) * 3.14159265358979323846f;
            const float t1 = (static_cast<float>(i + 1) / static_cast<float>(half_seg)) * 3.14159265358979323846f;

            // p0 hemisphere faces -u
            const glm::vec3 p0_v0 = p0_local + (v * std::cos(t0) - u * std::sin(t0)) * radius;
            const glm::vec3 p0_v1 = p0_local + (v * std::cos(t1) - u * std::sin(t1)) * radius;
            const glm::vec3 p0_w0 = p0_local + (w * std::cos(t0) - u * std::sin(t0)) * radius;
            const glm::vec3 p0_w1 = p0_local + (w * std::cos(t1) - u * std::sin(t1)) * radius;
            push_line(dst, p0_v0, p0_v1, color);
            push_line(dst, p0_w0, p0_w1, color);

            // p1 hemisphere faces +u
            const glm::vec3 p1_v0 = p1_local + (v * std::cos(t0) + u * std::sin(t0)) * radius;
            const glm::vec3 p1_v1 = p1_local + (v * std::cos(t1) + u * std::sin(t1)) * radius;
            const glm::vec3 p1_w0 = p1_local + (w * std::cos(t0) + u * std::sin(t0)) * radius;
            const glm::vec3 p1_w1 = p1_local + (w * std::cos(t1) + u * std::sin(t1)) * radius;
            push_line(dst, p1_v0, p1_v1, color);
            push_line(dst, p1_w0, p1_w1, color);
        }
    }
}

size_t DebugDrawSystem::command_count() const
{
    return _lines.size() + _aabbs.size() + _spheres.size() + _capsules.size() + _circles.size() + _cones.size() + _obbs.size();
}

void DebugDrawSystem::clear()
{
    _lines.clear();
    _aabbs.clear();
    _spheres.clear();
    _capsules.clear();
    _circles.clear();
    _cones.clear();
    _obbs.clear();
}

template <typename T>
void DebugDrawSystem::prune_list(std::vector<T> &cmds, float dt_seconds)
{
    if (cmds.empty())
    {
        return;
    }

    if (!std::isfinite(dt_seconds) || dt_seconds < 0.0f)
    {
        dt_seconds = 0.0f;
    }

    size_t dst = 0;
    for (size_t i = 0; i < cmds.size(); ++i)
    {
        T cmd = cmds[i];

        // one-frame commands are removed on begin_frame()
        if (cmd.ttl_seconds < 0.0f)
        {
            continue;
        }

        if (dt_seconds > 0.0f)
        {
            cmd.ttl_seconds -= dt_seconds;
        }

        if (cmd.ttl_seconds <= 0.0f)
        {
            continue;
        }

        cmds[dst++] = cmd;
    }
    cmds.resize(dst);
}

void DebugDrawSystem::begin_frame(float dt_seconds)
{
    prune_list(_lines, dt_seconds);
    prune_list(_aabbs, dt_seconds);
    prune_list(_spheres, dt_seconds);
    prune_list(_capsules, dt_seconds);
    prune_list(_circles, dt_seconds);
    prune_list(_cones, dt_seconds);
    prune_list(_obbs, dt_seconds);
}

void DebugDrawSystem::add_line(const WorldVec3 &a_world,
                               const WorldVec3 &b_world,
                               const glm::vec4 &color,
                               float seconds,
                               DebugDepth depth,
                               DebugDrawLayer layer)
{
    CmdLine cmd{};
    cmd.a_world = a_world;
    cmd.b_world = b_world;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _lines.push_back(cmd);
}

void DebugDrawSystem::add_ray(const WorldVec3 &origin_world,
                              const glm::dvec3 &dir_world,
                              double length,
                              const glm::vec4 &color,
                              float seconds,
                              DebugDepth depth,
                              DebugDrawLayer layer)
{
    if (!std::isfinite(length) || length <= 0.0)
    {
        return;
    }

    glm::dvec3 d = dir_world;
    const double len2 = glm::dot(d, d);
    if (!std::isfinite(len2) || len2 <= 1.0e-18)
    {
        d = glm::dvec3(0.0, 1.0, 0.0);
    }
    else
    {
        d *= 1.0 / std::sqrt(len2);
    }

    add_line(origin_world,
             origin_world + d * length,
             color,
             seconds,
             depth,
             layer);
}

void DebugDrawSystem::add_aabb(const WorldVec3 &center_world,
                               const glm::vec3 &half_extents,
                               const glm::vec4 &color,
                               float seconds,
                               DebugDepth depth,
                               DebugDrawLayer layer)
{
    CmdAabb cmd{};
    cmd.center_world = center_world;
    cmd.half_extents = half_extents;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _aabbs.push_back(cmd);
}

void DebugDrawSystem::add_sphere(const WorldVec3 &center_world,
                                 float radius,
                                 const glm::vec4 &color,
                                 float seconds,
                                 DebugDepth depth,
                                 DebugDrawLayer layer)
{
    CmdSphere cmd{};
    cmd.center_world = center_world;
    cmd.radius = radius;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _spheres.push_back(cmd);
}

void DebugDrawSystem::add_capsule(const WorldVec3 &p0_world,
                                  const WorldVec3 &p1_world,
                                  float radius,
                                  const glm::vec4 &color,
                                  float seconds,
                                  DebugDepth depth,
                                  DebugDrawLayer layer)
{
    CmdCapsule cmd{};
    cmd.p0_world = p0_world;
    cmd.p1_world = p1_world;
    cmd.radius = radius;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _capsules.push_back(cmd);
}

void DebugDrawSystem::add_circle(const WorldVec3 &center_world,
                                 const glm::dvec3 &normal_world,
                                 float radius,
                                 const glm::vec4 &color,
                                 float seconds,
                                 DebugDepth depth,
                                 DebugDrawLayer layer)
{
    CmdCircle cmd{};
    cmd.center_world = center_world;
    cmd.normal_world = normal_world;
    cmd.radius = radius;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _circles.push_back(cmd);
}

void DebugDrawSystem::add_cone(const WorldVec3 &apex_world,
                               const glm::dvec3 &direction_world,
                               float length,
                               float angle_degrees,
                               const glm::vec4 &color,
                               float seconds,
                               DebugDepth depth,
                               DebugDrawLayer layer)
{
    CmdCone cmd{};
    cmd.apex_world = apex_world;
    cmd.direction_world = direction_world;
    cmd.length = length;
    cmd.angle_degrees = angle_degrees;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _cones.push_back(cmd);
}

void DebugDrawSystem::add_obb(const WorldVec3 &center_world,
                              const glm::quat &rotation,
                              const glm::vec3 &half_extents,
                              const glm::vec4 &color,
                              float seconds,
                              DebugDepth depth,
                              DebugDrawLayer layer)
{
    const glm::vec3 e = glm::max(half_extents, glm::vec3(0.0f));
    const glm::vec3 corners_local[8] = {
        glm::vec3(-e.x, -e.y, -e.z),
        glm::vec3(+e.x, -e.y, -e.z),
        glm::vec3(-e.x, +e.y, -e.z),
        glm::vec3(+e.x, +e.y, -e.z),
        glm::vec3(-e.x, -e.y, +e.z),
        glm::vec3(+e.x, -e.y, +e.z),
        glm::vec3(-e.x, +e.y, +e.z),
        glm::vec3(+e.x, +e.y, +e.z),
    };

    std::array<WorldVec3, 8> corners_world{};
    for (int i = 0; i < 8; ++i)
    {
        const glm::vec3 rotated = rotation * corners_local[i];
        corners_world[i] = center_world + WorldVec3(rotated);
    }

    add_obb_corners(corners_world, color, seconds, depth, layer);
}

void DebugDrawSystem::add_obb_corners(const std::array<WorldVec3, 8> &corners_world,
                                      const glm::vec4 &color,
                                      float seconds,
                                      DebugDepth depth,
                                      DebugDrawLayer layer)
{
    CmdObb cmd{};
    cmd.corners_world = corners_world;
    cmd.color = color;
    cmd.depth = depth;
    cmd.layer = layer;
    cmd.ttl_seconds = ttl_from_seconds(seconds);
    _obbs.push_back(cmd);
}

void DebugDrawSystem::add_cylinder(const WorldVec3 &center_world,
                                   const glm::dvec3 &axis_world,
                                   float radius,
                                   float half_height,
                                   const glm::vec4 &color,
                                   float seconds,
                                   DebugDepth depth,
                                   DebugDrawLayer layer)
{
    radius = clamp_nonnegative_finite(radius, 0.0f);
    half_height = clamp_nonnegative_finite(half_height, 0.0f);
    if (radius <= 0.0f)
    {
        return;
    }

    const glm::vec3 axis_n = safe_normalize(glm::vec3(axis_world), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::dvec3 axis_nd = glm::dvec3(axis_n);
    const WorldVec3 top_world = center_world + axis_nd * static_cast<double>(half_height);
    const WorldVec3 bot_world = center_world - axis_nd * static_cast<double>(half_height);

    add_circle(top_world, axis_nd, radius, color, seconds, depth, layer);
    add_circle(bot_world, axis_nd, radius, color, seconds, depth, layer);

    glm::vec3 u{}, v{};
    basis_from_normal(axis_n, u, v);

    const glm::vec3 dirs[4] = {u, -u, v, -v};
    for (const glm::vec3 &d: dirs)
    {
        const WorldVec3 p0 = bot_world + WorldVec3(d * radius);
        const WorldVec3 p1 = top_world + WorldVec3(d * radius);
        add_line(p0, p1, color, seconds, depth, layer);
    }
}

void DebugDrawSystem::add_tapered_cylinder(const WorldVec3 &center_world,
                                          const glm::dvec3 &axis_world,
                                          float half_height,
                                          float top_radius,
                                          float bottom_radius,
                                          const glm::vec4 &color,
                                          float seconds,
                                          DebugDepth depth,
                                          DebugDrawLayer layer)
{
    half_height = clamp_nonnegative_finite(half_height, 0.0f);
    top_radius = clamp_nonnegative_finite(top_radius, 0.0f);
    bottom_radius = clamp_nonnegative_finite(bottom_radius, 0.0f);
    if (top_radius <= 0.0f && bottom_radius <= 0.0f)
    {
        return;
    }

    const glm::vec3 axis_n = safe_normalize(glm::vec3(axis_world), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::dvec3 axis_nd = glm::dvec3(axis_n);
    const WorldVec3 top_world = center_world + axis_nd * static_cast<double>(half_height);
    const WorldVec3 bot_world = center_world - axis_nd * static_cast<double>(half_height);

    if (top_radius > 0.0f)
    {
        add_circle(top_world, axis_nd, top_radius, color, seconds, depth, layer);
    }
    if (bottom_radius > 0.0f)
    {
        add_circle(bot_world, axis_nd, bottom_radius, color, seconds, depth, layer);
    }

    glm::vec3 u{}, v{};
    basis_from_normal(axis_n, u, v);

    const glm::vec3 dirs[4] = {u, -u, v, -v};
    for (const glm::vec3 &d: dirs)
    {
        const WorldVec3 p0 = bot_world + WorldVec3(d * bottom_radius);
        const WorldVec3 p1 = top_world + WorldVec3(d * top_radius);
        add_line(p0, p1, color, seconds, depth, layer);
    }
}

void DebugDrawSystem::add_plane_patch(const WorldVec3 &point_world,
                                      const glm::dvec3 &normal_world,
                                      float half_size,
                                      const glm::vec4 &color,
                                      float seconds,
                                      DebugDepth depth,
                                      DebugDrawLayer layer)
{
    half_size = clamp_nonnegative_finite(half_size, 0.0f);
    if (half_size <= 0.0f)
    {
        return;
    }

    const glm::vec3 n = safe_normalize(glm::vec3(normal_world), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 u{}, v{};
    basis_from_normal(n, u, v);

    const WorldVec3 du = WorldVec3(u * half_size);
    const WorldVec3 dv = WorldVec3(v * half_size);

    const WorldVec3 c0 = point_world + du + dv;
    const WorldVec3 c1 = point_world + du - dv;
    const WorldVec3 c2 = point_world - du - dv;
    const WorldVec3 c3 = point_world - du + dv;

    add_line(c0, c1, color, seconds, depth, layer);
    add_line(c1, c2, color, seconds, depth, layer);
    add_line(c2, c3, color, seconds, depth, layer);
    add_line(c3, c0, color, seconds, depth, layer);
}

DebugDrawSystem::LineVertexLists DebugDrawSystem::build_line_vertices(const WorldVec3 &origin_world) const
{
    LineVertexLists out{};
    if (!_settings.enabled)
    {
        return out;
    }

    std::vector<DebugDrawVertex> depth_vertices;
    std::vector<DebugDrawVertex> overlay_vertices;

    const int seg = clamp_segments(_settings.segments);

    for (const CmdLine &cmd : _lines)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        const glm::vec3 a = world_to_local(cmd.a_world, origin_world);
        const glm::vec3 b = world_to_local(cmd.b_world, origin_world);
        push_line(*dst, a, b, cmd.color);
    }

    for (const CmdAabb &cmd : _aabbs)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        const glm::vec3 c = world_to_local(cmd.center_world, origin_world);
        emit_aabb(*dst, c, cmd.half_extents, cmd.color);
    }

    for (const CmdSphere &cmd : _spheres)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        const glm::vec3 c = world_to_local(cmd.center_world, origin_world);
        emit_sphere(*dst, c, cmd.radius, seg, cmd.color);
    }

    for (const CmdCapsule &cmd : _capsules)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        const glm::vec3 p0 = world_to_local(cmd.p0_world, origin_world);
        const glm::vec3 p1 = world_to_local(cmd.p1_world, origin_world);
        emit_capsule(*dst, p0, p1, cmd.radius, seg, cmd.color);
    }

    for (const CmdCircle &cmd : _circles)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        const glm::vec3 c = world_to_local(cmd.center_world, origin_world);
        const glm::vec3 n = glm::vec3(cmd.normal_world);
        emit_circle(*dst, c, n, cmd.radius, seg, cmd.color);
    }

    for (const CmdCone &cmd : _cones)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        const glm::vec3 apex = world_to_local(cmd.apex_world, origin_world);
        const glm::vec3 dir = glm::vec3(cmd.direction_world);
        emit_cone(*dst, apex, dir, cmd.length, cmd.angle_degrees, seg, cmd.color);
    }

    for (const CmdObb &cmd : _obbs)
    {
        std::vector<DebugDrawVertex> *dst = select_bucket(cmd, _settings, depth_vertices, overlay_vertices);
        if (!dst) continue;

        std::array<glm::vec3, 8> corners_local{};
        for (size_t i = 0; i < 8; ++i)
        {
            corners_local[i] = world_to_local(cmd.corners_world[i], origin_world);
        }
        emit_obb(*dst, corners_local, cmd.color);
    }

    out.depth_vertex_count = static_cast<uint32_t>(depth_vertices.size());
    out.overlay_vertex_count = static_cast<uint32_t>(overlay_vertices.size());
    out.vertices.reserve(depth_vertices.size() + overlay_vertices.size());
    out.vertices.insert(out.vertices.end(), depth_vertices.begin(), depth_vertices.end());
    out.vertices.insert(out.vertices.end(), overlay_vertices.begin(), overlay_vertices.end());
    return out;
}
