#pragma once

#include <core/world.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <vector>

enum class DebugDepth : uint8_t
{
    DepthTested = 0,
    AlwaysOnTop = 1,
};

enum class DebugDrawLayer : uint32_t
{
    Physics = 1u << 0u,
    Picking = 1u << 1u,
    Lights = 1u << 2u,
    Particles = 1u << 3u,
    Volumetrics = 1u << 4u,
    Misc = 1u << 5u,
};

struct DebugDrawVertex
{
    glm::vec3 position{0.0f};
    float _pad0{0.0f};
    glm::vec4 color{1.0f};
};
static_assert(sizeof(DebugDrawVertex) == 32);

class DebugDrawSystem
{
public:
    struct Settings
    {
        bool enabled = false;
        bool show_depth_tested = true;
        bool show_overlay = true;
        uint32_t layer_mask = static_cast<uint32_t>(DebugDrawLayer::Physics)
                              | static_cast<uint32_t>(DebugDrawLayer::Picking)
                              | static_cast<uint32_t>(DebugDrawLayer::Lights)
                              | static_cast<uint32_t>(DebugDrawLayer::Particles)
                              | static_cast<uint32_t>(DebugDrawLayer::Volumetrics)
                              | static_cast<uint32_t>(DebugDrawLayer::Misc);
        int segments = 32;
    };

    struct LineVertexLists
    {
        std::vector<DebugDrawVertex> vertices;
        uint32_t depth_vertex_count = 0;
        uint32_t overlay_vertex_count = 0;
    };

    Settings &settings() { return _settings; }
    const Settings &settings() const { return _settings; }

    void clear();

    // Called once per frame before new submissions to expire one-frame and timed commands.
    void begin_frame(float dt_seconds);

    void add_line(const WorldVec3 &a_world,
                  const WorldVec3 &b_world,
                  const glm::vec4 &color,
                  float seconds = 0.0f,
                  DebugDepth depth = DebugDepth::DepthTested,
                  DebugDrawLayer layer = DebugDrawLayer::Physics);

    void add_ray(const WorldVec3 &origin_world,
                 const glm::dvec3 &dir_world,
                 double length,
                 const glm::vec4 &color,
                 float seconds = 0.0f,
                 DebugDepth depth = DebugDepth::DepthTested,
                 DebugDrawLayer layer = DebugDrawLayer::Physics);

    void add_aabb(const WorldVec3 &center_world,
                  const glm::vec3 &half_extents,
                  const glm::vec4 &color,
                  float seconds = 0.0f,
                  DebugDepth depth = DebugDepth::DepthTested,
                  DebugDrawLayer layer = DebugDrawLayer::Physics);

    void add_sphere(const WorldVec3 &center_world,
                    float radius,
                    const glm::vec4 &color,
                    float seconds = 0.0f,
                    DebugDepth depth = DebugDepth::DepthTested,
                    DebugDrawLayer layer = DebugDrawLayer::Physics);

    void add_capsule(const WorldVec3 &p0_world,
                     const WorldVec3 &p1_world,
                     float radius,
                     const glm::vec4 &color,
                     float seconds = 0.0f,
                     DebugDepth depth = DebugDepth::DepthTested,
                     DebugDrawLayer layer = DebugDrawLayer::Physics);

    void add_circle(const WorldVec3 &center_world,
                    const glm::dvec3 &normal_world,
                    float radius,
                    const glm::vec4 &color,
                    float seconds = 0.0f,
                    DebugDepth depth = DebugDepth::DepthTested,
                    DebugDrawLayer layer = DebugDrawLayer::Misc);

    void add_cone(const WorldVec3 &apex_world,
                  const glm::dvec3 &direction_world,
                  float length,
                  float angle_degrees,
                  const glm::vec4 &color,
                  float seconds = 0.0f,
                  DebugDepth depth = DebugDepth::DepthTested,
                  DebugDrawLayer layer = DebugDrawLayer::Misc);

    // Convenience: oriented box specified as center + rotation + half extents.
    void add_obb(const WorldVec3 &center_world,
                 const glm::quat &rotation,
                 const glm::vec3 &half_extents,
                 const glm::vec4 &color,
                 float seconds = 0.0f,
                 DebugDepth depth = DebugDepth::DepthTested,
                 DebugDrawLayer layer = DebugDrawLayer::Misc);

    void add_obb_corners(const std::array<WorldVec3, 8> &corners_world,
                         const glm::vec4 &color,
                         float seconds = 0.0f,
                         DebugDepth depth = DebugDepth::DepthTested,
                         DebugDrawLayer layer = DebugDrawLayer::Misc);

    // Convenience: draws cylinder wireframe using circles + 4 side lines.
    void add_cylinder(const WorldVec3 &center_world,
                      const glm::dvec3 &axis_world,
                      float radius,
                      float half_height,
                      const glm::vec4 &color,
                      float seconds = 0.0f,
                      DebugDepth depth = DebugDepth::DepthTested,
                      DebugDrawLayer layer = DebugDrawLayer::Misc);

    // Convenience: draws tapered cylinder / cone wireframe (top_radius or bottom_radius may be 0).
    void add_tapered_cylinder(const WorldVec3 &center_world,
                              const glm::dvec3 &axis_world,
                              float half_height,
                              float top_radius,
                              float bottom_radius,
                              const glm::vec4 &color,
                              float seconds = 0.0f,
                              DebugDepth depth = DebugDepth::DepthTested,
                              DebugDrawLayer layer = DebugDrawLayer::Misc);

    // Convenience: draws a square patch on a plane (useful for PlaneShape visualization).
    void add_plane_patch(const WorldVec3 &point_world,
                         const glm::dvec3 &normal_world,
                         float half_size,
                         const glm::vec4 &color,
                         float seconds = 0.0f,
                         DebugDepth depth = DebugDepth::DepthTested,
                         DebugDrawLayer layer = DebugDrawLayer::Misc);

    // Expand currently queued commands into render-local line vertices.
    LineVertexLists build_line_vertices(const WorldVec3 &origin_world) const;

    size_t command_count() const;

private:
    struct CmdBase
    {
        DebugDepth depth{DebugDepth::DepthTested};
        DebugDrawLayer layer{DebugDrawLayer::Misc};
        glm::vec4 color{1.0f};
        float ttl_seconds = -1.0f; // <0 = one-frame
    };

    struct CmdLine : CmdBase
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
    };
    struct CmdAabb : CmdBase
    {
        WorldVec3 center_world{0.0, 0.0, 0.0};
        glm::vec3 half_extents{0.5f, 0.5f, 0.5f};
    };
    struct CmdSphere : CmdBase
    {
        WorldVec3 center_world{0.0, 0.0, 0.0};
        float radius{1.0f};
    };
    struct CmdCapsule : CmdBase
    {
        WorldVec3 p0_world{0.0, 0.0, 0.0};
        WorldVec3 p1_world{0.0, 0.0, 0.0};
        float radius{0.5f};
    };
    struct CmdCircle : CmdBase
    {
        WorldVec3 center_world{0.0, 0.0, 0.0};
        glm::dvec3 normal_world{0.0, 1.0, 0.0};
        float radius{1.0f};
    };
    struct CmdCone : CmdBase
    {
        WorldVec3 apex_world{0.0, 0.0, 0.0};
        glm::dvec3 direction_world{0.0, -1.0, 0.0};
        float length{1.0f};
        float angle_degrees{15.0f};
    };
    struct CmdObb : CmdBase
    {
        std::array<WorldVec3, 8> corners_world{};
    };

    template <typename T>
    static void prune_list(std::vector<T> &cmds, float dt_seconds);

    Settings _settings{};
    std::vector<CmdLine> _lines;
    std::vector<CmdAabb> _aabbs;
    std::vector<CmdSphere> _spheres;
    std::vector<CmdCapsule> _capsules;
    std::vector<CmdCircle> _circles;
    std::vector<CmdCone> _cones;
    std::vector<CmdObb> _obbs;
};
