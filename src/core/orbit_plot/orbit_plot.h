#pragma once

#include <core/world.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

enum class OrbitPlotDepth : uint8_t
{
    DepthTested = 0,
    AlwaysOnTop = 1,
};

struct OrbitPlotVertex
{
    glm::vec3 position{0.0f};
    float _pad0{0.0f};
    glm::vec4 color{1.0f};
};
static_assert(sizeof(OrbitPlotVertex) == 32);

class OrbitPlotSystem
{
public:
    struct Settings
    {
        bool enabled = true;
        float line_width_px = 2.0f;
        float line_aa_px = 1.0f;
    };

    struct LineVertexLists
    {
        std::vector<OrbitPlotVertex> vertices;
        uint32_t depth_vertex_count = 0;
        uint32_t overlay_vertex_count = 0;
    };

    Settings &settings() { return _settings; }
    const Settings &settings() const { return _settings; }

    void clear_pending();
    void clear_all();
    void begin_frame();

    void add_line(const WorldVec3 &a_world,
                  const WorldVec3 &b_world,
                  const glm::vec4 &color,
                  OrbitPlotDepth depth = OrbitPlotDepth::DepthTested);

    bool has_active_lines() const;
    LineVertexLists build_line_vertices(const WorldVec3 &origin_world) const;

private:
    struct CmdLine
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
        glm::vec4 color{1.0f};
        OrbitPlotDepth depth{OrbitPlotDepth::DepthTested};
    };

    Settings _settings{};
    std::vector<CmdLine> _pending_lines{};
    std::vector<CmdLine> _active_lines{};
};
