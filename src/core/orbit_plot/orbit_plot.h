#pragma once

#include <core/world.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
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
        double render_error_px = 0.75;
        std::size_t upload_budget_bytes = 32ull * 1024ull * 1024ull;
    };

    struct LineVertexLists
    {
        std::vector<OrbitPlotVertex> vertices;
        uint32_t depth_vertex_count = 0;
        uint32_t overlay_vertex_count = 0;
    };

    struct LineVertexCounts
    {
        uint32_t depth_vertex_count = 0;
        uint32_t overlay_vertex_count = 0;
    };

    struct Stats
    {
        uint32_t active_line_count = 0;
        uint32_t pending_line_count = 0;

        uint32_t depth_segment_count = 0;
        uint32_t overlay_segment_count = 0;

        std::size_t upload_budget_bytes = 0;
        std::size_t upload_bytes_last_frame = 0;
        std::size_t upload_bytes_peak = 0;
        double upload_ms_last_frame = 0.0;
        double upload_ms_peak = 0.0;

        bool upload_cap_hit_last_frame = false;
        uint64_t upload_cap_hits_total = 0;
    };

    struct LineCommand
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
        glm::vec4 color{1.0f};
        OrbitPlotDepth depth{OrbitPlotDepth::DepthTested};
    };

    Settings &settings() { return _settings; }
    const Settings &settings() const { return _settings; }
    const Stats &stats() const { return _stats; }

    void clear_pending();
    void clear_all();
    void begin_frame();

    void add_line(const WorldVec3 &a_world,
                  const WorldVec3 &b_world,
                  const glm::vec4 &color,
                  OrbitPlotDepth depth = OrbitPlotDepth::DepthTested);

    bool has_active_lines() const;
    std::span<const LineCommand> active_lines() const;
    LineVertexCounts count_line_vertices() const;
    void write_line_vertices(const WorldVec3 &origin_world,
                             OrbitPlotVertex *dst,
                             uint32_t depth_vertex_count,
                             uint32_t overlay_vertex_count) const;
    LineVertexLists build_line_vertices(const WorldVec3 &origin_world) const;

    void record_upload_stats(std::size_t upload_bytes,
                             std::size_t upload_budget_bytes,
                             double upload_ms,
                             bool upload_cap_hit,
                             uint32_t depth_segment_count,
                             uint32_t overlay_segment_count);

private:
    Settings _settings{};
    Stats _stats{};
    std::vector<LineCommand> _pending_lines{};
    std::vector<LineCommand> _active_lines{};
};
