#include "orbit_plot.h"

#include <algorithm>

namespace
{
    static void push_line(std::vector<OrbitPlotVertex> &dst,
                          const glm::vec3 &a,
                          const glm::vec3 &b,
                          const glm::vec4 &color)
    {
        OrbitPlotVertex v0{};
        v0.position = a;
        v0.color = color;

        OrbitPlotVertex v1{};
        v1.position = b;
        v1.color = color;

        dst.push_back(v0);
        dst.push_back(v1);
    }
} // namespace

void OrbitPlotSystem::clear_pending()
{
    _pending_lines.clear();
    _stats.pending_line_count = 0;
}

void OrbitPlotSystem::clear_all()
{
    _pending_lines.clear();
    _active_lines.clear();
    _stats.active_line_count = 0;
    _stats.pending_line_count = 0;
    _stats.depth_segment_count = 0;
    _stats.overlay_segment_count = 0;
    _stats.upload_bytes_last_frame = 0;
    _stats.upload_bytes_peak = 0;
    _stats.upload_ms_last_frame = 0.0;
    _stats.upload_ms_peak = 0.0;
    _stats.upload_cap_hit_last_frame = false;
    _stats.upload_cap_hits_total = 0;
    _stats.upload_budget_bytes = _settings.upload_budget_bytes;
}

void OrbitPlotSystem::begin_frame()
{
    _active_lines = std::move(_pending_lines);
    _pending_lines.clear();
    _stats.active_line_count = static_cast<uint32_t>(_active_lines.size());
    _stats.pending_line_count = 0;
    _stats.depth_segment_count = 0;
    _stats.overlay_segment_count = 0;
    _stats.upload_bytes_last_frame = 0;
    _stats.upload_ms_last_frame = 0.0;
    _stats.upload_cap_hit_last_frame = false;
    _stats.upload_budget_bytes = _settings.upload_budget_bytes;
}

void OrbitPlotSystem::add_line(const WorldVec3 &a_world,
                               const WorldVec3 &b_world,
                               const glm::vec4 &color,
                               OrbitPlotDepth depth)
{
    CmdLine cmd{};
    cmd.a_world = a_world;
    cmd.b_world = b_world;
    cmd.color = color;
    cmd.depth = depth;
    _pending_lines.push_back(cmd);
    _stats.pending_line_count = static_cast<uint32_t>(_pending_lines.size());
}

bool OrbitPlotSystem::has_active_lines() const
{
    return !_active_lines.empty();
}

OrbitPlotSystem::LineVertexLists OrbitPlotSystem::build_line_vertices(const WorldVec3 &origin_world) const
{
    LineVertexLists out{};
    if (!_settings.enabled || _active_lines.empty())
    {
        return out;
    }

    std::vector<OrbitPlotVertex> depth_vertices{};
    std::vector<OrbitPlotVertex> overlay_vertices{};
    depth_vertices.reserve(_active_lines.size() * 2);
    overlay_vertices.reserve(_active_lines.size() * 2);

    for (const CmdLine &cmd : _active_lines)
    {
        std::vector<OrbitPlotVertex> &dst =
                (cmd.depth == OrbitPlotDepth::DepthTested) ? depth_vertices : overlay_vertices;

        const glm::vec3 a = world_to_local(cmd.a_world, origin_world);
        const glm::vec3 b = world_to_local(cmd.b_world, origin_world);
        push_line(dst, a, b, cmd.color);
    }

    out.depth_vertex_count = static_cast<uint32_t>(depth_vertices.size());
    out.overlay_vertex_count = static_cast<uint32_t>(overlay_vertices.size());
    out.vertices.reserve(depth_vertices.size() + overlay_vertices.size());
    out.vertices.insert(out.vertices.end(), depth_vertices.begin(), depth_vertices.end());
    out.vertices.insert(out.vertices.end(), overlay_vertices.begin(), overlay_vertices.end());
    return out;
}

void OrbitPlotSystem::record_upload_stats(const std::size_t upload_bytes,
                                          const std::size_t upload_budget_bytes,
                                          const double upload_ms,
                                          const bool upload_cap_hit,
                                          const uint32_t depth_segment_count,
                                          const uint32_t overlay_segment_count)
{
    _stats.upload_budget_bytes = upload_budget_bytes;
    _stats.upload_bytes_last_frame = upload_bytes;
    _stats.upload_bytes_peak = std::max(_stats.upload_bytes_peak, upload_bytes);
    _stats.upload_ms_last_frame = std::max(0.0, upload_ms);
    _stats.upload_ms_peak = std::max(_stats.upload_ms_peak, _stats.upload_ms_last_frame);
    _stats.upload_cap_hit_last_frame = upload_cap_hit;
    if (upload_cap_hit)
    {
        ++_stats.upload_cap_hits_total;
    }
    _stats.depth_segment_count = depth_segment_count;
    _stats.overlay_segment_count = overlay_segment_count;
}
