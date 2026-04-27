#include "orbit_plot.h"

#include <algorithm>

namespace
{
    static void push_line(OrbitPlotVertex *dst,
                          uint32_t &vertex_index,
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

        dst[vertex_index++] = v0;
        dst[vertex_index++] = v1;
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
    _active_lines.swap(_pending_lines);
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
    LineCommand cmd{};
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

std::span<const OrbitPlotSystem::LineCommand> OrbitPlotSystem::active_lines() const
{
    return std::span<const LineCommand>(_active_lines.data(), _active_lines.size());
}

OrbitPlotSystem::LineVertexCounts OrbitPlotSystem::count_line_vertices() const
{
    LineVertexCounts out{};
    if (!_settings.enabled || _active_lines.empty())
    {
        return out;
    }

    for (const LineCommand &cmd : _active_lines)
    {
        uint32_t &vertex_count =
                (cmd.depth == OrbitPlotDepth::DepthTested) ? out.depth_vertex_count : out.overlay_vertex_count;
        vertex_count += 2u;
    }

    return out;
}

void OrbitPlotSystem::write_line_vertices(const WorldVec3 &origin_world,
                                          OrbitPlotVertex *dst,
                                          const uint32_t depth_vertex_count,
                                          const uint32_t overlay_vertex_count) const
{
    if (!_settings.enabled || _active_lines.empty() || dst == nullptr)
    {
        return;
    }

    OrbitPlotVertex *const depth_dst = dst;
    OrbitPlotVertex *const overlay_dst = dst + depth_vertex_count;
    uint32_t depth_write_index = 0;
    uint32_t overlay_write_index = 0;

    for (const LineCommand &cmd : _active_lines)
    {
        OrbitPlotVertex *bucket_dst = depth_dst;
        uint32_t *bucket_write_index = &depth_write_index;
        uint32_t bucket_limit = depth_vertex_count;
        if (cmd.depth == OrbitPlotDepth::AlwaysOnTop)
        {
            bucket_dst = overlay_dst;
            bucket_write_index = &overlay_write_index;
            bucket_limit = overlay_vertex_count;
        }

        if ((*bucket_write_index + 2u) > bucket_limit)
        {
            continue;
        }

        const glm::vec3 a = world_to_local(cmd.a_world, origin_world);
        const glm::vec3 b = world_to_local(cmd.b_world, origin_world);
        push_line(bucket_dst, *bucket_write_index, a, b, cmd.color);
    }
}

OrbitPlotSystem::LineVertexLists OrbitPlotSystem::build_line_vertices(const WorldVec3 &origin_world) const
{
    LineVertexLists out{};
    if (!_settings.enabled || _active_lines.empty())
    {
        return out;
    }

    const LineVertexCounts counts = count_line_vertices();
    out.depth_vertex_count = counts.depth_vertex_count;
    out.overlay_vertex_count = counts.overlay_vertex_count;
    out.vertices.resize(static_cast<std::size_t>(counts.depth_vertex_count) +
                        static_cast<std::size_t>(counts.overlay_vertex_count));
    if (!out.vertices.empty())
    {
        write_line_vertices(origin_world,
                            out.vertices.data(),
                            out.depth_vertex_count,
                            out.overlay_vertex_count);
    }
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
