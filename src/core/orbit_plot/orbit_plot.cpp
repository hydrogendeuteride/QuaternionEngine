#include "orbit_plot.h"

#include <algorithm>
#include <cmath>

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
    _pending_gpu_root_segments.clear();
    _stats.pending_line_count = 0;
    _stats.pending_gpu_root_count = 0;
}

void OrbitPlotSystem::clear_all()
{
    _pending_lines.clear();
    _active_lines.clear();
    _pending_gpu_root_segments.clear();
    _active_gpu_root_segments.clear();
    _stats.active_line_count = 0;
    _stats.pending_line_count = 0;
    _stats.active_gpu_root_count = 0;
    _stats.pending_gpu_root_count = 0;
    _stats.depth_segment_count = 0;
    _stats.overlay_segment_count = 0;
    _stats.upload_bytes_last_frame = 0;
    _stats.upload_bytes_peak = 0;
    _stats.upload_ms_last_frame = 0.0;
    _stats.upload_ms_peak = 0.0;
    _stats.upload_cap_hit_last_frame = false;
    _stats.upload_cap_hits_total = 0;
    _stats.gpu_path_active_last_frame = false;
    _stats.gpu_generate_cap_hit_last_frame = false;
    _stats.gpu_generate_cap_hits_total = 0;
    _stats.gpu_fallback_count = 0;
    _stats.upload_budget_bytes = _settings.upload_budget_bytes;
}

void OrbitPlotSystem::begin_frame()
{
    _active_lines = std::move(_pending_lines);
    _active_gpu_root_segments = std::move(_pending_gpu_root_segments);
    _pending_lines.clear();
    _pending_gpu_root_segments.clear();
    _stats.active_line_count = static_cast<uint32_t>(_active_lines.size());
    _stats.pending_line_count = 0;
    _stats.active_gpu_root_count = static_cast<uint32_t>(_active_gpu_root_segments.size());
    _stats.pending_gpu_root_count = 0;
    _stats.depth_segment_count = 0;
    _stats.overlay_segment_count = 0;
    _stats.upload_bytes_last_frame = 0;
    _stats.upload_ms_last_frame = 0.0;
    _stats.upload_cap_hit_last_frame = false;
    _stats.gpu_path_active_last_frame = false;
    _stats.gpu_generate_cap_hit_last_frame = false;
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

bool OrbitPlotSystem::has_active_gpu_root_segments() const
{
    return !_active_gpu_root_segments.empty();
}

std::span<const OrbitPlotSystem::GpuRootSegment> OrbitPlotSystem::active_gpu_root_segments() const
{
    return std::span<const GpuRootSegment>(_active_gpu_root_segments.data(), _active_gpu_root_segments.size());
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

    for (const LineCommand &cmd : _active_lines)
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

void OrbitPlotSystem::record_gpu_path_stats(const bool gpu_path_active,
                                            const bool gpu_generate_cap_hit,
                                            const bool fallback)
{
    _stats.gpu_path_active_last_frame = gpu_path_active;
    _stats.gpu_generate_cap_hit_last_frame = gpu_generate_cap_hit;
    if (gpu_generate_cap_hit)
    {
        ++_stats.gpu_generate_cap_hits_total;
    }
    if (fallback)
    {
        ++_stats.gpu_fallback_count;
    }
}

void OrbitPlotSystem::set_gpu_frame_reference(const WorldVec3 &reference_body_world,
                                              const WorldVec3 &align_delta_world)
{
    _gpu_reference_body_world = reference_body_world;
    _gpu_align_delta_world = align_delta_world;
}

void OrbitPlotSystem::add_gpu_root_segment(const double dt_s,
                                           const glm::dvec3 &p0_bci,
                                           const glm::dvec3 &v0_bci,
                                           const glm::dvec3 &p1_bci,
                                           const glm::dvec3 &v1_bci,
                                           const double clip_u0,
                                           const double clip_u1,
                                           const bool dashed,
                                           const float dash_phase_start_px,
                                           const glm::vec4 &color,
                                           const OrbitPlotDepth depth)
{
    if (!(dt_s > 0.0) || !std::isfinite(dt_s))
    {
        return;
    }

    const double u0 = std::clamp(clip_u0, 0.0, 1.0);
    const double u1 = std::clamp(clip_u1, 0.0, 1.0);
    if (!(u1 > u0))
    {
        return;
    }

    GpuRootSegment segment{};
    segment.p0_bci = p0_bci;
    segment.v0_bci = v0_bci;
    segment.p1_bci = p1_bci;
    segment.v1_bci = v1_bci;
    segment.dt_s = dt_s;
    segment.clip_u0 = u0;
    segment.clip_u1 = u1;
    segment.dashed = dashed;
    segment.dash_phase_start_px = std::isfinite(dash_phase_start_px) ? std::max(0.0f, dash_phase_start_px) : 0.0f;
    segment.color = color;
    segment.depth = depth;
    _pending_gpu_root_segments.push_back(segment);
    _stats.pending_gpu_root_count = static_cast<uint32_t>(_pending_gpu_root_segments.size());
}
