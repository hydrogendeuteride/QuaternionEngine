#pragma once

#include <core/world.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
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
        bool gpu_generate_enabled = true;
        float line_width_px = 2.0f;
        float line_aa_px = 1.0f;
        double render_error_px = 0.75;
        std::size_t render_max_segments_gpu = 200'000ull;
        std::size_t upload_budget_bytes = 32ull * 1024ull * 1024ull;
    };

    struct LineVertexLists
    {
        std::vector<OrbitPlotVertex> vertices;
        uint32_t depth_vertex_count = 0;
        uint32_t overlay_vertex_count = 0;
    };

    struct Stats
    {
        uint32_t active_line_count = 0;
        uint32_t pending_line_count = 0;
        uint32_t active_gpu_root_count = 0;
        uint32_t pending_gpu_root_count = 0;

        uint32_t depth_segment_count = 0;
        uint32_t overlay_segment_count = 0;

        std::size_t upload_budget_bytes = 0;
        std::size_t upload_bytes_last_frame = 0;
        std::size_t upload_bytes_peak = 0;
        double upload_ms_last_frame = 0.0;
        double upload_ms_peak = 0.0;

        bool upload_cap_hit_last_frame = false;
        uint64_t upload_cap_hits_total = 0;

        bool gpu_path_active_last_frame = false;
        bool gpu_generate_cap_hit_last_frame = false;
        uint64_t gpu_generate_cap_hits_total = 0;
        uint64_t gpu_fallback_count = 0;
    };

    struct LineCommand
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
        glm::vec4 color{1.0f};
        OrbitPlotDepth depth{OrbitPlotDepth::DepthTested};
    };

    struct GpuRootSegment
    {
        double t0_s{0.0};
        glm::dvec3 p0_bci{0.0, 0.0, 0.0};
        glm::dvec3 v0_bci{0.0, 0.0, 0.0};
        glm::dvec3 p1_bci{0.0, 0.0, 0.0};
        glm::dvec3 v1_bci{0.0, 0.0, 0.0};
        double dt_s{0.0};
        double prefix_length_m{0.0};
    };

    struct GpuRootBatch
    {
        std::shared_ptr<const std::vector<GpuRootSegment>> owner{};
        const GpuRootSegment *segments{nullptr};
        std::size_t count{0};
        double t_start_s{0.0};
        double t_end_s{0.0};
        WorldVec3 reference_body_world{0.0, 0.0, 0.0};
        WorldVec3 align_delta_world{0.0, 0.0, 0.0};
        glm::dmat3 frame_to_world{1.0};
        glm::vec4 color{1.0f};
        bool dashed{false};
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
    bool has_active_gpu_root_batches() const;
    std::span<const GpuRootBatch> active_gpu_root_batches() const;
    LineVertexLists build_line_vertices(const WorldVec3 &origin_world) const;

    void record_upload_stats(std::size_t upload_bytes,
                             std::size_t upload_budget_bytes,
                             double upload_ms,
                             bool upload_cap_hit,
                             uint32_t depth_segment_count,
                             uint32_t overlay_segment_count);
    void record_gpu_path_stats(bool gpu_path_active, bool gpu_generate_cap_hit, bool fallback);

    void add_gpu_root_batch(std::shared_ptr<const std::vector<GpuRootSegment>> segments,
                            double t_start_s,
                            double t_end_s,
                            const WorldVec3 &reference_body_world,
                            const WorldVec3 &align_delta_world,
                            const glm::dmat3 &frame_to_world,
                            const glm::vec4 &color,
                            bool dashed,
                            OrbitPlotDepth depth = OrbitPlotDepth::DepthTested);

private:
    Settings _settings{};
    Stats _stats{};
    std::vector<LineCommand> _pending_lines{};
    std::vector<LineCommand> _active_lines{};
    std::vector<GpuRootBatch> _pending_gpu_root_batches{};
    std::vector<GpuRootBatch> _active_gpu_root_batches{};
};
