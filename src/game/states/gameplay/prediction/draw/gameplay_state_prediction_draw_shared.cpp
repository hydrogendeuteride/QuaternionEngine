#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game::PredictionDrawDetail
{
    namespace
    {
        constexpr double kPickWindowRebuildEpsilonS = 0.25;
        constexpr double kPickAlignRebuildDistanceM = 25.0;
        constexpr double kPickReferenceRebuildDistanceM = 1.0;
        constexpr double kPickFrustumOriginRebuildDistanceM = 1.0;
        constexpr double kPickCameraRebuildDistanceM = 1.0;
        constexpr double kPickMatrixRebuildEpsilon = 1.0e-6;
        constexpr double kPickScalarRebuildEpsilon = 1.0e-6;
        constexpr float kPickViewprojRebuildEpsilon = 1.0e-5f;
    }

    const std::vector<orbitsim::TrajectorySegment> &base_segments_world_basis(PredictionTrackDrawContext &track_ctx)
    {
        if (track_ctx.identity_frame_transform)
        {
            return *track_ctx.traj_base_segments;
        }

        if (track_ctx.traj_base_segments_world_basis.empty() && track_ctx.traj_base_segments &&
            !track_ctx.traj_base_segments->empty())
        {
            track_ctx.traj_base_segments_world_basis =
                    transform_segments_to_world_basis(*track_ctx.traj_base_segments, track_ctx.frame_to_world);
        }
        return track_ctx.traj_base_segments_world_basis;
    }

    const std::vector<orbitsim::TrajectorySegment> &planned_segments_world_basis(PredictionTrackDrawContext &track_ctx,
                                                                                 OrbitPredictionCache &cache)
    {
        if (track_ctx.identity_frame_transform)
        {
            return cache.trajectory_segments_frame_planned;
        }

        std::vector<orbitsim::TrajectorySegment> &world_basis_segments =
                (&cache == track_ctx.preview_planned_cache) ? track_ctx.traj_preview_planned_segments_world_basis
                                                            : track_ctx.traj_stable_planned_segments_world_basis;
        if (world_basis_segments.empty() && !cache.trajectory_segments_frame_planned.empty())
        {
            world_basis_segments =
                    transform_segments_to_world_basis(cache.trajectory_segments_frame_planned, track_ctx.frame_to_world);
        }
        return world_basis_segments;
    }

    bool same_matrix(const glm::dmat3 &a, const glm::dmat3 &b, const double epsilon)
    {
        for (int col = 0; col < 3; ++col)
        {
            for (int row = 0; row < 3; ++row)
            {
                if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                    std::abs(a[col][row] - b[col][row]) > epsilon)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool same_matrix(const glm::mat4 &a, const glm::mat4 &b, const float epsilon)
    {
        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                    std::abs(a[col][row] - b[col][row]) > epsilon)
                {
                    return false;
                }
            }
        }
        return true;
    }

    void enqueue_cached_orbit_window(
            OrbitPlotSystem *orbit_plot,
            const std::shared_ptr<const std::vector<OrbitPlotSystem::GpuRootSegment>> &cached_roots,
            const WorldVec3 &ref_body_world,
            const glm::dmat3 &frame_to_world,
            const WorldVec3 &align_delta,
            const double t_start_s,
            const double t_end_s,
            const glm::vec4 &color,
            const bool dashed,
            const float line_overlay_boost)
    {
        if (!orbit_plot || !cached_roots || cached_roots->empty() || !(t_end_s > t_start_s))
        {
            return;
        }

        orbit_plot->add_gpu_root_batch(cached_roots,
                                       t_start_s,
                                       t_end_s,
                                       ref_body_world,
                                       align_delta,
                                       frame_to_world,
                                       color,
                                       dashed,
                                       OrbitPlotDepth::DepthTested);

        if (line_overlay_boost <= 0.0f)
        {
            return;
        }

        glm::vec4 overlay_color = color;
        overlay_color.a = std::clamp(overlay_color.a * line_overlay_boost, 0.0f, 1.0f);
        if (overlay_color.a <= 0.0f)
        {
            return;
        }

        orbit_plot->add_gpu_root_batch(cached_roots,
                                       t_start_s,
                                       t_end_s,
                                       ref_body_world,
                                       align_delta,
                                       frame_to_world,
                                       overlay_color,
                                       dashed,
                                       OrbitPlotDepth::AlwaysOnTop);
    }

    ChunkAssemblyDrawResult draw_chunk_assembly_planned(
            const OrbitDrawWindowContext &draw_ctx,
            const OrbitPredictionDrawConfig &draw_config,
            OrbitPlotPerfStats &perf,
            PredictionChunkAssembly &assembly,
            const double t_start_s,
            const double t_end_s,
            const glm::vec4 &color,
            const bool dashed,
            const bool use_persistent_gpu_roots,
            const uint32_t max_chunk_builds)
    {
        ChunkAssemblyDrawResult result{};
        if (!assembly.valid || assembly.chunks.empty() || !(t_end_s > t_start_s))
        {
            return result;
        }

        for (OrbitChunk &chunk : assembly.chunks)
        {
            if (!chunk.valid || chunk.frame_segments.empty())
            {
                continue;
            }

            if (chunk.t1_s <= t_start_s || chunk.t0_s >= t_end_s)
            {
                continue;
            }

            const double draw_t0 = std::max(chunk.t0_s, t_start_s);
            const double draw_t1 = std::min(chunk.t1_s, t_end_s);
            if (!(draw_t1 > draw_t0))
            {
                continue;
            }

            bool drawn = false;
            if (use_persistent_gpu_roots && draw_ctx.orbit_plot)
            {
                if (!chunk.gpu_roots || chunk.gpu_roots->empty())
                {
                    if (result.chunks_built >= max_chunk_builds)
                    {
                        continue;
                    }

                    const auto build_start_tp = std::chrono::steady_clock::now();
                    chunk.gpu_roots = PredictionCacheInternal::build_gpu_root_cache(chunk.frame_segments);
                    result.gpu_root_build_ms +=
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start_tp)
                                    .count();
                    ++result.chunks_built;
                }

                if (chunk.gpu_roots && !chunk.gpu_roots->empty())
                {
                    enqueue_cached_orbit_window(draw_ctx.orbit_plot,
                                                chunk.gpu_roots,
                                                draw_ctx.ref_body_world,
                                                draw_ctx.frame_to_world,
                                                draw_ctx.align_delta,
                                                draw_t0,
                                                draw_t1,
                                                color,
                                                dashed,
                                                draw_ctx.line_overlay_boost);
                    drawn = true;
                }
            }

            if (!drawn)
            {
                if (!chunk.render_curve.empty())
                {
                    draw_adaptive_curve_window(draw_ctx,
                                               draw_config,
                                               perf,
                                               chunk.render_curve,
                                               draw_t0,
                                               draw_t1,
                                               color,
                                               dashed);
                }
                else
                {
                    draw_orbit_window(draw_ctx,
                                      draw_config,
                                      perf,
                                      chunk.frame_segments,
                                      draw_t0,
                                      draw_t1,
                                      color,
                                      dashed);
                }
            }

            ++result.chunks_drawn;
            result.covered_ranges.emplace_back(draw_t0, draw_t1);
        }

        return result;
    }

    std::vector<std::pair<double, double>> compute_uncovered_ranges(
            const double t_start_s,
            const double t_end_s,
            std::vector<std::pair<double, double>> covered_ranges)
    {
        std::vector<std::pair<double, double>> uncovered_ranges;
        if (!(t_end_s > t_start_s))
        {
            return uncovered_ranges;
        }

        if (covered_ranges.empty())
        {
            uncovered_ranges.emplace_back(t_start_s, t_end_s);
            return uncovered_ranges;
        }

        std::sort(covered_ranges.begin(),
                  covered_ranges.end(),
                  [](const auto &a, const auto &b) {
                      if (a.first == b.first)
                      {
                          return a.second < b.second;
                      }
                      return a.first < b.first;
                  });

        constexpr double kTimeEpsilonS = 1.0e-6;
        double cursor_t_s = t_start_s;
        for (const auto &[covered_t0_s, covered_t1_s] : covered_ranges)
        {
            if (!(covered_t1_s > covered_t0_s))
            {
                continue;
            }

            const double clamped_t0_s = std::max(covered_t0_s, t_start_s);
            const double clamped_t1_s = std::min(covered_t1_s, t_end_s);
            if (!(clamped_t1_s > clamped_t0_s))
            {
                continue;
            }

            if (clamped_t0_s > (cursor_t_s + kTimeEpsilonS))
            {
                uncovered_ranges.emplace_back(cursor_t_s, clamped_t0_s);
            }
            cursor_t_s = std::max(cursor_t_s, clamped_t1_s);
        }

        if (t_end_s > (cursor_t_s + kTimeEpsilonS))
        {
            uncovered_ranges.emplace_back(cursor_t_s, t_end_s);
        }

        return uncovered_ranges;
    }

    bool should_rebuild_pick_cache(const PredictionLinePickCache &cache,
                                   const uint64_t generation_id,
                                   const uint64_t display_frame_key,
                                   const uint64_t display_frame_revision,
                                   const WorldVec3 &ref_body_world,
                                   const glm::dmat3 &frame_to_world,
                                   const WorldVec3 &align_delta,
                                   const glm::dvec3 &camera_world,
                                   const double tan_half_fov,
                                   const double viewport_height_px,
                                   const double render_error_px,
                                   const OrbitRenderCurve::FrustumContext &pick_frustum,
                                   const double pick_frustum_margin_ratio,
                                   const double t0_s,
                                   const double t1_s,
                                   const std::size_t max_segments,
                                   const bool use_adaptive_curve,
                                   const bool planned)
    {
        const bool valid = planned ? cache.planned_valid : cache.base_valid;
        const double cached_t0_s = planned ? cache.planned_t0_s : cache.base_t0_s;
        const double cached_t1_s = planned ? cache.planned_t1_s : cache.base_t1_s;
        const std::size_t cached_max_segments = planned ? cache.planned_max_segments : cache.base_max_segments;
        const bool cached_use_adaptive_curve = planned ? cache.planned_use_adaptive_curve : cache.base_use_adaptive_curve;
        if (!valid || cache.generation_id != generation_id ||
            cache.display_frame_key != display_frame_key ||
            cache.display_frame_revision != display_frame_revision ||
            cached_max_segments != max_segments)
        {
            return true;
        }

        if (cached_use_adaptive_curve != use_adaptive_curve)
        {
            return true;
        }

        if (!same_matrix(cache.frame_to_world, frame_to_world, kPickMatrixRebuildEpsilon))
        {
            return true;
        }

        if (cache.pick_frustum_valid != pick_frustum.valid ||
            !same_matrix(cache.pick_frustum_viewproj, pick_frustum.viewproj, kPickViewprojRebuildEpsilon) ||
            glm::length(glm::dvec3(cache.pick_frustum_origin_world - pick_frustum.origin_world)) >
                    kPickFrustumOriginRebuildDistanceM ||
            std::abs(cache.pick_frustum_margin_ratio - pick_frustum_margin_ratio) > kPickMatrixRebuildEpsilon)
        {
            return true;
        }

        if (glm::length(glm::dvec3(cache.ref_body_world - ref_body_world)) > kPickReferenceRebuildDistanceM ||
            glm::length(glm::dvec3(cache.align_delta_world - align_delta)) > kPickAlignRebuildDistanceM)
        {
            return true;
        }

        if (use_adaptive_curve &&
            (glm::length(cache.camera_world - camera_world) > kPickCameraRebuildDistanceM ||
             std::abs(cache.tan_half_fov - tan_half_fov) > kPickScalarRebuildEpsilon ||
             std::abs(cache.viewport_height_px - viewport_height_px) > kPickScalarRebuildEpsilon ||
             std::abs(cache.render_error_px - render_error_px) > kPickScalarRebuildEpsilon))
        {
            return true;
        }

        return !std::isfinite(cached_t0_s) || !std::isfinite(cached_t1_s) ||
               std::abs(cached_t0_s - t0_s) > kPickWindowRebuildEpsilonS ||
               std::abs(cached_t1_s - t1_s) > kPickWindowRebuildEpsilonS;
    }

    void mark_pick_cache_valid(PredictionLinePickCache &cache,
                               const uint64_t generation_id,
                               const uint64_t display_frame_key,
                               const uint64_t display_frame_revision,
                               const WorldVec3 &ref_body_world,
                               const glm::dmat3 &frame_to_world,
                               const WorldVec3 &align_delta,
                               const glm::dvec3 &camera_world,
                               const double tan_half_fov,
                               const double viewport_height_px,
                               const double render_error_px,
                               const OrbitRenderCurve::FrustumContext &pick_frustum,
                               const double pick_frustum_margin_ratio,
                               const double t0_s,
                               const double t1_s,
                               const std::size_t max_segments,
                               const bool use_adaptive_curve,
                               const bool planned)
    {
        cache.generation_id = generation_id;
        cache.display_frame_key = display_frame_key;
        cache.display_frame_revision = display_frame_revision;
        cache.ref_body_world = ref_body_world;
        cache.frame_to_world = frame_to_world;
        cache.align_delta_world = align_delta;
        cache.camera_world = camera_world;
        cache.tan_half_fov = tan_half_fov;
        cache.viewport_height_px = viewport_height_px;
        cache.render_error_px = render_error_px;
        cache.pick_frustum_valid = pick_frustum.valid;
        cache.pick_frustum_viewproj = pick_frustum.viewproj;
        cache.pick_frustum_origin_world = pick_frustum.origin_world;
        cache.pick_frustum_margin_ratio = pick_frustum_margin_ratio;
        if (planned)
        {
            cache.planned_valid = true;
            cache.planned_t0_s = t0_s;
            cache.planned_t1_s = t1_s;
            cache.planned_max_segments = max_segments;
            cache.planned_use_adaptive_curve = use_adaptive_curve;
        }
        else
        {
            cache.base_valid = true;
            cache.base_t0_s = t0_s;
            cache.base_t1_s = t1_s;
            cache.base_max_segments = max_segments;
            cache.base_use_adaptive_curve = use_adaptive_curve;
        }
    }

    bool same_pick_time_window(const double cached_t0_s,
                               const double cached_t1_s,
                               const double t0_s,
                               const double t1_s)
    {
        return std::isfinite(cached_t0_s) &&
               std::isfinite(cached_t1_s) &&
               std::isfinite(t0_s) &&
               std::isfinite(t1_s) &&
               std::abs(cached_t0_s - t0_s) <= kPickWindowRebuildEpsilonS &&
               std::abs(cached_t1_s - t1_s) <= kPickWindowRebuildEpsilonS;
    }

    bool same_pick_ranges(const std::vector<std::pair<double, double>> &a,
                          const std::vector<std::pair<double, double>> &b)
    {
        if (a.size() != b.size())
        {
            return false;
        }

        for (std::size_t i = 0; i < a.size(); ++i)
        {
            if (!same_pick_time_window(a[i].first, a[i].second, b[i].first, b[i].second))
            {
                return false;
            }
        }

        return true;
    }

    const PredictionLinePickCache::PreviewChunkEntry *find_preview_pick_chunk_entry(
            const PredictionLinePickCache &cache,
            const uint32_t chunk_id)
    {
        const auto it = std::find_if(cache.preview_chunk_entries.begin(),
                                     cache.preview_chunk_entries.end(),
                                     [chunk_id](const PredictionLinePickCache::PreviewChunkEntry &entry) {
                                         return entry.chunk_id == chunk_id;
                                     });
        return (it != cache.preview_chunk_entries.end()) ? &(*it) : nullptr;
    }

    bool preview_pick_chunk_entry_matches(const PredictionLinePickCache::PreviewChunkEntry &entry,
                                          const OrbitChunk &chunk,
                                          const double t0_s,
                                          const double t1_s,
                                          const std::size_t max_segments,
                                          const bool use_adaptive_curve)
    {
        return entry.chunk_id == chunk.chunk_id &&
               entry.generation_id == chunk.generation_id &&
               entry.quality_state == chunk.quality_state &&
               entry.max_segments == max_segments &&
               entry.use_adaptive_curve == use_adaptive_curve &&
               same_pick_time_window(entry.t0_s, entry.t1_s, t0_s, t1_s);
    }

    bool preview_pick_fallback_matches(const PredictionLinePickCache::PreviewFallbackCache &fallback,
                                       const uint64_t source_generation_id,
                                       const std::vector<std::pair<double, double>> &uncovered_ranges,
                                       const std::size_t max_segments,
                                       const bool use_adaptive_curve)
    {
        return fallback.valid &&
               fallback.source_generation_id == source_generation_id &&
               fallback.max_segments == max_segments &&
               fallback.use_adaptive_curve == use_adaptive_curve &&
               same_pick_ranges(fallback.uncovered_ranges, uncovered_ranges);
    }

    void clear_preview_pick_chunk_cache(PredictionLinePickCache &cache)
    {
        cache.preview_chunk_cache_valid = false;
        cache.preview_chunk_entries.clear();
        cache.preview_fallback.clear();
    }
} // namespace Game::PredictionDrawDetail
