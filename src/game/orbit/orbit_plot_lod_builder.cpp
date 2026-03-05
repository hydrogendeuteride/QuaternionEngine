#include "game/orbit/orbit_plot_lod_builder.h"

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr int kMaxSubdivisionDepth = 24;
        constexpr double kMinSubdivisionDtS = 1.0e-6;

        struct WorkItem
        {
            const orbitsim::TrajectorySegment *segment{nullptr};
            double t0_s{0.0};
            double t1_s{0.0};
            int depth{0};
        };

        struct CandidateSegment
        {
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
            double t0_s{0.0};
            double t1_s{0.0};
        };

        double meters_per_px_at_world(const OrbitPlotLodBuilder::CameraContext &camera, const WorldVec3 &point_world)
        {
            const double dist_m = glm::length(glm::dvec3(point_world) - camera.camera_world);
            if (!std::isfinite(dist_m) || dist_m <= 1.0e-3 ||
                !std::isfinite(camera.tan_half_fov) || camera.tan_half_fov <= 1.0e-8 ||
                !std::isfinite(camera.viewport_height_px) || camera.viewport_height_px <= 1.0)
            {
                return 1.0;
            }

            const double meters_per_px = (2.0 * camera.tan_half_fov * dist_m) / camera.viewport_height_px;
            if (!std::isfinite(meters_per_px) || meters_per_px <= 1.0e-6)
            {
                return 1.0;
            }
            return meters_per_px;
        }

        glm::dvec3 eval_segment_local_position(const orbitsim::TrajectorySegment &segment, const double t_s)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                return glm::dvec3(segment.start.position_m);
            }

            double u = (t_s - segment.t0_s) / segment.dt_s;
            if (!std::isfinite(u))
            {
                u = 0.0;
            }
            u = std::clamp(u, 0.0, 1.0);

            const double u2 = u * u;
            const double u3 = u2 * u;
            const double h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
            const double h10 = u3 - (2.0 * u2) + u;
            const double h01 = (-2.0 * u3) + (3.0 * u2);
            const double h11 = u3 - u2;

            const glm::dvec3 p0 = glm::dvec3(segment.start.position_m);
            const glm::dvec3 p1 = glm::dvec3(segment.end.position_m);
            const glm::dvec3 m0 = glm::dvec3(segment.start.velocity_mps) * segment.dt_s;
            const glm::dvec3 m1 = glm::dvec3(segment.end.velocity_mps) * segment.dt_s;
            return (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
        }

        WorldVec3 eval_segment_world_position(const orbitsim::TrajectorySegment &segment,
                                              const double t_s,
                                              const WorldVec3 &reference_body_world,
                                              const WorldVec3 &align_delta_world)
        {
            return reference_body_world + eval_segment_local_position(segment, t_s) + align_delta_world;
        }

        bool should_split_interval(const orbitsim::TrajectorySegment &segment,
                                   const double t0_s,
                                   const double t1_s,
                                   const int depth,
                                   const OrbitPlotLodBuilder::CameraContext &camera,
                                   const double error_px_threshold,
                                   const WorldVec3 &reference_body_world,
                                   const WorldVec3 &align_delta_world,
                                   const WorldVec3 &a_world,
                                   const WorldVec3 &b_world)
        {
            if (depth >= kMaxSubdivisionDepth || !(error_px_threshold > 0.0))
            {
                return false;
            }

            const double dt_s = t1_s - t0_s;
            if (!std::isfinite(dt_s) || dt_s <= kMinSubdivisionDtS)
            {
                return false;
            }

            const double tm_s = 0.5 * (t0_s + t1_s);
            if (!(tm_s > t0_s) || !(tm_s < t1_s))
            {
                return false;
            }

            const WorldVec3 curve_mid_world =
                    eval_segment_world_position(segment, tm_s, reference_body_world, align_delta_world);
            const WorldVec3 chord_mid_world = 0.5 * (a_world + b_world);

            const double error_m = glm::length(glm::dvec3(curve_mid_world - chord_mid_world));
            if (!std::isfinite(error_m) || error_m <= 0.0)
            {
                return false;
            }

            const double meters_per_px = meters_per_px_at_world(camera, curve_mid_world);
            if (!std::isfinite(meters_per_px) || meters_per_px <= 0.0)
            {
                return false;
            }

            const double error_px = error_m / meters_per_px;
            return std::isfinite(error_px) && error_px > error_px_threshold;
        }

        bool frustum_contains_point_margin(const OrbitPlotLodBuilder::FrustumContext &frustum,
                                           const WorldVec3 &point_world,
                                           const double margin_ratio)
        {
            if (!frustum.valid)
            {
                return true;
            }

            const double safe_margin = std::isfinite(margin_ratio) ? std::max(0.0, margin_ratio) : 0.0;

            const glm::vec3 point_local = world_to_local(point_world, frustum.origin_world);
            const glm::vec4 clip = frustum.viewproj * glm::vec4(point_local, 1.0f);
            if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w))
            {
                return false;
            }

            const double w = static_cast<double>(clip.w);
            const double aw = std::abs(w);
            if (!(aw > 1.0e-6))
            {
                return false;
            }

            const double x = static_cast<double>(clip.x);
            const double y = static_cast<double>(clip.y);
            const double z = static_cast<double>(clip.z);

            const double xy_bound = (1.0 + safe_margin) * aw;
            const double z_min = -safe_margin * aw;
            const double z_max = (1.0 + safe_margin) * aw;

            return (x >= -xy_bound && x <= xy_bound) &&
                   (y >= -xy_bound && y <= xy_bound) &&
                   (z >= z_min && z <= z_max);
        }

        bool frustum_accept_segment_margin(const OrbitPlotLodBuilder::FrustumContext &frustum,
                                           const WorldVec3 &a_world,
                                           const WorldVec3 &b_world,
                                           const double margin_ratio)
        {
            if (!frustum.valid)
            {
                return true;
            }

            if (frustum_contains_point_margin(frustum, a_world, margin_ratio) ||
                frustum_contains_point_margin(frustum, b_world, margin_ratio))
            {
                return true;
            }

            const WorldVec3 mid_world = 0.5 * (a_world + b_world);
            return frustum_contains_point_margin(frustum, mid_world, margin_ratio);
        }

        std::size_t find_anchor_segment_index(const std::vector<CandidateSegment> &segments, const double anchor_time_s)
        {
            if (segments.empty() || !std::isfinite(anchor_time_s))
            {
                return std::numeric_limits<std::size_t>::max();
            }

            auto it = std::lower_bound(segments.begin(),
                                       segments.end(),
                                       anchor_time_s,
                                       [](const CandidateSegment &seg, const double t) {
                                           return seg.t1_s < t;
                                       });
            if (it == segments.end())
            {
                return segments.size() - 1;
            }
            return static_cast<std::size_t>(std::distance(segments.begin(), it));
        }
    } // namespace

    OrbitPlotLodBuilder::RenderResult OrbitPlotLodBuilder::build_render_lod(
            const std::span<const orbitsim::TrajectorySegment> segments_bci,
            const WorldVec3 &reference_body_world,
            const WorldVec3 &align_delta_world,
            const CameraContext &camera,
            const RenderSettings &settings,
            const double t_start_s,
            const double t_end_s)
    {
        RenderResult out{};
        if (segments_bci.empty() || !(t_end_s > t_start_s) || settings.max_segments == 0)
        {
            return out;
        }

        const double error_px_threshold = (std::isfinite(settings.error_px) && settings.error_px > 0.0)
                                              ? settings.error_px
                                              : 0.75;

        // Collect clipped solver segments with camera distance for sorting.
        struct IndexedSegment
        {
            const orbitsim::TrajectorySegment *segment{nullptr};
            double clip_t0_s{0.0};
            double clip_t1_s{0.0};
            double camera_dist_sq{0.0};
        };

        std::vector<IndexedSegment> clipped{};
        clipped.reserve(segments_bci.size());

        for (const orbitsim::TrajectorySegment &segment : segments_bci)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = seg_t0_s + segment.dt_s;
            const double clip_t0_s = std::max(seg_t0_s, t_start_s);
            const double clip_t1_s = std::min(seg_t1_s, t_end_s);
            if (!(clip_t1_s > clip_t0_s))
            {
                continue;
            }

            const double mid_t_s = 0.5 * (clip_t0_s + clip_t1_s);
            const glm::dvec3 mid_local = eval_segment_local_position(segment, mid_t_s);
            const glm::dvec3 mid_world = glm::dvec3(reference_body_world + align_delta_world) + mid_local;
            const glm::dvec3 delta = mid_world - camera.camera_world;

            clipped.push_back(IndexedSegment{
                    .segment = &segment,
                    .clip_t0_s = clip_t0_s,
                    .clip_t1_s = clip_t1_s,
                    .camera_dist_sq = glm::dot(delta, delta),
            });
        }

        // Sort by camera distance — nearest segments get subdivision budget first.
        std::sort(clipped.begin(), clipped.end(),
                  [](const IndexedSegment &a, const IndexedSegment &b) {
                      return a.camera_dist_sq < b.camera_dist_sq;
                  });

        std::vector<WorkItem> stack{};
        stack.reserve(64);
        out.segments.reserve(std::min(settings.max_segments, clipped.size() * 4));
        bool budget_exhausted = false;

        for (const IndexedSegment &indexed : clipped)
        {
            stack.push_back(WorkItem{
                    .segment = indexed.segment,
                    .t0_s = indexed.clip_t0_s,
                    .t1_s = indexed.clip_t1_s,
                    .depth = 0,
            });

            while (!stack.empty())
            {
                const WorkItem item = stack.back();
                stack.pop_back();
                if (!item.segment || !(item.t1_s > item.t0_s))
                {
                    continue;
                }

                const WorldVec3 a_world =
                        eval_segment_world_position(*item.segment, item.t0_s, reference_body_world, align_delta_world);
                const WorldVec3 b_world =
                        eval_segment_world_position(*item.segment, item.t1_s, reference_body_world, align_delta_world);

                // Once budget is exhausted, accept chord without further splitting.
                if (!budget_exhausted &&
                    should_split_interval(*item.segment,
                                          item.t0_s,
                                          item.t1_s,
                                          item.depth,
                                          camera,
                                          error_px_threshold,
                                          reference_body_world,
                                          align_delta_world,
                                          a_world,
                                          b_world))
                {
                    const double mid_t_s = 0.5 * (item.t0_s + item.t1_s);
                    stack.push_back(WorkItem{
                            .segment = item.segment,
                            .t0_s = mid_t_s,
                            .t1_s = item.t1_s,
                            .depth = item.depth + 1,
                    });
                    stack.push_back(WorkItem{
                            .segment = item.segment,
                            .t0_s = item.t0_s,
                            .t1_s = mid_t_s,
                            .depth = item.depth + 1,
                    });
                    continue;
                }

                out.segments.push_back(RenderSegment{
                        .a_world = a_world,
                        .b_world = b_world,
                        .t0_s = item.t0_s,
                        .t1_s = item.t1_s,
                });

                if (out.segments.size() >= settings.max_segments)
                {
                    budget_exhausted = true;
                }
            }
        }

        if (budget_exhausted)
        {
            out.cap_hit = true;
        }

        // Restore time order for rendering.
        std::sort(out.segments.begin(), out.segments.end(),
                  [](const RenderSegment &a, const RenderSegment &b) {
                      return a.t0_s < b.t0_s;
                  });

        return out;
    }

    OrbitPlotLodBuilder::PickResult OrbitPlotLodBuilder::build_pick_lod(
            const std::span<const orbitsim::TrajectorySegment> segments_bci,
            const WorldVec3 &reference_body_world,
            const WorldVec3 &align_delta_world,
            const FrustumContext &frustum,
            const PickSettings &settings,
            const double t_start_s,
            const double t_end_s,
            const std::span<const double> anchor_times_s)
    {
        PickResult out{};
        if (segments_bci.empty() || !(t_end_s > t_start_s))
        {
            return out;
        }

        const double frustum_margin_ratio =
                (std::isfinite(settings.frustum_margin_ratio) && settings.frustum_margin_ratio >= 0.0)
                        ? settings.frustum_margin_ratio
                        : 0.05;

        std::vector<CandidateSegment> visible_segments{};
        visible_segments.reserve(segments_bci.size());

        for (const orbitsim::TrajectorySegment &segment : segments_bci)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = seg_t0_s + segment.dt_s;
            const double clip_t0_s = std::max(seg_t0_s, t_start_s);
            const double clip_t1_s = std::min(seg_t1_s, t_end_s);
            if (!(clip_t1_s > clip_t0_s))
            {
                continue;
            }

            const WorldVec3 a_world =
                    eval_segment_world_position(segment, clip_t0_s, reference_body_world, align_delta_world);
            const WorldVec3 b_world =
                    eval_segment_world_position(segment, clip_t1_s, reference_body_world, align_delta_world);

            ++out.segments_before_cull;
            if (!frustum_accept_segment_margin(frustum, a_world, b_world, frustum_margin_ratio))
            {
                continue;
            }

            visible_segments.push_back(CandidateSegment{
                    .a_world = a_world,
                    .b_world = b_world,
                    .t0_s = clip_t0_s,
                    .t1_s = clip_t1_s,
            });
        }

        out.segments_after_cull = visible_segments.size();
        if (visible_segments.empty())
        {
            return out;
        }

        const std::size_t max_segments = settings.max_segments;
        if (max_segments == 0)
        {
            out.cap_hit = true;
            return out;
        }

        if (visible_segments.size() <= max_segments)
        {
            out.segments.reserve(visible_segments.size());
            for (const CandidateSegment &segment : visible_segments)
            {
                out.segments.push_back(PickSegment{
                        .a_world = segment.a_world,
                        .b_world = segment.b_world,
                        .t0_s = segment.t0_s,
                        .t1_s = segment.t1_s,
                });
            }
            return out;
        }

        out.cap_hit = true;

        const std::size_t n = visible_segments.size();
        std::vector<bool> keep(n, false);
        std::size_t keep_count = 0;

        auto mark_keep = [&](const std::size_t idx) {
            if (idx >= n || keep[idx] || keep_count >= max_segments)
            {
                return;
            }
            keep[idx] = true;
            ++keep_count;
        };

        // Always preserve path endpoints in visible-space.
        mark_keep(0);
        if (n > 1)
        {
            mark_keep(n - 1);
        }

        // Preserve anchor-near segments in requested order (e.g. now/maneuver times).
        for (const double anchor_time_s : anchor_times_s)
        {
            if (keep_count >= max_segments)
            {
                break;
            }
            const std::size_t anchor_idx = find_anchor_segment_index(visible_segments, anchor_time_s);
            if (anchor_idx != std::numeric_limits<std::size_t>::max())
            {
                mark_keep(anchor_idx);
            }
        }

        // Fill the remaining budget with uniform decimation over visible segments.
        if (keep_count < max_segments)
        {
            const std::size_t target_count = max_segments;
            const double span = static_cast<double>(n - 1);
            for (std::size_t i = 0; i < target_count && keep_count < max_segments; ++i)
            {
                const double u = (target_count > 1)
                                         ? (static_cast<double>(i) / static_cast<double>(target_count - 1))
                                         : 0.0;
                const std::size_t idx = static_cast<std::size_t>(std::llround(u * span));
                mark_keep(std::min(idx, n - 1));
            }
        }

        out.segments.reserve(keep_count);
        for (std::size_t i = 0; i < n; ++i)
        {
            if (!keep[i])
            {
                continue;
            }

            const CandidateSegment &segment = visible_segments[i];
            out.segments.push_back(PickSegment{
                    .a_world = segment.a_world,
                    .b_world = segment.b_world,
                    .t0_s = segment.t0_s,
                    .t1_s = segment.t1_s,
            });
        }

        return out;
    }
} // namespace Game
