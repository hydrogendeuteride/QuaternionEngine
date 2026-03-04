#include "game/orbit/orbit_plot_lod_builder.h"

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>

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
        if (segments_bci.empty() || !(t_end_s > t_start_s))
        {
            return out;
        }

        const double error_px_threshold = (std::isfinite(settings.error_px) && settings.error_px > 0.0)
                                              ? settings.error_px
                                              : 0.75;

        std::vector<WorkItem> stack{};
        stack.reserve(64);

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

            stack.push_back(WorkItem{
                    .segment = &segment,
                    .t0_s = clip_t0_s,
                    .t1_s = clip_t1_s,
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

                if (should_split_interval(*item.segment,
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
            }
        }

        if (settings.max_segments == 0)
        {
            if (!out.segments.empty())
            {
                out.cap_hit = true;
                out.segments.clear();
            }
            return out;
        }

        if (out.segments.size() > settings.max_segments)
        {
            out.cap_hit = true;

            struct PathPoint
            {
                WorldVec3 p_world{0.0, 0.0, 0.0};
                double t_s{0.0};
            };

            std::vector<PathPoint> path_points{};
            path_points.reserve(out.segments.size() + 1);
            path_points.push_back(PathPoint{
                    .p_world = out.segments.front().a_world,
                    .t_s = out.segments.front().t0_s,
            });
            for (const RenderSegment &segment : out.segments)
            {
                path_points.push_back(PathPoint{
                        .p_world = segment.b_world,
                        .t_s = segment.t1_s,
                });
            }

            std::vector<RenderSegment> decimated{};
            decimated.reserve(settings.max_segments);

            const double src_span = static_cast<double>(path_points.size() - 1);
            auto sample_point = [&](const double src_index_f) -> PathPoint {
                const double clamped = std::clamp(src_index_f, 0.0, src_span);
                const std::size_t lo = static_cast<std::size_t>(std::floor(clamped));
                const std::size_t hi = std::min(lo + 1, path_points.size() - 1);
                const double alpha = clamped - static_cast<double>(lo);

                const PathPoint &a = path_points[lo];
                const PathPoint &b = path_points[hi];
                return PathPoint{
                        .p_world = glm::mix(a.p_world, b.p_world, alpha),
                        .t_s = glm::mix(a.t_s, b.t_s, alpha),
                };
            };

            PathPoint prev = sample_point(0.0);
            for (std::size_t i = 1; i <= settings.max_segments; ++i)
            {
                const double src_index_f = (static_cast<double>(i) * src_span) / static_cast<double>(settings.max_segments);
                PathPoint cur = sample_point(src_index_f);
                if (!(cur.t_s > prev.t_s))
                {
                    const double safe_dt = 1.0e-9;
                    cur.t_s = prev.t_s + safe_dt;
                }

                decimated.push_back(RenderSegment{
                        .a_world = prev.p_world,
                        .b_world = cur.p_world,
                        .t0_s = prev.t_s,
                        .t1_s = cur.t_s,
                });
                prev = cur;
            }

            out.segments = std::move(decimated);
        }

        return out;
    }
} // namespace Game
