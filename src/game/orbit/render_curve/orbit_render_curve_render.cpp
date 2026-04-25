/// OrbitRenderCurve -- adaptive subdivision for rendering.
///
/// build_render_lod() walks each input segment and recursively bisects intervals
/// whose chord-vs-curve midpoint deviation exceeds a screen-pixel threshold.
/// Subdivision is stack-based (not recursive) to avoid deep call stacks.
/// Segments outside the frustum are skipped.

#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/render_curve/orbit_render_curve_internal.h"

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace Game
{
    namespace
    {
        constexpr int kMaxSubdivisionDepth = 24;       // hard limit to prevent runaway subdivision
        constexpr double kMinSubdivisionDtS = 1.0e-6;  // minimum time interval to subdivide (seconds)

        /// Pending subdivision work item on the explicit stack.
        struct WorkItem
        {
            const orbitsim::TrajectorySegment *segment{nullptr};
            double t0_s{0.0};
            double t1_s{0.0};
            int depth{0};
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
        };

        bool interval_midpoint(const double t0_s,
                               const double t1_s,
                               const int depth,
                               double &out_mid_t_s)
        {
            if (depth >= kMaxSubdivisionDepth)
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

            out_mid_t_s = tm_s;
            return true;
        }

        /// Decide whether to split a time interval based on pixel-error.
        /// Evaluates the curve midpoint, compares against the chord midpoint,
        /// and converts the distance to screen pixels using the camera's perspective.
        bool should_split_interval(const OrbitRenderCurve::CameraContext &camera,
                                   const OrbitRenderCurve::FrustumContext &frustum,
                                   const double error_px_threshold,
                                   const WorldVec3 &a_world,
                                   const WorldVec3 &b_world,
                                   const WorldVec3 &curve_mid_world)
        {
            if (!(error_px_threshold > 0.0))
            {
                return false;
            }

            const double projected_error_px = projected_point_to_segment_error_px(
                    frustum, camera.viewport_height_px, a_world, b_world, curve_mid_world);
            if (std::isfinite(projected_error_px) && projected_error_px > error_px_threshold)
            {
                return true;
            }

            if (std::isfinite(projected_error_px))
            {
                return false;
            }

            const WorldVec3 chord_mid_world = 0.5 * (a_world + b_world);
            const double error_m = glm::length(glm::dvec3(curve_mid_world - chord_mid_world));
            if (!std::isfinite(error_m) || error_m <= 0.0)
            {
                return false;
            }

            const double meters_per_px = OrbitPlotUtil::meters_per_px_at_world(
                    camera.camera_world, camera.tan_half_fov, camera.viewport_height_px, curve_mid_world);
            if (!std::isfinite(meters_per_px) || meters_per_px <= 0.0)
            {
                return false;
            }

            const double error_px = error_m / meters_per_px;
            if (std::isfinite(error_px) && error_px > error_px_threshold)
            {
                return true;
            }
            return false;
        }
    } // namespace

    OrbitRenderCurve::RenderResult OrbitRenderCurve::build_render_lod(
            const std::span<const orbitsim::TrajectorySegment> segments_bci,
            const WorldVec3 &reference_body_world,
            const WorldVec3 &align_delta_world,
            const CameraContext &camera,
            const RenderSettings &settings,
            const double t_start_s,
            const double t_end_s,
            const FrustumContext &frustum)
    {
        RenderResult out{};
        if (segments_bci.empty() || !(t_end_s > t_start_s) || settings.max_segments == 0)
        {
            return out;
        }

        const double error_px_threshold = (std::isfinite(settings.error_px) && settings.error_px > 0.0)
                                                  ? settings.error_px
                                                  : 0.75;

        std::vector<WorkItem> stack{};
        stack.reserve(64);
        out.segments.reserve(std::min(settings.max_segments, segments_bci.size() * 2));

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

            const WorldVec3 initial_a =
                    eval_segment_world_position(segment, clip_t0_s, reference_body_world, align_delta_world);
            const WorldVec3 initial_b =
                    eval_segment_world_position(segment, clip_t1_s, reference_body_world, align_delta_world);

            stack.push_back(WorkItem{
                    .segment = &segment,
                    .t0_s = clip_t0_s,
                    .t1_s = clip_t1_s,
                    .depth = 0,
                    .a_world = initial_a,
                    .b_world = initial_b,
            });

            while (!stack.empty())
            {
                if (out.segments.size() >= settings.max_segments)
                {
                    out.cap_hit = true;
                    stack.clear();
                    break;
                }

                const WorkItem item = stack.back();
                stack.pop_back();
                if (!item.segment || !(item.t1_s > item.t0_s))
                {
                    continue;
                }

                constexpr double kRenderFrustumMargin = 0.1;
                const bool chord_in_frustum = frustum_accept_segment_margin(
                        frustum, item.a_world, item.b_world, kRenderFrustumMargin);
                const bool curve_may_enter_frustum =
                        frustum_accept_hermite_interval_margin(frustum,
                                                               *item.segment,
                                                               item.t0_s,
                                                               item.t1_s,
                                                               reference_body_world,
                                                               align_delta_world,
                                                               kRenderFrustumMargin);
                if (!curve_may_enter_frustum)
                {
                    continue;
                }

                WorldVec3 mid_world{};
                double mid_t_s = 0.0;
                const bool can_split = interval_midpoint(item.t0_s, item.t1_s, item.depth, mid_t_s);
                if (curve_may_enter_frustum && can_split)
                {
                    mid_world = eval_segment_world_position(
                            *item.segment, mid_t_s, reference_body_world, align_delta_world);
                    const bool split_for_frustum_curve = !chord_in_frustum;
                    const bool split_for_screen_error =
                            should_split_interval(camera,
                                                  frustum,
                                                  error_px_threshold,
                                                  item.a_world,
                                                  item.b_world,
                                                  mid_world);

                    if (split_for_frustum_curve || split_for_screen_error)
                    {
                        stack.push_back(WorkItem{
                                .segment = item.segment,
                                .t0_s = mid_t_s,
                                .t1_s = item.t1_s,
                                .depth = item.depth + 1,
                                .a_world = mid_world,
                                .b_world = item.b_world,
                        });
                        stack.push_back(WorkItem{
                                .segment = item.segment,
                                .t0_s = item.t0_s,
                                .t1_s = mid_t_s,
                                .depth = item.depth + 1,
                                .a_world = item.a_world,
                                .b_world = mid_world,
                        });
                        continue;
                    }
                }

                out.segments.push_back(LineSegment{
                        .a_world = item.a_world,
                        .b_world = item.b_world,
                        .t0_s = item.t0_s,
                        .t1_s = item.t1_s,
                });
            }

            if (out.cap_hit)
            {
                break;
            }
        }

        return out;
    }

    OrbitRenderCurve::RenderResult OrbitRenderCurve::build_render_lod(
            const OrbitRenderCurve &curve,
            const SelectionContext &ctx,
            const FrustumContext &frustum,
            const std::size_t max_segments,
            const double t_start_s,
            const double t_end_s)
    {
        if (curve.empty() || !(t_end_s > t_start_s) || max_segments == 0)
        {
            return {};
        }

        const auto segments_world_basis = resolve_curve_segments(curve, ctx, t_start_s, t_end_s);
        if (segments_world_basis.empty())
        {
            return {};
        }

        CameraContext camera{};
        camera.camera_world = ctx.camera_world;
        camera.tan_half_fov = ctx.tan_half_fov;
        camera.viewport_height_px = ctx.viewport_height_px;

        RenderSettings settings{};
        settings.error_px = ctx.error_px;
        settings.max_segments = max_segments;

        return build_render_lod(
                segments_world_basis,
                ctx.reference_body_world,
                ctx.align_delta_world,
                camera,
                settings,
                t_start_s,
                t_end_s,
                frustum);
    }
} // namespace Game
