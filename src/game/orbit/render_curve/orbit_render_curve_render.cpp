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


        std::size_t source_segment_index_for_time(
                const std::span<const orbitsim::TrajectorySegment> source_segments,
                const double t_s)
        {
            if (source_segments.empty())
            {
                return 0u;
            }

            if (!std::isfinite(t_s) || t_s <= source_segments.front().t0_s)
            {
                return 0u;
            }

            std::size_t lo = 0u;
            std::size_t hi = source_segments.size();
            while (lo < hi)
            {
                const std::size_t mid = lo + ((hi - lo) / 2u);
                const double mid_t1_s = segment_end_time(source_segments[mid]);
                if (!std::isfinite(mid_t1_s) || t_s < mid_t1_s)
                {
                    hi = mid;
                }
                else
                {
                    lo = mid + 1u;
                }
            }

            return std::min(lo, source_segments.size() - 1u);
        }

        bool sample_source_world_position(
                const std::span<const orbitsim::TrajectorySegment> source_segments,
                const OrbitRenderCurve::SelectionContext &ctx,
                const double t_s,
                WorldVec3 &out_world)
        {
            if (source_segments.empty() || !std::isfinite(t_s))
            {
                return false;
            }

            const std::size_t segment_index = source_segment_index_for_time(source_segments, t_s);
            if (segment_index >= source_segments.size())
            {
                return false;
            }

            const glm::dvec3 local_position =
                    OrbitPlotUtil::eval_segment_local_position(source_segments[segment_index], t_s);
            if (!std::isfinite(local_position.x) ||
                !std::isfinite(local_position.y) ||
                !std::isfinite(local_position.z))
            {
                return false;
            }

            out_world = ctx.reference_body_world +
                        WorldVec3(ctx.frame_to_world * local_position) +
                        ctx.align_delta_world;
            return std::isfinite(out_world.x) &&
                   std::isfinite(out_world.y) &&
                   std::isfinite(out_world.z);
        }

        bool source_sample_is_visible_hint(
                const OrbitRenderCurve::FrustumContext &frustum,
                const WorldVec3 &sample_world,
                const double margin_ratio)
        {
            return !frustum.valid || frustum_contains_point_margin(frustum, sample_world, margin_ratio);
        }

        bool source_interval_spans_many_segments(
                const std::span<const orbitsim::TrajectorySegment> source_segments,
                const double t0_s,
                const double t1_s,
                const std::size_t max_source_segments_per_interval)
        {
            if (source_segments.empty() ||
                max_source_segments_per_interval == 0u ||
                !(t1_s > t0_s))
            {
                return false;
            }

            const std::size_t i0 = source_segment_index_for_time(source_segments, t0_s);
            const std::size_t i1 = source_segment_index_for_time(source_segments, std::nextafter(t1_s, t0_s));
            if (i0 >= source_segments.size() || i1 >= source_segments.size())
            {
                return false;
            }

            const std::size_t covered_segments = (i1 >= i0) ? ((i1 - i0) + 1u) : ((i0 - i1) + 1u);
            return covered_segments > max_source_segments_per_interval;
        }

        bool should_split_source_interval(
                const std::span<const orbitsim::TrajectorySegment> source_segments,
                const OrbitRenderCurve::SelectionContext &ctx,
                const OrbitRenderCurve::CameraContext &camera,
                const OrbitRenderCurve::FrustumContext &frustum,
                const double error_px_threshold,
                const double t0_s,
                const double mid_t_s,
                const double t1_s,
                const WorldVec3 &a_world,
                const WorldVec3 &mid_world,
                const WorldVec3 &b_world)
        {
            if (should_split_interval(camera, frustum, error_px_threshold, a_world, b_world, mid_world))
            {
                return true;
            }

            const double q0_t_s = 0.5 * (t0_s + mid_t_s);
            const double q1_t_s = 0.5 * (mid_t_s + t1_s);

            WorldVec3 q0_world{};
            if (q0_t_s > t0_s && q0_t_s < t1_s &&
                sample_source_world_position(source_segments, ctx, q0_t_s, q0_world) &&
                should_split_interval(camera, frustum, error_px_threshold, a_world, b_world, q0_world))
            {
                return true;
            }

            WorldVec3 q1_world{};
            if (q1_t_s > t0_s && q1_t_s < t1_s &&
                sample_source_world_position(source_segments, ctx, q1_t_s, q1_world) &&
                should_split_interval(camera, frustum, error_px_threshold, a_world, b_world, q1_world))
            {
                return true;
            }

            return false;
        }

        OrbitRenderCurve::RenderResult build_source_backed_render_lod(
                const std::span<const orbitsim::TrajectorySegment> source_segments,
                const std::span<const orbitsim::TrajectorySegment> selected_window_segments_world_basis,
                const OrbitRenderCurve::SelectionContext &ctx,
                const OrbitRenderCurve::CameraContext &camera,
                const OrbitRenderCurve::RenderSettings &settings,
                const double t_start_s,
                const double t_end_s,
                const OrbitRenderCurve::FrustumContext &frustum)
        {
            OrbitRenderCurve::RenderResult out{};
            if (source_segments.empty() ||
                selected_window_segments_world_basis.empty() ||
                !(t_end_s > t_start_s) ||
                settings.max_segments == 0)
            {
                return out;
            }

            const double error_px_threshold = (std::isfinite(settings.error_px) && settings.error_px > 0.0)
                                                      ? settings.error_px
                                                      : 0.75;

            std::vector<WorkItem> stack{};
            stack.reserve(64);
            out.segments.reserve(std::min(settings.max_segments, selected_window_segments_world_basis.size() * 2u));

            for (const orbitsim::TrajectorySegment &window_segment : selected_window_segments_world_basis)
            {
                if (!(window_segment.dt_s > 0.0) || !std::isfinite(window_segment.dt_s))
                {
                    continue;
                }

                const double seg_t0_s = window_segment.t0_s;
                const double seg_t1_s = segment_end_time(window_segment);
                const double clip_t0_s = std::max(seg_t0_s, t_start_s);
                const double clip_t1_s = std::min(seg_t1_s, t_end_s);
                if (!(clip_t1_s > clip_t0_s))
                {
                    continue;
                }

                WorldVec3 initial_a{};
                WorldVec3 initial_b{};
                if (!sample_source_world_position(source_segments, ctx, clip_t0_s, initial_a) ||
                    !sample_source_world_position(source_segments, ctx, clip_t1_s, initial_b))
                {
                    continue;
                }

                stack.push_back(WorkItem{
                        .segment = &window_segment,
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
                    const bool approximate_curve_may_enter_frustum =
                            frustum_accept_hermite_interval_margin(frustum,
                                                                   *item.segment,
                                                                   item.t0_s,
                                                                   item.t1_s,
                                                                   ctx.reference_body_world,
                                                                   ctx.align_delta_world,
                                                                   kRenderFrustumMargin);

                    WorldVec3 mid_world{};
                    double mid_t_s = 0.0;
                    const bool can_split = interval_midpoint(item.t0_s, item.t1_s, item.depth, mid_t_s);
                    bool have_mid = false;
                    const auto ensure_mid_sample = [&]() {
                        if (have_mid)
                        {
                            return true;
                        }
                        have_mid = sample_source_world_position(source_segments, ctx, mid_t_s, mid_world);
                        return have_mid;
                    };

                    bool source_visibility_hint = false;
                    if (!approximate_curve_may_enter_frustum && !chord_in_frustum && can_split)
                    {
                        if (ensure_mid_sample())
                        {
                            source_visibility_hint = source_sample_is_visible_hint(
                                    frustum, mid_world, kRenderFrustumMargin);
                        }

                        const double q0_t_s = 0.5 * (item.t0_s + mid_t_s);
                        const double q1_t_s = 0.5 * (mid_t_s + item.t1_s);
                        WorldVec3 q_world{};
                        if (!source_visibility_hint &&
                            sample_source_world_position(source_segments, ctx, q0_t_s, q_world))
                        {
                            source_visibility_hint = source_sample_is_visible_hint(
                                    frustum, q_world, kRenderFrustumMargin);
                        }
                        if (!source_visibility_hint &&
                            sample_source_world_position(source_segments, ctx, q1_t_s, q_world))
                        {
                            source_visibility_hint = source_sample_is_visible_hint(
                                    frustum, q_world, kRenderFrustumMargin);
                        }
                    }

                    if (!approximate_curve_may_enter_frustum && !chord_in_frustum && !source_visibility_hint)
                    {
                        continue;
                    }

                    if (can_split && ensure_mid_sample())
                    {
                        constexpr std::size_t kMaxSourceSegmentsPerRenderInterval = 16u;
                        const bool split_for_frustum_curve = !chord_in_frustum;
                        const bool split_for_source_granularity =
                                source_interval_spans_many_segments(source_segments,
                                                                    item.t0_s,
                                                                    item.t1_s,
                                                                    kMaxSourceSegmentsPerRenderInterval);
                        const bool split_for_screen_error =
                                should_split_source_interval(source_segments,
                                                             ctx,
                                                             camera,
                                                             frustum,
                                                             error_px_threshold,
                                                             item.t0_s,
                                                             mid_t_s,
                                                             item.t1_s,
                                                             item.a_world,
                                                             mid_world,
                                                             item.b_world);

                        if (split_for_frustum_curve || split_for_source_granularity || split_for_screen_error)
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

                    out.segments.push_back(OrbitRenderCurve::LineSegment{
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
        RenderSettings settings{};
        settings.error_px = ctx.error_px;
        settings.max_segments = max_segments;
        return build_render_lod(curve, ctx, frustum, settings, t_start_s, t_end_s);
    }

    OrbitRenderCurve::RenderResult OrbitRenderCurve::build_render_lod(
            const OrbitRenderCurve &curve,
            const SelectionContext &ctx,
            const FrustumContext &frustum,
            const RenderSettings &settings,
            const double t_start_s,
            const double t_end_s)
    {
        if (curve.empty() || !(t_end_s > t_start_s) || settings.max_segments == 0)
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

        const std::span<const orbitsim::TrajectorySegment> source_segments = curve.source_segments();
        if (!source_segments.empty())
        {
            return build_source_backed_render_lod(source_segments,
                                                  segments_world_basis,
                                                  ctx,
                                                  camera,
                                                  settings,
                                                  t_start_s,
                                                  t_end_s,
                                                  frustum);
        }

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
