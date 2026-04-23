/// OrbitRenderCurve -- frustum cull and downsampling for picking.
///
/// build_pick_lod() converts trajectory segments to world-space line segments,
/// frustum-culls them, then downsamples if the count exceeds the budget.
/// Downsampling preserves endpoints, anchor-time segments, and distributes
/// the remaining budget uniformly across the visible range.

#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/render_curve/orbit_render_curve_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace Game
{
    namespace
    {
        using LineSegment = OrbitRenderCurve::LineSegment;

        constexpr int kMaxPickFrustumSubdivisionDepth = 10;
        constexpr double kMinPickSubdivisionDtS = 1.0e-6;

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
            if (depth >= kMaxPickFrustumSubdivisionDepth)
            {
                return false;
            }

            const double dt_s = t1_s - t0_s;
            if (!std::isfinite(dt_s) || dt_s <= kMinPickSubdivisionDtS)
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

        /// Binary search for the segment whose time range contains anchor_time_s.
        /// Returns the index into the sorted segments vector, or size_t::max on failure.
        std::size_t find_anchor_segment_index(const std::vector<LineSegment> &segments, const double anchor_time_s)
        {
            if (segments.empty() || !std::isfinite(anchor_time_s))
            {
                return std::numeric_limits<std::size_t>::max();
            }

            auto it = std::lower_bound(segments.begin(),
                                       segments.end(),
                                       anchor_time_s,
                                       [](const LineSegment &seg, const double t) {
                                           return seg.t1_s < t;
                                       });
            if (it == segments.end())
            {
                return segments.size() - 1;
            }
            return static_cast<std::size_t>(std::distance(segments.begin(), it));
        }
    } // namespace

    OrbitRenderCurve::PickResult OrbitRenderCurve::build_pick_lod(
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

        std::vector<LineSegment> visible_segments{};
        visible_segments.reserve(segments_bci.size());
        std::vector<WorkItem> stack{};
        stack.reserve(32);

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
            stack.clear();
            stack.push_back(WorkItem{
                    .segment = &segment,
                    .t0_s = clip_t0_s,
                    .t1_s = clip_t1_s,
                    .depth = 0,
                    .a_world = a_world,
                    .b_world = b_world,
            });

            while (!stack.empty())
            {
                const WorkItem item = stack.back();
                stack.pop_back();
                if (!item.segment || !(item.t1_s > item.t0_s))
                {
                    continue;
                }

                const bool chord_in_frustum =
                        frustum_accept_segment_margin(frustum, item.a_world, item.b_world, frustum_margin_ratio);
                if (chord_in_frustum)
                {
                    visible_segments.push_back(LineSegment{
                            .a_world = item.a_world,
                            .b_world = item.b_world,
                            .t0_s = item.t0_s,
                            .t1_s = item.t1_s,
                    });
                    continue;
                }

                const bool curve_may_enter_frustum =
                        frustum_accept_hermite_interval_margin(frustum,
                                                               *item.segment,
                                                               item.t0_s,
                                                               item.t1_s,
                                                               reference_body_world,
                                                               align_delta_world,
                                                               frustum_margin_ratio);
                if (!curve_may_enter_frustum)
                {
                    continue;
                }

                double mid_t_s = 0.0;
                if (interval_midpoint(item.t0_s, item.t1_s, item.depth, mid_t_s))
                {
                    const WorldVec3 mid_world =
                            eval_segment_world_position(*item.segment, mid_t_s, reference_body_world, align_delta_world);
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

                visible_segments.push_back(LineSegment{
                        .a_world = item.a_world,
                        .b_world = item.b_world,
                        .t0_s = item.t0_s,
                        .t1_s = item.t1_s,
                });
            }
        }

        out.segments_after_cull = visible_segments.size();
        if (visible_segments.empty())
        {
            return out;
        }

        std::vector<double> scratch_anchor_times_s{};
        const std::span<const double> normalized_anchor_times_s =
                normalized_anchor_times(anchor_times_s, scratch_anchor_times_s);

        const std::size_t max_segments = settings.max_segments;
        if (max_segments == 0)
        {
            out.cap_hit = true;
            return out;
        }

        if (visible_segments.size() <= max_segments)
        {
            out.segments = visible_segments;
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

        mark_keep(0);
        if (n > 1)
        {
            mark_keep(n - 1);
        }

        for (const double anchor_time_s : normalized_anchor_times_s)
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
            if (keep[i])
            {
                out.segments.push_back(visible_segments[i]);
            }
        }

        return out;
    }

    OrbitRenderCurve::PickResult OrbitRenderCurve::build_pick_lod(
            const OrbitRenderCurve &curve,
            const SelectionContext &ctx,
            const FrustumContext &frustum,
            const PickSettings &settings,
            const double t_start_s,
            const double t_end_s)
    {
        if (curve.empty() || !(t_end_s > t_start_s))
        {
            return {};
        }

        const auto segments_world_basis = resolve_curve_segments(curve, ctx, t_start_s, t_end_s);
        if (segments_world_basis.empty())
        {
            return {};
        }

        return build_pick_lod(
                segments_world_basis,
                ctx.reference_body_world,
                ctx.align_delta_world,
                frustum,
                settings,
                t_start_s,
                t_end_s,
                ctx.anchor_times_s);
    }
} // namespace Game
