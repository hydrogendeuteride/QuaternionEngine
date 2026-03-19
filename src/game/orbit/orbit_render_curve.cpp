#include "game/orbit/orbit_render_curve.h"
#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_plot_util.h"

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr std::size_t kUniformErrorSamples = 9;
        constexpr std::size_t kMidpointErrorSamples = 8;
        constexpr std::size_t kBoundaryErrorSamples = 8;
        constexpr int kMaxSubdivisionDepth = 24;
        constexpr double kMinSubdivisionDtS = 1.0e-6;

        struct WorkItem
        {
            const orbitsim::TrajectorySegment *segment{nullptr};
            double t0_s{0.0};
            double t1_s{0.0};
            int depth{0};
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
        };

        struct CandidateSegment
        {
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
            double t0_s{0.0};
            double t1_s{0.0};
        };

        double segment_end_time(const orbitsim::TrajectorySegment &segment)
        {
            return segment.t0_s + segment.dt_s;
        }

        WorldVec3 eval_segment_world_position(const orbitsim::TrajectorySegment &segment,
                                              const double t_s,
                                              const WorldVec3 &reference_body_world,
                                              const WorldVec3 &align_delta_world)
        {
            return reference_body_world + OrbitPlotUtil::eval_segment_local_position(segment, t_s) + align_delta_world;
        }

        bool node_overlaps_window(const OrbitRenderCurve::Node &node,
                                  const double t_start_s,
                                  const double t_end_s)
        {
            const double node_t0_s = node.segment.t0_s;
            const double node_t1_s = segment_end_time(node.segment);
            return std::isfinite(node_t0_s) &&
                   std::isfinite(node_t1_s) &&
                   node_t1_s > t_start_s &&
                   node_t0_s < t_end_s;
        }

        bool interval_contains_interior_time(const double interval_t0_s,
                                             const double interval_t1_s,
                                             const double t_s)
        {
            if (!(interval_t1_s > interval_t0_s) || !std::isfinite(t_s))
            {
                return false;
            }

            const double epsilon_s =
                    std::max(1.0e-9, std::abs(interval_t1_s - interval_t0_s) * 1.0e-9);
            return t_s > (interval_t0_s + epsilon_s) && t_s < (interval_t1_s - epsilon_s);
        }

        bool node_requires_anchor_descend(const OrbitRenderCurve::Node &node,
                                          const double t_start_s,
                                          const double t_end_s,
                                          const std::span<const double> anchor_times_s)
        {
            if (node.is_leaf())
            {
                return false;
            }

            const double node_t0_s = node.segment.t0_s;
            const double node_t1_s = segment_end_time(node.segment);
            if (!(node_t1_s > node_t0_s))
            {
                return false;
            }

            if (interval_contains_interior_time(node_t0_s, node_t1_s, t_start_s) ||
                interval_contains_interior_time(node_t0_s, node_t1_s, t_end_s))
            {
                return true;
            }

            for (const double anchor_time_s : anchor_times_s)
            {
                if (interval_contains_interior_time(node_t0_s, node_t1_s, anchor_time_s))
                {
                    return true;
                }
            }

            return false;
        }

        bool should_split_interval(const orbitsim::TrajectorySegment &segment,
                                   const double t0_s,
                                   const double t1_s,
                                   const int depth,
                                   const OrbitRenderCurve::CameraContext &camera,
                                   const double error_px_threshold,
                                   const WorldVec3 &reference_body_world,
                                   const WorldVec3 &align_delta_world,
                                   const WorldVec3 &a_world,
                                   const WorldVec3 &b_world,
                                   WorldVec3 &out_mid_world)
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

            const double meters_per_px = OrbitPlotUtil::meters_per_px_at_world(
                    camera.camera_world, camera.tan_half_fov, camera.viewport_height_px, curve_mid_world);
            if (!std::isfinite(meters_per_px) || meters_per_px <= 0.0)
            {
                return false;
            }

            const double error_px = error_m / meters_per_px;
            if (std::isfinite(error_px) && error_px > error_px_threshold)
            {
                out_mid_world = curve_mid_world;
                return true;
            }
            return false;
        }

        std::size_t find_segment_for_time(const std::span<const orbitsim::TrajectorySegment> segments,
                                          const std::size_t first_index,
                                          const std::size_t last_index,
                                          const double t_s)
        {
            if (last_index <= first_index + 1)
            {
                return first_index;
            }

            std::size_t lo = first_index;
            std::size_t hi = last_index - 1;
            while (lo < hi)
            {
                const std::size_t mid = lo + ((hi - lo) / 2);
                const double mid_t1_s = segment_end_time(segments[mid]);
                if (t_s < mid_t1_s)
                {
                    hi = mid;
                }
                else
                {
                    lo = mid + 1;
                }
            }
            return lo;
        }

        bool sample_source_position(const std::span<const orbitsim::TrajectorySegment> segments,
                                    const std::size_t first_index,
                                    const std::size_t last_index,
                                    const double t_s,
                                    glm::dvec3 &out_position_m)
        {
            if (first_index >= last_index || last_index > segments.size() || !std::isfinite(t_s))
            {
                return false;
            }

            const std::size_t segment_index = find_segment_for_time(segments, first_index, last_index, t_s);
            if (segment_index >= last_index)
            {
                return false;
            }

            out_position_m = OrbitPlotUtil::eval_segment_local_position(segments[segment_index], t_s);
            return std::isfinite(out_position_m.x) &&
                   std::isfinite(out_position_m.y) &&
                   std::isfinite(out_position_m.z);
        }

        orbitsim::TrajectorySegment merge_segment_range(const std::span<const orbitsim::TrajectorySegment> segments,
                                                        const std::size_t first_index,
                                                        const std::size_t last_index)
        {
            const orbitsim::TrajectorySegment &first = segments[first_index];
            const orbitsim::TrajectorySegment &last = segments[last_index - 1];
            const double t0_s = first.t0_s;
            const double t1_s = segment_end_time(last);

            orbitsim::TrajectorySegment merged{};
            merged.t0_s = t0_s;
            merged.dt_s = std::max(0.0, t1_s - t0_s);
            merged.start = first.start;
            merged.end = last.end;
            merged.flags = 0u;
            return merged;
        }

        double estimate_merge_error(const std::span<const orbitsim::TrajectorySegment> segments,
                                    const std::size_t first_index,
                                    const std::size_t last_index,
                                    const orbitsim::TrajectorySegment &merged)
        {
            if (last_index <= first_index + 1 || !(merged.dt_s > 0.0))
            {
                return 0.0;
            }

            const double t0_s = merged.t0_s;
            const double t1_s = segment_end_time(merged);
            if (!(t1_s > t0_s))
            {
                return 0.0;
            }

            double max_error_m = 0.0;
            const auto observe_time = [&](const double sample_time_s) {
                if (!(sample_time_s > t0_s) || !(sample_time_s < t1_s))
                {
                    return;
                }

                glm::dvec3 source_position_m{0.0, 0.0, 0.0};
                if (!sample_source_position(segments, first_index, last_index, sample_time_s, source_position_m))
                {
                    return;
                }

                const glm::dvec3 merged_position_m = OrbitPlotUtil::eval_segment_local_position(merged, sample_time_s);
                const double error_m = OrbitPredictionMath::safe_length(source_position_m - merged_position_m);
                if (std::isfinite(error_m))
                {
                    max_error_m = std::max(max_error_m, error_m);
                }
            };

            for (std::size_t i = 1; i + 1 < kUniformErrorSamples; ++i)
            {
                const double u = static_cast<double>(i) / static_cast<double>(kUniformErrorSamples - 1);
                observe_time(t0_s + ((t1_s - t0_s) * u));
            }

            const std::size_t range_count = last_index - first_index;
            const std::size_t midpoint_samples = std::min(range_count, kMidpointErrorSamples);
            for (std::size_t i = 0; i < midpoint_samples; ++i)
            {
                const std::size_t offset =
                        std::min(range_count - 1,
                                 static_cast<std::size_t>(
                                         ((i * 2u) + 1u) * range_count / (midpoint_samples * 2u)));
                const std::size_t segment_index = first_index + offset;
                const orbitsim::TrajectorySegment &segment = segments[segment_index];
                observe_time(segment.t0_s + (segment.dt_s * 0.5));
            }

            const std::size_t boundary_count = std::min(range_count - 1, kBoundaryErrorSamples);
            for (std::size_t i = 0; i < boundary_count; ++i)
            {
                const std::size_t boundary_offset =
                        1 + (((i + 1u) * (range_count - 1u)) / (boundary_count + 1u));
                observe_time(segment_end_time(segments[first_index + boundary_offset - 1]));
            }

            return max_error_m;
        }

        bool frame_transform_is_identity(const glm::dmat3 &frame_to_world)
        {
            constexpr double kIdentityEpsilon = 1.0e-12;

            for (int col = 0; col < 3; ++col)
            {
                for (int row = 0; row < 3; ++row)
                {
                    const double expected = (col == row) ? 1.0 : 0.0;
                    const double value = frame_to_world[col][row];
                    if (!std::isfinite(value) || std::abs(value - expected) > kIdentityEpsilon)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        std::vector<orbitsim::TrajectorySegment> transform_segments_to_world_basis(
                const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                const glm::dmat3 &frame_to_world)
        {
            std::vector<orbitsim::TrajectorySegment> transformed_segments{};
            transformed_segments.reserve(traj_segments.size());

            for (const orbitsim::TrajectorySegment &segment : traj_segments)
            {
                orbitsim::TrajectorySegment transformed = segment;
                transformed.start.position_m = frame_to_world * glm::dvec3(segment.start.position_m);
                transformed.start.velocity_mps = frame_to_world * glm::dvec3(segment.start.velocity_mps);
                transformed.end.position_m = frame_to_world * glm::dvec3(segment.end.position_m);
                transformed.end.velocity_mps = frame_to_world * glm::dvec3(segment.end.velocity_mps);
                transformed_segments.push_back(transformed);
            }

            return transformed_segments;
        }

        bool frustum_contains_point_margin(const OrbitRenderCurve::FrustumContext &frustum,
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

        bool frustum_accept_segment_margin(const OrbitRenderCurve::FrustumContext &frustum,
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

    void OrbitRenderCurve::clear()
    {
        _nodes.clear();
        _root_index = kInvalidNodeIndex;
    }

    OrbitRenderCurve OrbitRenderCurve::build(const std::span<const orbitsim::TrajectorySegment> source_segments)
    {
        OrbitRenderCurve curve{};
        if (source_segments.empty())
        {
            return curve;
        }

        curve._nodes.reserve((source_segments.size() * 2) - 1);
        const auto build_node = [&](auto &&self, const std::size_t first_index, const std::size_t last_index) -> uint32_t {
            if (last_index <= first_index || last_index > source_segments.size())
            {
                return kInvalidNodeIndex;
            }

            if (last_index == first_index + 1)
            {
                Node node{};
                node.segment = source_segments[first_index];
                node.max_error_m = 0.0;
                curve._nodes.push_back(std::move(node));
                return static_cast<uint32_t>(curve._nodes.size() - 1);
            }

            const std::size_t mid_index = first_index + ((last_index - first_index) / 2);
            const uint32_t left_child = self(self, first_index, mid_index);
            const uint32_t right_child = self(self, mid_index, last_index);

            Node node{};
            node.segment = merge_segment_range(source_segments, first_index, last_index);
            node.left_child = left_child;
            node.right_child = right_child;
            node.max_error_m = estimate_merge_error(source_segments, first_index, last_index, node.segment);
            curve._nodes.push_back(std::move(node));
            return static_cast<uint32_t>(curve._nodes.size() - 1);
        };

        curve._root_index = build_node(build_node, 0, source_segments.size());
        return curve;
    }

    void OrbitRenderCurve::select_segments(const OrbitRenderCurve &curve,
                                           const SelectionContext &ctx,
                                           const double t_start_s,
                                           const double t_end_s,
                                           std::vector<orbitsim::TrajectorySegment> &out_segments)
    {
        out_segments.clear();
        if (curve.empty() || !(t_end_s > t_start_s))
        {
            return;
        }

        std::vector<uint32_t> stack{};
        stack.push_back(curve.root_index());

        while (!stack.empty())
        {
            const uint32_t node_index = stack.back();
            stack.pop_back();
            if (node_index >= curve._nodes.size())
            {
                continue;
            }

            const Node &node = curve._nodes[node_index];
            if (!node_overlaps_window(node, t_start_s, t_end_s))
            {
                continue;
            }

            bool descend = node_requires_anchor_descend(node, t_start_s, t_end_s, ctx.anchor_times_s);
            if (!node.is_leaf() &&
                !descend &&
                std::isfinite(node.max_error_m) &&
                node.max_error_m > 0.0 &&
                std::isfinite(ctx.error_px) &&
                ctx.error_px > 0.0)
            {
                const double tm_s = node.segment.t0_s + (node.segment.dt_s * 0.5);
                const WorldVec3 midpoint_world =
                        ctx.reference_body_world +
                        WorldVec3(ctx.frame_to_world * OrbitPlotUtil::eval_segment_local_position(node.segment, tm_s)) +
                        ctx.align_delta_world;
                const double error_px = node.max_error_m / OrbitPlotUtil::meters_per_px_at_world(
                        ctx.camera_world, ctx.tan_half_fov, ctx.viewport_height_px, midpoint_world);
                descend = std::isfinite(error_px) && error_px > ctx.error_px;
            }

            if (descend)
            {
                if (node.right_child != kInvalidNodeIndex)
                {
                    stack.push_back(node.right_child);
                }
                if (node.left_child != kInvalidNodeIndex)
                {
                    stack.push_back(node.left_child);
                }
                continue;
            }

            out_segments.push_back(node.segment);
        }
    }

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
                const bool in_frustum = frustum_accept_segment_margin(
                        frustum, item.a_world, item.b_world, kRenderFrustumMargin);

                WorldVec3 mid_world{};
                if (in_frustum &&
                    should_split_interval(*item.segment,
                                          item.t0_s,
                                          item.t1_s,
                                          item.depth,
                                          camera,
                                          error_px_threshold,
                                          reference_body_world,
                                          align_delta_world,
                                          item.a_world,
                                          item.b_world,
                                          mid_world))
                {
                    const double mid_t_s = 0.5 * (item.t0_s + item.t1_s);
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

                out.segments.push_back(RenderSegment{
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
        RenderResult out{};
        if (curve.empty() || !(t_end_s > t_start_s) || max_segments == 0)
        {
            return out;
        }

        std::vector<orbitsim::TrajectorySegment> selected_segments{};
        select_segments(curve, ctx, t_start_s, t_end_s, selected_segments);
        if (selected_segments.empty())
        {
            return out;
        }

        const bool needs_world_basis_transform = !frame_transform_is_identity(ctx.frame_to_world);
        const std::vector<orbitsim::TrajectorySegment> transformed_segments =
                needs_world_basis_transform
                        ? transform_segments_to_world_basis(selected_segments, ctx.frame_to_world)
                        : std::vector<orbitsim::TrajectorySegment>{};
        const std::vector<orbitsim::TrajectorySegment> &segments_world_basis =
                needs_world_basis_transform ? transformed_segments : selected_segments;

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

        mark_keep(0);
        if (n > 1)
        {
            mark_keep(n - 1);
        }

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

    OrbitRenderCurve::PickResult OrbitRenderCurve::build_pick_lod(
            const OrbitRenderCurve &curve,
            const SelectionContext &ctx,
            const FrustumContext &frustum,
            const PickSettings &settings,
            const double t_start_s,
            const double t_end_s)
    {
        PickResult out{};
        if (curve.empty() || !(t_end_s > t_start_s))
        {
            return out;
        }

        std::vector<orbitsim::TrajectorySegment> selected_segments{};
        select_segments(curve, ctx, t_start_s, t_end_s, selected_segments);
        if (selected_segments.empty())
        {
            return out;
        }

        const bool needs_world_basis_transform = !frame_transform_is_identity(ctx.frame_to_world);
        const std::vector<orbitsim::TrajectorySegment> transformed_segments =
                needs_world_basis_transform
                        ? transform_segments_to_world_basis(selected_segments, ctx.frame_to_world)
                        : std::vector<orbitsim::TrajectorySegment>{};
        const std::vector<orbitsim::TrajectorySegment> &segments_world_basis =
                needs_world_basis_transform ? transformed_segments : selected_segments;

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
