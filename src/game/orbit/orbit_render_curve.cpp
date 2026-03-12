#include "game/orbit/orbit_render_curve.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace Game
{
    namespace
    {
        constexpr std::size_t kUniformErrorSamples = 9;
        constexpr std::size_t kMidpointErrorSamples = 8;
        constexpr std::size_t kBoundaryErrorSamples = 8;

        double safe_length(const glm::dvec3 &v)
        {
            const double len2 = glm::dot(v, v);
            if (!std::isfinite(len2) || len2 <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(len2);
        }

        glm::dvec3 eval_segment_position(const orbitsim::TrajectorySegment &segment, const double t_s)
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

        double segment_end_time(const orbitsim::TrajectorySegment &segment)
        {
            return segment.t0_s + segment.dt_s;
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

        double meters_per_px_at_world(const OrbitRenderCurve::SelectionContext &ctx,
                                      const WorldVec3 &point_world)
        {
            const double dist_m = safe_length(glm::dvec3(point_world) - ctx.camera_world);
            if (!std::isfinite(dist_m) || dist_m <= 1.0e-3 ||
                !std::isfinite(ctx.tan_half_fov) || ctx.tan_half_fov <= 1.0e-8 ||
                !std::isfinite(ctx.viewport_height_px) || ctx.viewport_height_px <= 1.0)
            {
                return 1.0;
            }

            const double meters_per_px = (2.0 * ctx.tan_half_fov * dist_m) / ctx.viewport_height_px;
            if (!std::isfinite(meters_per_px) || meters_per_px <= 1.0e-6)
            {
                return 1.0;
            }
            return meters_per_px;
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

            out_position_m = eval_segment_position(segments[segment_index], t_s);
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

                const glm::dvec3 merged_position_m = eval_segment_position(merged, sample_time_s);
                const double error_m = safe_length(source_position_m - merged_position_m);
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

            bool descend = false;
            if (!node.is_leaf() &&
                std::isfinite(node.max_error_m) &&
                node.max_error_m > 0.0 &&
                std::isfinite(ctx.error_px) &&
                ctx.error_px > 0.0)
            {
                const double tm_s = node.segment.t0_s + (node.segment.dt_s * 0.5);
                const WorldVec3 midpoint_world =
                        ctx.reference_body_world +
                        WorldVec3(ctx.frame_to_world * eval_segment_position(node.segment, tm_s)) +
                        ctx.align_delta_world;
                const double error_px = node.max_error_m / meters_per_px_at_world(ctx, midpoint_world);
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
} // namespace Game
