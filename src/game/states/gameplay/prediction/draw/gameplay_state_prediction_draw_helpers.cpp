#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include "game/orbit/orbit_plot_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace Game::PredictionDrawDetail
{
    namespace
    {
        bool sample_segment_world(const WorldVec3 &frame_origin_world,
                                  const glm::dmat3 &frame_to_world,
                                  const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                  double t_s,
                                  WorldVec3 &out_world);
    }

    void reset_orbit_plot_state(PickingSystem *picking,
                                OrbitPlotSystem *orbit_plot,
                                OrbitPlotPerfStats &perf,
                                const bool prediction_enabled)
    {
        if (picking)
        {
            // Orbit plot is emitted per-frame, so refresh pickable segments the same way.
            picking->clear_line_picks();
            picking->settings().enable_line_hover = prediction_enabled;
        }

        if (orbit_plot)
        {
            orbit_plot->clear_pending();
        }

        perf.render_lod_ms_last = 0.0;
        perf.pick_lod_ms_last = 0.0;
        perf.solver_segments_base = 0;
        perf.solver_segments_planned = 0;
        perf.pick_segments_before_cull = 0;
        perf.pick_segments = 0;
        perf.render_cap_hit_last_frame = false;
        perf.pick_cap_hit_last_frame = false;
        perf.planned_chunk_count = 0;
        perf.planned_chunks_drawn = 0;
        perf.planned_fallback_range_count = 0;
        perf.planned_chunk_enqueue_ms_last = 0.0;
        perf.planned_fallback_draw_ms_last = 0.0;
    }

    glm::vec4 scale_line_color(glm::vec4 color, const float line_alpha_scale)
    {
        color.a = std::clamp(color.a * line_alpha_scale, 0.0f, 1.0f);
        return color;
    }

    double compute_prediction_display_time_s(const double sim_time_s,
                                             const double last_sim_step_dt_s,
                                             const float fixed_delta_time,
                                             const float alpha_f)
    {
        const double interp_dt_s =
                (last_sim_step_dt_s > 0.0) ? last_sim_step_dt_s : static_cast<double>(fixed_delta_time);

        double now_s = sim_time_s;
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }

        if (!std::isfinite(now_s))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return now_s;
    }

    double compute_prediction_now_s(const double display_time_s,
                                    const double t0,
                                    const double t1)
    {
        if (!std::isfinite(display_time_s))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return std::clamp(display_time_s, t0, t1);
    }

    double meters_per_px_at_world(const OrbitDrawWindowContext &ctx, const WorldVec3 &p_world)
    {
        return OrbitPlotUtil::meters_per_px_at_world(ctx.camera_world, ctx.tan_half_fov, ctx.viewport_height_px, p_world);
    }

    double snap_time_past_straddling_segment(const std::vector<orbitsim::TrajectorySegment> &traj_segments, double t_s)
    {
        double snapped_t_s = t_s;
        for (const orbitsim::TrajectorySegment &segment : traj_segments)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                continue;
            }

            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = seg_t0_s + segment.dt_s;
            if (!std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s))
            {
                continue;
            }

            if (seg_t0_s < t_s && t_s < seg_t1_s)
            {
                snapped_t_s = seg_t1_s;
                break;
            }
        }

        return snapped_t_s;
    }

    std::vector<double> collect_maneuver_node_times(const std::vector<ManeuverNode> &nodes)
    {
        std::vector<double> node_times_s;
        node_times_s.reserve(nodes.size());

        for (const ManeuverNode &node : nodes)
        {
            if (std::isfinite(node.time_s))
            {
                node_times_s.push_back(node.time_s);
            }
        }

        std::sort(node_times_s.begin(), node_times_s.end());
        node_times_s.erase(std::unique(node_times_s.begin(), node_times_s.end()), node_times_s.end());
        return node_times_s;
    }

    std::size_t lower_bound_sample_index(const std::vector<orbitsim::TrajectorySample> &traj, const double t_s)
    {
        const auto it = std::lower_bound(traj.cbegin(),
                                         traj.cend(),
                                         t_s,
                                         [](const orbitsim::TrajectorySample &sample, const double t) {
                                             return sample.t_s < t;
                                         });
        return static_cast<std::size_t>(std::distance(traj.cbegin(), it));
    }

    WorldVec3 sample_polyline_world(const WorldVec3 &frame_origin_world,
                                    const glm::dmat3 &frame_to_world,
                                    const std::vector<orbitsim::TrajectorySample> &traj,
                                    const std::size_t i_lo,
                                    const std::size_t i_hi,
                                    const double t_s)
    {
        if (i_lo >= traj.size())
        {
            return WorldVec3(0.0);
        }

        const auto transform_local = [&](const glm::dvec3 &local) -> WorldVec3 {
            return frame_origin_world + WorldVec3(frame_to_world * local);
        };

        if (i_hi >= traj.size() || i_lo == i_hi)
        {
            return transform_local(glm::dvec3(traj[i_lo].position_m));
        }

        const orbitsim::TrajectorySample &a = traj[i_lo];
        const orbitsim::TrajectorySample &b = traj[i_hi];
        const double h = b.t_s - a.t_s;
        if (!(h > 0.0) || !std::isfinite(h))
        {
            return transform_local(glm::dvec3(a.position_m));
        }

        double u = (t_s - a.t_s) / h;
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

        const glm::dvec3 p0 = glm::dvec3(a.position_m);
        const glm::dvec3 p1 = glm::dvec3(b.position_m);
        const glm::dvec3 m0 = glm::dvec3(a.velocity_mps) * h;
        const glm::dvec3 m1 = glm::dvec3(b.velocity_mps) * h;
        const glm::dvec3 local = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
        return transform_local(local);
    }

    bool sample_prediction_path_world(const OrbitDrawWindowContext &ctx,
                                      const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                      const std::vector<orbitsim::TrajectorySample> &traj_samples,
                                      const double t_s,
                                      WorldVec3 &out_world)
    {
        out_world = WorldVec3(0.0);
        if (!std::isfinite(t_s))
        {
            return false;
        }

        if (!traj_segments.empty())
        {
            WorldVec3 sampled_world{0.0};
            if (sample_segment_world(ctx.ref_body_world, ctx.frame_to_world, traj_segments, t_s, sampled_world))
            {
                out_world = sampled_world + ctx.align_delta;
                return true;
            }
        }

        if (traj_samples.empty())
        {
            return false;
        }

        std::size_t i_hi = lower_bound_sample_index(traj_samples, t_s);
        if (i_hi >= traj_samples.size())
        {
            i_hi = traj_samples.size() - 1u;
        }
        const std::size_t i_lo = (i_hi > 0u) ? (i_hi - 1u) : i_hi;
        out_world = sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj_samples, i_lo, i_hi, t_s) +
                    ctx.align_delta;
        return true;
    }

    namespace
    {
        bool sample_segment_world(const WorldVec3 &frame_origin_world,
                                  const glm::dmat3 &frame_to_world,
                                  const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                  const double t_s,
                                  WorldVec3 &out_world)
        {
            if (traj_segments.empty() || !std::isfinite(t_s))
            {
                return false;
            }

            const orbitsim::TrajectorySegment *selected = nullptr;
            for (const orbitsim::TrajectorySegment &segment : traj_segments)
            {
                if (!(segment.dt_s > 0.0) || !std::isfinite(segment.t0_s))
                {
                    continue;
                }

                const double seg_t1_s = segment.t0_s + segment.dt_s;
                if (!std::isfinite(seg_t1_s))
                {
                    continue;
                }

                selected = &segment;
                if (t_s <= seg_t1_s)
                {
                    break;
                }
            }

            if (!selected)
            {
                return false;
            }

            const double sample_t_s = std::clamp(t_s, selected->t0_s, selected->t0_s + selected->dt_s);
            out_world = frame_origin_world +
                        WorldVec3(frame_to_world * OrbitPlotUtil::eval_segment_local_position(*selected, sample_t_s));
            return true;
        }
    }

    WorldVec3 compute_align_delta(const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                  const std::vector<orbitsim::TrajectorySample> &traj_base,
                                  const std::size_t i_hi,
                                  const WorldVec3 &ship_pos_world,
                                  const double now_s,
                                  const WorldVec3 &frame_origin_world,
                                  const glm::dmat3 &frame_to_world)
    {
        WorldVec3 predicted_now_world{0.0, 0.0, 0.0};
        if (!traj_segments.empty() &&
            sample_segment_world(frame_origin_world, frame_to_world, traj_segments, now_s, predicted_now_world))
        {
        }
        else if (i_hi > 0)
        {
            predicted_now_world = sample_polyline_world(frame_origin_world, frame_to_world, traj_base, i_hi - 1, i_hi, now_s);
        }
        else if (i_hi < traj_base.size())
        {
            predicted_now_world = sample_polyline_world(frame_origin_world, frame_to_world, traj_base, i_hi, i_hi, now_s);
        }

        const WorldVec3 align_delta = ship_pos_world - predicted_now_world;
        const double align_len = glm::length(glm::dvec3(align_delta));
        if (!std::isfinite(align_len) || align_len > 10'000.0)
        {
            return WorldVec3(0.0, 0.0, 0.0);
        }

        return align_delta;
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
} // namespace Game::PredictionDrawDetail
