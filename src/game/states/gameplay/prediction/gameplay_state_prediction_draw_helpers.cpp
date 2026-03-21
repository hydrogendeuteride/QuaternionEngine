#include "game/states/gameplay/prediction/gameplay_state_prediction_draw_internal.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/orbit/orbit_plot_util.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>

namespace Game::PredictionDrawDetail
{
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

        // Match render interpolation: entities are rendered between prev/curr using `alpha_f`,
        // so treat "now" as within the previous->current fixed step interval.
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

    WorldVec3 compute_align_delta(const std::vector<orbitsim::TrajectorySample> &traj_base,
                                  const std::size_t i_hi,
                                  const WorldVec3 &ship_pos_world,
                                  const double now_s,
                                  const WorldVec3 &frame_origin_world,
                                  const glm::dmat3 &frame_to_world)
    {
        WorldVec3 predicted_now_world{0.0, 0.0, 0.0};
        if (i_hi > 0)
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

    namespace
    {
        WorldVec3 eval_segment_world_pos(const OrbitDrawWindowContext &ctx,
                                         const orbitsim::TrajectorySegment &segment,
                                         const double t_s)
        {
            return ctx.ref_body_world + OrbitPlotUtil::eval_segment_local_position(segment, t_s) + ctx.align_delta;
        }

        void emit_orbit_line(const OrbitDrawWindowContext &ctx,
                             const glm::vec4 &color,
                             const WorldVec3 &a_world,
                             const WorldVec3 &b_world)
        {
            if (ctx.orbit_plot)
            {
                ctx.orbit_plot->add_line(a_world, b_world, color, OrbitPlotDepth::DepthTested);
            }

            if (ctx.line_overlay_boost <= 0.0f || !ctx.orbit_plot)
            {
                return;
            }

            glm::vec4 overlay_color = color;
            overlay_color.a = std::clamp(overlay_color.a * ctx.line_overlay_boost, 0.0f, 1.0f);
            if (overlay_color.a > 0.0f)
            {
                ctx.orbit_plot->add_line(a_world, b_world, overlay_color, OrbitPlotDepth::AlwaysOnTop);
            }
        }

        void emit_gpu_root_segments(const OrbitDrawWindowContext &ctx,
                                    const OrbitPredictionDrawConfig &draw_config,
                                    const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                    const double t_start_s,
                                    const double t_end_s,
                                    const glm::vec4 &color,
                                    const bool dashed,
                                    const OrbitPlotDepth depth)
        {
            (void) draw_config;
            if (!ctx.orbit_plot || traj_segments.empty() || !(t_end_s > t_start_s))
            {
                return;
            }

            std::vector<OrbitPlotSystem::GpuRootSegment> roots;
            roots.reserve(traj_segments.size());
            double prefix_length_m = 0.0;
            for (const orbitsim::TrajectorySegment &segment : traj_segments)
            {
                if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
                {
                    continue;
                }

                OrbitPlotSystem::GpuRootSegment root{};
                root.t0_s = segment.t0_s;
                root.p0_bci = glm::dvec3(segment.start.position_m);
                root.v0_bci = glm::dvec3(segment.start.velocity_mps);
                root.p1_bci = glm::dvec3(segment.end.position_m);
                root.v1_bci = glm::dvec3(segment.end.velocity_mps);
                root.dt_s = segment.dt_s;
                root.prefix_length_m = prefix_length_m;
                roots.push_back(root);

                const double chord_m = glm::length(root.p1_bci - root.p0_bci);
                if (std::isfinite(chord_m) && chord_m > 0.0)
                {
                    prefix_length_m += chord_m;
                }
            }

            if (roots.empty())
            {
                return;
            }

            ctx.orbit_plot->add_gpu_root_batch(
                    std::make_shared<const std::vector<OrbitPlotSystem::GpuRootSegment>>(std::move(roots)),
                                               t_start_s,
                                               t_end_s,
                                               ctx.ref_body_world,
                                               ctx.align_delta,
                                               ctx.frame_to_world,
                                               color,
                                               dashed,
                                               depth);
        }

        void emit_render_lod_segments(const OrbitDrawWindowContext &ctx,
                                      const OrbitPredictionDrawConfig &draw_config,
                                      OrbitPlotPerfStats &perf,
                                      const OrbitRenderCurve::RenderResult &lod,
                                      const glm::vec4 &color,
                                      const bool dashed)
        {
            if (lod.cap_hit)
            {
                perf.render_cap_hit_last_frame = true;
                ++perf.render_cap_hits_total;
            }
            if (lod.segments.empty())
            {
                return;
            }

            if (!dashed)
            {
                for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
                {
                    emit_orbit_line(ctx, color, segment.a_world, segment.b_world);
                }
                return;
            }

            const double dash_on_px = draw_config.dashed_segment_on_px;
            const double dash_off_px = draw_config.dashed_segment_off_px;
            const double dash_period_px = dash_on_px + dash_off_px;

            double dash_phase_px = 0.0;
            for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
            {
                const double seg_m = glm::length(glm::dvec3(segment.b_world - segment.a_world));
                const double seg_dt_s = segment.t1_s - segment.t0_s;
                if (!std::isfinite(seg_m) || !(seg_m > 1.0e-9) || !std::isfinite(seg_dt_s) || !(seg_dt_s > 0.0))
                {
                    continue;
                }

                const glm::dvec3 seg_mid = glm::mix(glm::dvec3(segment.a_world), glm::dvec3(segment.b_world), 0.5);
                const double seg_mpp = meters_per_px_at_world(ctx, WorldVec3(seg_mid));
                if (!std::isfinite(seg_mpp) || !(seg_mpp > 1.0e-6))
                {
                    continue;
                }

                const double seg_px = seg_m / seg_mpp;
                if (!std::isfinite(seg_px) || !(seg_px > 1.0e-6))
                {
                    continue;
                }

                double cursor_px = 0.0;
                int dash_chunks = 0;
                while ((cursor_px + 1.0e-6) < seg_px &&
                       dash_chunks < draw_config.dash_max_chunks_per_segment)
                {
                    const bool phase_on = dash_phase_px < dash_on_px;
                    double phase_remaining_px = phase_on ? (dash_on_px - dash_phase_px) : (dash_period_px - dash_phase_px);
                    if (!std::isfinite(phase_remaining_px) || !(phase_remaining_px > 1.0e-6))
                    {
                        phase_remaining_px = 1.0;
                    }

                    const double step_px = std::min(phase_remaining_px, seg_px - cursor_px);
                    const double next_px = cursor_px + step_px;

                    if (phase_on)
                    {
                        const double u0 = std::clamp(cursor_px / seg_px, 0.0, 1.0);
                        const double u1 = std::clamp(next_px / seg_px, 0.0, 1.0);
                        if (u1 > u0)
                        {
                            const WorldVec3 a_world = glm::mix(segment.a_world, segment.b_world, u0);
                            const WorldVec3 b_world = glm::mix(segment.a_world, segment.b_world, u1);
                            emit_orbit_line(ctx, color, a_world, b_world);
                        }
                    }

                    cursor_px = next_px;
                    dash_phase_px += step_px;
                    if (dash_phase_px >= dash_period_px)
                    {
                        dash_phase_px = std::fmod(dash_phase_px, dash_period_px);
                    }
                    ++dash_chunks;
                }

                if ((cursor_px + 1.0e-6) < seg_px)
                {
                    const double remaining_px = seg_px - cursor_px;
                    dash_phase_px = std::fmod(dash_phase_px + remaining_px, dash_period_px);
                }
            }
        }

        void emit_cpu_render_lod(const OrbitDrawWindowContext &ctx,
                                 const OrbitPredictionDrawConfig &draw_config,
                                 OrbitPlotPerfStats &perf,
                                 const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                 const double t_start_s,
                                 const double t_end_s,
                                 const glm::vec4 &color,
                                 const bool dashed)
        {
            OrbitRenderCurve::RenderSettings lod_settings{};
            lod_settings.error_px = ctx.render_error_px;
            lod_settings.max_segments = ctx.render_max_segments;

            const auto render_lod_start_tp = std::chrono::steady_clock::now();
            const OrbitRenderCurve::RenderResult lod =
                    OrbitRenderCurve::build_render_lod(traj_segments,
                                                       ctx.ref_body_world,
                                                       ctx.align_delta,
                                                       ctx.lod_camera,
                                                       lod_settings,
                                                       t_start_s,
                                                       t_end_s,
                                                       ctx.render_frustum);
            perf.render_lod_ms_last +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                            .count();
            emit_render_lod_segments(ctx, draw_config, perf, lod, color, dashed);
        }

    } // namespace

    void draw_orbit_window(const OrbitDrawWindowContext &ctx,
                           const OrbitPredictionDrawConfig &draw_config,
                           OrbitPlotPerfStats &perf,
                           const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                           const double t_start_s,
                           const double t_end_s,
                           const glm::vec4 &color,
                           const bool dashed)
    {
        if (!(t_end_s > t_start_s) || traj_segments.empty())
        {
            return;
        }

        const bool needs_world_basis_transform = !frame_transform_is_identity(ctx.frame_to_world);
        const std::vector<orbitsim::TrajectorySegment> transformed_segments =
                needs_world_basis_transform
                        ? transform_segments_to_world_basis(traj_segments, ctx.frame_to_world)
                        : std::vector<orbitsim::TrajectorySegment>{};
        const std::vector<orbitsim::TrajectorySegment> &segments_world_basis =
                needs_world_basis_transform ? transformed_segments : traj_segments;

        const bool gpu_subdivision_enabled = false;
        if (gpu_subdivision_enabled)
        {
            emit_gpu_root_segments(ctx,
                                   draw_config,
                                   segments_world_basis,
                                   t_start_s,
                                   t_end_s,
                                   color,
                                   dashed,
                                   OrbitPlotDepth::DepthTested);

            if (ctx.line_overlay_boost > 0.0f)
            {
                glm::vec4 overlay_color = color;
                overlay_color.a = std::clamp(overlay_color.a * ctx.line_overlay_boost, 0.0f, 1.0f);
                if (overlay_color.a > 0.0f)
                {
                    emit_gpu_root_segments(ctx,
                                           draw_config,
                                           segments_world_basis,
                                           t_start_s,
                                           t_end_s,
                                           overlay_color,
                                           dashed,
                                           OrbitPlotDepth::AlwaysOnTop);
                }
            }
            return;
        }

        emit_cpu_render_lod(ctx, draw_config, perf, segments_world_basis, t_start_s, t_end_s, color, dashed);
    }

    void draw_adaptive_curve_window(const OrbitDrawWindowContext &ctx,
                                    const OrbitPredictionDrawConfig &draw_config,
                                    OrbitPlotPerfStats &perf,
                                    const OrbitRenderCurve &curve,
                                    const double t_start_s,
                                    const double t_end_s,
                                    const glm::vec4 &color,
                                    const bool dashed)
    {
        if (curve.empty() || !(t_end_s > t_start_s))
        {
            return;
        }

        OrbitRenderCurve::SelectionContext selection_ctx{};
        selection_ctx.reference_body_world = ctx.ref_body_world;
        selection_ctx.align_delta_world = ctx.align_delta;
        selection_ctx.frame_to_world = ctx.frame_to_world;
        selection_ctx.camera_world = ctx.camera_world;
        selection_ctx.tan_half_fov = ctx.tan_half_fov;
        selection_ctx.viewport_height_px = ctx.viewport_height_px;
        selection_ctx.error_px = ctx.render_error_px;

        const auto render_lod_start_tp = std::chrono::steady_clock::now();
        const OrbitRenderCurve::RenderResult lod =
                OrbitRenderCurve::build_render_lod(
                        curve, selection_ctx, ctx.render_frustum, ctx.render_max_segments, t_start_s, t_end_s);
        perf.render_lod_ms_last +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                        .count();
        if (lod.segments.empty())
        {
            return;
        }

        emit_render_lod_segments(ctx, draw_config, perf, lod, color, dashed);
    }

    PickWindow build_planned_pick_window(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const std::vector<ManeuverNode> &nodes,
                                         const double now_s,
                                         const double future_window_s,
                                         const bool draw_future_segment,
                                         const bool draw_full_orbit,
                                         const double orbital_period_s)
    {
        PickWindow planned_window{};
        if (traj_planned_segments.empty() || nodes.empty())
        {
            return planned_window;
        }

        const double t0p = traj_planned_segments.front().t0_s;
        const double t1p = traj_planned_segments.back().t0_s + traj_planned_segments.back().dt_s;
        if (!(t1p > t0p))
        {
            return planned_window;
        }

        double first_future_node_time_s = std::numeric_limits<double>::infinity();
        double first_relevant_node_time_s = std::numeric_limits<double>::infinity();
        for (const ManeuverNode &node : nodes)
        {
            if (!std::isfinite(node.time_s) ||
                node.time_s < (t0p - draw_config.node_time_tolerance_s) ||
                node.time_s > (t1p + draw_config.node_time_tolerance_s))
            {
                continue;
            }

            first_relevant_node_time_s = std::min(first_relevant_node_time_s, node.time_s);
            if (node.time_s >= (now_s + draw_config.node_time_tolerance_s))
            {
                first_future_node_time_s = std::min(first_future_node_time_s, node.time_s);
            }
        }

        const bool has_future_node = std::isfinite(first_future_node_time_s);
        const bool has_relevant_node = std::isfinite(first_relevant_node_time_s);

        double anchor_time_s = first_future_node_time_s;
        if (!std::isfinite(anchor_time_s))
        {
            anchor_time_s = first_relevant_node_time_s;
        }
        if (!std::isfinite(anchor_time_s))
        {
            return planned_window;
        }

        double t_plan_start = t0p;
        if (has_future_node)
        {
            // When the solver output is split at the node time, start exactly on the node so the
            // visible preview length matches the authored preview window. Fall back to the old
            // "snap past the containing segment" behavior only if the node still lands inside
            // a straddling segment.
            t_plan_start = std::clamp(anchor_time_s, t0p, t1p);
            const double snapped_t_plan_start =
                    std::clamp(snap_time_past_straddling_segment(traj_planned_segments, t_plan_start), t0p, t1p);
            if (snapped_t_plan_start > (t_plan_start + draw_config.node_time_tolerance_s))
            {
                t_plan_start = snapped_t_plan_start;
            }
        }
        else if (has_relevant_node)
        {
            // Once every authored node is in the past, keep the planned plot anchored to the ship's
            // current predicted state instead of the original node placement time.
            t_plan_start = std::clamp(now_s, t0p, t1p);
        }

        double t_plan_end = t_plan_start;
        if (draw_future_segment && future_window_s > 0.0)
        {
            t_plan_end = std::min(t_plan_start + future_window_s, t1p);
        }
        else if (draw_full_orbit)
        {
            double t_full_end = t1p;
            if (orbital_period_s > 0.0 && std::isfinite(orbital_period_s))
            {
                // Measure the full-orbit extent from the node (plan start), not from the trajectory data start.
                t_full_end = std::min(t_plan_start + (orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale), t1p);
            }
            t_plan_end = t_full_end;
        }

        if (!(t_plan_end > t_plan_start))
        {
            return planned_window;
        }

        planned_window.valid = true;
        planned_window.t0_s = t_plan_start;
        planned_window.t1_s = t_plan_end;
        planned_window.anchor_time_s = anchor_time_s;
        return planned_window;
    }

    std::size_t build_pick_segment_cache(const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                         const WorldVec3 &ref_body_world,
                                         const glm::dmat3 &frame_to_world,
                                         const WorldVec3 &align_delta,
                                         const OrbitRenderCurve::FrustumContext &pick_frustum,
                                         const OrbitRenderCurve::PickSettings &pick_settings,
                                         const double t0_s,
                                         const double t1_s,
                                         const std::span<const double> anchor_times_s,
                                         const bool segments_are_world_basis,
                                         std::vector<PickingSystem::LinePickSegmentData> &out_segments,
                                         bool &out_cap_hit,
                                         OrbitPlotPerfStats &perf)
    {
        const bool needs_world_basis_transform =
                !segments_are_world_basis && !frame_transform_is_identity(frame_to_world);
        const std::vector<orbitsim::TrajectorySegment> transformed_segments =
                needs_world_basis_transform
                        ? transform_segments_to_world_basis(traj_segments, frame_to_world)
                        : std::vector<orbitsim::TrajectorySegment>{};
        const std::vector<orbitsim::TrajectorySegment> &segments_world_basis =
                needs_world_basis_transform ? transformed_segments : traj_segments;
        out_segments.clear();
        out_cap_hit = false;
        if (segments_world_basis.empty())
        {
            return 0;
        }

        const auto pick_start_tp = std::chrono::steady_clock::now();
        const OrbitRenderCurve::PickResult lod =
                OrbitRenderCurve::build_pick_lod(segments_world_basis,
                                                 ref_body_world,
                                                 align_delta,
                                                 pick_frustum,
                                                 pick_settings,
                                                 t0_s,
                                                 t1_s,
                                                 anchor_times_s);
        perf.pick_lod_ms_last +=
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pick_start_tp).count();
        perf.pick_segments_before_cull += static_cast<uint32_t>(lod.segments_before_cull);
        perf.pick_segments += static_cast<uint32_t>(lod.segments_after_cull);

        out_segments.reserve(lod.segments.size());
        for (const OrbitRenderCurve::LineSegment &segment : lod.segments)
        {
            out_segments.push_back(PickingSystem::LinePickSegmentData{
                    .a_world = segment.a_world,
                    .b_world = segment.b_world,
                    .a_time_s = segment.t0_s,
                    .b_time_s = segment.t1_s,
            });
        }
        const std::size_t emitted = out_segments.size();
        out_cap_hit = lod.cap_hit;
        perf.pick_cap_hit_last_frame = out_cap_hit;
        if (out_cap_hit)
        {
            ++perf.pick_cap_hits_total;
        }
        return emitted;
    }

    bool frame_spec_uses_direct_world_polyline(const orbitsim::TrajectoryFrameSpec &spec)
    {
        (void) spec;
        return false;
    }

    void draw_polyline_window(const OrbitDrawWindowContext &ctx,
                              const OrbitPredictionDrawConfig &draw_config,
                              const std::vector<orbitsim::TrajectorySample> &traj,
                              const double t_start_s,
                              const double t_end_s,
                              const glm::vec4 &color,
                              const bool dashed)
    {
        if (!(t_end_s > t_start_s) || traj.size() < 2)
        {
            return;
        }

        const double dash_on_px = draw_config.dashed_segment_on_px;
        const double dash_off_px = draw_config.dashed_segment_off_px;
        const double dash_period_px = dash_on_px + dash_off_px;
        double dash_phase_px = 0.0;

        for (std::size_t i = 1; i < traj.size(); ++i)
        {
            const double seg_t0_s = traj[i - 1].t_s;
            const double seg_t1_s = traj[i].t_s;
            const double clip_t0_s = std::max(seg_t0_s, t_start_s);
            const double clip_t1_s = std::min(seg_t1_s, t_end_s);
            if (!(clip_t1_s > clip_t0_s))
            {
                continue;
            }

            const WorldVec3 a_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t0_s) + ctx.align_delta;
            const WorldVec3 b_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t1_s) + ctx.align_delta;

            if (!dashed)
            {
                emit_orbit_line(ctx, color, a_world, b_world);
                continue;
            }

            const double seg_m = glm::length(glm::dvec3(b_world - a_world));
            const glm::dvec3 seg_mid = glm::mix(glm::dvec3(a_world), glm::dvec3(b_world), 0.5);
            const double seg_mpp = meters_per_px_at_world(ctx, WorldVec3(seg_mid));
            const double seg_px = seg_m / seg_mpp;
            if (!std::isfinite(seg_px) || !(seg_px > 1.0e-6) || !std::isfinite(seg_mpp) || !(seg_mpp > 1.0e-6))
            {
                continue;
            }

            double cursor_px = 0.0;
            while ((cursor_px + 1.0e-6) < seg_px)
            {
                const bool phase_on = dash_phase_px < dash_on_px;
                double phase_remaining_px = phase_on ? (dash_on_px - dash_phase_px) : (dash_period_px - dash_phase_px);
                if (!std::isfinite(phase_remaining_px) || !(phase_remaining_px > 1.0e-6))
                {
                    phase_remaining_px = 1.0;
                }

                const double step_px = std::min(phase_remaining_px, seg_px - cursor_px);
                const double next_px = cursor_px + step_px;
                if (phase_on)
                {
                    const double u0 = std::clamp(cursor_px / seg_px, 0.0, 1.0);
                    const double u1 = std::clamp(next_px / seg_px, 0.0, 1.0);
                    emit_orbit_line(ctx, color, glm::mix(a_world, b_world, u0), glm::mix(a_world, b_world, u1));
                }

                cursor_px = next_px;
                dash_phase_px += step_px;
                if (dash_phase_px >= dash_period_px)
                {
                    dash_phase_px = std::fmod(dash_phase_px, dash_period_px);
                }
            }
        }
    }

    std::size_t emit_polyline_pick_segments(const OrbitDrawWindowContext &ctx,
                                            PickingSystem *picking,
                                            const uint32_t pick_group,
                                            const std::vector<orbitsim::TrajectorySample> &traj,
                                            const double t0_s,
                                            const double t1_s,
                                            const std::size_t max_segments,
                                            OrbitPlotPerfStats &perf)
    {
        if (!picking || pick_group == kInvalidPickGroup || traj.size() < 2)
        {
            return 0;
        }

        std::size_t emitted = 0;
        for (std::size_t i = 1; i < traj.size() && emitted < max_segments; ++i)
        {
            const double seg_t0_s = traj[i - 1].t_s;
            const double seg_t1_s = traj[i].t_s;
            const double clip_t0_s = std::max(seg_t0_s, t0_s);
            const double clip_t1_s = std::min(seg_t1_s, t1_s);
            if (!(clip_t1_s > clip_t0_s))
            {
                continue;
            }

            const WorldVec3 a_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t0_s) + ctx.align_delta;
            const WorldVec3 b_world =
                    sample_polyline_world(ctx.ref_body_world, ctx.frame_to_world, traj, i - 1, i, clip_t1_s) + ctx.align_delta;
            picking->add_line_pick_segment(pick_group, a_world, b_world, clip_t0_s, clip_t1_s);
            ++emitted;
        }

        perf.pick_segments_before_cull += static_cast<uint32_t>(emitted);
        perf.pick_segments += static_cast<uint32_t>(emitted);
        return emitted;
    }

    void emit_velocity_ray(GameAPI::Engine *api,
                           const WorldVec3 &ship_pos_world,
                           const glm::dvec3 &ship_vel_world,
                           const float ttl_s,
                           const glm::vec4 &color)
    {
        const double speed_mps = glm::length(ship_vel_world);
        double len_m = 40.0;
        if (std::isfinite(speed_mps) && speed_mps > 1.0)
        {
            len_m = std::clamp(speed_mps * 0.002, 10.0, 250.0);
        }

        api->debug_draw_ray(glm::dvec3(ship_pos_world), ship_vel_world, len_m, color, ttl_s, true);
    }
} // namespace Game::PredictionDrawDetail
