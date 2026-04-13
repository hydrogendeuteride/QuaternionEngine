#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include "game/orbit/orbit_plot_util.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game::PredictionDrawDetail
{
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
                    double phase_remaining_px =
                            phase_on ? (dash_on_px - dash_phase_px) : (dash_period_px - dash_phase_px);
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

    namespace
    {
        PickWindow build_planned_window_from_policy(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                                    const OrbitPredictionDrawConfig &draw_config,
                                                    const PredictionWindowPolicyResult &policy,
                                                    const double window_span_s)
        {
            PickWindow planned_window{};
            if (traj_planned_segments.empty() || !policy.valid || !std::isfinite(policy.anchor_time_s))
            {
                return planned_window;
            }

            const double t0p = traj_planned_segments.front().t0_s;
            const double t1p = traj_planned_segments.back().t0_s + traj_planned_segments.back().dt_s;
            if (!(t1p > t0p))
            {
                return planned_window;
            }

            double t_plan_start = std::clamp(policy.anchor_time_s, t0p, t1p);
            if (policy.anchor_is_future)
            {
                const double snapped_t_plan_start =
                        std::clamp(snap_time_past_straddling_segment(traj_planned_segments, t_plan_start), t0p, t1p);
                if (snapped_t_plan_start > (t_plan_start + draw_config.node_time_tolerance_s))
                {
                    t_plan_start = snapped_t_plan_start;
                }
            }

            if (!(window_span_s > 0.0))
            {
                return planned_window;
            }

            const double t_plan_end = std::min(t_plan_start + window_span_s, t1p);
            if (!(t_plan_end > t_plan_start))
            {
                return planned_window;
            }

            planned_window.valid = true;
            planned_window.t0_s = t_plan_start;
            planned_window.t1_s = t_plan_end;
            planned_window.anchor_time_s = policy.anchor_time_s;
            return planned_window;
        }
    } // namespace

    PickWindow build_planned_draw_window(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const PredictionWindowPolicyResult &policy)
    {
        return build_planned_window_from_policy(traj_planned_segments, draw_config, policy, policy.visual_window_s);
    }

    PickWindow build_planned_pick_window(const std::vector<orbitsim::TrajectorySegment> &traj_planned_segments,
                                         const OrbitPredictionDrawConfig &draw_config,
                                         const PredictionWindowPolicyResult &policy)
    {
        return build_planned_window_from_policy(traj_planned_segments, draw_config, policy, policy.pick_window_s);
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
