#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_plot_lod_builder.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr uint32_t kInvalidPickGroup = std::numeric_limits<uint32_t>::max();

        struct OrbitDrawWindowContext
        {
            OrbitPlotSystem *orbit_plot{nullptr};
            WorldVec3 ref_body_world{0.0, 0.0, 0.0};
            WorldVec3 align_delta{0.0, 0.0, 0.0};
            OrbitPlotLodBuilder::CameraContext lod_camera{};
            glm::dvec3 camera_world{0.0, 0.0, 0.0};
            double tan_half_fov{0.0};
            double viewport_height_px{1.0};
            double render_error_px{0.75};
            std::size_t render_max_segments{1};
            float line_overlay_boost{0.0f};
        };

        struct PickWindow
        {
            bool valid{false};
            double t0_s{0.0};
            double t1_s{0.0};
            double anchor_time_s{std::numeric_limits<double>::quiet_NaN()};
        };

        // Clear per-frame orbit plot state before emitting new lines and pick segments.
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

        // Apply the user-controlled alpha multiplier to a base orbit color.
        glm::vec4 scale_line_color(glm::vec4 color, const float line_alpha_scale)
        {
            color.a = std::clamp(color.a * line_alpha_scale, 0.0f, 1.0f);
            return color;
        }

        // Convert simulation time into the interpolated render-time sample used for orbit drawing.
        double compute_prediction_now_s(const double sim_time_s,
                                        const double last_sim_step_dt_s,
                                        const float fixed_delta_time,
                                        const float alpha_f,
                                        const double t0,
                                        const double t1)
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

            return std::clamp(now_s, t0, t1);
        }

        // Estimate world-space meters per screen pixel at a specific point for dash and LOD sizing.
        double meters_per_px_at_world(const OrbitDrawWindowContext &ctx, const WorldVec3 &p_world)
        {
            constexpr double kFallbackMetersPerPx = 1.0;

            const double dist_m = OrbitPredictionMath::safe_length(glm::dvec3(p_world) - ctx.camera_world);
            if (!std::isfinite(dist_m) || dist_m <= 1.0e-3 || !std::isfinite(ctx.tan_half_fov) || ctx.tan_half_fov <= 1.0e-8)
            {
                return kFallbackMetersPerPx;
            }

            const double mpp = (2.0 * ctx.tan_half_fov * dist_m) / std::max(1.0, ctx.viewport_height_px);
            if (!std::isfinite(mpp) || mpp <= 1.0e-6)
            {
                return kFallbackMetersPerPx;
            }

            return mpp;
        }

        // Build interpolation-ready trajectory segments when the solver only returned raw samples.
        std::vector<orbitsim::TrajectorySegment> trajectory_segments_from_samples(
                const std::vector<orbitsim::TrajectorySample> &samples)
        {
            std::vector<orbitsim::TrajectorySegment> out;
            if (samples.size() < 2)
            {
                return out;
            }

            out.reserve(samples.size() - 1);
            for (size_t i = 1; i < samples.size(); ++i)
            {
                const orbitsim::TrajectorySample &a = samples[i - 1];
                const orbitsim::TrajectorySample &b = samples[i];
                const double dt_s = b.t_s - a.t_s;
                if (!(dt_s > 0.0) || !std::isfinite(dt_s))
                {
                    continue;
                }

                orbitsim::State start{};
                start.position_m = a.position_m;
                start.velocity_mps = a.velocity_mps;
                orbitsim::State end{};
                end.position_m = b.position_m;
                end.velocity_mps = b.velocity_mps;

                out.push_back(orbitsim::TrajectorySegment{
                        .t0_s = a.t_s,
                        .dt_s = dt_s,
                        .start = start,
                        .end = end,
                        .flags = 0u,
                });
            }

            return out;
        }

        // Push a timestamp to the end of an impulse-straddling segment so post-burn paths start cleanly.
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

        // Collect sorted unique maneuver-node times used as anchors for pick LOD generation.
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

        // Find the first trajectory sample at or after the requested time.
        size_t lower_bound_sample_index(const std::vector<orbitsim::TrajectorySample> &traj, const double t_s)
        {
            const auto it = std::lower_bound(traj.cbegin(),
                                             traj.cend(),
                                             t_s,
                                             [](const orbitsim::TrajectorySample &sample, double t) {
                                                 return sample.t_s < t;
                                             });
            return static_cast<size_t>(std::distance(traj.cbegin(), it));
        }

        // Offset the predicted curve so it visually meets the ship at the current render time.
        WorldVec3 compute_align_delta(const std::vector<orbitsim::TrajectorySample> &traj_base,
                                      const std::vector<WorldVec3> &points_base,
                                      const size_t i_hi,
                                      const WorldVec3 &ship_pos_world,
                                      const WorldVec3 &ref_body_world,
                                      const double now_s)
        {
            WorldVec3 predicted_now_world = points_base[i_hi];
            if (i_hi > 0)
            {
                predicted_now_world = OrbitPredictionMath::hermite_position_world(ref_body_world,
                                                                                  traj_base[i_hi - 1],
                                                                                  traj_base[i_hi],
                                                                                  now_s);
            }

            WorldVec3 align_delta = ship_pos_world - predicted_now_world;
            const double align_len = glm::length(glm::dvec3(align_delta));
            if (!std::isfinite(align_len) || align_len > 10'000.0)
            {
                return WorldVec3(0.0, 0.0, 0.0);
            }

            return align_delta;
        }

        // Evaluate a trajectory segment in world space using Hermite interpolation.
        WorldVec3 eval_segment_world_pos(const OrbitDrawWindowContext &ctx,
                                         const orbitsim::TrajectorySegment &segment,
                                         const double t_s)
        {
            if (!(segment.dt_s > 0.0) || !std::isfinite(segment.dt_s))
            {
                return ctx.ref_body_world + WorldVec3(glm::dvec3(segment.start.position_m)) + ctx.align_delta;
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
            const glm::dvec3 local = (h00 * p0) + (h10 * m0) + (h01 * p1) + (h11 * m1);
            return ctx.ref_body_world + WorldVec3(local) + ctx.align_delta;
        }

        // Emit one orbit line segment, including the optional always-on-top overlay pass.
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

        // Send clipped root segments to the GPU orbit plot path for adaptive subdivision.
        void emit_gpu_root_segments(const OrbitDrawWindowContext &ctx,
                                    const OrbitPredictionDrawConfig &draw_config,
                                    const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                    const double t_start_s,
                                    const double t_end_s,
                                    const glm::vec4 &color,
                                    const bool dashed,
                                    const OrbitPlotDepth depth)
        {
            double dash_phase_px = 0.0;
            const double dash_on_px = draw_config.dashed_segment_on_px;
            const double dash_off_px = draw_config.dashed_segment_off_px;
            const double dash_period_px = dash_on_px + dash_off_px;

            for (const orbitsim::TrajectorySegment &segment : traj_segments)
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

                float dash_phase_start_px = 0.0f;
                if (dashed)
                {
                    const WorldVec3 a_world = eval_segment_world_pos(ctx, segment, clip_t0_s);
                    const WorldVec3 b_world = eval_segment_world_pos(ctx, segment, clip_t1_s);
                    const double seg_m = glm::length(glm::dvec3(b_world - a_world));
                    if (std::isfinite(seg_m) && seg_m > 1.0e-9)
                    {
                        const glm::dvec3 mid_world = glm::mix(glm::dvec3(a_world), glm::dvec3(b_world), 0.5);
                        const double seg_mpp = meters_per_px_at_world(ctx, WorldVec3(mid_world));
                        if (std::isfinite(seg_mpp) && seg_mpp > 1.0e-6)
                        {
                            const double seg_px = seg_m / seg_mpp;
                            dash_phase_start_px = static_cast<float>(std::fmod(dash_phase_px, dash_period_px));
                            if (std::isfinite(seg_px) && seg_px > 0.0)
                            {
                                dash_phase_px = std::fmod(dash_phase_px + seg_px, dash_period_px);
                            }
                        }
                    }
                }

                const double clip_u0 = (clip_t0_s - seg_t0_s) / segment.dt_s;
                const double clip_u1 = (clip_t1_s - seg_t0_s) / segment.dt_s;
                ctx.orbit_plot->add_gpu_root_segment(segment.dt_s,
                                                     glm::dvec3(segment.start.position_m),
                                                     glm::dvec3(segment.start.velocity_mps),
                                                     glm::dvec3(segment.end.position_m),
                                                     glm::dvec3(segment.end.velocity_mps),
                                                     clip_u0,
                                                     clip_u1,
                                                     dashed,
                                                     dash_phase_start_px,
                                                     color,
                                                     depth);
            }
        }

        // Emit a coarse CPU fallback line set while the GPU orbit path is warming up.
        void emit_cpu_fallback_chords(const OrbitDrawWindowContext &ctx,
                                      const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                      const double t_start_s,
                                      const double t_end_s,
                                      const glm::vec4 &color)
        {
            for (const orbitsim::TrajectorySegment &segment : traj_segments)
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

                const WorldVec3 a_world = eval_segment_world_pos(ctx, segment, clip_t0_s);
                const WorldVec3 b_world = eval_segment_world_pos(ctx, segment, clip_t1_s);
                emit_orbit_line(ctx, color, a_world, b_world);
            }
        }

        // Build CPU render LOD and emit either solid or dashed orbit segments.
        void emit_cpu_render_lod(const OrbitDrawWindowContext &ctx,
                                 const OrbitPredictionDrawConfig &draw_config,
                                 OrbitPlotPerfStats &perf,
                                 const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                 const double t_start_s,
                                 const double t_end_s,
                                 const glm::vec4 &color,
                                 const bool dashed)
        {
            OrbitPlotLodBuilder::RenderSettings lod_settings{};
            lod_settings.error_px = ctx.render_error_px;
            lod_settings.max_segments = ctx.render_max_segments;

            const auto render_lod_start_tp = std::chrono::steady_clock::now();
            const OrbitPlotLodBuilder::RenderResult lod =
                    OrbitPlotLodBuilder::build_render_lod(traj_segments,
                                                          ctx.ref_body_world,
                                                          ctx.align_delta,
                                                          ctx.lod_camera,
                                                          lod_settings,
                                                          t_start_s,
                                                          t_end_s);
            perf.render_lod_ms_last +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                            .count();
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
                for (const OrbitPlotLodBuilder::RenderSegment &segment : lod.segments)
                {
                    emit_orbit_line(ctx, color, segment.a_world, segment.b_world);
                }
                return;
            }

            const double dash_on_px = draw_config.dashed_segment_on_px;
            const double dash_off_px = draw_config.dashed_segment_off_px;
            const double dash_period_px = dash_on_px + dash_off_px;

            double dash_phase_px = 0.0;
            for (const OrbitPlotLodBuilder::RenderSegment &segment : lod.segments)
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

        // Render one visible time window of an orbit using the GPU path when available.
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

            const bool gpu_subdivision_enabled = ctx.orbit_plot && ctx.orbit_plot->settings().gpu_generate_enabled;
            if (gpu_subdivision_enabled)
            {
                ctx.orbit_plot->set_gpu_frame_reference(ctx.ref_body_world, ctx.align_delta);
                emit_gpu_root_segments(ctx,
                                       draw_config,
                                       traj_segments,
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
                                               traj_segments,
                                               t_start_s,
                                               t_end_s,
                                               overlay_color,
                                               dashed,
                                               OrbitPlotDepth::AlwaysOnTop);
                    }
                }

                // If GPU path wasn't active in the last frame, emit a coarse CPU fallback
                // (single clipped chord per adaptive segment) to avoid losing trajectory visuals.
                if (ctx.orbit_plot->stats().gpu_path_active_last_frame)
                {
                    return;
                }

                if (!dashed)
                {
                    emit_cpu_fallback_chords(ctx, traj_segments, t_start_s, t_end_s, color);
                    return;
                }
            }

            emit_cpu_render_lod(ctx, draw_config, perf, traj_segments, t_start_s, t_end_s, color, dashed);
        }

        // Compute the planned-orbit draw and pick window anchored to the first relevant maneuver.
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

            double anchor_time_s = first_future_node_time_s;
            if (!std::isfinite(anchor_time_s))
            {
                anchor_time_s = first_relevant_node_time_s;
            }
            if (!std::isfinite(anchor_time_s))
            {
                return planned_window;
            }

            double t_plan_start = std::clamp(anchor_time_s + draw_config.node_time_tolerance_s, t0p, t1p);
            t_plan_start = std::clamp(snap_time_past_straddling_segment(traj_planned_segments, t_plan_start), t0p, t1p);

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
                    t_full_end = std::min(t0p + (orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale), t1p);
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

        // Fold pick-LOD stats into the per-frame orbit performance counters.
        void update_pick_perf_stats(OrbitPlotPerfStats &perf, const OrbitPlotLodBuilder::PickResult &pick_lod)
        {
            perf.pick_segments_before_cull += static_cast<uint32_t>(pick_lod.segments_before_cull);
            perf.pick_segments += static_cast<uint32_t>(pick_lod.segments.size());
            if (pick_lod.cap_hit)
            {
                perf.pick_cap_hit_last_frame = true;
                ++perf.pick_cap_hits_total;
            }
        }

        // Build pick LOD for one orbit branch and register the emitted segments with the picker.
        std::size_t emit_pick_segments(PickingSystem *picking,
                                       const uint32_t pick_group,
                                       const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                                       const WorldVec3 &ref_body_world,
                                       const WorldVec3 &align_delta,
                                       const OrbitPlotLodBuilder::FrustumContext &pick_frustum,
                                       const OrbitPlotLodBuilder::PickSettings &pick_settings,
                                       const double t0_s,
                                       const double t1_s,
                                       const std::vector<double> &anchor_times,
                                       OrbitPlotPerfStats &perf)
        {
            const auto pick_lod_start_tp = std::chrono::steady_clock::now();
            const OrbitPlotLodBuilder::PickResult pick_lod =
                    OrbitPlotLodBuilder::build_pick_lod(traj_segments,
                                                        ref_body_world,
                                                        align_delta,
                                                        pick_frustum,
                                                        pick_settings,
                                                        t0_s,
                                                        t1_s,
                                                        anchor_times);
            perf.pick_lod_ms_last +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pick_lod_start_tp)
                            .count();
            update_pick_perf_stats(perf, pick_lod);

            for (const OrbitPlotLodBuilder::PickSegment &segment : pick_lod.segments)
            {
                picking->add_line_pick_segment(pick_group,
                                               segment.a_world,
                                               segment.b_world,
                                               segment.t0_s,
                                               segment.t1_s);
            }

            return pick_lod.segments.size();
        }

        // Draw a debug velocity ray scaled from the ship's current speed.
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
    } // namespace

    // Build per-frame orbit visuals, pick data, and debug overlays from the latest prediction cache.
    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        PickingSystem *picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        reset_orbit_plot_state(picking, orbit_plot, _orbit_plot_perf, _prediction_enabled);

        if (!_prediction_enabled || !ctx.api || !_orbitsim)
        {
            return;
        }

        const PredictionTrackState *active_track = active_prediction_track();
        if (!active_track)
        {
            return;
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);

        // Debug velocity ray is emitted into DebugDrawSystem, which prunes commands in
        // engine draw begin_frame(dt) after update_scene(), so ttl must be > dt.
        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        const float line_alpha_scale = std::clamp(_prediction_line_alpha_scale, 0.1f, 8.0f);
        const glm::vec4 color_orbit_full =
                scale_line_color(_prediction_draw_config.palette.orbit_full, line_alpha_scale);
        const glm::vec4 color_orbit_future =
                scale_line_color(_prediction_draw_config.palette.orbit_future, line_alpha_scale);
        const glm::vec4 color_orbit_plan =
                scale_line_color(_prediction_draw_config.palette.orbit_planned, line_alpha_scale);

        glm::dvec3 camera_world = ctx.api->get_camera_position_d();
        float viewport_h_px = 720.0f;
        float camera_fov_deg = 70.0f;
        if (ctx.renderer && ctx.renderer->_sceneManager)
        {
            const Camera &cam = ctx.renderer->_sceneManager->getMainCamera();
            camera_world = cam.position_world;
            camera_fov_deg = cam.fovDegrees;
            if (ctx.renderer->_logicalRenderExtent.height > 0)
            {
                viewport_h_px = static_cast<float>(ctx.renderer->_logicalRenderExtent.height);
            }
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();
        const double tan_half_fov = std::tan(glm::radians(static_cast<double>(camera_fov_deg)) * 0.5);
        OrbitPlotLodBuilder::CameraContext lod_camera{};
        lod_camera.camera_world = camera_world;
        lod_camera.tan_half_fov = tan_half_fov;
        lod_camera.viewport_height_px = std::max(1.0, static_cast<double>(viewport_h_px));

        const double render_error_px =
                (std::isfinite(_orbit_plot_render_error_px) && _orbit_plot_render_error_px > 0.0)
                        ? _orbit_plot_render_error_px
                        : 0.75;
        if (orbit_plot)
        {
            orbit_plot->settings().render_error_px = render_error_px;
        }

        std::vector<PredictionTrackState *> visible_tracks;
        visible_tracks.reserve(1 + _prediction_selection.overlay_subjects.size());
        for (PredictionSubjectKey key : collect_visible_prediction_subjects())
        {
            if (PredictionTrackState *track = find_prediction_track(key))
            {
                visible_tracks.push_back(track);
            }
        }
        if (visible_tracks.empty())
        {
            return;
        }

        size_t overlay_draw_index = 0;
        for (PredictionTrackState *track : visible_tracks)
        {
            if (!track)
            {
                continue;
            }

            refresh_prediction_world_points(*track);
            if (!track->cache.valid ||
                track->cache.points_world.size() < 2 ||
                track->cache.trajectory_bci.size() != track->cache.points_world.size())
            {
                continue;
            }

            const auto &traj_base = track->cache.trajectory_bci;
            const auto &points_base = track->cache.points_world;
            const auto &traj_planned = track->cache.trajectory_bci_planned;

            const double t0 = traj_base.front().t_s;
            const double t1 = traj_base.back().t_s;
            if (!(t1 > t0))
            {
                continue;
            }

            const double now_s =
                    compute_prediction_now_s(_orbitsim->sim.time_s(), _last_sim_step_dt_s, ctx.fixed_delta_time(), alpha_f, t0, t1);
            if (!std::isfinite(now_s))
            {
                continue;
            }

            WorldVec3 subject_pos_world_state{0.0, 0.0, 0.0};
            glm::dvec3 subject_vel_world(0.0);
            glm::vec3 subject_vel_local_f(0.0f);
            if (!get_prediction_subject_world_state(track->key,
                                                    subject_pos_world_state,
                                                    subject_vel_world,
                                                    subject_vel_local_f))
            {
                continue;
            }

            WorldVec3 subject_pos_world = subject_pos_world_state;
            if (track->key.kind == PredictionSubjectKind::Orbiter)
            {
                if (const Entity *entity = _world.entities().find(EntityId{track->key.value}))
                {
                    subject_pos_world = entity->get_render_position_world(alpha_f);
                }
            }

            std::vector<orbitsim::TrajectorySegment> base_segments_fallback;
            const std::vector<orbitsim::TrajectorySegment> *traj_base_segments = &track->cache.trajectory_segments_bci;
            if (traj_base_segments->empty())
            {
                base_segments_fallback = trajectory_segments_from_samples(traj_base);
                traj_base_segments = &base_segments_fallback;
            }
            if (traj_base_segments->empty())
            {
                continue;
            }

            std::vector<orbitsim::TrajectorySegment> planned_segments_fallback;
            const std::vector<orbitsim::TrajectorySegment> *traj_planned_segments = &track->cache.trajectory_segments_bci_planned;
            if (traj_planned_segments->empty() && !traj_planned.empty())
            {
                planned_segments_fallback = trajectory_segments_from_samples(traj_planned);
                traj_planned_segments = &planned_segments_fallback;
            }

            const bool is_active = track->key == _prediction_selection.active_subject;
            const bool active_player_track = is_active && prediction_subject_is_player(track->key);
            if (is_active)
            {
                _orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(traj_base_segments->size());
                _orbit_plot_perf.solver_segments_planned = static_cast<uint32_t>(traj_planned_segments->size());
            }

            const size_t i_hi = lower_bound_sample_index(traj_base, now_s);
            if (i_hi >= traj_base.size())
            {
                continue;
            }

            const WorldVec3 align_delta =
                    compute_align_delta(traj_base, points_base, i_hi, subject_pos_world, ref_body_world, now_s);

            OrbitDrawWindowContext draw_ctx{};
            draw_ctx.orbit_plot = orbit_plot;
            draw_ctx.ref_body_world = ref_body_world;
            draw_ctx.align_delta = align_delta;
            draw_ctx.lod_camera = lod_camera;
            draw_ctx.camera_world = camera_world;
            draw_ctx.tan_half_fov = tan_half_fov;
            draw_ctx.viewport_height_px = std::max(1.0, static_cast<double>(viewport_h_px));
            draw_ctx.render_error_px = render_error_px;
            draw_ctx.render_max_segments = static_cast<std::size_t>(std::max(1, _orbit_plot_render_max_segments_cpu));
            draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost, 0.0f, 1.0f);

            glm::vec4 track_color_full = color_orbit_full;
            glm::vec4 track_color_future = color_orbit_future;
            glm::vec4 track_color_plan = color_orbit_plan;
            if (!is_active)
            {
                const glm::vec4 seed = prediction_overlay_seed_color(overlay_draw_index);
                ++overlay_draw_index;
                track_color_full = scale_line_color(glm::vec4(seed.r, seed.g, seed.b, 0.18f), line_alpha_scale);
                track_color_future = scale_line_color(glm::vec4(seed.r, seed.g, seed.b, 0.58f), line_alpha_scale);
                track_color_plan = scale_line_color(glm::vec4(seed.r, seed.g, seed.b, 0.75f), line_alpha_scale);
                draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost * 0.35f, 0.0f, 1.0f);
            }

            const double future_window_s = prediction_future_window_s(track->key);
            PickWindow base_pick_window{};
            if (_prediction_draw_full_orbit)
            {
                double t_full_end = t1;
                if (track->cache.orbital_period_s > 0.0 && std::isfinite(track->cache.orbital_period_s))
                {
                    t_full_end = std::min(t0 + (track->cache.orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale), t1);
                }

                draw_orbit_window(draw_ctx,
                                  _prediction_draw_config,
                                  _orbit_plot_perf,
                                  *traj_base_segments,
                                  t0,
                                  t_full_end,
                                  track_color_full,
                                  false);
                if (is_active && !_prediction_draw_future_segment && t_full_end > t0)
                {
                    base_pick_window.valid = true;
                    base_pick_window.t0_s = t0;
                    base_pick_window.t1_s = t_full_end;
                }
            }

            if (_prediction_draw_future_segment)
            {
                const double t_end = (future_window_s > 0.0) ? std::min(now_s + future_window_s, t1) : t1;

                draw_orbit_window(draw_ctx,
                                  _prediction_draw_config,
                                  _orbit_plot_perf,
                                  *traj_base_segments,
                                  now_s,
                                  t_end,
                                  track_color_future,
                                  false);
                if (is_active && t_end > now_s)
                {
                    base_pick_window.valid = true;
                    base_pick_window.t0_s = now_s;
                    base_pick_window.t1_s = t_end;
                }
            }

            const PickWindow planned_pick_window =
                    (active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty() && !traj_planned_segments->empty())
                            ? build_planned_pick_window(*traj_planned_segments,
                                                        _prediction_draw_config,
                                                        _maneuver_state.nodes,
                                                        now_s,
                                                        future_window_s,
                                                        _prediction_draw_future_segment,
                                                        _prediction_draw_full_orbit,
                                                        track->cache.orbital_period_s)
                            : PickWindow{};
            if (planned_pick_window.valid)
            {
                draw_orbit_window(draw_ctx,
                                  _prediction_draw_config,
                                  _orbit_plot_perf,
                                  *traj_planned_segments,
                                  planned_pick_window.t0_s,
                                  planned_pick_window.t1_s,
                                  track_color_plan,
                                  _prediction_draw_config.draw_planned_as_dashed);
            }

            if (picking && active_player_track)
            {
                const bool allow_base_pick = _maneuver_state.nodes.empty();
                const bool allow_planned_pick = !_maneuver_state.nodes.empty();
                const uint32_t pick_group_base = picking->add_line_pick_group("OrbitPlot/Base");
                const uint32_t pick_group_planned = picking->add_line_pick_group("OrbitPlot/Planned");
                const std::vector<double> maneuver_node_times_s = collect_maneuver_node_times(_maneuver_state.nodes);

                OrbitPlotLodBuilder::FrustumContext pick_frustum{};
                if (ctx.renderer && ctx.renderer->_context && ctx.renderer->_sceneManager)
                {
                    pick_frustum.valid = true;
                    pick_frustum.viewproj = ctx.renderer->_context->getSceneData().viewproj;
                    pick_frustum.origin_world = ctx.renderer->_sceneManager->get_world_origin();
                }

                const std::size_t pick_max_segments =
                        static_cast<std::size_t>(std::max(1, _orbit_plot_pick_max_segments));
                const double pick_frustum_margin_ratio =
                        (std::isfinite(_orbit_plot_pick_frustum_margin_ratio) && _orbit_plot_pick_frustum_margin_ratio >= 0.0)
                                ? _orbit_plot_pick_frustum_margin_ratio
                                : 0.05;
                const std::size_t pick_planned_reserve_target = std::min(
                        _prediction_draw_config.pick_planned_reserve_segments,
                        static_cast<std::size_t>(std::max<std::size_t>(
                                1,
                                static_cast<std::size_t>(
                                        std::llround(static_cast<double>(pick_max_segments) *
                                                     _prediction_draw_config.pick_planned_reserve_ratio)))));
                std::size_t remaining_pick_budget = pick_max_segments;

                if (allow_base_pick &&
                    base_pick_window.valid &&
                    pick_group_base != kInvalidPickGroup &&
                    remaining_pick_budget > 0)
                {
                    const std::size_t planned_reserve =
                            planned_pick_window.valid
                                    ? std::min(pick_planned_reserve_target, remaining_pick_budget)
                                    : 0;
                    OrbitPlotLodBuilder::PickSettings pick_settings{};
                    pick_settings.max_segments = remaining_pick_budget - planned_reserve;
                    pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;

                    std::vector<double> base_anchor_times{};
                    base_anchor_times.reserve(1 + maneuver_node_times_s.size());
                    base_anchor_times.push_back(now_s);
                    base_anchor_times.insert(base_anchor_times.end(), maneuver_node_times_s.begin(), maneuver_node_times_s.end());

                    const std::size_t emitted = emit_pick_segments(picking,
                                                                   pick_group_base,
                                                                   *traj_base_segments,
                                                                   ref_body_world,
                                                                   align_delta,
                                                                   pick_frustum,
                                                                   pick_settings,
                                                                   base_pick_window.t0_s,
                                                                   base_pick_window.t1_s,
                                                                   base_anchor_times,
                                                                   _orbit_plot_perf);
                    remaining_pick_budget = (emitted >= remaining_pick_budget) ? 0 : (remaining_pick_budget - emitted);
                }

                if (allow_planned_pick &&
                    planned_pick_window.valid &&
                    pick_group_planned != kInvalidPickGroup &&
                    remaining_pick_budget > 0)
                {
                    OrbitPlotLodBuilder::PickSettings pick_settings{};
                    pick_settings.max_segments = remaining_pick_budget;
                    pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;

                    std::vector<double> planned_anchor_times{};
                    planned_anchor_times.reserve(1 + maneuver_node_times_s.size());
                    if (std::isfinite(planned_pick_window.anchor_time_s))
                    {
                        planned_anchor_times.push_back(planned_pick_window.anchor_time_s);
                    }
                    planned_anchor_times.insert(planned_anchor_times.end(),
                                                maneuver_node_times_s.begin(),
                                                maneuver_node_times_s.end());

                    emit_pick_segments(picking,
                                       pick_group_planned,
                                       *traj_planned_segments,
                                       ref_body_world,
                                       align_delta,
                                       pick_frustum,
                                       pick_settings,
                                       planned_pick_window.t0_s,
                                       planned_pick_window.t1_s,
                                       planned_anchor_times,
                                       _orbit_plot_perf);
                }
            }

            if (is_active &&
                _prediction_draw_velocity_ray &&
                _debug_draw_enabled &&
                track->key.kind == PredictionSubjectKind::Orbiter)
            {
                emit_velocity_ray(ctx.api,
                                  subject_pos_world,
                                  subject_vel_world,
                                  ttl_s,
                                  _prediction_draw_config.palette.velocity_ray);
            }
        }
    }
} // namespace Game
