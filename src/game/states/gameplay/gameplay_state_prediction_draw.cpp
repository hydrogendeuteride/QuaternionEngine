#include "gameplay_state.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_plot_lod_builder.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr double kOrbitRenderErrorPx = 0.75;
        constexpr int kOrbitDrawMaxSegmentsSolid = 4'000;
        constexpr int kOrbitDrawMaxSegmentsDashed = 2'500;
        constexpr double kOrbitDashedPickMaxSpanS = 4'000.0;
        constexpr bool kPredictionDrawPlannedAsDashed = true;
    } // namespace

    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        PickingSystem *picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        if (picking)
        {
            // Orbit plot is emitted per-frame, so refresh pickable segments the same way.
            picking->clear_line_picks();
        }

        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        if (orbit_plot)
        {
            orbit_plot->clear_pending();
        }

        if (!_prediction_enabled)
        {
            return;
        }

        if (!ctx.api)
        {
            return;
        }

        if (!_orbitsim || !_prediction_cache.valid)
        {
            return;
        }

        refresh_prediction_world_points();
        if (_prediction_cache.points_world.size() < 2 ||
            _prediction_cache.trajectory_bci.size() != _prediction_cache.points_world.size())
        {
            return;
        }

        const double t0 = _prediction_cache.trajectory_bci.front().t_s;
        const double t1 = _prediction_cache.trajectory_bci.back().t_s;
        if (!(t1 > t0))
        {
            return;
        }

        const float alpha_f = std::clamp(ctx.interpolation_alpha(), 0.0f, 1.0f);
        const double interp_dt_s =
                (_last_sim_step_dt_s > 0.0) ? _last_sim_step_dt_s : static_cast<double>(ctx.fixed_delta_time());
        double now_s = _orbitsim->sim.time_s();

        // Match render interpolation: entities are rendered between prev/curr using `alpha_f`,
        // so treat "now" as within the previous->current fixed step interval.
        if (std::isfinite(interp_dt_s) && interp_dt_s > 0.0)
        {
            now_s -= (1.0 - static_cast<double>(alpha_f)) * interp_dt_s;
        }

        if (!std::isfinite(now_s))
        {
            return;
        }

        now_s = std::clamp(now_s, t0, t1);

        // Debug velocity ray is emitted into DebugDrawSystem, which prunes commands in
        // engine draw begin_frame(dt) after update_scene(), so ttl must be > dt.
        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        constexpr glm::vec4 color_orbit_full_current{0.75f, 0.20f, 0.92f, 0.22f};
        constexpr glm::vec4 color_orbit_future_current{0.75f, 0.20f, 0.92f, 0.80f};
        constexpr glm::vec4 color_orbit_planned{1.00f, 0.62f, 0.10f, 0.90f};
        constexpr glm::vec4 color_velocity{1.0f, 0.35f, 0.1f, 1.0f};

        const float line_alpha_scale = std::clamp(_prediction_line_alpha_scale, 0.1f, 8.0f);
        const float line_overlay_boost = std::clamp(_prediction_line_overlay_boost, 0.0f, 1.0f);
        auto scaled_line_color = [line_alpha_scale](glm::vec4 color) {
            color.a = std::clamp(color.a * line_alpha_scale, 0.0f, 1.0f);
            return color;
        };

        const glm::vec4 color_orbit_full = scaled_line_color(color_orbit_full_current);
        const glm::vec4 color_orbit_future = scaled_line_color(color_orbit_future_current);
        const glm::vec4 color_orbit_plan = scaled_line_color(color_orbit_planned);

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
        const double tan_half_fov =
                std::tan(glm::radians(static_cast<double>(camera_fov_deg)) * 0.5);
        const double safe_viewport_h_px = std::max(1.0, static_cast<double>(viewport_h_px));
        const double fallback_meters_per_px = 1.0;

        auto meters_per_px_at_world = [&](const WorldVec3 &p_world) -> double {
            const double dist_m = OrbitPredictionMath::safe_length(glm::dvec3(p_world) - camera_world);
            if (!std::isfinite(dist_m) || dist_m <= 1.0e-3 || !std::isfinite(tan_half_fov) || tan_half_fov <= 1.0e-8)
            {
                return fallback_meters_per_px;
            }
            const double mpp = (2.0 * tan_half_fov * dist_m) / safe_viewport_h_px;
            if (!std::isfinite(mpp) || mpp <= 1.0e-6)
            {
                return fallback_meters_per_px;
            }
            return mpp;
        };

        WorldVec3 ship_pos_world_state{0.0, 0.0, 0.0};
        glm::dvec3 ship_vel_world(0.0);
        glm::vec3 ship_vel_local_f(0.0f);
        if (!get_player_world_state(ship_pos_world_state, ship_vel_world, ship_vel_local_f))
        {
            return;
        }

        WorldVec3 ship_pos_world = ship_pos_world_state;
        const EntityId player_eid = player_entity();
        if (const Entity *player = _world.entities().find(player_eid))
        {
            ship_pos_world = player->get_render_position_world(alpha_f);
        }

        const WorldVec3 ref_body_world = prediction_reference_body_world();

        const auto &traj_base = _prediction_cache.trajectory_bci;
        const auto &points_base = _prediction_cache.points_world;
        const auto &traj_planned = _prediction_cache.trajectory_bci_planned;

        auto segments_from_samples = [](const std::vector<orbitsim::TrajectorySample> &samples) {
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
        };

        std::vector<orbitsim::TrajectorySegment> base_segments_fallback{};
        const std::vector<orbitsim::TrajectorySegment> *traj_base_segments = &_prediction_cache.trajectory_segments_bci;
        if (traj_base_segments->empty())
        {
            base_segments_fallback = segments_from_samples(traj_base);
            traj_base_segments = &base_segments_fallback;
        }
        if (traj_base_segments->empty())
        {
            return;
        }

        std::vector<orbitsim::TrajectorySegment> planned_segments_fallback{};
        const std::vector<orbitsim::TrajectorySegment> *traj_planned_segments = &_prediction_cache.trajectory_segments_bci_planned;
        if (traj_planned_segments->empty())
        {
            planned_segments_fallback = segments_from_samples(traj_planned);
            traj_planned_segments = &planned_segments_fallback;
        }

        const bool have_planned = !traj_planned_segments->empty();

        constexpr uint32_t kInvalidPickGroup = std::numeric_limits<uint32_t>::max();
        const uint32_t pick_group_base = picking ? picking->add_line_pick_group("OrbitPlot/Base") : kInvalidPickGroup;
        const uint32_t pick_group_planned = picking ? picking->add_line_pick_group("OrbitPlot/Planned") : kInvalidPickGroup;

        auto lower_bound_by_time = [&](const std::vector<orbitsim::TrajectorySample> &traj, double t_s) {
            return std::lower_bound(traj.cbegin(),
                                    traj.cend(),
                                    t_s,
                                    [](const orbitsim::TrajectorySample &s, double t) { return s.t_s < t; });
        };

        const auto it_hi = lower_bound_by_time(traj_base, now_s);
        size_t i_hi = static_cast<size_t>(std::distance(traj_base.cbegin(), it_hi));
        if (i_hi >= traj_base.size())
        {
            return;
        }

        // Align the curve to the ship at "now" to hide polyline chord error and keep the plot
        // visually attached even with small solver/physics drift.
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
            align_delta = WorldVec3(0.0, 0.0, 0.0);
        }

        OrbitPlotLodBuilder::CameraContext lod_camera{};
        lod_camera.camera_world = camera_world;
        lod_camera.tan_half_fov = tan_half_fov;
        lod_camera.viewport_height_px = safe_viewport_h_px;

        auto draw_window = [&](const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                               double t_start_s,
                               double t_end_s,
                               const glm::vec4 &color,
                               const bool dashed,
                               const uint32_t pick_group_id) {
            if (!(t_end_s > t_start_s) || traj_segments.empty())
            {
                return;
            }

            OrbitPlotLodBuilder::RenderSettings lod_settings{};
            lod_settings.error_px = kOrbitRenderErrorPx;
            lod_settings.max_segments = static_cast<size_t>(std::max(
                    1,
                    dashed ? kOrbitDrawMaxSegmentsDashed : kOrbitDrawMaxSegmentsSolid));

            const OrbitPlotLodBuilder::RenderResult lod =
                    OrbitPlotLodBuilder::build_render_lod(traj_segments,
                                                          ref_body_world,
                                                          align_delta,
                                                          lod_camera,
                                                          lod_settings,
                                                          t_start_s,
                                                          t_end_s);
            if (lod.segments.empty())
            {
                return;
            }

            const double span_s = std::max(0.0, t_end_s - t_start_s);
            const bool allow_pick_segments = !dashed || (span_s <= kOrbitDashedPickMaxSpanS);

            const double dash_on_px = 14.0;
            const double dash_off_px = 9.0;
            const double dash_period_px = dash_on_px + dash_off_px;
            double dash_phase_px = 0.0;
            constexpr int kMaxDashChunksPerSegment = 256;

            auto emit_segment = [&](const WorldVec3 &a_world,
                                    const WorldVec3 &b_world,
                                    const double a_t_s,
                                    const double b_t_s) {
                if (orbit_plot)
                {
                    orbit_plot->add_line(a_world, b_world, color, OrbitPlotDepth::DepthTested);
                }
                if (line_overlay_boost > 0.0f)
                {
                    glm::vec4 overlay_color = color;
                    overlay_color.a = std::clamp(overlay_color.a * line_overlay_boost, 0.0f, 1.0f);
                    if (overlay_color.a > 0.0f)
                    {
                        if (orbit_plot)
                        {
                            orbit_plot->add_line(a_world,
                                                 b_world,
                                                 overlay_color,
                                                 OrbitPlotDepth::AlwaysOnTop);
                        }
                    }
                }

                if (allow_pick_segments && picking && pick_group_id != kInvalidPickGroup)
                {
                    picking->add_line_pick_segment(pick_group_id,
                                                   a_world,
                                                   b_world,
                                                   a_t_s,
                                                   b_t_s);
                }
            };

            for (const OrbitPlotLodBuilder::RenderSegment &segment : lod.segments)
            {
                if (!dashed)
                {
                    emit_segment(segment.a_world, segment.b_world, segment.t0_s, segment.t1_s);
                    continue;
                }

                const double seg_m = glm::length(glm::dvec3(segment.b_world - segment.a_world));
                const double seg_dt_s = segment.t1_s - segment.t0_s;
                if (!std::isfinite(seg_m) || !(seg_m > 1.0e-9) || !std::isfinite(seg_dt_s) || !(seg_dt_s > 0.0))
                {
                    continue;
                }

                const glm::dvec3 seg_mid = glm::mix(glm::dvec3(segment.a_world), glm::dvec3(segment.b_world), 0.5);
                const double seg_mpp = meters_per_px_at_world(WorldVec3(seg_mid));
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
                while ((cursor_px + 1.0e-6) < seg_px && dash_chunks < kMaxDashChunksPerSegment)
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
                            const double a_t_s = glm::mix(segment.t0_s, segment.t1_s, u0);
                            const double b_t_s = glm::mix(segment.t0_s, segment.t1_s, u1);
                            emit_segment(a_world, b_world, a_t_s, b_t_s);
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
        };

        // Full orbit (typically one period) for context.
        if (_prediction_draw_full_orbit)
        {
            double t_full_end = t1;
            if (_prediction_cache.orbital_period_s > 0.0 && std::isfinite(_prediction_cache.orbital_period_s))
            {
                t_full_end = std::min(t0 + (_prediction_cache.orbital_period_s *
                                            OrbitPredictionTuning::kFullOrbitDrawPeriodScale),
                                      t1);
            }

            if (!_prediction_draw_future_segment)
            {
                draw_window(*traj_base_segments, t0, t_full_end, color_orbit_full, false, pick_group_base);
            }
            else
            {
                draw_window(*traj_base_segments, t0, t_full_end, color_orbit_full, false, kInvalidPickGroup);
            }
        }

        // Future segment highlight (windowed).
        if (_prediction_draw_future_segment)
        {
            const double window_s = std::max(0.0, _prediction_future_window_s);
            const double t_end = (window_s > 0.0) ? std::min(now_s + window_s, t1) : t1;

            draw_window(*traj_base_segments, now_s, t_end, color_orbit_future, false, pick_group_base);
        }

        // Planned trajectory (maneuver nodes): draw as a dashed line in a distinct color.
        if (have_planned && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
        {
            double first_node_time_s = std::numeric_limits<double>::infinity();
            double first_future_node_time_s = std::numeric_limits<double>::infinity();
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (std::isfinite(node.time_s))
                {
                    first_node_time_s = std::min(first_node_time_s, node.time_s);
                    if (node.time_s >= now_s)
                    {
                        first_future_node_time_s = std::min(first_future_node_time_s, node.time_s);
                    }
                }
            }

            if (std::isfinite(first_node_time_s))
            {
                const double t0p = traj_planned_segments->front().t0_s;
                const double t1p = traj_planned_segments->back().t0_s + traj_planned_segments->back().dt_s;
                const double anchor_time_s =
                        std::isfinite(first_future_node_time_s) ? first_future_node_time_s : first_node_time_s;
                double t_plan_start = anchor_time_s;
                t_plan_start = std::clamp(t_plan_start, t0p, t1p);

                double t_plan_end = t_plan_start;
                const double window_s = std::max(0.0, _prediction_future_window_s);
                if (_prediction_draw_future_segment && window_s > 0.0)
                {
                    t_plan_end = std::min(t_plan_start + window_s, t1p);
                }
                else if (_prediction_draw_full_orbit)
                {
                    double t_full_end = t1p;
                    if (_prediction_cache.orbital_period_s > 0.0 && std::isfinite(_prediction_cache.orbital_period_s))
                    {
                        t_full_end = std::min(t0p + (_prediction_cache.orbital_period_s *
                                                     OrbitPredictionTuning::kFullOrbitDrawPeriodScale),
                                              t1p);
                    }
                    t_plan_end = t_full_end;
                }

                if (t_plan_end > t_plan_start)
                {
                    draw_window(*traj_planned_segments,
                                t_plan_start,
                                t_plan_end,
                                color_orbit_plan,
                                kPredictionDrawPlannedAsDashed,
                                pick_group_planned);
                }
            }
        }

        if (_prediction_draw_velocity_ray && _debug_draw_enabled)
        {
            const double speed_mps = glm::length(ship_vel_world);
            double len_m = 40.0;
            if (std::isfinite(speed_mps) && speed_mps > 1.0)
            {
                len_m = std::clamp(speed_mps * 0.002, 10.0, 250.0);
            }

            ctx.api->debug_draw_ray(glm::dvec3(ship_pos_world), ship_vel_world, len_m, color_velocity, ttl_s, true);
        }
    }
} // namespace Game
