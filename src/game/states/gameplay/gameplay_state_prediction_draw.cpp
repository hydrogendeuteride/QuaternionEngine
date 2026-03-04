#include "gameplay_state.h"

#include "game/orbit/orbit_prediction_math.h"
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
        constexpr double kOrbitDrawMaxDtS = 1.0; // visual subdivision step (reduces polyline chord error)
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

        const bool have_planned =
                !_prediction_cache.trajectory_bci_planned.empty() &&
                _prediction_cache.trajectory_bci_planned.size() == _prediction_cache.points_world_planned.size();
        const auto &traj_planned = _prediction_cache.trajectory_bci_planned;
        const auto &points_planned = _prediction_cache.points_world_planned;

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

        auto draw_world = [&](const WorldVec3 &p_world) -> WorldVec3 { return p_world + align_delta; };

        auto draw_hermite = [&](const orbitsim::TrajectorySample &a,
                                const orbitsim::TrajectorySample &b,
                                const double t_s) -> WorldVec3 {
            return OrbitPredictionMath::hermite_position_world(ref_body_world, a, b, t_s) + align_delta;
        };

        auto draw_position_at = [&](const std::vector<orbitsim::TrajectorySample> &traj,
                                    const std::vector<WorldVec3> &points_world,
                                    const double t_s) -> WorldVec3 {
            if (traj.size() < 2 || points_world.size() != traj.size())
            {
                return ship_pos_world;
            }

            const double tc = std::clamp(t_s, traj.front().t_s, traj.back().t_s);
            const auto it = lower_bound_by_time(traj, tc);
            const size_t i = static_cast<size_t>(std::distance(traj.cbegin(), it));
            if (i == 0)
            {
                return draw_world(points_world.front());
            }
            if (i >= traj.size())
            {
                return draw_world(points_world.back());
            }
            return draw_hermite(traj[i - 1], traj[i], tc);
        };

        auto draw_window = [&](const std::vector<orbitsim::TrajectorySample> &traj,
                               double t_start_s,
                               double t_end_s,
                               const glm::vec4 &color,
                               WorldVec3 prev_world,
                               const bool dashed,
                               const uint32_t pick_group_id) {
            if (!(t_end_s > t_start_s))
            {
                return;
            }

            const size_t n = traj.size();
            if (n < 2)
            {
                return;
            }

            auto it_start_hi = lower_bound_by_time(traj, t_start_s);
            const size_t i_start_hi = static_cast<size_t>(std::distance(traj.cbegin(), it_start_hi));
            if (i_start_hi >= n)
            {
                return;
            }

            size_t seg = (i_start_hi == 0) ? 0 : (i_start_hi - 1);
            double t = std::clamp(t_start_s, traj.front().t_s, traj.back().t_s);
            const double t_end = std::clamp(t_end_s, traj.front().t_s, traj.back().t_s);
            const double span_s = std::max(0.0, t_end - t);
            const int max_segments = dashed ? kOrbitDrawMaxSegmentsDashed : kOrbitDrawMaxSegmentsSolid;
            const double draw_dt_s = std::max(kOrbitDrawMaxDtS, span_s / static_cast<double>(std::max(1, max_segments)));
            const bool allow_pick_segments = !dashed || (span_s <= kOrbitDashedPickMaxSpanS);

            const double dash_on_px = 14.0;
            const double dash_off_px = 9.0;
            const double dash_period_px = dash_on_px + dash_off_px;
            double dash_phase_px = 0.0;
            double prev_t_s = t_start_s;

            while (t < t_end && (seg + 1) < n)
            {
                const orbitsim::TrajectorySample &a = traj[seg];
                const orbitsim::TrajectorySample &b = traj[seg + 1];

                const double seg_start = std::max(t, a.t_s);
                const double seg_end = std::min(t_end, b.t_s);
                const double seg_len = seg_end - seg_start;
                if (!(seg_len > 0.0) || !std::isfinite(seg_len))
                {
                    ++seg;
                    continue;
                }

                const int sub = std::max(1, static_cast<int>(std::ceil(seg_len / draw_dt_s)));
                for (int j = 1; j <= sub; ++j)
                {
                    const double u = static_cast<double>(j) / static_cast<double>(sub);
                    const double tj = seg_start + seg_len * u;
                    const WorldVec3 p = draw_hermite(a, b, tj);

                    bool draw = true;
                    if (dashed)
                    {
                        draw = dash_phase_px < dash_on_px;

                        const double seg_m = glm::length(glm::dvec3(p - prev_world));
                        if (std::isfinite(seg_m) && seg_m > 0.0)
                        {
                            const glm::dvec3 seg_mid = glm::mix(glm::dvec3(prev_world), glm::dvec3(p), 0.5);
                            const double seg_mpp = meters_per_px_at_world(WorldVec3(seg_mid));
                            if (std::isfinite(seg_mpp) && seg_mpp > 1.0e-6)
                            {
                                const double seg_px = seg_m / seg_mpp;
                                if (std::isfinite(seg_px) && seg_px > 0.0)
                                {
                                    dash_phase_px += seg_px;
                                    if (dash_phase_px >= dash_period_px)
                                    {
                                        dash_phase_px = std::fmod(dash_phase_px, dash_period_px);
                                    }
                                }
                            }
                        }
                    }

                    if (draw)
                    {
                        if (orbit_plot)
                        {
                            orbit_plot->add_line(prev_world, p, color, OrbitPlotDepth::DepthTested);
                        }
                        if (line_overlay_boost > 0.0f)
                        {
                            glm::vec4 overlay_color = color;
                            overlay_color.a = std::clamp(overlay_color.a * line_overlay_boost, 0.0f, 1.0f);
                            if (overlay_color.a > 0.0f)
                            {
                                if (orbit_plot)
                                {
                                    orbit_plot->add_line(prev_world, p, overlay_color, OrbitPlotDepth::AlwaysOnTop);
                                }
                            }
                        }

                        if (allow_pick_segments && picking && pick_group_id != kInvalidPickGroup)
                        {
                            picking->add_line_pick_segment(pick_group_id,
                                                           prev_world,
                                                           p,
                                                           prev_t_s,
                                                           tj);
                        }
                    }
                    prev_world = p;
                    prev_t_s = tj;
                }

                t = seg_end;
                if (t >= b.t_s)
                {
                    ++seg;
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
                draw_window(traj_base, t0, t_full_end, color_orbit_full, draw_world(points_base.front()), false, pick_group_base);
            }
            else
            {
                draw_window(traj_base, t0, t_full_end, color_orbit_full, draw_world(points_base.front()), false, kInvalidPickGroup);
            }
        }

        // Future segment highlight (windowed).
        if (_prediction_draw_future_segment)
        {
            const double window_s = std::max(0.0, _prediction_future_window_s);
            const double t_end = (window_s > 0.0) ? std::min(now_s + window_s, t1) : t1;

            draw_window(traj_base, now_s, t_end, color_orbit_future, ship_pos_world, false, pick_group_base);
        }

        // Planned trajectory (maneuver nodes): draw as a dashed line in a distinct color.
        if (have_planned && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
        {
            double first_node_time_s = std::numeric_limits<double>::infinity();
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (std::isfinite(node.time_s))
                {
                    first_node_time_s = std::min(first_node_time_s, node.time_s);
                }
            }

            if (std::isfinite(first_node_time_s))
            {
                const double t0p = traj_planned.front().t_s;
                const double t1p = traj_planned.back().t_s;
                double t_plan_start = std::max(first_node_time_s, now_s);
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
                    const WorldVec3 p_start = draw_position_at(traj_planned, points_planned, t_plan_start);
                    draw_window(traj_planned,
                                t_plan_start,
                                t_plan_end,
                                color_orbit_plan,
                                p_start,
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
