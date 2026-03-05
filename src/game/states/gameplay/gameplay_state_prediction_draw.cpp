#include "gameplay_state.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_plot_lod_builder.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr std::size_t kOrbitPickPlannedReserveSegments = 2'000;
        constexpr double kOrbitPickPlannedReserveRatio = 0.25;
        constexpr double kOrbitNodeTimeToleranceS = 1.0e-3;
        constexpr int kOrbitDashMaxChunksPerSegment = 4096;
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

        _orbit_plot_perf.render_lod_ms_last = 0.0;
        _orbit_plot_perf.pick_lod_ms_last = 0.0;
        _orbit_plot_perf.solver_segments_base = 0;
        _orbit_plot_perf.solver_segments_planned = 0;
        _orbit_plot_perf.pick_segments_before_cull = 0;
        _orbit_plot_perf.pick_segments = 0;
        _orbit_plot_perf.render_cap_hit_last_frame = false;
        _orbit_plot_perf.pick_cap_hit_last_frame = false;

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
        _orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(traj_base_segments->size());
        _orbit_plot_perf.solver_segments_planned = static_cast<uint32_t>(traj_planned_segments->size());

        constexpr uint32_t kInvalidPickGroup = std::numeric_limits<uint32_t>::max();
        const uint32_t pick_group_base = picking ? picking->add_line_pick_group("OrbitPlot/Base") : kInvalidPickGroup;
        const uint32_t pick_group_planned = picking ? picking->add_line_pick_group("OrbitPlot/Planned") : kInvalidPickGroup;

        std::vector<double> maneuver_node_times_s{};
        if (_maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
        {
            maneuver_node_times_s.reserve(_maneuver_state.nodes.size());
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (std::isfinite(node.time_s))
                {
                    maneuver_node_times_s.push_back(node.time_s);
                }
            }
            std::sort(maneuver_node_times_s.begin(), maneuver_node_times_s.end());
            maneuver_node_times_s.erase(std::unique(maneuver_node_times_s.begin(), maneuver_node_times_s.end()),
                                        maneuver_node_times_s.end());
        }

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
        const double render_error_px =
                (std::isfinite(_orbit_plot_render_error_px) && _orbit_plot_render_error_px > 0.0)
                        ? _orbit_plot_render_error_px
                        : 0.75;
        const std::size_t render_max_segments =
                static_cast<std::size_t>(std::max(1, _orbit_plot_render_max_segments_cpu));

        auto draw_window = [&](const std::vector<orbitsim::TrajectorySegment> &traj_segments,
                               double t_start_s,
                               double t_end_s,
                               const glm::vec4 &color,
                               const bool dashed) {
            if (!(t_end_s > t_start_s) || traj_segments.empty())
            {
                return;
            }

            OrbitPlotLodBuilder::RenderSettings lod_settings{};
            lod_settings.error_px = render_error_px;
            lod_settings.max_segments = render_max_segments;

            const auto render_lod_start_tp = std::chrono::steady_clock::now();
            const OrbitPlotLodBuilder::RenderResult lod =
                    OrbitPlotLodBuilder::build_render_lod(traj_segments,
                                                          ref_body_world,
                                                          align_delta,
                                                          lod_camera,
                                                          lod_settings,
                                                          t_start_s,
                                                          t_end_s);
            _orbit_plot_perf.render_lod_ms_last +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - render_lod_start_tp)
                            .count();
            if (lod.cap_hit)
            {
                _orbit_plot_perf.render_cap_hit_last_frame = true;
                ++_orbit_plot_perf.render_cap_hits_total;
            }
            if (lod.segments.empty())
            {
                return;
            }

            const double dash_on_px = 14.0;
            const double dash_off_px = 9.0;
            const double dash_period_px = dash_on_px + dash_off_px;
            double dash_phase_px = 0.0;

            auto emit_segment = [&](const WorldVec3 &a_world, const WorldVec3 &b_world) {
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
            };

            for (const OrbitPlotLodBuilder::RenderSegment &segment : lod.segments)
            {
                if (!dashed)
                {
                    emit_segment(segment.a_world, segment.b_world);
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
                while ((cursor_px + 1.0e-6) < seg_px && dash_chunks < kOrbitDashMaxChunksPerSegment)
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
                            emit_segment(a_world, b_world);
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

        bool base_pick_window_valid = false;
        double base_pick_t0_s = 0.0;
        double base_pick_t1_s = 0.0;

        bool planned_pick_window_valid = false;
        double planned_pick_t0_s = 0.0;
        double planned_pick_t1_s = 0.0;
        double planned_anchor_time_s = std::numeric_limits<double>::quiet_NaN();

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

            draw_window(*traj_base_segments, t0, t_full_end, color_orbit_full, false);
            if (!_prediction_draw_future_segment && t_full_end > t0)
            {
                base_pick_window_valid = true;
                base_pick_t0_s = t0;
                base_pick_t1_s = t_full_end;
            }
        }

        // Future segment highlight (windowed).
        if (_prediction_draw_future_segment)
        {
            const double window_s = std::max(0.0, _prediction_future_window_s);
            const double t_end = (window_s > 0.0) ? std::min(now_s + window_s, t1) : t1;

            draw_window(*traj_base_segments, now_s, t_end, color_orbit_future, false);
            if (t_end > now_s)
            {
                base_pick_window_valid = true;
                base_pick_t0_s = now_s;
                base_pick_t1_s = t_end;
            }
        }

        // Planned trajectory (maneuver nodes): draw as a dashed line in a distinct color.
        if (have_planned && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
        {
            const double t0p = traj_planned_segments->front().t0_s;
            const double t1p = traj_planned_segments->back().t0_s + traj_planned_segments->back().dt_s;
            if (t1p > t0p)
            {
                auto node_time_in_planned_range = [&](const double t_s) -> bool {
                    return std::isfinite(t_s) &&
                           t_s >= (t0p - kOrbitNodeTimeToleranceS) &&
                           t_s <= (t1p + kOrbitNodeTimeToleranceS);
                };

                double selected_node_time_s = std::numeric_limits<double>::infinity();
                if (const ManeuverNode *selected = _maneuver_state.find_node(_maneuver_state.selected_node_id))
                {
                    if (node_time_in_planned_range(selected->time_s))
                    {
                        selected_node_time_s = selected->time_s;
                    }
                }

                double first_future_node_time_s = std::numeric_limits<double>::infinity();
                double first_relevant_node_time_s = std::numeric_limits<double>::infinity();
                for (const ManeuverNode &node : _maneuver_state.nodes)
                {
                    if (!node_time_in_planned_range(node.time_s))
                    {
                        continue;
                    }

                    first_relevant_node_time_s = std::min(first_relevant_node_time_s, node.time_s);
                    if (node.time_s >= (now_s + kOrbitNodeTimeToleranceS))
                    {
                        first_future_node_time_s = std::min(first_future_node_time_s, node.time_s);
                    }
                }

                double anchor_time_s = selected_node_time_s;
                if (!std::isfinite(anchor_time_s))
                {
                    anchor_time_s = first_future_node_time_s;
                }
                if (!std::isfinite(anchor_time_s))
                {
                    anchor_time_s = first_relevant_node_time_s;
                }

                if (std::isfinite(anchor_time_s))
                {
                    const double t_plan_start = std::clamp(anchor_time_s, t0p, t1p);

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
                                    kPredictionDrawPlannedAsDashed);

                        planned_pick_window_valid = true;
                        planned_pick_t0_s = t_plan_start;
                        planned_pick_t1_s = t_plan_end;
                        planned_anchor_time_s = anchor_time_s;
                    }
                }
            }
        }

        if (picking)
        {
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
                    (std::isfinite(_orbit_plot_pick_frustum_margin_ratio) &&
                     _orbit_plot_pick_frustum_margin_ratio >= 0.0)
                            ? _orbit_plot_pick_frustum_margin_ratio
                            : 0.05;
            const std::size_t pick_planned_reserve_target = std::min(
                    kOrbitPickPlannedReserveSegments,
                    static_cast<std::size_t>(std::max<std::size_t>(
                            1,
                            static_cast<std::size_t>(
                                    std::llround(static_cast<double>(pick_max_segments) * kOrbitPickPlannedReserveRatio)))));
            std::size_t remaining_pick_budget = pick_max_segments;

            if (base_pick_window_valid && pick_group_base != kInvalidPickGroup && remaining_pick_budget > 0)
            {
                const std::size_t planned_reserve =
                        planned_pick_window_valid
                                ? std::min(pick_planned_reserve_target, remaining_pick_budget)
                                : 0;
                const std::size_t base_budget = remaining_pick_budget - planned_reserve;

                OrbitPlotLodBuilder::PickSettings pick_settings{};
                pick_settings.max_segments = base_budget;
                pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;

                std::vector<double> base_anchor_times{};
                base_anchor_times.reserve(1 + maneuver_node_times_s.size());
                base_anchor_times.push_back(now_s);
                base_anchor_times.insert(base_anchor_times.end(),
                                         maneuver_node_times_s.begin(),
                                         maneuver_node_times_s.end());

                const auto pick_lod_start_tp = std::chrono::steady_clock::now();
                const OrbitPlotLodBuilder::PickResult pick_lod =
                        OrbitPlotLodBuilder::build_pick_lod(*traj_base_segments,
                                                            ref_body_world,
                                                            align_delta,
                                                            pick_frustum,
                                                            pick_settings,
                                                            base_pick_t0_s,
                                                            base_pick_t1_s,
                                                            base_anchor_times);
                _orbit_plot_perf.pick_lod_ms_last +=
                        std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - pick_lod_start_tp)
                                .count();
                _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(pick_lod.segments_before_cull);
                _orbit_plot_perf.pick_segments += static_cast<uint32_t>(pick_lod.segments.size());
                if (pick_lod.cap_hit)
                {
                    _orbit_plot_perf.pick_cap_hit_last_frame = true;
                    ++_orbit_plot_perf.pick_cap_hits_total;
                }
                for (const OrbitPlotLodBuilder::PickSegment &segment : pick_lod.segments)
                {
                    picking->add_line_pick_segment(pick_group_base,
                                                   segment.a_world,
                                                   segment.b_world,
                                                   segment.t0_s,
                                                   segment.t1_s);
                }

                if (pick_lod.segments.size() >= remaining_pick_budget)
                {
                    remaining_pick_budget = 0;
                }
                else
                {
                    remaining_pick_budget -= pick_lod.segments.size();
                }
            }

            if (planned_pick_window_valid && pick_group_planned != kInvalidPickGroup && remaining_pick_budget > 0)
            {
                OrbitPlotLodBuilder::PickSettings pick_settings{};
                pick_settings.max_segments = remaining_pick_budget;
                pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;

                std::vector<double> planned_anchor_times{};
                planned_anchor_times.reserve(1 + maneuver_node_times_s.size());
                if (std::isfinite(planned_anchor_time_s))
                {
                    planned_anchor_times.push_back(planned_anchor_time_s);
                }
                planned_anchor_times.insert(planned_anchor_times.end(),
                                            maneuver_node_times_s.begin(),
                                            maneuver_node_times_s.end());

                const auto pick_lod_start_tp = std::chrono::steady_clock::now();
                const OrbitPlotLodBuilder::PickResult pick_lod =
                        OrbitPlotLodBuilder::build_pick_lod(*traj_planned_segments,
                                                            ref_body_world,
                                                            align_delta,
                                                            pick_frustum,
                                                            pick_settings,
                                                            planned_pick_t0_s,
                                                            planned_pick_t1_s,
                                                            planned_anchor_times);
                _orbit_plot_perf.pick_lod_ms_last +=
                        std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - pick_lod_start_tp)
                                .count();
                _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(pick_lod.segments_before_cull);
                _orbit_plot_perf.pick_segments += static_cast<uint32_t>(pick_lod.segments.size());
                if (pick_lod.cap_hit)
                {
                    _orbit_plot_perf.pick_cap_hit_last_frame = true;
                    ++_orbit_plot_perf.pick_cap_hits_total;
                }
                for (const OrbitPlotLodBuilder::PickSegment &segment : pick_lod.segments)
                {
                    picking->add_line_pick_segment(pick_group_planned,
                                                   segment.a_world,
                                                   segment.b_world,
                                                   segment.t0_s,
                                                   segment.t1_s);
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
