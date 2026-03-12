#include "game/states/gameplay/prediction/gameplay_state_prediction_draw_internal.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    // Build per-frame orbit visuals, pick data, and debug overlays from the latest prediction cache.
    void GameplayState::emit_orbit_prediction_debug(GameStateContext &ctx)
    {
        PickingSystem *picking = (ctx.renderer != nullptr) ? ctx.renderer->picking() : nullptr;
        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        Draw::reset_orbit_plot_state(picking, orbit_plot, _orbit_plot_perf, _prediction_enabled);

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
        const double display_time_s =
                Draw::compute_prediction_display_time_s(_orbitsim->sim.time_s(),
                                                        _last_sim_step_dt_s,
                                                        ctx.fixed_delta_time(),
                                                        alpha_f);
        if (!std::isfinite(display_time_s))
        {
            return;
        }

        // Debug velocity ray is emitted into DebugDrawSystem, which prunes commands in
        // engine draw begin_frame(dt) after update_scene(), so ttl must be > dt.
        const float ttl_s = std::clamp(ctx.delta_time(), 0.0f, 0.1f) + 0.002f;

        const float line_alpha_scale = std::clamp(_prediction_line_alpha_scale, 0.1f, 8.0f);
        const glm::vec4 color_orbit_full =
                Draw::scale_line_color(_prediction_draw_config.palette.orbit_full, line_alpha_scale);
        const glm::vec4 color_orbit_future =
                Draw::scale_line_color(_prediction_draw_config.palette.orbit_future, line_alpha_scale);
        const glm::vec4 color_orbit_plan =
                Draw::scale_line_color(_prediction_draw_config.palette.orbit_planned, line_alpha_scale);

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

        std::size_t overlay_draw_index = 0;
        for (PredictionTrackState *track : visible_tracks)
        {
            if (!track)
            {
                continue;
            }

            refresh_prediction_derived_cache(*track, display_time_s);
            if (!track->cache.valid || track->cache.trajectory_frame.size() < 2)
            {
                continue;
            }

            const auto &traj_base = track->cache.trajectory_frame;
            const auto &traj_planned = track->cache.trajectory_frame_planned;
            WorldVec3 ref_body_world{0.0, 0.0, 0.0};
            glm::dmat3 frame_to_world(1.0);
            if (!build_prediction_display_transform(track->cache, ref_body_world, frame_to_world, display_time_s))
            {
                ref_body_world = prediction_frame_origin_world(track->cache, display_time_s);
                frame_to_world = glm::dmat3(1.0);
            }

            const double t0 = traj_base.front().t_s;
            const double t1 = traj_base.back().t_s;
            if (!(t1 > t0))
            {
                continue;
            }

            const double now_s = Draw::compute_prediction_now_s(display_time_s, t0, t1);
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
            const std::vector<orbitsim::TrajectorySegment> *traj_base_segments = &track->cache.trajectory_segments_frame;
            if (traj_base_segments->empty())
            {
                base_segments_fallback = Draw::trajectory_segments_from_samples(traj_base);
                traj_base_segments = &base_segments_fallback;
            }
            if (traj_base_segments->empty())
            {
                continue;
            }

            std::vector<orbitsim::TrajectorySegment> planned_segments_fallback;
            const std::vector<orbitsim::TrajectorySegment> *traj_planned_segments = &track->cache.trajectory_segments_frame_planned;
            if (traj_planned_segments->empty() && !traj_planned.empty())
            {
                planned_segments_fallback = Draw::trajectory_segments_from_samples(traj_planned);
                traj_planned_segments = &planned_segments_fallback;
            }

            const bool is_active = track->key == _prediction_selection.active_subject;
            const bool active_player_track = is_active && prediction_subject_is_player(track->key);
            if (is_active)
            {
                _orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(traj_base_segments->size());
                _orbit_plot_perf.solver_segments_planned = static_cast<uint32_t>(traj_planned_segments->size());
            }

            const std::size_t i_hi = Draw::lower_bound_sample_index(traj_base, now_s);
            if (i_hi >= traj_base.size())
            {
                continue;
            }

            const WorldVec3 align_delta =
                    Draw::compute_align_delta(traj_base, i_hi, subject_pos_world, now_s, ref_body_world, frame_to_world);
            const bool direct_world_polyline =
                    Draw::frame_spec_uses_direct_world_polyline(
                            track->cache.resolved_frame_spec_valid ? track->cache.resolved_frame_spec
                                                                   : _prediction_frame_selection.spec);

            Draw::OrbitDrawWindowContext draw_ctx{};
            draw_ctx.orbit_plot = orbit_plot;
            draw_ctx.ref_body_world = ref_body_world;
            draw_ctx.frame_to_world = frame_to_world;
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
                track_color_full = Draw::scale_line_color(glm::vec4(seed.r, seed.g, seed.b, 0.18f), line_alpha_scale);
                track_color_future = Draw::scale_line_color(glm::vec4(seed.r, seed.g, seed.b, 0.58f), line_alpha_scale);
                track_color_plan = Draw::scale_line_color(glm::vec4(seed.r, seed.g, seed.b, 0.75f), line_alpha_scale);
                draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost * 0.35f, 0.0f, 1.0f);
            }

            const double future_window_s = prediction_future_window_s(track->key);
            Draw::PickWindow base_pick_window{};
            if (_prediction_draw_full_orbit)
            {
                double t_full_end = t1;
                if (track->cache.orbital_period_s > 0.0 && std::isfinite(track->cache.orbital_period_s))
                {
                    t_full_end = std::min(t0 + (track->cache.orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale), t1);
                }

                if (direct_world_polyline)
                {
                    Draw::draw_polyline_window(draw_ctx,
                                               _prediction_draw_config,
                                               traj_base,
                                               t0,
                                               t_full_end,
                                               track_color_full,
                                               false);
                }
                else
                {
                    Draw::draw_orbit_window(draw_ctx,
                                            _prediction_draw_config,
                                            _orbit_plot_perf,
                                            *traj_base_segments,
                                            t0,
                                            t_full_end,
                                            track_color_full,
                                            false);
                }
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

                if (direct_world_polyline)
                {
                    Draw::draw_polyline_window(draw_ctx,
                                               _prediction_draw_config,
                                               traj_base,
                                               now_s,
                                               t_end,
                                               track_color_future,
                                               false);
                }
                else
                {
                    Draw::draw_orbit_window(draw_ctx,
                                            _prediction_draw_config,
                                            _orbit_plot_perf,
                                            *traj_base_segments,
                                            now_s,
                                            t_end,
                                            track_color_future,
                                            false);
                }
                if (is_active && t_end > now_s)
                {
                    base_pick_window.valid = true;
                    base_pick_window.t0_s = now_s;
                    base_pick_window.t1_s = t_end;
                }
            }

            const Draw::PickWindow planned_pick_window =
                    (active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty() && !traj_planned_segments->empty())
                            ? Draw::build_planned_pick_window(*traj_planned_segments,
                                                              _prediction_draw_config,
                                                              _maneuver_state.nodes,
                                                              now_s,
                                                              future_window_s,
                                                              _prediction_draw_future_segment,
                                                              _prediction_draw_full_orbit,
                                                              track->cache.orbital_period_s)
                            : Draw::PickWindow{};
            if (planned_pick_window.valid)
            {
                if (direct_world_polyline)
                {
                    Draw::draw_polyline_window(draw_ctx,
                                               _prediction_draw_config,
                                               traj_planned,
                                               planned_pick_window.t0_s,
                                               planned_pick_window.t1_s,
                                               track_color_plan,
                                               _prediction_draw_config.draw_planned_as_dashed);
                }
                else
                {
                    Draw::draw_orbit_window(draw_ctx,
                                            _prediction_draw_config,
                                            _orbit_plot_perf,
                                            *traj_planned_segments,
                                            planned_pick_window.t0_s,
                                            planned_pick_window.t1_s,
                                            track_color_plan,
                                            _prediction_draw_config.draw_planned_as_dashed);
                }
            }

            if (picking && active_player_track)
            {
                const bool allow_base_pick = _maneuver_state.nodes.empty();
                const bool allow_planned_pick = !_maneuver_state.nodes.empty();
                const uint32_t pick_group_base = picking->add_line_pick_group("OrbitPlot/Base");
                const uint32_t pick_group_planned = picking->add_line_pick_group("OrbitPlot/Planned");
                const std::vector<double> maneuver_node_times_s = Draw::collect_maneuver_node_times(_maneuver_state.nodes);

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
                    pick_group_base != Draw::kInvalidPickGroup &&
                    remaining_pick_budget > 0)
                {
                    std::size_t emitted = 0;
                    const std::size_t planned_reserve =
                            planned_pick_window.valid
                                    ? std::min(pick_planned_reserve_target, remaining_pick_budget)
                                    : 0;
                    if (direct_world_polyline)
                    {
                        emitted = Draw::emit_polyline_pick_segments(draw_ctx,
                                                                    picking,
                                                                    pick_group_base,
                                                                    traj_base,
                                                                    base_pick_window.t0_s,
                                                                    base_pick_window.t1_s,
                                                                    remaining_pick_budget - planned_reserve,
                                                                    _orbit_plot_perf);
                    }
                    else
                    {
                        OrbitPlotLodBuilder::PickSettings pick_settings{};
                        pick_settings.max_segments = remaining_pick_budget - planned_reserve;
                        pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;

                        std::vector<double> base_anchor_times{};
                        base_anchor_times.reserve(1 + maneuver_node_times_s.size());
                        base_anchor_times.push_back(now_s);
                        base_anchor_times.insert(base_anchor_times.end(),
                                                 maneuver_node_times_s.begin(),
                                                 maneuver_node_times_s.end());

                        emitted = Draw::emit_pick_segments(picking,
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
                    }
                    remaining_pick_budget = (emitted >= remaining_pick_budget) ? 0 : (remaining_pick_budget - emitted);
                }

                if (allow_planned_pick &&
                    planned_pick_window.valid &&
                    pick_group_planned != Draw::kInvalidPickGroup &&
                    remaining_pick_budget > 0)
                {
                    if (direct_world_polyline)
                    {
                        Draw::emit_polyline_pick_segments(draw_ctx,
                                                          picking,
                                                          pick_group_planned,
                                                          traj_planned,
                                                          planned_pick_window.t0_s,
                                                          planned_pick_window.t1_s,
                                                          remaining_pick_budget,
                                                          _orbit_plot_perf);
                    }
                    else
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

                        Draw::emit_pick_segments(picking,
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
            }

            if (is_active &&
                _prediction_draw_velocity_ray &&
                _debug_draw_enabled &&
                track->key.kind == PredictionSubjectKind::Orbiter)
            {
                Draw::emit_velocity_ray(ctx.api,
                                        subject_pos_world,
                                        subject_vel_world,
                                        ttl_s,
                                        _prediction_draw_config.palette.velocity_ray);
            }
        }
    }
} // namespace Game
