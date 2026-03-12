#include "game/states/gameplay/prediction/gameplay_state_prediction_draw_internal.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    namespace
    {
        constexpr double kPickWindowRebuildEpsilonS = 0.25;
        constexpr double kPickAlignRebuildDistanceM = 25.0;
        constexpr double kPickReferenceRebuildDistanceM = 1.0;
        constexpr double kPickMatrixRebuildEpsilon = 1.0e-6;

        bool same_matrix(const glm::dmat3 &a, const glm::dmat3 &b, const double epsilon)
        {
            for (int col = 0; col < 3; ++col)
            {
                for (int row = 0; row < 3; ++row)
                {
                    if (!std::isfinite(a[col][row]) || !std::isfinite(b[col][row]) ||
                        std::abs(a[col][row] - b[col][row]) > epsilon)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        void enqueue_cached_orbit_window(OrbitPlotSystem *orbit_plot,
                                         const std::shared_ptr<const std::vector<OrbitPlotSystem::GpuRootSegment>> &cached_roots,
                                         const WorldVec3 &ref_body_world,
                                         const glm::dmat3 &frame_to_world,
                                         const WorldVec3 &align_delta,
                                         const double t_start_s,
                                         const double t_end_s,
                                         const glm::vec4 &color,
                                         const bool dashed,
                                         const float line_overlay_boost)
        {
            if (!orbit_plot || !cached_roots || cached_roots->empty() || !(t_end_s > t_start_s) ||
                !Draw::frame_transform_is_identity(frame_to_world))
            {
                return;
            }

            orbit_plot->add_gpu_root_batch(cached_roots,
                                           t_start_s,
                                           t_end_s,
                                           ref_body_world,
                                           align_delta,
                                           color,
                                           dashed,
                                           OrbitPlotDepth::DepthTested);

            if (line_overlay_boost <= 0.0f)
            {
                return;
            }

            glm::vec4 overlay_color = color;
            overlay_color.a = std::clamp(overlay_color.a * line_overlay_boost, 0.0f, 1.0f);
            if (overlay_color.a <= 0.0f)
            {
                return;
            }

            orbit_plot->add_gpu_root_batch(cached_roots,
                                           t_start_s,
                                           t_end_s,
                                           ref_body_world,
                                           align_delta,
                                           overlay_color,
                                           dashed,
                                           OrbitPlotDepth::AlwaysOnTop);
        }

        bool should_rebuild_pick_cache(const PredictionLinePickCache &cache,
                                       const uint64_t generation_id,
                                       const WorldVec3 &ref_body_world,
                                       const glm::dmat3 &frame_to_world,
                                       const WorldVec3 &align_delta,
                                       const double t0_s,
                                       const double t1_s,
                                       const std::size_t max_segments,
                                       const bool planned)
        {
            const bool valid = planned ? cache.planned_valid : cache.base_valid;
            const double cached_t0_s = planned ? cache.planned_t0_s : cache.base_t0_s;
            const double cached_t1_s = planned ? cache.planned_t1_s : cache.base_t1_s;
            const std::size_t cached_max_segments = planned ? cache.planned_max_segments : cache.base_max_segments;
            if (!valid || cache.generation_id != generation_id || cached_max_segments != max_segments)
            {
                return true;
            }

            if (!same_matrix(cache.frame_to_world, frame_to_world, kPickMatrixRebuildEpsilon))
            {
                return true;
            }

            if (glm::length(glm::dvec3(cache.ref_body_world - ref_body_world)) > kPickReferenceRebuildDistanceM ||
                glm::length(glm::dvec3(cache.align_delta_world - align_delta)) > kPickAlignRebuildDistanceM)
            {
                return true;
            }

            return !std::isfinite(cached_t0_s) || !std::isfinite(cached_t1_s) ||
                   std::abs(cached_t0_s - t0_s) > kPickWindowRebuildEpsilonS ||
                   std::abs(cached_t1_s - t1_s) > kPickWindowRebuildEpsilonS;
        }

        void mark_pick_cache_valid(PredictionLinePickCache &cache,
                                   const uint64_t generation_id,
                                   const WorldVec3 &ref_body_world,
                                   const glm::dmat3 &frame_to_world,
                                   const WorldVec3 &align_delta,
                                   const double t0_s,
                                   const double t1_s,
                                   const std::size_t max_segments,
                                   const bool planned)
        {
            cache.generation_id = generation_id;
            cache.ref_body_world = ref_body_world;
            cache.frame_to_world = frame_to_world;
            cache.align_delta_world = align_delta;
            if (planned)
            {
                cache.planned_valid = true;
                cache.planned_t0_s = t0_s;
                cache.planned_t1_s = t1_s;
                cache.planned_max_segments = max_segments;
            }
            else
            {
                cache.base_valid = true;
                cache.base_t0_s = t0_s;
                cache.base_t1_s = t1_s;
                cache.base_max_segments = max_segments;
            }
        }
    } // namespace

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
            const bool use_persistent_gpu_roots =
                    orbit_plot && orbit_plot->settings().gpu_generate_enabled &&
                    !direct_world_polyline &&
                    Draw::frame_transform_is_identity(frame_to_world);

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
                else if (use_persistent_gpu_roots && track->cache.gpu_roots_frame && !track->cache.gpu_roots_frame->empty())
                {
                    enqueue_cached_orbit_window(orbit_plot,
                                                track->cache.gpu_roots_frame,
                                                ref_body_world,
                                                frame_to_world,
                                                align_delta,
                                                t0,
                                                t_full_end,
                                                track_color_full,
                                                false,
                                                draw_ctx.line_overlay_boost);
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
                else if (use_persistent_gpu_roots && track->cache.gpu_roots_frame && !track->cache.gpu_roots_frame->empty())
                {
                    enqueue_cached_orbit_window(orbit_plot,
                                                track->cache.gpu_roots_frame,
                                                ref_body_world,
                                                frame_to_world,
                                                align_delta,
                                                now_s,
                                                t_end,
                                                track_color_future,
                                                false,
                                                draw_ctx.line_overlay_boost);
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
                else if (use_persistent_gpu_roots && track->cache.gpu_roots_frame_planned &&
                         !track->cache.gpu_roots_frame_planned->empty())
                {
                    enqueue_cached_orbit_window(orbit_plot,
                                                track->cache.gpu_roots_frame_planned,
                                                ref_body_world,
                                                frame_to_world,
                                                align_delta,
                                                planned_pick_window.t0_s,
                                                planned_pick_window.t1_s,
                                                track_color_plan,
                                                _prediction_draw_config.draw_planned_as_dashed,
                                                draw_ctx.line_overlay_boost);
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

                const std::size_t pick_max_segments =
                        static_cast<std::size_t>(std::max(1, _orbit_plot_pick_max_segments));
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
                        const std::size_t base_pick_budget = remaining_pick_budget - planned_reserve;
                        bool rebuilt_pick_cache = false;
                        if (base_pick_budget > 0 &&
                            should_rebuild_pick_cache(track->pick_cache,
                                                      track->cache.generation_id,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      base_pick_window.t0_s,
                                                      base_pick_window.t1_s,
                                                      base_pick_budget,
                                                      false))
                        {
                            rebuilt_pick_cache = true;
                            bool cap_hit = false;
                            emitted = Draw::build_pick_segment_cache(*traj_base_segments,
                                                                     ref_body_world,
                                                                     frame_to_world,
                                                                     align_delta,
                                                                     base_pick_window.t0_s,
                                                                     base_pick_window.t1_s,
                                                                     base_pick_budget,
                                                                     track->pick_cache.base_segments,
                                                                     cap_hit,
                                                                     _orbit_plot_perf);
                            if (emitted > 0)
                            {
                                mark_pick_cache_valid(track->pick_cache,
                                                      track->cache.generation_id,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      base_pick_window.t0_s,
                                                      base_pick_window.t1_s,
                                                      base_pick_budget,
                                                      false);
                            }
                            else
                            {
                                track->pick_cache.base_valid = false;
                                track->pick_cache.base_segments.clear();
                            }
                        }

                        if (base_pick_budget > 0 &&
                            track->pick_cache.base_valid &&
                            !track->pick_cache.base_segments.empty())
                        {
                            emitted = track->pick_cache.base_segments.size();
                            if (!rebuilt_pick_cache)
                            {
                                _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(emitted);
                                _orbit_plot_perf.pick_segments += static_cast<uint32_t>(emitted);
                            }
                            picking->add_line_pick_segments(
                                    pick_group_base,
                                    std::span<const PickingSystem::LinePickSegmentData>(
                                            track->pick_cache.base_segments.data(),
                                            track->pick_cache.base_segments.size()));
                        }
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
                        bool rebuilt_pick_cache = false;
                        if (should_rebuild_pick_cache(track->pick_cache,
                                                      track->cache.generation_id,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      planned_pick_window.t0_s,
                                                      planned_pick_window.t1_s,
                                                      remaining_pick_budget,
                                                      true))
                        {
                            rebuilt_pick_cache = true;
                            bool cap_hit = false;
                            Draw::build_pick_segment_cache(*traj_planned_segments,
                                                           ref_body_world,
                                                           frame_to_world,
                                                           align_delta,
                                                           planned_pick_window.t0_s,
                                                           planned_pick_window.t1_s,
                                                           remaining_pick_budget,
                                                           track->pick_cache.planned_segments,
                                                           cap_hit,
                                                           _orbit_plot_perf);
                            if (!track->pick_cache.planned_segments.empty())
                            {
                                mark_pick_cache_valid(track->pick_cache,
                                                      track->cache.generation_id,
                                                      ref_body_world,
                                                      frame_to_world,
                                                      align_delta,
                                                      planned_pick_window.t0_s,
                                                      planned_pick_window.t1_s,
                                                      remaining_pick_budget,
                                                      true);
                            }
                            else
                            {
                                track->pick_cache.planned_valid = false;
                                track->pick_cache.planned_segments.clear();
                            }
                        }

                        if (track->pick_cache.planned_valid && !track->pick_cache.planned_segments.empty())
                        {
                            if (!rebuilt_pick_cache)
                            {
                                _orbit_plot_perf.pick_segments_before_cull +=
                                        static_cast<uint32_t>(track->pick_cache.planned_segments.size());
                                _orbit_plot_perf.pick_segments +=
                                        static_cast<uint32_t>(track->pick_cache.planned_segments.size());
                            }
                            picking->add_line_pick_segments(
                                    pick_group_planned,
                                    std::span<const PickingSystem::LinePickSegmentData>(
                                            track->pick_cache.planned_segments.data(),
                                            track->pick_cache.planned_segments.size()));
                        }
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
