#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    void GameplayState::draw_orbit_prediction_track_windows(Draw::PredictionTrackDrawContext &track_ctx)
    {
        OrbitPredictionCache &stable_cache = *track_ctx.stable_cache;
        OrbitPredictionCache &planned_cache = *track_ctx.planned_cache;
        const auto draw_raw_base_window = [&](const double split_t0_s,
                                              const double split_t1_s,
                                              const glm::vec4 &color) {
            if (!(split_t1_s > split_t0_s))
            {
                return;
            }
            Draw::draw_orbit_window(track_ctx.identity_frame_transform ? track_ctx.draw_ctx : track_ctx.world_basis_draw_ctx,
                                    _prediction_draw_config,
                                    _orbit_plot_perf,
                                    track_ctx.identity_frame_transform ? *track_ctx.traj_base_segments
                                                                       : Draw::base_segments_world_basis(track_ctx),
                                    split_t0_s,
                                    split_t1_s,
                                    color,
                                    false);
        };

        const auto draw_cpu_base_window = [&](const double window_t0_s,
                                              const double window_t1_s,
                                              const glm::vec4 &color) {
            if (!(window_t1_s > window_t0_s))
            {
                return;
            }
            const auto draw_cpu_base_window_once = [&](const double split_t0_s, const double split_t1_s) {
                if (!(split_t1_s > split_t0_s))
                {
                    return;
                }
                if (!track_ctx.use_base_adaptive_curve)
                {
                    draw_raw_base_window(split_t0_s, split_t1_s, color);
                    return;
                }
                std::size_t anchor_hi = track_ctx.i_hi;
                if (anchor_hi >= track_ctx.traj_base->size())
                {
                    anchor_hi = track_ctx.traj_base->size() - 1;
                }
                std::size_t anchor_lo = (anchor_hi > 0) ? (anchor_hi - 1) : 0;
                if (anchor_hi == anchor_lo && (anchor_hi + 1) < track_ctx.traj_base->size())
                {
                    anchor_hi = anchor_lo + 1;
                }
                const double anchor_source_t0_s = (*track_ctx.traj_base)[anchor_lo].t_s;
                const double anchor_source_t1_s = (*track_ctx.traj_base)[anchor_hi].t_s;
                const double anchor_t0_s = std::max(split_t0_s, anchor_source_t0_s);
                const double anchor_t1_s = std::min(split_t1_s, anchor_source_t1_s);
                const bool anchor_window_valid =
                        track_ctx.is_active &&
                        std::isfinite(anchor_source_t0_s) &&
                        std::isfinite(anchor_source_t1_s) &&
                        (anchor_t1_s > anchor_t0_s);

                if (!anchor_window_valid)
                {
                    Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                     _prediction_draw_config,
                                                     _orbit_plot_perf,
                                                     stable_cache.render_curve_frame,
                                                     split_t0_s,
                                                     split_t1_s,
                                                     color,
                                                     false);
                    return;
                }

                if (anchor_t0_s > split_t0_s)
                {
                    Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                     _prediction_draw_config,
                                                     _orbit_plot_perf,
                                                     stable_cache.render_curve_frame,
                                                     split_t0_s,
                                                     anchor_t0_s,
                                                     color,
                                                     false);
                }

                draw_raw_base_window(anchor_t0_s, anchor_t1_s, color);

                if (split_t1_s > anchor_t1_s)
                {
                    Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                     _prediction_draw_config,
                                                     _orbit_plot_perf,
                                                     stable_cache.render_curve_frame,
                                                     anchor_t1_s,
                                                     split_t1_s,
                                                     color,
                                                     false);
                }
            };

            if (track_ctx.now_s > window_t0_s && track_ctx.now_s < window_t1_s)
            {
                draw_cpu_base_window_once(window_t0_s, track_ctx.now_s);
                draw_cpu_base_window_once(track_ctx.now_s, window_t1_s);
                return;
            }
            draw_cpu_base_window_once(window_t0_s, window_t1_s);
        };

        const auto draw_planned_window_from_cache = [&](OrbitPredictionCache &cache,
                                                        const double window_t0_s,
                                                        const double window_t1_s,
                                                        const glm::vec4 &color) {
            if (!(window_t1_s > window_t0_s))
            {
                return;
            }
            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           _prediction_draw_config,
                                           cache.trajectory_frame_planned,
                                           window_t0_s,
                                           window_t1_s,
                                           color,
                                           _prediction_draw_config.draw_planned_as_dashed);
                return;
            }

            if (!cache.render_curve_frame_planned.empty())
            {
                Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                 _prediction_draw_config,
                                                 _orbit_plot_perf,
                                                 cache.render_curve_frame_planned,
                                                 window_t0_s,
                                                 window_t1_s,
                                                 color,
                                                 _prediction_draw_config.draw_planned_as_dashed);
                return;
            }

            Draw::draw_orbit_window(track_ctx.identity_frame_transform ? track_ctx.draw_ctx : track_ctx.world_basis_draw_ctx,
                                    _prediction_draw_config,
                                    _orbit_plot_perf,
                                    track_ctx.identity_frame_transform
                                            ? cache.trajectory_segments_frame_planned
                                            : Draw::planned_segments_world_basis(track_ctx, cache),
                                    window_t0_s,
                                    window_t1_s,
                                    color,
                                    _prediction_draw_config.draw_planned_as_dashed);
        };

        const double planned_draw_future_window_s = track_ctx.planned_visual_window_s;

        track_ctx.base_pick_window = {};
        track_ctx.planned_draw_window =
                (track_ctx.active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty() &&
                 track_ctx.planned_window_segments && !track_ctx.planned_window_segments->empty())
                        ? Draw::build_planned_pick_window(*track_ctx.planned_window_segments,
                                                          _prediction_draw_config,
                                                          _maneuver_state.nodes,
                                                          track_ctx.now_s,
                                                          planned_draw_future_window_s,
                                                          _prediction_draw_future_segment,
                                                          _prediction_draw_full_orbit,
                                                          stable_cache.orbital_period_s)
                        : Draw::PickWindow{};
        track_ctx.planned_pick_window =
                (track_ctx.active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty() &&
                 track_ctx.planned_window_segments && !track_ctx.planned_window_segments->empty())
                        ? Draw::build_planned_pick_window(*track_ctx.planned_window_segments,
                                                          _prediction_draw_config,
                                                          _maneuver_state.nodes,
                                                          track_ctx.now_s,
                                                          track_ctx.planned_pick_window_s,
                                                          _prediction_draw_future_segment,
                                                          _prediction_draw_full_orbit,
                                                          stable_cache.orbital_period_s)
                        : Draw::PickWindow{};

        if (_prediction_draw_full_orbit)
        {
            double t_full_end = track_ctx.t1_s;
            if (stable_cache.orbital_period_s > 0.0 && std::isfinite(stable_cache.orbital_period_s))
            {
                t_full_end = std::min(track_ctx.t0_s +
                                              (stable_cache.orbital_period_s *
                                               OrbitPredictionTuning::kFullOrbitDrawPeriodScale),
                                      track_ctx.t1_s);
            }

            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           _prediction_draw_config,
                                           *track_ctx.traj_base,
                                           track_ctx.t0_s,
                                           t_full_end,
                                           track_ctx.track_color_full,
                                           false);
            }
            else
            {
                draw_cpu_base_window(track_ctx.t0_s, t_full_end, track_ctx.track_color_full);
            }

            if (track_ctx.is_active && !_prediction_draw_future_segment && t_full_end > track_ctx.t0_s)
            {
                track_ctx.base_pick_window.valid = true;
                track_ctx.base_pick_window.t0_s = track_ctx.t0_s;
                track_ctx.base_pick_window.t1_s = t_full_end;
            }
        }

        if (_prediction_draw_future_segment)
        {
            const double t_end =
                    (track_ctx.future_window_s > 0.0) ? std::min(track_ctx.now_s + track_ctx.future_window_s, track_ctx.t1_s)
                                                      : track_ctx.t1_s;

            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           _prediction_draw_config,
                                           *track_ctx.traj_base,
                                           track_ctx.now_s,
                                           t_end,
                                           track_ctx.track_color_future,
                                           false);
            }
            else
            {
                draw_cpu_base_window(track_ctx.now_s, t_end, track_ctx.track_color_future);
            }

            if (track_ctx.is_active && t_end > track_ctx.now_s)
            {
                track_ctx.base_pick_window.valid = true;
                track_ctx.base_pick_window.t0_s = track_ctx.now_s;
                track_ctx.base_pick_window.t1_s = t_end;
            }
        }

        if (track_ctx.active_player_track)
        {
            _orbit_plot_perf.planned_window_valid = track_ctx.planned_draw_window.valid;
            _orbit_plot_perf.planned_window_now_s = track_ctx.now_s;
            _orbit_plot_perf.planned_window_anchor_s = track_ctx.planned_draw_window.anchor_time_s;
            _orbit_plot_perf.planned_window_t_start = track_ctx.planned_draw_window.t0_s;
            _orbit_plot_perf.planned_window_t_end = track_ctx.planned_draw_window.t1_s;
            _orbit_plot_perf.planned_window_t0p =
                    (track_ctx.traj_planned_segments && !track_ctx.traj_planned_segments->empty())
                            ? track_ctx.traj_planned_segments->front().t0_s
                            : 0.0;
            _orbit_plot_perf.planned_chunk_count = 0;
            _orbit_plot_perf.planned_chunks_drawn = 0;
            _orbit_plot_perf.planned_chunk_enqueue_ms_last = 0.0;
            _orbit_plot_perf.planned_fallback_range_count = 0;
            _orbit_plot_perf.planned_fallback_draw_ms_last = 0.0;
        }

        if (!track_ctx.planned_draw_window.valid)
        {
            return;
        }

        draw_planned_window_from_cache(planned_cache,
                                       track_ctx.planned_draw_window.t0_s,
                                       track_ctx.planned_draw_window.t1_s,
                                       track_ctx.track_color_plan);
    }
} // namespace Game
