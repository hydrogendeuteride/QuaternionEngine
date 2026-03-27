#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    void GameplayState::draw_orbit_prediction_track_windows(Draw::PredictionTrackDrawContext &track_ctx)
    {
        PredictionTrackState &track = *track_ctx.track;
        OrbitPredictionCache &stable_cache = *track_ctx.stable_cache;
        OrbitPredictionCache &planned_cache = *track_ctx.planned_cache;
        PredictionChunkAssembly *const planned_chunk_assembly = track_ctx.planned_chunk_assembly;
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

        const auto collect_planned_cache_covered_ranges = [&](const OrbitPredictionCache &cache,
                                                              const double window_t0_s,
                                                              const double window_t1_s) {
            std::vector<std::pair<double, double>> covered_ranges;
            if (!(window_t1_s > window_t0_s) || cache.trajectory_segments_frame_planned.empty())
            {
                return covered_ranges;
            }

            const double cache_t0_s = cache.trajectory_segments_frame_planned.front().t0_s;
            const orbitsim::TrajectorySegment &last_segment = cache.trajectory_segments_frame_planned.back();
            const double cache_t1_s = last_segment.t0_s + last_segment.dt_s;
            if (!std::isfinite(cache_t0_s) || !std::isfinite(cache_t1_s) || !(cache_t1_s > cache_t0_s))
            {
                return covered_ranges;
            }

            const double covered_t0_s = std::max(window_t0_s, cache_t0_s);
            const double covered_t1_s = std::min(window_t1_s, cache_t1_s);
            if (covered_t1_s > covered_t0_s)
            {
                covered_ranges.emplace_back(covered_t0_s, covered_t1_s);
            }
            return covered_ranges;
        };

        const auto build_planned_window_from_chunk_assembly = [&](const double now_s,
                                                                  const double future_window_s) {
            Draw::PickWindow planned_window{};
            if (!planned_chunk_assembly || !planned_chunk_assembly->valid || planned_chunk_assembly->chunks.empty() ||
                _maneuver_state.nodes.empty())
            {
                return planned_window;
            }

            const double t0p = planned_chunk_assembly->start_time_s();
            const double t1p = planned_chunk_assembly->end_time_s();
            if (!std::isfinite(t0p) || !std::isfinite(t1p) || !(t1p > t0p))
            {
                return planned_window;
            }

            double first_future_node_time_s = std::numeric_limits<double>::infinity();
            double first_relevant_node_time_s = std::numeric_limits<double>::infinity();
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s) ||
                    node.time_s < (t0p - _prediction_draw_config.node_time_tolerance_s) ||
                    node.time_s > (t1p + _prediction_draw_config.node_time_tolerance_s))
                {
                    continue;
                }
                first_relevant_node_time_s = std::min(first_relevant_node_time_s, node.time_s);
                if (node.time_s >= (now_s + _prediction_draw_config.node_time_tolerance_s))
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

            double t_plan_start = std::isfinite(first_future_node_time_s) ? std::clamp(anchor_time_s, t0p, t1p)
                                                                          : std::clamp(now_s, t0p, t1p);
            double t_plan_end = t_plan_start;
            if (_prediction_draw_future_segment && future_window_s > 0.0)
            {
                t_plan_end = std::min(t_plan_start + future_window_s, t1p);
            }
            else if (_prediction_draw_full_orbit)
            {
                double t_full_end = t1p;
                if (stable_cache.orbital_period_s > 0.0 && std::isfinite(stable_cache.orbital_period_s))
                {
                    t_full_end = std::min(
                            t_plan_start +
                                    (stable_cache.orbital_period_s *
                                     OrbitPredictionTuning::kFullOrbitDrawPeriodScale),
                            t1p);
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
        };

        const double planned_draw_future_window_s =
                track_ctx.maneuver_drag_active
                        ? std::min(track_ctx.planned_visual_window_s, track_ctx.planned_exact_window_s)
                        : track_ctx.planned_visual_window_s;

        track_ctx.base_pick_window = {};
        track_ctx.planned_draw_window =
                (track_ctx.active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
                        ? ((track_ctx.planned_window_segments && !track_ctx.planned_window_segments->empty())
                                   ? Draw::build_planned_pick_window(*track_ctx.planned_window_segments,
                                                                     _prediction_draw_config,
                                                                     _maneuver_state.nodes,
                                                                     track_ctx.now_s,
                                                                     planned_draw_future_window_s,
                                                                     _prediction_draw_future_segment,
                                                                     _prediction_draw_full_orbit,
                                                                     stable_cache.orbital_period_s)
                                   : (track_ctx.has_preview_planned_overlay && track_ctx.has_chunk_planned_overlay
                                                      ? build_planned_window_from_chunk_assembly(
                                                                track_ctx.now_s,
                                                                planned_draw_future_window_s)
                                                      : Draw::PickWindow{}))
                        : Draw::PickWindow{};
        track_ctx.planned_pick_window =
                (track_ctx.active_player_track && _maneuver_nodes_enabled && !_maneuver_state.nodes.empty())
                        ? ((track_ctx.planned_window_segments && !track_ctx.planned_window_segments->empty())
                                   ? Draw::build_planned_pick_window(*track_ctx.planned_window_segments,
                                                                     _prediction_draw_config,
                                                                     _maneuver_state.nodes,
                                                                     track_ctx.now_s,
                                                                     track_ctx.planned_pick_window_s,
                                                                     _prediction_draw_future_segment,
                                                                     _prediction_draw_full_orbit,
                                                                     stable_cache.orbital_period_s)
                                   : (track_ctx.has_preview_planned_overlay && track_ctx.has_chunk_planned_overlay
                                                      ? build_planned_window_from_chunk_assembly(
                                                                track_ctx.now_s,
                                                                track_ctx.planned_pick_window_s)
                                                      : Draw::PickWindow{}))
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
            if (track_ctx.has_chunk_planned_overlay &&
                planned_chunk_assembly &&
                !planned_chunk_assembly->chunks.empty())
            {
                _orbit_plot_perf.planned_window_t0p = planned_chunk_assembly->start_time_s();
            }
            else if (track_ctx.traj_planned_segments && !track_ctx.traj_planned_segments->empty())
            {
                _orbit_plot_perf.planned_window_t0p = track_ctx.traj_planned_segments->front().t0_s;
            }
        }

        const auto compute_drag_prefix_fallback_ranges = [&](const double window_t0_s,
                                                             const double window_t1_s,
                                                             std::vector<std::pair<double, double>> covered_ranges = {}) {
            std::vector<std::pair<double, double>> fallback_ranges;
            if (!track_ctx.drag_anchor_valid || !(window_t1_s > window_t0_s))
            {
                return fallback_ranges;
            }

            const double prefix_t1_s = std::min(window_t1_s, track_ctx.drag_anchor_time_s);
            if (!(prefix_t1_s > window_t0_s))
            {
                return fallback_ranges;
            }

            return Draw::compute_uncovered_ranges(window_t0_s, prefix_t1_s, std::move(covered_ranges));
        };

        if (!track_ctx.planned_draw_window.valid)
        {
            return;
        }

        if (track_ctx.suppress_stale_planned_preview)
        {
            if (track_ctx.active_player_track)
            {
                _orbit_plot_perf.planned_chunk_count = 0;
            }

            OrbitPredictionCache *const stable_planned_prefix_cache =
                    (!stable_cache.trajectory_segments_frame_planned.empty() ||
                     !stable_cache.trajectory_frame_planned.empty())
                            ? &stable_cache
                            : nullptr;
            const auto fallback_ranges =
                    (stable_planned_prefix_cache != nullptr)
                            ? compute_drag_prefix_fallback_ranges(track_ctx.planned_draw_window.t0_s,
                                                                  track_ctx.planned_draw_window.t1_s)
                            : std::vector<std::pair<double, double>>{};
            if (track_ctx.active_player_track)
            {
                _orbit_plot_perf.planned_fallback_range_count = static_cast<uint32_t>(fallback_ranges.size());
            }
            if (!fallback_ranges.empty())
            {
                const auto fallback_draw_start_tp = std::chrono::steady_clock::now();
                for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                {
                    draw_planned_window_from_cache(*stable_planned_prefix_cache,
                                                   fallback_t0_s,
                                                   fallback_t1_s,
                                                   track_ctx.track_color_plan);
                }
                if (track_ctx.active_player_track)
                {
                    _orbit_plot_perf.planned_fallback_draw_ms_last =
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fallback_draw_start_tp)
                                    .count();
                }
            }
            return;
        }

        if (track_ctx.active_player_track)
        {
            _orbit_plot_perf.planned_chunk_count =
                    track_ctx.has_chunk_planned_overlay ? static_cast<uint32_t>(planned_chunk_assembly->chunks.size()) : 0;
        }

        OrbitPredictionCache *const fallback_planned_cache =
                track_ctx.has_chunk_planned_overlay
                        ? ((!stable_cache.trajectory_segments_frame_planned.empty() ||
                            !stable_cache.trajectory_frame_planned.empty())
                                   ? &stable_cache
                                   : nullptr)
                        : ((!stable_cache.trajectory_segments_frame_planned.empty() ||
                            !stable_cache.trajectory_frame_planned.empty())
                                   ? &stable_cache
                                   : &planned_cache);

        if (track_ctx.has_chunk_planned_overlay)
        {
            const auto chunk_draw_start_tp = std::chrono::steady_clock::now();
            const Draw::ChunkAssemblyDrawResult chunk_draw_result =
                    Draw::draw_chunk_assembly_planned(track_ctx.draw_ctx,
                                                      _prediction_draw_config,
                                                      _orbit_plot_perf,
                                                      *planned_chunk_assembly,
                                                      track_ctx.planned_draw_window.t0_s,
                                                      track_ctx.planned_draw_window.t1_s,
                                                      track_ctx.track_color_plan,
                                                      _prediction_draw_config.draw_planned_as_dashed);
            const double chunk_draw_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - chunk_draw_start_tp)
                            .count();

            if (track_ctx.active_player_track)
            {
                _orbit_plot_perf.planned_chunks_drawn = chunk_draw_result.chunks_drawn;
                _orbit_plot_perf.planned_chunk_enqueue_ms_last = chunk_draw_ms;
            }

            const auto fallback_ranges =
                    track_ctx.maneuver_drag_active
                            ? compute_drag_prefix_fallback_ranges(track_ctx.planned_draw_window.t0_s,
                                                                  track_ctx.planned_draw_window.t1_s,
                                                                  chunk_draw_result.covered_ranges)
                            : Draw::compute_uncovered_ranges(track_ctx.planned_draw_window.t0_s,
                                                             track_ctx.planned_draw_window.t1_s,
                                                             chunk_draw_result.covered_ranges);
            if (track_ctx.active_player_track)
            {
                _orbit_plot_perf.planned_fallback_range_count = static_cast<uint32_t>(fallback_ranges.size());
            }
            if (!fallback_ranges.empty() && fallback_planned_cache)
            {
                const auto fallback_draw_start_tp = std::chrono::steady_clock::now();
                for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                {
                    draw_planned_window_from_cache(*fallback_planned_cache,
                                                   fallback_t0_s,
                                                   fallback_t1_s,
                                                   track_ctx.track_color_plan);
                }
                if (track_ctx.active_player_track)
                {
                    _orbit_plot_perf.planned_fallback_draw_ms_last =
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fallback_draw_start_tp)
                                    .count();
                }
            }
            return;
        }

        if (!track_ctx.has_preview_planned_overlay)
        {
            draw_planned_window_from_cache(planned_cache,
                                           track_ctx.planned_draw_window.t0_s,
                                           track_ctx.planned_draw_window.t1_s,
                                           track_ctx.track_color_plan);
            return;
        }

        draw_planned_window_from_cache(planned_cache,
                                       track_ctx.planned_draw_window.t0_s,
                                       track_ctx.planned_draw_window.t1_s,
                                       track_ctx.track_color_plan);

        const auto covered_ranges = collect_planned_cache_covered_ranges(planned_cache,
                                                                         track_ctx.planned_draw_window.t0_s,
                                                                         track_ctx.planned_draw_window.t1_s);
        const auto fallback_ranges =
                (fallback_planned_cache && fallback_planned_cache != &planned_cache)
                        ? (track_ctx.maneuver_drag_active
                                   ? compute_drag_prefix_fallback_ranges(track_ctx.planned_draw_window.t0_s,
                                                                         track_ctx.planned_draw_window.t1_s,
                                                                         covered_ranges)
                                   : Draw::compute_uncovered_ranges(track_ctx.planned_draw_window.t0_s,
                                                                    track_ctx.planned_draw_window.t1_s,
                                                                    covered_ranges))
                        : std::vector<std::pair<double, double>>{};
        if (track_ctx.active_player_track)
        {
            _orbit_plot_perf.planned_fallback_range_count = static_cast<uint32_t>(fallback_ranges.size());
        }

        if (!fallback_ranges.empty() && fallback_planned_cache)
        {
            const auto fallback_draw_start_tp = std::chrono::steady_clock::now();
            for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
            {
                draw_planned_window_from_cache(*fallback_planned_cache,
                                               fallback_t0_s,
                                               fallback_t1_s,
                                               track_ctx.track_color_plan);
            }
            if (track_ctx.active_player_track)
            {
                _orbit_plot_perf.planned_fallback_draw_ms_last =
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fallback_draw_start_tp)
                                .count();
            }
        }
    }
} // namespace Game
