#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

        const auto planned_cache_drawable = [&](const OrbitPredictionCache &cache) {
            const bool primary_cache_enabled =
                    &cache == track_ctx.planned_cache &&
                    track_ctx.planned_cache_drawable;
            const bool stale_cache_enabled =
                    &cache == track_ctx.stale_planned_cache &&
                    track_ctx.stale_planned_cache_drawable;
            if (!primary_cache_enabled && !stale_cache_enabled)
            {
                return false;
            }

            if (track_ctx.direct_world_polyline)
            {
                return cache.trajectory_frame_planned.size() >= 2;
            }

            if (track_ctx.use_planned_adaptive_curve)
            {
                return !cache.render_curve_frame_planned.empty();
            }

            return !cache.trajectory_segments_frame_planned.empty();
        };

        const auto draw_planned_window_from_cache = [&](OrbitPredictionCache &cache,
                                                        const double window_t0_s,
                                                        const double window_t1_s,
                                                         const glm::vec4 &color,
                                                         const bool force_solid = false,
                                                         const bool ignore_prefix_clip = false) {
            double clipped_window_t1_s = window_t1_s;
            const bool drawing_stale_cache =
                    &cache == track_ctx.stale_planned_cache &&
                    track_ctx.stale_planned_cache_drawable;
            if (drawing_stale_cache)
            {
                if (!std::isfinite(track_ctx.stale_planned_cache_prefix_cutoff_s))
                {
                    return;
                }
                clipped_window_t1_s = std::min(clipped_window_t1_s,
                                               track_ctx.stale_planned_cache_prefix_cutoff_s);
            }
            else if (track_ctx.planned_cache_prefix_only && !ignore_prefix_clip)
            {
                if (!std::isfinite(track_ctx.planned_cache_prefix_cutoff_s))
                {
                    return;
                }
                clipped_window_t1_s = std::min(clipped_window_t1_s, track_ctx.planned_cache_prefix_cutoff_s);
            }
            if (!(clipped_window_t1_s > window_t0_s) || !planned_cache_drawable(cache))
            {
                return;
            }
            const bool dashed = _prediction_draw_config.draw_planned_as_dashed && !force_solid;
            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           _prediction_draw_config,
                                           cache.trajectory_frame_planned,
                                           window_t0_s,
                                           clipped_window_t1_s,
                                           color,
                                           dashed);
                return;
            }

            if (track_ctx.use_planned_adaptive_curve)
            {
                Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                 _prediction_draw_config,
                                                 _orbit_plot_perf,
                                                 cache.render_curve_frame_planned,
                                                 window_t0_s,
                                                 clipped_window_t1_s,
                                                 color,
                                                 dashed);
                return;
            }

            Draw::draw_orbit_window(track_ctx.identity_frame_transform ? track_ctx.draw_ctx : track_ctx.world_basis_draw_ctx,
                                    _prediction_draw_config,
                                    _orbit_plot_perf,
                                    track_ctx.identity_frame_transform
                                            ? cache.trajectory_segments_frame_planned
                                            : Draw::planned_segments_world_basis(track_ctx, cache),
                                    window_t0_s,
                                    clipped_window_t1_s,
                                    color,
                                    dashed);
        };

        const auto chunk_drawable = [&](const OrbitChunk &chunk) {
            if (track_ctx.direct_world_polyline)
            {
                return chunk.frame_samples.size() >= 2 ||
                       !chunk.render_curve.empty() ||
                       !chunk.frame_segments.empty();
            }

            return !chunk.render_curve.empty() || !chunk.frame_segments.empty();
        };

        const auto draw_planned_window_from_chunk = [&](const OrbitChunk &chunk,
                                                        const double window_t0_s,
                                                        const double window_t1_s,
                                                        const glm::vec4 &color,
                                                        const bool force_solid = false) {
            if (!(window_t1_s > window_t0_s) || !chunk_drawable(chunk))
            {
                return false;
            }
            const bool dashed = _prediction_draw_config.draw_planned_as_dashed && !force_solid;

            if (track_ctx.direct_world_polyline && chunk.frame_samples.size() >= 2)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           _prediction_draw_config,
                                           chunk.frame_samples,
                                           window_t0_s,
                                           window_t1_s,
                                           color,
                                           dashed);
                return true;
            }

            if (!chunk.render_curve.empty())
            {
                Draw::draw_adaptive_curve_window(track_ctx.draw_ctx,
                                                 _prediction_draw_config,
                                                 _orbit_plot_perf,
                                                 chunk.render_curve,
                                                 window_t0_s,
                                                 window_t1_s,
                                                 color,
                                                 dashed);
                return true;
            }

            if (chunk.frame_segments.empty())
            {
                return false;
            }

            Draw::draw_orbit_window(track_ctx.draw_ctx,
                                    _prediction_draw_config,
                                    _orbit_plot_perf,
                                    chunk.frame_segments,
                                    window_t0_s,
                                    window_t1_s,
                                    color,
                                    dashed);
            return true;
        };

        const auto prefix_fallback_cache = [&]() -> OrbitPredictionCache * {
            if (track_ctx.stale_planned_cache && track_ctx.stale_planned_cache_drawable)
            {
                return track_ctx.stale_planned_cache;
            }
            if (planned_cache_drawable(planned_cache))
            {
                return &planned_cache;
            }
            return nullptr;
        };

        const auto clamp_prefix_cutoff_for_cache = [&](OrbitPredictionCache *cache,
                                                       const double requested_cutoff_s) {
            double cutoff_s = requested_cutoff_s;
            if (cache == track_ctx.stale_planned_cache &&
                std::isfinite(track_ctx.stale_planned_cache_prefix_cutoff_s))
            {
                cutoff_s = std::isfinite(cutoff_s)
                                   ? std::min(cutoff_s, track_ctx.stale_planned_cache_prefix_cutoff_s)
                                   : track_ctx.stale_planned_cache_prefix_cutoff_s;
            }
            return cutoff_s;
        };

        track_ctx.base_pick_window = {};
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(*track_ctx.track);
        const bool active_maneuver_track =
                track_ctx.active_player_track &&
                track_ctx.track->supports_maneuvers &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty();
        const bool preview_lifecycle_fallback_active =
                PredictionRuntimeDetail::prediction_track_preview_fallback_active(lifecycle);
        const bool preview_window_available =
                active_maneuver_track &&
                track_ctx.track->preview_anchor.valid &&
                preview_lifecycle_fallback_active &&
                std::isfinite(track_ctx.track->preview_anchor.anchor_time_s) &&
                track_ctx.track->preview_anchor.visual_window_s > 0.0;
        const auto preview_anchor_window = [&]() {
            Draw::PickWindow window{};
            if (!preview_window_available)
            {
                return window;
            }

            window.valid = true;
            window.t0_s = track_ctx.track->preview_anchor.anchor_time_s;
            window.t1_s = window.t0_s + track_ctx.track->preview_anchor.visual_window_s;
            window.anchor_time_s = track_ctx.track->preview_anchor.anchor_time_s;
            return window;
        };
        const auto preview_anchor_pick_window = [&]() {
            Draw::PickWindow window = preview_anchor_window();
            if (window.valid && track_ctx.track->preview_anchor.exact_window_s > 0.0)
            {
                window.t1_s = window.t0_s + track_ctx.track->preview_anchor.exact_window_s;
            }
            return window;
        };
        track_ctx.planned_draw_window =
                (active_maneuver_track &&
                 track_ctx.planned_window_segments && !track_ctx.planned_window_segments->empty())
                        ? Draw::build_planned_draw_window(*track_ctx.planned_window_segments,
                                                          _prediction_draw_config,
                                                          track_ctx.planned_window_policy)
                        : preview_anchor_window();
        track_ctx.planned_pick_window =
                (active_maneuver_track &&
                 track_ctx.planned_window_segments && !track_ctx.planned_window_segments->empty())
                        ? Draw::build_planned_pick_window(*track_ctx.planned_window_segments,
                                                          _prediction_draw_config,
                                                          track_ctx.planned_window_policy)
                        : preview_anchor_pick_window();

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
            const double t_full_start =
                    std::isfinite(track_ctx.now_s) ? std::clamp(track_ctx.now_s, track_ctx.t0_s, t_full_end)
                                                   : track_ctx.t0_s;

            if (track_ctx.direct_world_polyline)
            {
                Draw::draw_polyline_window(track_ctx.draw_ctx,
                                           _prediction_draw_config,
                                           *track_ctx.traj_base,
                                           t_full_start,
                                           t_full_end,
                                           track_ctx.track_color_full,
                                           false);
            }
            else
            {
                draw_cpu_base_window(t_full_start, t_full_end, track_ctx.track_color_full);
            }

            if (track_ctx.is_active && !_prediction_draw_future_segment && t_full_end > t_full_start)
            {
                track_ctx.base_pick_window.valid = true;
                track_ctx.base_pick_window.t0_s = t_full_start;
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

        const double planned_window_t0_s = track_ctx.planned_draw_window.t0_s;
        const double planned_window_t1_s = track_ctx.planned_draw_window.t1_s;
        const glm::vec4 preview_plan_color = glm::vec4(track_ctx.track_color_plan.r,
                                                       track_ctx.track_color_plan.g,
                                                       track_ctx.track_color_plan.b,
                                                       std::clamp(std::max(track_ctx.track_color_plan.a, 0.98f), 0.0f, 1.0f));
        const bool preview_overlay_draw_active =
                PredictionRuntimeDetail::prediction_track_preview_overlay_draw_active(
                        lifecycle,
                        track_ctx.track->preview_anchor.valid);
        const PredictionChunkAssembly preview_assembly_snapshot =
                preview_overlay_draw_active ? track_ctx.track->preview_overlay.chunk_assembly
                                            : PredictionChunkAssembly{};
        const PredictionChunkAssembly &preview_assembly = preview_assembly_snapshot;
        const bool full_stream_overlay_draw_active =
                !active_maneuver_track &&
                !track_ctx.maneuver_drag_active &&
                !preview_lifecycle_fallback_active;
        const PredictionChunkAssembly full_stream_assembly_snapshot =
                (full_stream_overlay_draw_active &&
                 track_ctx.track->full_stream_overlay.ready_for_draw(planned_cache.generation_id,
                                                                     planned_cache.display_frame_key,
                                                                     planned_cache.display_frame_revision))
                        ? track_ctx.track->full_stream_overlay.chunk_assembly
                        : PredictionChunkAssembly{};
        const PredictionChunkAssembly *full_stream_assembly =
                full_stream_assembly_snapshot.valid && !full_stream_assembly_snapshot.chunks.empty()
                        ? &full_stream_assembly_snapshot
                        : nullptr;
        const auto draw_chunk_assembly_ranges =
                [&](const PredictionChunkAssembly &assembly,
                    const glm::vec4 &color,
                    const std::vector<std::pair<double, double>> *masked_ranges,
                    std::vector<std::pair<double, double>> &out_drawn_ranges) {
                    _orbit_plot_perf.planned_chunk_count += static_cast<uint32_t>(assembly.chunks.size());
                    for (const OrbitChunk &chunk : assembly.chunks)
                    {
                        const double clipped_t0_s = std::max(planned_window_t0_s, chunk.t0_s);
                        const double clipped_t1_s = std::min(planned_window_t1_s, chunk.t1_s);
                        if (!(clipped_t1_s > clipped_t0_s))
                        {
                            continue;
                        }

                        std::vector<std::pair<double, double>> draw_ranges;
                        if (masked_ranges && !masked_ranges->empty())
                        {
                            draw_ranges = Draw::compute_uncovered_ranges(clipped_t0_s, clipped_t1_s, *masked_ranges);
                        }
                        else
                        {
                            draw_ranges.emplace_back(clipped_t0_s, clipped_t1_s);
                        }

                        bool drew_chunk = false;
                        for (const auto &[range_t0_s, range_t1_s] : draw_ranges)
                        {
                            if (!draw_planned_window_from_chunk(chunk,
                                                                range_t0_s,
                                                                range_t1_s,
                                                                color,
                                                                track_ctx.maneuver_drag_active))
                            {
                                continue;
                            }

                            out_drawn_ranges.emplace_back(range_t0_s, range_t1_s);
                            drew_chunk = true;
                        }

                        if (drew_chunk)
                        {
                            ++_orbit_plot_perf.planned_chunks_drawn;
                        }
                    }
                };
        const auto draw_cache_fallback_ranges =
                [&](const std::vector<std::pair<double, double>> &covered_ranges,
                    const double fresh_cutoff_s) {
                    OrbitPredictionCache *fallback_cache = prefix_fallback_cache();
                    if (!fallback_cache || !planned_cache_drawable(*fallback_cache))
                    {
                        return;
                    }
                    const double prefix_cutoff_s =
                            clamp_prefix_cutoff_for_cache(fallback_cache, fresh_cutoff_s);

                    const std::vector<std::pair<double, double>> uncovered_ranges =
                            Draw::compute_uncovered_ranges(planned_window_t0_s,
                                                           planned_window_t1_s,
                                                           covered_ranges);
                    _orbit_plot_perf.planned_fallback_range_count =
                            static_cast<uint32_t>(uncovered_ranges.size());
                    for (const auto &[range_t0_s, range_t1_s] : uncovered_ranges)
                    {
                        const bool prefix_range =
                                std::isfinite(prefix_cutoff_s) &&
                                range_t1_s <= (prefix_cutoff_s + 1.0e-6);
                        if (!prefix_range)
                        {
                            continue;
                        }
                        draw_planned_window_from_cache(*fallback_cache,
                                                       range_t0_s,
                                                       range_t1_s,
                                                       track_ctx.track_color_plan);
                    }
                };
        const auto preview_tail_matches_planned_cache = [&]() {
            if (!preview_assembly.valid || preview_assembly.chunks.empty())
            {
                return false;
            }
            if (planned_cache.trajectory_segments_frame_planned.empty() &&
                planned_cache.trajectory_frame_planned.size() < 2)
            {
                return false;
            }

            constexpr std::size_t kSamplesPerChunk = 5u;
            constexpr std::size_t kMaxTotalSamples = 24u;
            constexpr double kMaxPointErrorPx = 1.75;
            constexpr double kMaxAverageErrorPx = 0.85;

            double total_error_px = 0.0;
            std::size_t sample_count = 0u;
            for (const OrbitChunk &chunk : preview_assembly.chunks)
            {
                const double clipped_t0_s = std::max(planned_window_t0_s, chunk.t0_s);
                const double clipped_t1_s = std::min(planned_window_t1_s, chunk.t1_s);
                if (!(clipped_t1_s > clipped_t0_s))
                {
                    continue;
                }

                const std::size_t samples_this_chunk =
                        std::min<std::size_t>(kSamplesPerChunk, kMaxTotalSamples - sample_count);
                if (samples_this_chunk == 0u)
                {
                    break;
                }

                for (std::size_t i = 0; i < samples_this_chunk; ++i)
                {
                    const double u =
                            (samples_this_chunk > 1u) ? static_cast<double>(i) / static_cast<double>(samples_this_chunk - 1u)
                                                      : 0.5;
                    const double sample_t_s = clipped_t0_s + ((clipped_t1_s - clipped_t0_s) * u);

                    WorldVec3 preview_world{0.0};
                    WorldVec3 planned_world{0.0};
                    if (!Draw::sample_prediction_path_world(track_ctx.draw_ctx,
                                                            chunk.frame_segments,
                                                            chunk.frame_samples,
                                                            sample_t_s,
                                                            preview_world) ||
                        !Draw::sample_prediction_path_world(track_ctx.draw_ctx,
                                                            planned_cache.trajectory_segments_frame_planned,
                                                            planned_cache.trajectory_frame_planned,
                                                            sample_t_s,
                                                            planned_world))
                    {
                        continue;
                    }

                    const double error_m = glm::length(glm::dvec3(preview_world - planned_world));
                    const WorldVec3 error_mid_world =
                            WorldVec3(glm::mix(glm::dvec3(preview_world), glm::dvec3(planned_world), 0.5));
                    const double meters_per_px =
                            std::max(1.0e-6, Draw::meters_per_px_at_world(track_ctx.draw_ctx, error_mid_world));
                    const double error_px = error_m / meters_per_px;
                    if (!std::isfinite(error_px) || error_px > kMaxPointErrorPx)
                    {
                        return false;
                    }

                    total_error_px += error_px;
                    ++sample_count;
                }
            }

            return sample_count > 0u &&
                   (total_error_px / static_cast<double>(sample_count)) <= kMaxAverageErrorPx;
        };
        const bool draw_matching_cached_tail = preview_tail_matches_planned_cache();
        if (!preview_assembly.valid || preview_assembly.chunks.empty())
        {
            const bool preview_fallback_active =
                    track_ctx.track->preview_anchor.valid &&
                    PredictionRuntimeDetail::prediction_track_preview_fallback_active(lifecycle) &&
                    std::isfinite(track_ctx.track->preview_anchor.anchor_time_s) &&
                    track_ctx.track->preview_anchor.visual_window_s > 0.0;
            if (preview_fallback_active)
            {
                const double preview_t0_s = std::clamp(track_ctx.track->preview_anchor.anchor_time_s,
                                                       planned_window_t0_s,
                                                       planned_window_t1_s);
                const double preview_t1_s =
                        std::min(planned_window_t1_s,
                                 preview_t0_s + track_ctx.track->preview_anchor.visual_window_s);
                if (preview_t0_s > planned_window_t0_s)
                {
                    if (OrbitPredictionCache *fallback_cache = prefix_fallback_cache())
                    {
                        draw_planned_window_from_cache(*fallback_cache,
                                                       planned_window_t0_s,
                                                       preview_t0_s,
                                                       track_ctx.track_color_plan);
                    }
                }
                if (preview_t1_s > preview_t0_s && planned_cache_drawable(planned_cache))
                {
                    draw_planned_window_from_cache(planned_cache,
                                                   preview_t0_s,
                                                   preview_t1_s,
                                                   preview_plan_color,
                                                   track_ctx.maneuver_drag_active,
                                                   true);
                }
                return;
            }
            if (full_stream_assembly)
            {
                std::vector<std::pair<double, double>> full_stream_covered_ranges;
                full_stream_covered_ranges.reserve(full_stream_assembly->chunks.size());
                draw_chunk_assembly_ranges(
                        *full_stream_assembly,
                        track_ctx.track_color_plan,
                        nullptr,
                        full_stream_covered_ranges);
                double first_full_stream_t0_s = std::numeric_limits<double>::infinity();
                for (const auto &[range_t0_s, range_t1_s] : full_stream_covered_ranges)
                {
                    (void) range_t1_s;
                    first_full_stream_t0_s = std::min(first_full_stream_t0_s, range_t0_s);
                }
                draw_cache_fallback_ranges(full_stream_covered_ranges, first_full_stream_t0_s);
                return;
            }
            if (track_ctx.maneuver_drag_active && std::isfinite(track_ctx.planned_draw_window.anchor_time_s))
            {
                if (OrbitPredictionCache *fallback_cache = prefix_fallback_cache())
                {
                    draw_planned_window_from_cache(*fallback_cache,
                                                   planned_window_t0_s,
                                                   std::min(planned_window_t1_s,
                                                            track_ctx.planned_draw_window.anchor_time_s),
                                                   track_ctx.track_color_plan);
                }
                return;
            }
            draw_planned_window_from_cache(planned_cache,
                                           planned_window_t0_s,
                                           planned_window_t1_s,
                                           track_ctx.track_color_plan);
            return;
        }

        std::vector<std::pair<double, double>> covered_ranges;
        covered_ranges.reserve(preview_assembly.chunks.size());
        double first_preview_t0_s = std::numeric_limits<double>::infinity();
        _orbit_plot_perf.planned_chunk_count += static_cast<uint32_t>(preview_assembly.chunks.size());
        for (const OrbitChunk &chunk : preview_assembly.chunks)
        {
            const double clipped_t0_s = std::max(planned_window_t0_s, chunk.t0_s);
            const double clipped_t1_s = std::min(planned_window_t1_s, chunk.t1_s);
            if (!(clipped_t1_s > clipped_t0_s))
            {
                continue;
            }

            if (!draw_planned_window_from_chunk(chunk,
                                                clipped_t0_s,
                                                clipped_t1_s,
                                                preview_plan_color,
                                                track_ctx.maneuver_drag_active))
            {
                continue;
            }

            covered_ranges.emplace_back(clipped_t0_s, clipped_t1_s);
            first_preview_t0_s = std::min(first_preview_t0_s, clipped_t0_s);
            ++_orbit_plot_perf.planned_chunks_drawn;
        }

        if (full_stream_assembly)
        {
            std::vector<std::pair<double, double>> full_stream_covered_ranges;
            full_stream_covered_ranges.reserve(full_stream_assembly->chunks.size());
            draw_chunk_assembly_ranges(
                    *full_stream_assembly,
                    track_ctx.track_color_plan,
                    &covered_ranges,
                    full_stream_covered_ranges);
            covered_ranges.insert(covered_ranges.end(),
                                  full_stream_covered_ranges.begin(),
                                  full_stream_covered_ranges.end());
            const double fresh_cutoff_s =
                    std::isfinite(first_preview_t0_s) ? first_preview_t0_s : std::numeric_limits<double>::infinity();
            draw_cache_fallback_ranges(covered_ranges, fresh_cutoff_s);
            return;
        }

        const std::vector<std::pair<double, double>> uncovered_ranges =
                Draw::compute_uncovered_ranges(planned_window_t0_s, planned_window_t1_s, covered_ranges);
        _orbit_plot_perf.planned_fallback_range_count = static_cast<uint32_t>(uncovered_ranges.size());
        const double drag_prefix_cutoff_s =
                track_ctx.maneuver_drag_active && std::isfinite(track_ctx.planned_draw_window.anchor_time_s)
                        ? track_ctx.planned_draw_window.anchor_time_s
                        : first_preview_t0_s;
        OrbitPredictionCache *fallback_cache = prefix_fallback_cache();
        const double fallback_prefix_cutoff_s =
                fallback_cache ? clamp_prefix_cutoff_for_cache(fallback_cache, drag_prefix_cutoff_s)
                               : std::numeric_limits<double>::quiet_NaN();
        for (const auto &[range_t0_s, range_t1_s] : uncovered_ranges)
        {
            const bool prefix_range =
                    std::isfinite(fallback_prefix_cutoff_s) &&
                    range_t1_s <= (fallback_prefix_cutoff_s + 1.0e-6);
            if (prefix_range)
            {
                if (fallback_cache)
                {
                    draw_planned_window_from_cache(*fallback_cache,
                                                   range_t0_s,
                                                   range_t1_s,
                                                   track_ctx.track_color_plan);
                }
                continue;
            }
            if (track_ctx.maneuver_drag_active || !draw_matching_cached_tail)
            {
                continue;
            }
            draw_planned_window_from_cache(planned_cache,
                                           range_t0_s,
                                           range_t1_s,
                                           preview_plan_color);
        }
    }
} // namespace Game
