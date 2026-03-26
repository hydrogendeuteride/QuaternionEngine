#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <span>

namespace Game
{
    namespace Draw = PredictionDrawDetail;

    void GameplayState::emit_orbit_prediction_track_picks(
            const Draw::PredictionGlobalDrawContext &global_ctx,
            Draw::PredictionTrackDrawContext &track_ctx)
    {
        if (!global_ctx.picking || !track_ctx.active_player_track)
        {
            return;
        }

        PredictionTrackState &track = *track_ctx.track;
        OrbitPredictionCache &stable_cache = *track_ctx.stable_cache;
        OrbitPredictionCache &planned_cache = *track_ctx.planned_cache;
        PredictionChunkAssembly *const planned_chunk_assembly = track_ctx.planned_chunk_assembly;

        const bool allow_base_pick = _maneuver_state.nodes.empty();
        const bool allow_planned_pick = !_maneuver_state.nodes.empty();
        const uint32_t pick_group_base = global_ctx.picking->add_line_pick_group("OrbitPlot/Base");
        const uint32_t pick_group_planned = global_ctx.picking->add_line_pick_group("OrbitPlot/Planned");

        const std::size_t pick_max_segments =
                static_cast<std::size_t>(std::max(1, _orbit_plot_budget.pick_max_segments));
        const double pick_frustum_margin_ratio = std::max(0.0, _orbit_plot_budget.pick_frustum_margin_ratio);
        OrbitRenderCurve::PickSettings pick_settings{};
        pick_settings.frustum_margin_ratio = pick_frustum_margin_ratio;

        std::vector<double> pick_anchor_times = Draw::collect_maneuver_node_times(_maneuver_state.nodes);
        pick_anchor_times.insert(pick_anchor_times.begin(), track_ctx.now_s);
        if (track_ctx.planned_pick_window.valid && std::isfinite(track_ctx.planned_pick_window.anchor_time_s))
        {
            pick_anchor_times.push_back(track_ctx.planned_pick_window.anchor_time_s);
        }
        std::sort(pick_anchor_times.begin(), pick_anchor_times.end());
        pick_anchor_times.erase(std::unique(pick_anchor_times.begin(), pick_anchor_times.end()), pick_anchor_times.end());
        const std::span<const double> pick_anchor_times_span(pick_anchor_times);

        const auto make_pick_selection_context = [&]() {
            OrbitRenderCurve::SelectionContext selection_ctx{};
            selection_ctx.reference_body_world = track_ctx.ref_body_world;
            selection_ctx.align_delta_world = track_ctx.align_delta;
            selection_ctx.frame_to_world = track_ctx.frame_to_world;
            selection_ctx.camera_world = global_ctx.camera_world;
            selection_ctx.tan_half_fov = global_ctx.tan_half_fov;
            selection_ctx.viewport_height_px = track_ctx.draw_ctx.viewport_height_px;
            selection_ctx.error_px = global_ctx.render_error_px;
            selection_ctx.anchor_times_s = pick_anchor_times_span;
            return selection_ctx;
        };

        const std::size_t pick_planned_reserve_target = std::min(
                _prediction_draw_config.pick_planned_reserve_segments,
                static_cast<std::size_t>(std::max<std::size_t>(
                        1,
                        static_cast<std::size_t>(
                                std::llround(static_cast<double>(pick_max_segments) *
                                             _prediction_draw_config.pick_planned_reserve_ratio)))));
        std::size_t remaining_pick_budget = pick_max_segments;

        const auto build_pick_curve_cache = [&](const OrbitRenderCurve &curve,
                                                const double t_start_s,
                                                const double t_end_s,
                                                std::vector<PickingSystem::LinePickSegmentData> &out_segments,
                                                bool &out_cap_hit) {
            const auto pick_start_tp = std::chrono::steady_clock::now();
            const OrbitRenderCurve::PickResult lod = OrbitRenderCurve::build_pick_lod(
                    curve,
                    make_pick_selection_context(),
                    global_ctx.render_frustum,
                    pick_settings,
                    t_start_s,
                    t_end_s);
            _orbit_plot_perf.pick_lod_ms_last +=
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pick_start_tp).count();
            _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(lod.segments_before_cull);
            _orbit_plot_perf.pick_segments += static_cast<uint32_t>(lod.segments_after_cull);

            out_segments.clear();
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

            out_cap_hit = lod.cap_hit;
            _orbit_plot_perf.pick_cap_hit_last_frame = out_cap_hit;
            if (out_cap_hit)
            {
                ++_orbit_plot_perf.pick_cap_hits_total;
            }
            return out_segments.size();
        };

        const auto build_pick_segments_for_chunk = [&](OrbitChunk &chunk,
                                                       const double chunk_t0_s,
                                                       const double chunk_t1_s,
                                                       const std::size_t max_segments,
                                                       std::vector<PickingSystem::LinePickSegmentData> &out_segments,
                                                       bool &out_cap_hit) {
            if (!chunk.valid || chunk.frame_segments.empty() || !(chunk_t1_s > chunk_t0_s) || max_segments == 0)
            {
                out_segments.clear();
                out_cap_hit = false;
                return std::size_t{0};
            }

            pick_settings.max_segments = std::max<std::size_t>(1, max_segments);
            if (!chunk.render_curve.empty())
            {
                return build_pick_curve_cache(chunk.render_curve, chunk_t0_s, chunk_t1_s, out_segments, out_cap_hit);
            }

            return Draw::build_pick_segment_cache(chunk.frame_segments,
                                                  track_ctx.ref_body_world,
                                                  track_ctx.frame_to_world,
                                                  track_ctx.align_delta,
                                                  global_ctx.render_frustum,
                                                  pick_settings,
                                                  chunk_t0_s,
                                                  chunk_t1_s,
                                                  pick_anchor_times_span,
                                                  false,
                                                  out_segments,
                                                  out_cap_hit,
                                                  _orbit_plot_perf);
        };

        if (allow_base_pick &&
            track_ctx.base_pick_window.valid &&
            pick_group_base != Draw::kInvalidPickGroup &&
            remaining_pick_budget > 0)
        {
            std::size_t emitted = 0;
            const std::size_t planned_reserve =
                    track_ctx.planned_pick_window.valid
                            ? std::min(pick_planned_reserve_target, remaining_pick_budget)
                            : 0;
            if (track_ctx.direct_world_polyline)
            {
                emitted = Draw::emit_polyline_pick_segments(track_ctx.draw_ctx,
                                                            global_ctx.picking,
                                                            pick_group_base,
                                                            *track_ctx.traj_base,
                                                            track_ctx.base_pick_window.t0_s,
                                                            track_ctx.base_pick_window.t1_s,
                                                            remaining_pick_budget - planned_reserve,
                                                            _orbit_plot_perf);
            }
            else
            {
                const std::size_t base_pick_budget = remaining_pick_budget - planned_reserve;
                pick_settings.max_segments = std::max<std::size_t>(1, base_pick_budget);
                bool rebuilt_pick_cache = false;
                if (base_pick_budget > 0 &&
                    Draw::should_rebuild_pick_cache(track.pick_cache,
                                                    stable_cache.generation_id,
                                                    track_ctx.ref_body_world,
                                                    track_ctx.frame_to_world,
                                                    track_ctx.align_delta,
                                                    global_ctx.camera_world,
                                                    global_ctx.tan_half_fov,
                                                    track_ctx.draw_ctx.viewport_height_px,
                                                    global_ctx.render_error_px,
                                                    global_ctx.render_frustum,
                                                    pick_frustum_margin_ratio,
                                                    track_ctx.base_pick_window.t0_s,
                                                    track_ctx.base_pick_window.t1_s,
                                                    base_pick_budget,
                                                    track_ctx.use_base_adaptive_curve,
                                                    false))
                {
                    rebuilt_pick_cache = true;
                    bool cap_hit = false;
                    emitted = 0;
                    if (track_ctx.use_base_adaptive_curve)
                    {
                        emitted = build_pick_curve_cache(stable_cache.render_curve_frame,
                                                         track_ctx.base_pick_window.t0_s,
                                                         track_ctx.base_pick_window.t1_s,
                                                         track.pick_cache.base_segments,
                                                         cap_hit);
                    }
                    else
                    {
                        const std::vector<orbitsim::TrajectorySegment> &pick_base_segments =
                                track_ctx.identity_frame_transform ? *track_ctx.traj_base_segments
                                                                   : Draw::base_segments_world_basis(track_ctx);
                        emitted = Draw::build_pick_segment_cache(pick_base_segments,
                                                                 track_ctx.ref_body_world,
                                                                 track_ctx.frame_to_world,
                                                                 track_ctx.align_delta,
                                                                 global_ctx.render_frustum,
                                                                 pick_settings,
                                                                 track_ctx.base_pick_window.t0_s,
                                                                 track_ctx.base_pick_window.t1_s,
                                                                 pick_anchor_times_span,
                                                                 !track_ctx.identity_frame_transform,
                                                                 track.pick_cache.base_segments,
                                                                 cap_hit,
                                                                 _orbit_plot_perf);
                    }

                    if (emitted > 0)
                    {
                        Draw::mark_pick_cache_valid(track.pick_cache,
                                                    stable_cache.generation_id,
                                                    track_ctx.ref_body_world,
                                                    track_ctx.frame_to_world,
                                                    track_ctx.align_delta,
                                                    global_ctx.camera_world,
                                                    global_ctx.tan_half_fov,
                                                    track_ctx.draw_ctx.viewport_height_px,
                                                    global_ctx.render_error_px,
                                                    global_ctx.render_frustum,
                                                    pick_frustum_margin_ratio,
                                                    track_ctx.base_pick_window.t0_s,
                                                    track_ctx.base_pick_window.t1_s,
                                                    base_pick_budget,
                                                    track_ctx.use_base_adaptive_curve,
                                                    false);
                    }
                    else
                    {
                        track.pick_cache.base_valid = false;
                        track.pick_cache.base_segments.clear();
                    }
                }

                if (base_pick_budget > 0 && track.pick_cache.base_valid && !track.pick_cache.base_segments.empty())
                {
                    emitted = track.pick_cache.base_segments.size();
                    if (!rebuilt_pick_cache)
                    {
                        _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(emitted);
                        _orbit_plot_perf.pick_segments += static_cast<uint32_t>(emitted);
                    }
                    global_ctx.picking->add_line_pick_segments(
                            pick_group_base,
                            std::span<const PickingSystem::LinePickSegmentData>(track.pick_cache.base_segments.data(),
                                                                               track.pick_cache.base_segments.size()));
                }
            }

            remaining_pick_budget = (emitted >= remaining_pick_budget) ? 0 : (remaining_pick_budget - emitted);
        }

        if (allow_planned_pick &&
            track_ctx.planned_pick_window.valid &&
            pick_group_planned != Draw::kInvalidPickGroup &&
            remaining_pick_budget > 0 &&
            !track_ctx.suppress_stale_planned_preview)
        {
            const bool has_chunk_planned_pick =
                    planned_chunk_assembly && planned_chunk_assembly->valid && !planned_chunk_assembly->chunks.empty();
            PredictionLinePickCache &planned_pick_cache =
                    has_chunk_planned_pick ? track.preview_pick_cache : track.pick_cache;
            OrbitPredictionCache *const stable_planned_pick_cache =
                    (!track_ctx.maneuver_drag_active && !stable_cache.trajectory_segments_frame_planned.empty())
                            ? &stable_cache
                            : nullptr;

            if (track_ctx.direct_world_polyline &&
                stable_planned_pick_cache &&
                !stable_planned_pick_cache->trajectory_frame_planned.empty())
            {
                Draw::emit_polyline_pick_segments(track_ctx.draw_ctx,
                                                  global_ctx.picking,
                                                  pick_group_planned,
                                                  stable_planned_pick_cache->trajectory_frame_planned,
                                                  track_ctx.planned_pick_window.t0_s,
                                                  track_ctx.planned_pick_window.t1_s,
                                                  remaining_pick_budget,
                                                  _orbit_plot_perf);
                return;
            }

            const uint64_t planned_pick_generation_id =
                    has_chunk_planned_pick
                            ? planned_chunk_assembly->generation_id
                            : (stable_planned_pick_cache ? stable_planned_pick_cache->generation_id
                                                         : planned_cache.generation_id);
            const bool planned_pick_uses_adaptive_curve =
                    has_chunk_planned_pick ||
                    (stable_planned_pick_cache && !stable_planned_pick_cache->render_curve_frame_planned.empty());
            pick_settings.max_segments = std::max<std::size_t>(1, remaining_pick_budget);
            const bool rebuild_cache_signature = Draw::should_rebuild_pick_cache(planned_pick_cache,
                                                                                 planned_pick_generation_id,
                                                                                 track_ctx.ref_body_world,
                                                                                 track_ctx.frame_to_world,
                                                                                 track_ctx.align_delta,
                                                                                 global_ctx.camera_world,
                                                                                 global_ctx.tan_half_fov,
                                                                                 track_ctx.draw_ctx.viewport_height_px,
                                                                                 global_ctx.render_error_px,
                                                                                 global_ctx.render_frustum,
                                                                                 pick_frustum_margin_ratio,
                                                                                 track_ctx.planned_pick_window.t0_s,
                                                                                 track_ctx.planned_pick_window.t1_s,
                                                                                 remaining_pick_budget,
                                                                                 planned_pick_uses_adaptive_curve,
                                                                                 true);
            bool rebuilt_pick_cache = false;

            if (has_chunk_planned_pick)
            {
                planned_pick_cache.planned_segments.clear();
                if (rebuild_cache_signature)
                {
                    rebuilt_pick_cache = true;
                    Draw::clear_preview_pick_chunk_cache(planned_pick_cache);
                }

                std::vector<PredictionLinePickCache::PreviewChunkEntry> rebuilt_chunk_entries;
                rebuilt_chunk_entries.reserve(planned_chunk_assembly->chunks.size());
                PredictionLinePickCache::PreviewFallbackCache rebuilt_fallback{};
                std::vector<std::pair<double, double>> covered_ranges;
                std::size_t emitted_total = 0;
                const bool can_reuse_preview_chunks =
                        !rebuild_cache_signature && planned_pick_cache.preview_chunk_cache_valid;

                for (OrbitChunk &chunk : planned_chunk_assembly->chunks)
                {
                    if (!chunk.valid || chunk.frame_segments.empty() ||
                        chunk.t1_s <= track_ctx.planned_pick_window.t0_s ||
                        chunk.t0_s >= track_ctx.planned_pick_window.t1_s ||
                        emitted_total >= remaining_pick_budget)
                    {
                        continue;
                    }

                    const double chunk_t0_s = std::max(chunk.t0_s, track_ctx.planned_pick_window.t0_s);
                    const double chunk_t1_s = std::min(chunk.t1_s, track_ctx.planned_pick_window.t1_s);
                    if (!(chunk_t1_s > chunk_t0_s))
                    {
                        continue;
                    }

                    const std::size_t chunk_budget = remaining_pick_budget - emitted_total;
                    const bool chunk_use_adaptive_curve = !chunk.render_curve.empty();
                    const PredictionLinePickCache::PreviewChunkEntry *cached_entry =
                            can_reuse_preview_chunks
                                    ? Draw::find_preview_pick_chunk_entry(planned_pick_cache, chunk.chunk_id)
                                    : nullptr;

                    PredictionLinePickCache::PreviewChunkEntry entry{};
                    if (cached_entry &&
                        Draw::preview_pick_chunk_entry_matches(*cached_entry,
                                                               chunk,
                                                               chunk_t0_s,
                                                               chunk_t1_s,
                                                               chunk_budget,
                                                               chunk_use_adaptive_curve))
                    {
                        entry = *cached_entry;
                        if (entry.cap_hit)
                        {
                            _orbit_plot_perf.pick_cap_hit_last_frame = true;
                        }
                        _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(entry.segments.size());
                        _orbit_plot_perf.pick_segments += static_cast<uint32_t>(entry.segments.size());
                    }
                    else
                    {
                        rebuilt_pick_cache = true;
                        entry.chunk_id = chunk.chunk_id;
                        entry.generation_id = chunk.generation_id;
                        entry.quality_state = chunk.quality_state;
                        entry.t0_s = chunk_t0_s;
                        entry.t1_s = chunk_t1_s;
                        entry.max_segments = chunk_budget;
                        entry.use_adaptive_curve = chunk_use_adaptive_curve;
                        build_pick_segments_for_chunk(chunk,
                                                      chunk_t0_s,
                                                      chunk_t1_s,
                                                      chunk_budget,
                                                      entry.segments,
                                                      entry.cap_hit);
                    }

                    if (entry.segments.empty())
                    {
                        continue;
                    }

                    emitted_total += entry.segments.size();
                    covered_ranges.emplace_back(chunk_t0_s, chunk_t1_s);
                    rebuilt_chunk_entries.push_back(std::move(entry));
                }

                if (stable_planned_pick_cache && emitted_total < remaining_pick_budget)
                {
                    const auto fallback_ranges =
                            Draw::compute_uncovered_ranges(track_ctx.planned_pick_window.t0_s,
                                                           track_ctx.planned_pick_window.t1_s,
                                                           covered_ranges);
                    const std::size_t fallback_budget = remaining_pick_budget - emitted_total;
                    const bool fallback_use_adaptive_curve =
                            !stable_planned_pick_cache->render_curve_frame_planned.empty();
                    if (!fallback_ranges.empty() && fallback_budget > 0)
                    {
                        if (can_reuse_preview_chunks &&
                            Draw::preview_pick_fallback_matches(planned_pick_cache.preview_fallback,
                                                                stable_planned_pick_cache->generation_id,
                                                                fallback_ranges,
                                                                fallback_budget,
                                                                fallback_use_adaptive_curve))
                        {
                            rebuilt_fallback = planned_pick_cache.preview_fallback;
                            if (rebuilt_fallback.cap_hit)
                            {
                                _orbit_plot_perf.pick_cap_hit_last_frame = true;
                            }
                            _orbit_plot_perf.pick_segments_before_cull +=
                                    static_cast<uint32_t>(rebuilt_fallback.segments.size());
                            _orbit_plot_perf.pick_segments +=
                                    static_cast<uint32_t>(rebuilt_fallback.segments.size());
                        }
                        else
                        {
                            rebuilt_pick_cache = true;
                            rebuilt_fallback.valid = true;
                            rebuilt_fallback.source_generation_id = stable_planned_pick_cache->generation_id;
                            rebuilt_fallback.max_segments = fallback_budget;
                            rebuilt_fallback.use_adaptive_curve = fallback_use_adaptive_curve;
                            rebuilt_fallback.uncovered_ranges = fallback_ranges;
                            rebuilt_fallback.cap_hit = false;
                            rebuilt_fallback.segments.clear();

                            for (const auto &[fallback_t0_s, fallback_t1_s] : fallback_ranges)
                            {
                                const std::size_t remaining_budget =
                                        fallback_budget - rebuilt_fallback.segments.size();
                                if (remaining_budget == 0)
                                {
                                    break;
                                }

                                pick_settings.max_segments = std::max<std::size_t>(1, remaining_budget);
                                std::vector<PickingSystem::LinePickSegmentData> fallback_segments;
                                bool cap_hit = false;
                                if (fallback_use_adaptive_curve)
                                {
                                    build_pick_curve_cache(stable_planned_pick_cache->render_curve_frame_planned,
                                                           fallback_t0_s,
                                                           fallback_t1_s,
                                                           fallback_segments,
                                                           cap_hit);
                                }
                                else
                                {
                                    const std::vector<orbitsim::TrajectorySegment> &pick_planned_segments =
                                            track_ctx.identity_frame_transform
                                                    ? stable_planned_pick_cache->trajectory_segments_frame_planned
                                                    : Draw::planned_segments_world_basis(track_ctx, *stable_planned_pick_cache);
                                    Draw::build_pick_segment_cache(pick_planned_segments,
                                                                   track_ctx.ref_body_world,
                                                                   track_ctx.frame_to_world,
                                                                   track_ctx.align_delta,
                                                                   global_ctx.render_frustum,
                                                                   pick_settings,
                                                                   fallback_t0_s,
                                                                   fallback_t1_s,
                                                                   pick_anchor_times_span,
                                                                   !track_ctx.identity_frame_transform,
                                                                   fallback_segments,
                                                                   cap_hit,
                                                                   _orbit_plot_perf);
                                }

                                rebuilt_fallback.cap_hit = rebuilt_fallback.cap_hit || cap_hit;
                                rebuilt_fallback.segments.insert(rebuilt_fallback.segments.end(),
                                                                 fallback_segments.begin(),
                                                                 fallback_segments.end());
                            }
                        }
                    }
                }

                if (!rebuilt_chunk_entries.empty() ||
                    (rebuilt_fallback.valid && !rebuilt_fallback.segments.empty()))
                {
                    planned_pick_cache.preview_chunk_cache_valid = true;
                    planned_pick_cache.preview_chunk_entries = std::move(rebuilt_chunk_entries);
                    planned_pick_cache.preview_fallback = std::move(rebuilt_fallback);
                    Draw::mark_pick_cache_valid(planned_pick_cache,
                                                planned_pick_generation_id,
                                                track_ctx.ref_body_world,
                                                track_ctx.frame_to_world,
                                                track_ctx.align_delta,
                                                global_ctx.camera_world,
                                                global_ctx.tan_half_fov,
                                                track_ctx.draw_ctx.viewport_height_px,
                                                global_ctx.render_error_px,
                                                global_ctx.render_frustum,
                                                pick_frustum_margin_ratio,
                                                track_ctx.planned_pick_window.t0_s,
                                                track_ctx.planned_pick_window.t1_s,
                                                remaining_pick_budget,
                                                planned_pick_uses_adaptive_curve,
                                                true);
                }
                else
                {
                    planned_pick_cache.planned_valid = false;
                    Draw::clear_preview_pick_chunk_cache(planned_pick_cache);
                }

                if (planned_pick_cache.planned_valid && planned_pick_cache.preview_chunk_cache_valid)
                {
                    for (const PredictionLinePickCache::PreviewChunkEntry &entry :
                         planned_pick_cache.preview_chunk_entries)
                    {
                        if (entry.segments.empty())
                        {
                            continue;
                        }

                        global_ctx.picking->add_line_pick_segments(
                                pick_group_planned,
                                std::span<const PickingSystem::LinePickSegmentData>(entry.segments.data(),
                                                                                   entry.segments.size()));
                    }

                    if (planned_pick_cache.preview_fallback.valid &&
                        !planned_pick_cache.preview_fallback.segments.empty())
                    {
                        global_ctx.picking->add_line_pick_segments(
                                pick_group_planned,
                                std::span<const PickingSystem::LinePickSegmentData>(
                                        planned_pick_cache.preview_fallback.segments.data(),
                                        planned_pick_cache.preview_fallback.segments.size()));
                    }
                }
                return;
            }

            if (rebuild_cache_signature)
            {
                rebuilt_pick_cache = true;
                planned_pick_cache.planned_segments.clear();
                Draw::clear_preview_pick_chunk_cache(planned_pick_cache);
                if (stable_planned_pick_cache)
                {
                    bool cap_hit = false;
                    if (!stable_planned_pick_cache->render_curve_frame_planned.empty())
                    {
                        build_pick_curve_cache(stable_planned_pick_cache->render_curve_frame_planned,
                                               track_ctx.planned_pick_window.t0_s,
                                               track_ctx.planned_pick_window.t1_s,
                                               planned_pick_cache.planned_segments,
                                               cap_hit);
                    }
                    else
                    {
                        const std::vector<orbitsim::TrajectorySegment> &pick_planned_segments =
                                track_ctx.identity_frame_transform
                                        ? stable_planned_pick_cache->trajectory_segments_frame_planned
                                        : Draw::planned_segments_world_basis(track_ctx, *stable_planned_pick_cache);
                        Draw::build_pick_segment_cache(pick_planned_segments,
                                                       track_ctx.ref_body_world,
                                                       track_ctx.frame_to_world,
                                                       track_ctx.align_delta,
                                                       global_ctx.render_frustum,
                                                       pick_settings,
                                                       track_ctx.planned_pick_window.t0_s,
                                                       track_ctx.planned_pick_window.t1_s,
                                                       pick_anchor_times_span,
                                                       !track_ctx.identity_frame_transform,
                                                       planned_pick_cache.planned_segments,
                                                       cap_hit,
                                                       _orbit_plot_perf);
                    }
                }

                if (!planned_pick_cache.planned_segments.empty())
                {
                    Draw::mark_pick_cache_valid(planned_pick_cache,
                                                planned_pick_generation_id,
                                                track_ctx.ref_body_world,
                                                track_ctx.frame_to_world,
                                                track_ctx.align_delta,
                                                global_ctx.camera_world,
                                                global_ctx.tan_half_fov,
                                                track_ctx.draw_ctx.viewport_height_px,
                                                global_ctx.render_error_px,
                                                global_ctx.render_frustum,
                                                pick_frustum_margin_ratio,
                                                track_ctx.planned_pick_window.t0_s,
                                                track_ctx.planned_pick_window.t1_s,
                                                remaining_pick_budget,
                                                planned_pick_uses_adaptive_curve,
                                                true);
                }
                else
                {
                    planned_pick_cache.planned_valid = false;
                    planned_pick_cache.planned_segments.clear();
                }
            }

            if (planned_pick_cache.planned_valid && !planned_pick_cache.planned_segments.empty())
            {
                if (!rebuilt_pick_cache)
                {
                    _orbit_plot_perf.pick_segments_before_cull +=
                            static_cast<uint32_t>(planned_pick_cache.planned_segments.size());
                    _orbit_plot_perf.pick_segments +=
                            static_cast<uint32_t>(planned_pick_cache.planned_segments.size());
                }
                global_ctx.picking->add_line_pick_segments(
                        pick_group_planned,
                        std::span<const PickingSystem::LinePickSegmentData>(planned_pick_cache.planned_segments.data(),
                                                                           planned_pick_cache.planned_segments.size()));
            }
        }
    }
} // namespace Game
