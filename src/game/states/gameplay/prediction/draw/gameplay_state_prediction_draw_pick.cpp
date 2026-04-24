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
        // During maneuver-axis drags, orbit hover/pick feedback is not actionable and
        // rebuilding line-pick LOD every frame is wasted work.
        if (!global_ctx.picking || !track_ctx.active_player_track || track_ctx.maneuver_drag_active)
        {
            return;
        }

        PredictionTrackState &track = *track_ctx.track;
        OrbitPredictionCache &stable_cache = *track_ctx.stable_cache;
        OrbitPredictionCache &planned_cache = *track_ctx.planned_cache;

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
            selection_ctx.error_frustum = global_ctx.render_frustum;
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
            _orbit_plot_perf.pick_cap_hit_last_frame = _orbit_plot_perf.pick_cap_hit_last_frame || out_cap_hit;
            if (out_cap_hit)
            {
                ++_orbit_plot_perf.pick_cap_hits_total;
            }
            return out_segments.size();
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
                                                    stable_cache.display_frame_key,
                                                    stable_cache.display_frame_revision,
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
                                                    stable_cache.display_frame_key,
                                                    stable_cache.display_frame_revision,
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
            remaining_pick_budget > 0)
        {
            const PredictionChunkAssembly preview_assembly_snapshot = track.preview_overlay.chunk_assembly;
            const PredictionChunkAssembly &preview_assembly = preview_assembly_snapshot;
            const PredictionChunkAssembly *full_stream_assembly =
                    track.full_stream_overlay.ready_for_draw(planned_cache.generation_id,
                                                             planned_cache.display_frame_key,
                                                             planned_cache.display_frame_revision)
                            ? &track.full_stream_overlay.chunk_assembly
                            : nullptr;
            const bool preview_overlay_active = preview_assembly.valid && !preview_assembly.chunks.empty();
            const bool full_stream_overlay_active = full_stream_assembly && !full_stream_assembly->chunks.empty();
            const auto assembly_has_pick_curve = [](const PredictionChunkAssembly &assembly) {
                return std::any_of(assembly.chunks.begin(),
                                   assembly.chunks.end(),
                                   [](const OrbitChunk &chunk) { return !chunk.render_curve.empty(); });
            };
            const bool overlay_pick_active = preview_overlay_active || full_stream_overlay_active;
            const bool planned_pick_uses_adaptive_curve =
                    overlay_pick_active
                            ? ((preview_overlay_active && assembly_has_pick_curve(preview_assembly)) ||
                               (full_stream_overlay_active && assembly_has_pick_curve(*full_stream_assembly)))
                            : track_ctx.use_planned_adaptive_curve;
            const bool rebuild_cache = Draw::should_rebuild_pick_cache(track.pick_cache,
                                                                       planned_cache.generation_id,
                                                                       planned_cache.display_frame_key,
                                                                       planned_cache.display_frame_revision,
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
            if (rebuild_cache)
            {
                rebuilt_pick_cache = true;
                track.pick_cache.planned_segments.clear();
                const auto build_pick_polyline_segments =
                        [&](const std::vector<orbitsim::TrajectorySample> &traj,
                            const double t_start_s,
                            const double t_end_s,
                            const std::size_t max_segments,
                            std::vector<PickingSystem::LinePickSegmentData> &out_segments) {
                            out_segments.clear();
                            if (traj.size() < 2 || max_segments == 0)
                            {
                                return std::size_t{0};
                            }

                            out_segments.reserve(std::min(max_segments, traj.size() - 1u));
                            std::size_t emitted = 0;
                            for (std::size_t i = 1; i < traj.size() && emitted < max_segments; ++i)
                            {
                                const double seg_t0_s = traj[i - 1].t_s;
                                const double seg_t1_s = traj[i].t_s;
                                const double clip_t0_s = std::max(seg_t0_s, t_start_s);
                                const double clip_t1_s = std::min(seg_t1_s, t_end_s);
                                if (!(clip_t1_s > clip_t0_s))
                                {
                                    continue;
                                }

                                const WorldVec3 a_world =
                                        Draw::sample_polyline_world(track_ctx.ref_body_world,
                                                                    track_ctx.frame_to_world,
                                                                    traj,
                                                                    i - 1,
                                                                    i,
                                                                    clip_t0_s) +
                                        track_ctx.align_delta;
                                const WorldVec3 b_world =
                                        Draw::sample_polyline_world(track_ctx.ref_body_world,
                                                                    track_ctx.frame_to_world,
                                                                    traj,
                                                                    i - 1,
                                                                    i,
                                                                    clip_t1_s) +
                                        track_ctx.align_delta;
                                out_segments.push_back(PickingSystem::LinePickSegmentData{
                                        .a_world = a_world,
                                        .b_world = b_world,
                                        .a_time_s = clip_t0_s,
                                        .b_time_s = clip_t1_s,
                                });
                                ++emitted;
                            }

                            _orbit_plot_perf.pick_segments_before_cull += static_cast<uint32_t>(emitted);
                            _orbit_plot_perf.pick_segments += static_cast<uint32_t>(emitted);
                            return emitted;
                        };
                const auto append_chunk_pick_segments =
                        [&](const OrbitChunk &chunk, const double t_start_s, const double t_end_s, const std::size_t budget_left) {
                            if (!(t_end_s > t_start_s) || budget_left == 0)
                            {
                                return std::size_t{0};
                            }

                            std::vector<PickingSystem::LinePickSegmentData> chunk_segments;
                            bool cap_hit = false;
                            std::size_t emitted = 0;
                            pick_settings.max_segments = std::max<std::size_t>(1, budget_left);
                            if (track_ctx.direct_world_polyline && chunk.frame_samples.size() >= 2)
                            {
                                emitted = build_pick_polyline_segments(chunk.frame_samples,
                                                                       t_start_s,
                                                                       t_end_s,
                                                                       budget_left,
                                                                       chunk_segments);
                            }
                            else if (!chunk.render_curve.empty())
                            {
                                emitted = build_pick_curve_cache(chunk.render_curve,
                                                                 t_start_s,
                                                                 t_end_s,
                                                                 chunk_segments,
                                                                 cap_hit);
                            }
                            else if (!chunk.frame_segments.empty())
                            {
                                emitted = Draw::build_pick_segment_cache(chunk.frame_segments,
                                                                         track_ctx.ref_body_world,
                                                                         track_ctx.frame_to_world,
                                                                         track_ctx.align_delta,
                                                                         global_ctx.render_frustum,
                                                                         pick_settings,
                                                                         t_start_s,
                                                                         t_end_s,
                                                                         pick_anchor_times_span,
                                                                         false,
                                                                         chunk_segments,
                                                                         cap_hit,
                                                                         _orbit_plot_perf);
                            }

                            if (emitted == 0 || chunk_segments.empty())
                            {
                                return std::size_t{0};
                            }

                            track.pick_cache.planned_segments.insert(track.pick_cache.planned_segments.end(),
                                                                     chunk_segments.begin(),
                                                                     chunk_segments.end());
                            return chunk_segments.size();
                        };
                const auto append_chunk_assembly_pick_ranges =
                        [&](const PredictionChunkAssembly &assembly,
                            const std::vector<std::pair<double, double>> *masked_ranges,
                            std::vector<std::pair<double, double>> &out_covered_ranges) {
                            std::size_t emitted_total = 0;
                            for (const OrbitChunk &chunk : assembly.chunks)
                            {
                                const double clipped_t0_s = std::max(track_ctx.planned_pick_window.t0_s, chunk.t0_s);
                                const double clipped_t1_s = std::min(track_ctx.planned_pick_window.t1_s, chunk.t1_s);
                                if (!(clipped_t1_s > clipped_t0_s))
                                {
                                    continue;
                                }

                                std::vector<std::pair<double, double>> pick_ranges;
                                if (masked_ranges && !masked_ranges->empty())
                                {
                                    pick_ranges = Draw::compute_uncovered_ranges(clipped_t0_s, clipped_t1_s, *masked_ranges);
                                }
                                else
                                {
                                    pick_ranges.emplace_back(clipped_t0_s, clipped_t1_s);
                                }

                                for (const auto &[range_t0_s, range_t1_s] : pick_ranges)
                                {
                                    const std::size_t budget_left =
                                            (remaining_pick_budget > emitted_total)
                                                    ? (remaining_pick_budget - emitted_total)
                                                    : 0u;
                                    if (budget_left == 0)
                                    {
                                        return emitted_total;
                                    }

                                    const std::size_t emitted =
                                            append_chunk_pick_segments(chunk, range_t0_s, range_t1_s, budget_left);
                                    if (emitted == 0)
                                    {
                                        continue;
                                    }

                                    out_covered_ranges.emplace_back(range_t0_s, range_t1_s);
                                    emitted_total += emitted;
                                }
                            }
                            return emitted_total;
                        };

                if (overlay_pick_active)
                {
                    std::vector<std::pair<double, double>> covered_ranges;
                    if (preview_overlay_active)
                    {
                        covered_ranges.reserve(preview_assembly.chunks.size());
                        append_chunk_assembly_pick_ranges(preview_assembly, nullptr, covered_ranges);
                    }

                    if (full_stream_overlay_active)
                    {
                        std::vector<std::pair<double, double>> full_stream_covered_ranges;
                        full_stream_covered_ranges.reserve(full_stream_assembly->chunks.size());
                        append_chunk_assembly_pick_ranges(*full_stream_assembly,
                                                          preview_overlay_active ? &covered_ranges : nullptr,
                                                          full_stream_covered_ranges);
                    }
                    else if (preview_overlay_active)
                    {
                        const std::vector<std::pair<double, double>> uncovered_ranges =
                                Draw::compute_uncovered_ranges(track_ctx.planned_pick_window.t0_s,
                                                               track_ctx.planned_pick_window.t1_s,
                                                               covered_ranges);
                        for (const auto &[range_t0_s, range_t1_s] : uncovered_ranges)
                        {
                            const std::size_t budget_left =
                                    remaining_pick_budget > track.pick_cache.planned_segments.size()
                                            ? (remaining_pick_budget - track.pick_cache.planned_segments.size())
                                            : 0u;
                            if (budget_left == 0)
                            {
                                break;
                            }

                            if (track_ctx.direct_world_polyline && !planned_cache.trajectory_frame_planned.empty())
                            {
                                std::vector<PickingSystem::LinePickSegmentData> fallback_segments;
                                build_pick_polyline_segments(planned_cache.trajectory_frame_planned,
                                                             range_t0_s,
                                                             range_t1_s,
                                                             budget_left,
                                                             fallback_segments);
                                track.pick_cache.planned_segments.insert(track.pick_cache.planned_segments.end(),
                                                                         fallback_segments.begin(),
                                                                         fallback_segments.end());
                            }
                            else
                            {
                                std::vector<PickingSystem::LinePickSegmentData> fallback_segments;
                                bool cap_hit = false;
                                pick_settings.max_segments = std::max<std::size_t>(1, budget_left);
                                const std::vector<orbitsim::TrajectorySegment> &pick_planned_segments =
                                        track_ctx.identity_frame_transform
                                                ? planned_cache.trajectory_segments_frame_planned
                                                : Draw::planned_segments_world_basis(track_ctx, planned_cache);
                                Draw::build_pick_segment_cache(pick_planned_segments,
                                                               track_ctx.ref_body_world,
                                                               track_ctx.frame_to_world,
                                                               track_ctx.align_delta,
                                                               global_ctx.render_frustum,
                                                               pick_settings,
                                                               range_t0_s,
                                                               range_t1_s,
                                                               pick_anchor_times_span,
                                                               !track_ctx.identity_frame_transform,
                                                               fallback_segments,
                                                               cap_hit,
                                                               _orbit_plot_perf);
                                track.pick_cache.planned_segments.insert(track.pick_cache.planned_segments.end(),
                                                                         fallback_segments.begin(),
                                                                         fallback_segments.end());
                            }
                        }
                    }
                }
                else if (track_ctx.direct_world_polyline && !planned_cache.trajectory_frame_planned.empty())
                {
                    build_pick_polyline_segments(planned_cache.trajectory_frame_planned,
                                                 track_ctx.planned_pick_window.t0_s,
                                                 track_ctx.planned_pick_window.t1_s,
                                                 remaining_pick_budget,
                                                 track.pick_cache.planned_segments);
                }
                else if (planned_pick_uses_adaptive_curve)
                {
                    bool cap_hit = false;
                    build_pick_curve_cache(planned_cache.render_curve_frame_planned,
                                           track_ctx.planned_pick_window.t0_s,
                                           track_ctx.planned_pick_window.t1_s,
                                           track.pick_cache.planned_segments,
                                           cap_hit);
                }
                else
                {
                    bool cap_hit = false;
                    pick_settings.max_segments = std::max<std::size_t>(1, remaining_pick_budget);
                    const std::vector<orbitsim::TrajectorySegment> &pick_planned_segments =
                            track_ctx.identity_frame_transform
                                    ? planned_cache.trajectory_segments_frame_planned
                                    : Draw::planned_segments_world_basis(track_ctx, planned_cache);
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
                                                   track.pick_cache.planned_segments,
                                                   cap_hit,
                                                   _orbit_plot_perf);
                }

                if (!track.pick_cache.planned_segments.empty())
                {
                    Draw::mark_pick_cache_valid(track.pick_cache,
                                                planned_cache.generation_id,
                                                planned_cache.display_frame_key,
                                                planned_cache.display_frame_revision,
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
                    track.pick_cache.planned_valid = false;
                }
            }

            if (track.pick_cache.planned_valid && !track.pick_cache.planned_segments.empty())
            {
                if (!rebuilt_pick_cache)
                {
                    _orbit_plot_perf.pick_segments_before_cull +=
                            static_cast<uint32_t>(track.pick_cache.planned_segments.size());
                    _orbit_plot_perf.pick_segments +=
                            static_cast<uint32_t>(track.pick_cache.planned_segments.size());
                }
                global_ctx.picking->add_line_pick_segments(
                        pick_group_planned,
                        std::span<const PickingSystem::LinePickSegmentData>(track.pick_cache.planned_segments.data(),
                                                                           track.pick_cache.planned_segments.size()));
            }
        }
    }
} // namespace Game
