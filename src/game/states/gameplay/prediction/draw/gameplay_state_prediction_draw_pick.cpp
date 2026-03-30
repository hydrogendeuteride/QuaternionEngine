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
            if (track_ctx.direct_world_polyline && !planned_cache.trajectory_frame_planned.empty())
            {
                Draw::emit_polyline_pick_segments(track_ctx.draw_ctx,
                                                  global_ctx.picking,
                                                  pick_group_planned,
                                                  planned_cache.trajectory_frame_planned,
                                                  track_ctx.planned_pick_window.t0_s,
                                                  track_ctx.planned_pick_window.t1_s,
                                                  remaining_pick_budget,
                                                  _orbit_plot_perf);
                return;
            }

            const bool planned_pick_uses_adaptive_curve = !planned_cache.render_curve_frame_planned.empty();
            pick_settings.max_segments = std::max<std::size_t>(1, remaining_pick_budget);
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
                bool cap_hit = false;
                if (!planned_cache.render_curve_frame_planned.empty())
                {
                    build_pick_curve_cache(planned_cache.render_curve_frame_planned,
                                           track_ctx.planned_pick_window.t0_s,
                                           track_ctx.planned_pick_window.t1_s,
                                           track.pick_cache.planned_segments,
                                           cap_hit);
                }
                else
                {
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
