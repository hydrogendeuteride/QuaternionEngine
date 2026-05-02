#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_internal.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    namespace Draw = PredictionDrawDetail;
    namespace
    {
        constexpr double kPredictionTimeEpsilonS = 1.0e-6;

        [[nodiscard]] bool preview_pick_clamp_active(const PredictionTrackState &track)
        {
            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                    PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
            return PredictionRuntimeDetail::prediction_track_preview_pick_clamp_active(lifecycle);
        }

        [[nodiscard]] double resolve_preview_pick_coverage_end_s(const PredictionTrackState &track)
        {
            double preview_coverage_end_s = std::numeric_limits<double>::quiet_NaN();
            const PredictionChunkAssembly &assembly = track.preview_overlay.chunk_assembly;
            if (assembly.valid && !assembly.chunks.empty())
            {
                for (const OrbitChunk &chunk : assembly.chunks)
                {
                    if (std::isfinite(chunk.t1_s))
                    {
                        preview_coverage_end_s = std::isfinite(preview_coverage_end_s)
                                                         ? std::max(preview_coverage_end_s, chunk.t1_s)
                                                         : chunk.t1_s;
                    }
                }
                return preview_coverage_end_s;
            }

            if (track.preview_anchor.valid &&
                std::isfinite(track.preview_anchor.anchor_time_s) &&
                track.preview_anchor.exact_window_s > 0.0)
            {
                return track.preview_anchor.anchor_time_s + (2.0 * track.preview_anchor.exact_window_s);
            }

            return std::numeric_limits<double>::quiet_NaN();
        }

        [[nodiscard]] bool draw_frame_specs_compatible(const orbitsim::TrajectoryFrameSpec &a,
                                                       const orbitsim::TrajectoryFrameSpec &b)
        {
            return a.type == b.type &&
                   a.primary_body_id == b.primary_body_id &&
                   a.secondary_body_id == b.secondary_body_id &&
                   a.target_spacecraft_id == b.target_spacecraft_id;
        }

        [[nodiscard]] bool planned_cache_frame_compatible(const OrbitPredictionCache &candidate,
                                                          const OrbitPredictionCache &reference)
        {
            if (candidate.display.display_frame_key != reference.display.display_frame_key ||
                candidate.display.display_frame_revision != reference.display.display_frame_revision)
            {
                return false;
            }

            return !candidate.display.resolved_frame_spec_valid ||
                   !reference.display.resolved_frame_spec_valid ||
                   draw_frame_specs_compatible(candidate.display.resolved_frame_spec,
                                              reference.display.resolved_frame_spec);
        }

        [[nodiscard]] const OrbitPredictionCache *planned_window_source_cache(
                const Draw::PredictionTrackDrawContext &ctx)
        {
            if (ctx.stale_planned_cache &&
                ctx.planned_window_segments == &ctx.stale_planned_cache->display.trajectory_segments_frame_planned)
            {
                return ctx.stale_planned_cache;
            }

            if (ctx.stable_cache &&
                ctx.planned_window_segments == &ctx.stable_cache->display.trajectory_segments_frame_planned)
            {
                return ctx.stable_cache;
            }

            return ctx.planned_cache;
        }
    }

    bool GameplayState::build_orbit_prediction_track_draw_context(
            PredictionTrackState &track,
            const Draw::PredictionGlobalDrawContext &global_ctx,
            Draw::PredictionTrackDrawContext &out)
    {
        out = {};
        out.track = &track;

        refresh_prediction_derived_cache(track, global_ctx.display_time_s);
        if (!prediction_track_has_current_derived_cache(track, global_ctx.display_time_s))
        {
            return false;
        }

        out.stable_cache = &track.cache;
        out.planned_cache = out.stable_cache;

        out.traj_base = &out.stable_cache->display.trajectory_frame;
        out.traj_planned = &out.planned_cache->display.trajectory_frame_planned;
        out.display_cache = out.planned_cache->display.resolved_frame_spec_valid ? out.planned_cache : out.stable_cache;

        if (!build_prediction_display_transform(
                    *out.display_cache,
                    out.ref_body_world,
                    out.frame_to_world,
                    global_ctx.display_time_s))
        {
            out.ref_body_world = prediction_frame_origin_world(*out.display_cache, global_ctx.display_time_s);
            out.frame_to_world = glm::dmat3(1.0);
        }

        out.t0_s = out.traj_base->front().t_s;
        out.t1_s = out.traj_base->back().t_s;
        if (!(out.t1_s > out.t0_s))
        {
            return false;
        }

        out.now_s = Draw::compute_prediction_now_s(global_ctx.display_time_s, out.t0_s, out.t1_s);
        if (!std::isfinite(out.now_s))
        {
            return false;
        }

        if (!get_prediction_subject_world_state(
                    track.key,
                    out.subject_pos_world_state,
                    out.subject_vel_world,
                    out.subject_vel_local))
        {
            return false;
        }

        out.subject_pos_world = out.subject_pos_world_state;
        if (track.key.kind == PredictionSubjectKind::Orbiter)
        {
            if (const Entity *entity = _world.entities().find(EntityId{track.key.value}))
            {
                out.subject_pos_world = entity->get_render_physics_center_of_mass_world(global_ctx.alpha_f);
            }
        }

        out.traj_base_segments = &out.stable_cache->display.trajectory_segments_frame;
        if (out.traj_base_segments->empty())
        {
            return false;
        }

        out.traj_planned_segments = &out.planned_cache->display.trajectory_segments_frame_planned;
        out.planned_window_segments =
                !out.stable_cache->display.trajectory_segments_frame_planned.empty()
                        ? &out.stable_cache->display.trajectory_segments_frame_planned
                        : (!out.planned_cache->display.trajectory_segments_frame_planned.empty()
                                   ? &out.planned_cache->display.trajectory_segments_frame_planned
                                   : out.traj_planned_segments);

        out.is_active = track.key == _prediction->state().selection.active_subject;
        out.active_player_track = out.is_active && prediction_subject_is_player(track.key);
        const bool with_maneuver_live_preview =
                out.active_player_track &&
                prediction_subject_supports_maneuvers(track.key) &&
                _maneuver.settings().nodes_enabled &&
                !_maneuver.plan().nodes.empty();
        out.active_maneuver_track = with_maneuver_live_preview;
        const bool live_preview_active = maneuver_live_preview_active(with_maneuver_live_preview);
        out.maneuver_drag_active =
                out.active_player_track &&
                live_preview_active;
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        const bool keep_stale_planned_visible =
                PredictionRuntimeDetail::prediction_track_should_keep_stale_planned_visible(
                        lifecycle,
                        out.active_player_track,
                        live_preview_active);
        const bool planned_cache_current =
                with_maneuver_live_preview &&
                out.planned_cache &&
                out.planned_cache->identity.maneuver_plan_signature_valid &&
                out.planned_cache->identity.maneuver_plan_signature == current_maneuver_plan_signature();
        out.planned_cache_current = planned_cache_current;
        out.planned_cache_drawable = planned_cache_current &&
                                      out.planned_cache &&
                                      out.planned_cache->display.has_planned_draw_data();
        if (!planned_cache_current || !out.planned_cache_drawable)
        {
            const auto resolve_stale_prefix_cutoff_s = [&]() {
                double cutoff_s = std::numeric_limits<double>::quiet_NaN();
                if (_maneuver.edit_preview().state == ManeuverNodeEditPreview::State::EditingTime)
                {
                    if (const ManeuverNode *edit_node =
                                _maneuver.plan().find_node(_maneuver.edit_preview().node_id))
                    {
                        cutoff_s = edit_node->time_s;
                        if (std::isfinite(_maneuver.edit_preview().start_time_s))
                        {
                            cutoff_s = std::isfinite(cutoff_s)
                                               ? std::min(cutoff_s, _maneuver.edit_preview().start_time_s)
                                               : _maneuver.edit_preview().start_time_s;
                        }
                    }
                    return cutoff_s;
                }

                cutoff_s = track.preview_anchor.valid ? track.preview_anchor.anchor_time_s
                                                      : std::numeric_limits<double>::quiet_NaN();
                if (!std::isfinite(cutoff_s) && out.active_player_track)
                {
                    if (const ManeuverNode *anchor_node =
                                _maneuver.plan().find_node(active_maneuver_preview_anchor_node_id()))
                    {
                        cutoff_s = anchor_node->time_s;
                    }
                }
                if (!std::isfinite(cutoff_s) && out.active_player_track &&
                    (live_preview_active || lifecycle.preview_state != PredictionPreviewRuntimeState::Idle))
                {
                    if (const ManeuverNode *selected_node =
                                _maneuver.plan().find_node(_maneuver.plan().selected_node_id))
                    {
                        cutoff_s = selected_node->time_s;
                    }
                }
                return cutoff_s;
            };

            const auto planned_cache_end_time_s = [](const OrbitPredictionCache &candidate) {
                if (!candidate.display.trajectory_segments_frame_planned.empty())
                {
                    const orbitsim::TrajectorySegment &last =
                            candidate.display.trajectory_segments_frame_planned.back();
                    return last.t0_s + last.dt_s;
                }
                if (candidate.display.trajectory_frame_planned.size() >= 2)
                {
                    return candidate.display.trajectory_frame_planned.back().t_s;
                }
                return std::numeric_limits<double>::quiet_NaN();
            };

            double stale_prefix_cutoff_s = resolve_stale_prefix_cutoff_s();
            const auto stale_candidate_drawable = [&](const OrbitPredictionCache &candidate) {
                return candidate.identity.valid &&
                       candidate.display.has_planned_draw_data() &&
                       out.planned_cache &&
                       planned_cache_frame_compatible(candidate, *out.planned_cache);
            };

            OrbitPredictionCache *stale_candidate = nullptr;
            if (out.active_player_track &&
                track.authoritative_cache.identity.valid &&
                stale_candidate_drawable(track.authoritative_cache))
            {
                stale_candidate = &track.authoritative_cache;
            }
            else if (out.active_player_track &&
                     out.planned_cache &&
                     stale_candidate_drawable(*out.planned_cache))
            {
                stale_candidate = out.planned_cache;
            }

            if (stale_candidate && !std::isfinite(stale_prefix_cutoff_s) &&
                lifecycle.preview_state == PredictionPreviewRuntimeState::Idle &&
                (lifecycle.dirty || lifecycle.request_pending || lifecycle.derived_request_pending ||
                 lifecycle.awaiting_authoritative_publish))
            {
                stale_prefix_cutoff_s = planned_cache_end_time_s(*stale_candidate);
            }

            const bool can_draw_stale_prefix =
                    out.active_player_track &&
                    stale_candidate &&
                    std::isfinite(stale_prefix_cutoff_s);
            if (can_draw_stale_prefix)
            {
                out.stale_planned_cache = stale_candidate;
                out.stale_planned_cache_drawable = keep_stale_planned_visible;
                out.stale_planned_cache_prefix_cutoff_s = stale_prefix_cutoff_s;
                if (!out.stale_planned_cache->display.trajectory_segments_frame_planned.empty())
                {
                    out.planned_window_segments =
                            &out.stale_planned_cache->display.trajectory_segments_frame_planned;
                }
            }
            else
            {
                out.traj_planned = nullptr;
                out.traj_planned_segments = nullptr;
                out.planned_window_segments = nullptr;
            }
        }

        if (out.is_active)
        {
            _prediction->state().orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(out.traj_base_segments->size());
            _prediction->state().orbit_plot_perf.solver_segments_planned =
                    out.traj_planned_segments
                            ? static_cast<uint32_t>(out.traj_planned_segments->size())
                            : 0u;
        }

        out.i_hi = Draw::lower_bound_sample_index(*out.traj_base, out.now_s);
        if (out.i_hi >= out.traj_base->size())
        {
            out.i_hi = out.traj_base->size() - 1;
        }

        out.align_delta = Draw::compute_align_delta(*out.traj_base_segments,
                                                    *out.traj_base,
                                                    out.i_hi,
                                                    out.subject_pos_world,
                                                    out.now_s,
                                                    out.ref_body_world,
                                                    out.frame_to_world);
        out.direct_world_polyline = Draw::frame_spec_uses_direct_world_polyline(
                out.display_cache->display.resolved_frame_spec_valid ? out.display_cache->display.resolved_frame_spec
                                                                     : _prediction->state().frame_selection.spec);

        out.draw_ctx.orbit_plot = global_ctx.orbit_plot;
        out.draw_ctx.ref_body_world = out.ref_body_world;
        out.draw_ctx.frame_to_world = out.frame_to_world;
        out.draw_ctx.align_delta = out.align_delta;
        out.draw_ctx.lod_camera = global_ctx.lod_camera;
        out.draw_ctx.render_frustum = global_ctx.render_frustum;
        out.draw_ctx.camera_world = global_ctx.camera_world;
        out.draw_ctx.tan_half_fov = global_ctx.tan_half_fov;
        out.draw_ctx.viewport_height_px = std::max(1.0, static_cast<double>(global_ctx.viewport_height_px));
        out.draw_ctx.render_error_px = global_ctx.render_error_px;
        out.draw_ctx.render_max_segments =
                static_cast<std::size_t>(std::max(1, _prediction->budget().render_max_segments_cpu));
        out.draw_ctx.line_overlay_boost = out.maneuver_drag_active
                                                  ? 0.0f
                                                  : std::clamp(_prediction->state().line_overlay_boost, 0.0f, 1.0f);

        out.identity_frame_transform = Draw::frame_transform_is_identity(out.frame_to_world);
        out.use_base_adaptive_curve = !out.stable_cache->display.render_curve_frame.empty();
        const bool planned_preview_like = PredictionRuntimeDetail::prediction_track_planned_preview_like(lifecycle);
        const bool stale_planned_curve_drawable =
                out.stale_planned_cache_drawable &&
                out.stale_planned_cache &&
                !out.stale_planned_cache->display.render_curve_frame_planned.empty();
        const bool current_planned_curve_drawable =
                out.planned_cache_drawable &&
                out.planned_cache &&
                !out.planned_cache->display.render_curve_frame_planned.empty();
        out.use_planned_adaptive_curve =
                !out.maneuver_drag_active &&
                (!planned_preview_like || stale_planned_curve_drawable) &&
                (current_planned_curve_drawable || stale_planned_curve_drawable);

        out.world_basis_draw_ctx = out.draw_ctx;
        if (!out.identity_frame_transform)
        {
            out.world_basis_draw_ctx.frame_to_world = glm::dmat3(1.0);
        }

        const glm::vec3 track_rgb = prediction_subject_orbit_rgb(track.key);
        out.track_color_full = Draw::scale_line_color(glm::vec4(track_rgb, 0.22f), global_ctx.line_alpha_scale);
        out.track_color_future = Draw::scale_line_color(glm::vec4(track_rgb, 0.80f), global_ctx.line_alpha_scale);
        out.track_color_plan = global_ctx.color_orbit_plan;
        if (out.maneuver_drag_active)
        {
            out.track_color_plan.a = out.track_color_future.a;
        }
        if (!out.maneuver_drag_active && !out.is_active)
        {
            out.draw_ctx.line_overlay_boost = std::clamp(_prediction->state().line_overlay_boost * 0.35f, 0.0f, 1.0f);
        }
        out.world_basis_draw_ctx.line_overlay_boost = out.draw_ctx.line_overlay_boost;

        out.future_window_s = prediction_future_window_s(track.key);
        const OrbitPredictionCache *planned_window_source = planned_window_source_cache(out);

        if (out.planned_window_segments && !out.planned_window_segments->empty() &&
            planned_window_source && planned_window_source->has_planned_frame_draw_data())
        {
            const double planned_segments_t0_s = out.planned_window_segments->front().t0_s;
            const double planned_segments_t1_s =
                    out.planned_window_segments->back().t0_s + out.planned_window_segments->back().dt_s;
            const PredictionTimeContext time_ctx =
                    build_prediction_time_context(track.key, out.now_s, planned_segments_t0_s, planned_segments_t1_s);
            out.planned_window_policy = resolve_prediction_window_policy(&track, time_ctx, true);
            if (out.active_player_track)
            {
                const PredictionTimeContext authored_plan_ctx = build_prediction_time_context(track.key, out.now_s);
                if (authored_plan_ctx.has_plan &&
                    (std::isfinite(authored_plan_ctx.first_relevant_node_time_s) ||
                     std::isfinite(authored_plan_ctx.first_future_node_time_s)))
                {
                    const double authored_plan_start_s =
                            std::isfinite(authored_plan_ctx.first_future_node_time_s)
                                    ? authored_plan_ctx.first_future_node_time_s
                                    : authored_plan_ctx.first_relevant_node_time_s;
                    const double authored_plan_tail_anchor_s =
                            std::isfinite(authored_plan_ctx.last_future_node_time_s)
                                    ? authored_plan_ctx.last_future_node_time_s
                                    : authored_plan_start_s;
                    const double authored_plan_end_s =
                            std::isfinite(authored_plan_start_s) && std::isfinite(authored_plan_tail_anchor_s)
                                    ? std::max(authored_plan_start_s + maneuver_plan_horizon_s(),
                                               authored_plan_tail_anchor_s +
                                                       OrbitPredictionTuning::kPostNodeCoverageMinS)
                                    : std::numeric_limits<double>::quiet_NaN();
                    if (std::isfinite(authored_plan_start_s) &&
                        authored_plan_end_s > (authored_plan_start_s + kPredictionTimeEpsilonS))
                    {
                        out.planned_window_policy.visual_window_start_time_s =
                                authored_plan_start_s;
                        out.planned_window_policy.pick_window_start_time_s =
                                authored_plan_start_s;
                    }

                    if (std::isfinite(authored_plan_end_s))
                    {
                        out.planned_window_policy.visual_window_end_time_s = authored_plan_end_s;
                        out.planned_window_policy.pick_window_end_time_s = authored_plan_end_s;
                    }

                    if (out.stale_planned_cache_drawable &&
                        std::isfinite(out.stale_planned_cache_prefix_cutoff_s) &&
                        out.planned_window_segments &&
                        !out.planned_window_segments->empty())
                    {
                        const double prefix_start_s =
                                std::max(out.now_s, out.planned_window_segments->front().t0_s);
                        if (out.stale_planned_cache_prefix_cutoff_s >
                            prefix_start_s + kPredictionTimeEpsilonS)
                        {
                            out.planned_window_policy.visual_window_start_time_s = prefix_start_s;
                            out.planned_window_policy.pick_window_start_time_s = prefix_start_s;
                        }
                    }
                }

                // Clamp picking to the preview patch while the preview overlay is
                // active so stale tail geometry stays visible but cannot be selected.
                if (preview_pick_clamp_active(track))
                {
                    const double preview_coverage_end_s = resolve_preview_pick_coverage_end_s(track);
                    if (std::isfinite(preview_coverage_end_s))
                    {
                        if (std::isfinite(out.planned_window_policy.pick_window_end_time_s))
                        {
                            out.planned_window_policy.pick_window_end_time_s =
                                    std::min(out.planned_window_policy.pick_window_end_time_s,
                                             preview_coverage_end_s);
                        }
                        else
                        {
                            out.planned_window_policy.pick_window_end_time_s = preview_coverage_end_s;
                        }
                    }
                }
            }
            out.planned_visual_window_s = out.planned_window_policy.visual_window_s;
            out.planned_exact_window_s = out.planned_window_policy.exact_window_s;
            out.planned_pick_window_s = out.planned_window_policy.pick_window_s;
        }
        return true;
    }
} // namespace Game
