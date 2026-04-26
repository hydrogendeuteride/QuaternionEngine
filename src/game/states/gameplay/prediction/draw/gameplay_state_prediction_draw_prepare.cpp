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
            if (candidate.display_frame_key != reference.display_frame_key ||
                candidate.display_frame_revision != reference.display_frame_revision)
            {
                return false;
            }

            return !candidate.resolved_frame_spec_valid ||
                   !reference.resolved_frame_spec_valid ||
                   draw_frame_specs_compatible(candidate.resolved_frame_spec, reference.resolved_frame_spec);
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

        out.traj_base = &out.stable_cache->trajectory_frame;
        out.traj_planned = &out.planned_cache->trajectory_frame_planned;
        out.display_cache = out.planned_cache->resolved_frame_spec_valid ? out.planned_cache : out.stable_cache;

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

        out.traj_base_segments = &out.stable_cache->trajectory_segments_frame;
        if (out.traj_base_segments->empty())
        {
            return false;
        }

        out.traj_planned_segments = &out.planned_cache->trajectory_segments_frame_planned;
        out.planned_window_segments =
                !out.stable_cache->trajectory_segments_frame_planned.empty()
                        ? &out.stable_cache->trajectory_segments_frame_planned
                        : (!out.planned_cache->trajectory_segments_frame_planned.empty()
                                   ? &out.planned_cache->trajectory_segments_frame_planned
                                   : out.traj_planned_segments);

        out.is_active = track.key == _prediction_selection.active_subject;
        out.active_player_track = out.is_active && prediction_subject_is_player(track.key);
        const bool with_maneuver_live_preview =
                out.active_player_track &&
                prediction_subject_supports_maneuvers(track.key) &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty();
        out.maneuver_drag_active =
                out.active_player_track &&
                maneuver_live_preview_active(with_maneuver_live_preview);
        const bool planned_cache_current =
                with_maneuver_live_preview &&
                out.planned_cache &&
                out.planned_cache->maneuver_plan_signature_valid &&
                out.planned_cache->maneuver_plan_signature == current_maneuver_plan_signature();
        out.planned_cache_current = planned_cache_current;
        out.planned_cache_drawable = planned_cache_current;
        if (!planned_cache_current)
        {
            const auto resolve_stale_prefix_cutoff_s = [&]() {
                double cutoff_s = std::numeric_limits<double>::quiet_NaN();
                if (_maneuver_node_edit_preview.state == ManeuverNodeEditPreview::State::EditingTime)
                {
                    if (const ManeuverNode *edit_node =
                                _maneuver_state.find_node(_maneuver_node_edit_preview.node_id))
                    {
                        cutoff_s = edit_node->time_s;
                        if (std::isfinite(_maneuver_node_edit_preview.start_time_s))
                        {
                            cutoff_s = std::isfinite(cutoff_s)
                                               ? std::min(cutoff_s, _maneuver_node_edit_preview.start_time_s)
                                               : _maneuver_node_edit_preview.start_time_s;
                        }
                    }
                    return cutoff_s;
                }

                cutoff_s = track.preview_anchor.valid ? track.preview_anchor.anchor_time_s
                                                      : std::numeric_limits<double>::quiet_NaN();
                if (!std::isfinite(cutoff_s) && out.active_player_track)
                {
                    if (const ManeuverNode *anchor_node =
                                _maneuver_state.find_node(active_maneuver_preview_anchor_node_id()))
                    {
                        cutoff_s = anchor_node->time_s;
                    }
                }
                return cutoff_s;
            };

            const double stale_prefix_cutoff_s = resolve_stale_prefix_cutoff_s();
            const auto stale_candidate_drawable = [&](const OrbitPredictionCache &candidate) {
                return candidate.valid &&
                       candidate.has_planned_frame_draw_data() &&
                       out.planned_cache &&
                       planned_cache_frame_compatible(candidate, *out.planned_cache);
            };

            OrbitPredictionCache *stale_candidate = nullptr;
            if (out.active_player_track &&
                track.authoritative_cache.valid &&
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

            const bool can_draw_stale_prefix =
                    out.active_player_track &&
                    stale_candidate &&
                    std::isfinite(stale_prefix_cutoff_s);
            if (can_draw_stale_prefix)
            {
                out.stale_planned_cache = stale_candidate;
                out.stale_planned_cache_drawable = maneuver_live_preview_active(with_maneuver_live_preview);
                out.stale_planned_cache_prefix_cutoff_s = stale_prefix_cutoff_s;
                if (!out.stale_planned_cache->trajectory_segments_frame_planned.empty())
                {
                    out.planned_window_segments = &out.stale_planned_cache->trajectory_segments_frame_planned;
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
            _orbit_plot_perf.solver_segments_base = static_cast<uint32_t>(out.traj_base_segments->size());
            _orbit_plot_perf.solver_segments_planned =
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
                out.display_cache->resolved_frame_spec_valid ? out.display_cache->resolved_frame_spec
                                                             : _prediction_frame_selection.spec);

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
                static_cast<std::size_t>(std::max(1, _orbit_plot_budget.render_max_segments_cpu));
        out.draw_ctx.line_overlay_boost = out.maneuver_drag_active
                                                  ? 0.0f
                                                  : std::clamp(_prediction_line_overlay_boost, 0.0f, 1.0f);

        out.identity_frame_transform = Draw::frame_transform_is_identity(out.frame_to_world);
        out.use_base_adaptive_curve = !out.stable_cache->render_curve_frame.empty();
        // Planned maneuver paths need to stay on the segment path. Preview streaming
        // publishes segment chunks, and switching the tail back to the cached adaptive
        // render curve on final publish can visibly move the post-preview tail.
        out.use_planned_adaptive_curve = false;

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
            out.draw_ctx.line_overlay_boost = std::clamp(_prediction_line_overlay_boost * 0.35f, 0.0f, 1.0f);
        }
        out.world_basis_draw_ctx.line_overlay_boost = out.draw_ctx.line_overlay_boost;

        out.future_window_s = prediction_future_window_s(track.key);
        if (out.planned_window_segments && !out.planned_window_segments->empty() &&
            out.planned_cache && out.planned_cache->has_planned_frame_draw_data())
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
                            std::isfinite(authored_plan_ctx.sim_now_s) &&
                                            std::isfinite(authored_plan_tail_anchor_s)
                                    ? std::max(authored_plan_ctx.sim_now_s + maneuver_plan_horizon_s(),
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
