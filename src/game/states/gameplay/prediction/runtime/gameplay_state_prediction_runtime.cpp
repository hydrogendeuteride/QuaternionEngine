#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace Game
{
    namespace
    {
        struct PreviewWindowPolicy
        {
            double visual_window_s{0.0};
            double exact_window_s{0.0};
            double pick_window_s{0.0};
            double request_window_s{0.0};
        };

        const ManeuverNode *select_preview_anchor_node(const ManeuverPlanState &plan, const double now_s)
        {
            if (const ManeuverNode *selected = plan.find_node(plan.selected_node_id))
            {
                if (std::isfinite(selected->time_s))
                {
                    return selected;
                }
            }

            const ManeuverNode *first_future = nullptr;
            for (const ManeuverNode &node : plan.nodes)
            {
                if (!std::isfinite(node.time_s) || node.time_s < now_s)
                {
                    continue;
                }

                if (!first_future || node.time_s < first_future->time_s)
                {
                    first_future = &node;
                }
            }
            return first_future;
        }

        template<typename T>
        void hash_combine(uint64_t &seed, const T &value)
        {
            seed ^= static_cast<uint64_t>(std::hash<T>{}(value)) + 0x9e3779b97f4a7c15ULL + (seed << 6u) + (seed >> 2u);
        }

        uint64_t hash_upstream_maneuvers(const ManeuverPlanState &plan,
                                         const int anchor_node_id,
                                         const double anchor_time_s)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            for (const ManeuverNode &node : plan.nodes)
            {
                if (!std::isfinite(node.time_s) ||
                    node.id == anchor_node_id ||
                    node.time_s > anchor_time_s)
                {
                    continue;
                }

                hash_combine(seed, node.id);
                hash_combine(seed, node.time_s);
                hash_combine(seed, node.primary_body_id);
                hash_combine(seed, node.primary_body_auto);
                hash_combine(seed, node.dv_rtn_mps.x);
                hash_combine(seed, node.dv_rtn_mps.y);
                hash_combine(seed, node.dv_rtn_mps.z);
            }
            return seed;
        }

        glm::dmat3 snapshot_gizmo_basis(const ManeuverGizmoInteraction &interaction)
        {
            if (PredictionRuntimeDetail::maneuver_drag_active(interaction.state))
            {
                return glm::dmat3(interaction.drag_basis_r_world,
                                  interaction.drag_basis_t_world,
                                  interaction.drag_basis_n_world);
            }
            return glm::dmat3(1.0);
        }

        PreviewWindowPolicy build_preview_window_policy(const double now_s,
                                                        const double anchor_time_s,
                                                        const double visual_window_hint_s,
                                                        const double anchored_visual_window_hint_s,
                                                        const double coverage_window_s,
                                                        const bool drag_active)
        {
            PreviewWindowPolicy policy{};
            policy.visual_window_s =
                    std::max({visual_window_hint_s, anchored_visual_window_hint_s, coverage_window_s});
            policy.exact_window_s = policy.visual_window_s;
            if (drag_active)
            {
                policy.exact_window_s = std::min(policy.exact_window_s,
                                                 OrbitPredictionTuning::kDragInteractivePreviewWindowMaxS);
            }
            policy.pick_window_s = policy.exact_window_s;
            policy.request_window_s =
                    std::max(0.0,
                             (anchor_time_s - now_s) + std::max(policy.visual_window_s, policy.exact_window_s));
            return policy;
        }

        double resolve_interactive_prediction_reference_time_s(
                const double sim_time_s,
                const GameplayState::ManeuverGizmoInteraction &interaction)
        {
            if (PredictionRuntimeDetail::maneuver_drag_active(interaction.state) &&
                std::isfinite(interaction.drag_display_reference_time_s))
            {
                return std::min(sim_time_s, interaction.drag_display_reference_time_s);
            }
            return sim_time_s;
        }

        bool preview_anchor_matches(const PreviewAnchorCache &a, const PreviewAnchorCache &b)
        {
            return a.valid == b.valid &&
                   a.anchor_node_id == b.anchor_node_id &&
                   a.anchor_time_s == b.anchor_time_s &&
                   a.baseline_generation_id == b.baseline_generation_id &&
                   a.upstream_maneuver_hash == b.upstream_maneuver_hash &&
                   a.display_frame_key == b.display_frame_key &&
                   a.display_frame_revision == b.display_frame_revision &&
                   a.visual_window_s == b.visual_window_s &&
                   a.exact_window_s == b.exact_window_s &&
                   a.pick_window_s == b.pick_window_s &&
                   a.downstream_maneuver_node_ids == b.downstream_maneuver_node_ids;
        }
    } // namespace

    bool GameplayState::maneuver_fast_preview_active(const bool with_maneuvers) const
    {
        return _prediction_fast_preview_enabled &&
               PredictionRuntimeDetail::maneuver_live_preview(
                       with_maneuvers,
                       _maneuver_plan_live_preview_active,
                       _maneuver_gizmo_interaction.state);
    }

    void GameplayState::sync_prediction_dirty_flag()
    {
        // Collapse only visible-track rebuild demand into one cheap UI-facing flag.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        _prediction_dirty = false;
        for (const PredictionTrackState &track : _prediction_tracks)
        {
            if (!PredictionRuntimeDetail::contains_key(visible_subjects, track.key))
            {
                continue;
            }

            if (track.dirty)
            {
                _prediction_dirty = true;
                return;
            }
        }
    }

    void GameplayState::mark_prediction_dirty()
    {
        // Force only the active/overlay-visible tracks to rebuild on the next prediction update.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        for (PredictionTrackState &track : _prediction_tracks)
        {
            if (!PredictionRuntimeDetail::contains_key(visible_subjects, track.key))
            {
                continue;
            }

            if (track.request_pending)
            {
                track.invalidated_while_pending = true;
                continue;
            }

            track.dirty = true;
        }
        sync_prediction_dirty_flag();
    }

    void GameplayState::mark_maneuver_plan_dirty()
    {
        // Maneuver edits should live-update until the latest authored plan finishes solving once.
        _maneuver_plan_live_preview_active = _prediction_fast_preview_enabled;
        mark_prediction_dirty();
    }

    void GameplayState::refresh_prediction_preview_anchor(PredictionTrackState &track,
                                                          const double now_s,
                                                          const bool with_maneuvers)
    {
        const bool preview_track =
                with_maneuvers &&
                track.key == _prediction_selection.active_subject;
        const bool preview_live = preview_track && maneuver_fast_preview_active(with_maneuvers);
        const bool drag_active =
                PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state);

        if (!preview_track)
        {
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
            return;
        }

        if (!preview_live)
        {
            if (track.preview_state == PredictionPreviewRuntimeState::EnterDrag ||
                track.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
                track.preview_state == PredictionPreviewRuntimeState::PreviewStreaming)
            {
                track.preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
            }
            return;
        }

        const ManeuverNode *anchor_node = select_preview_anchor_node(_maneuver_state, now_s);
        if (!anchor_node || !std::isfinite(anchor_node->time_s))
        {
            track.preview_state = PredictionPreviewRuntimeState::Idle;
            track.preview_anchor.clear();
            return;
        }

        PreviewAnchorCache refreshed{};
        refreshed.valid = true;
        refreshed.anchor_node_id = anchor_node->id;
        refreshed.anchor_time_s = std::max(now_s, anchor_node->time_s);
        refreshed.baseline_generation_id = track.cache.generation_id;
        refreshed.upstream_maneuver_hash = hash_upstream_maneuvers(
                _maneuver_state,
                refreshed.anchor_node_id,
                refreshed.anchor_time_s);
        refreshed.gizmo_basis_snapshot = snapshot_gizmo_basis(_maneuver_gizmo_interaction);
        refreshed.display_frame_snapshot =
                (track.cache.valid && track.cache.trajectory_inertial.size() >= 2)
                        ? resolve_prediction_display_frame_spec(track.cache, now_s)
                        : _prediction_frame_selection.spec;
        refreshed.display_frame_key = prediction_display_frame_key(refreshed.display_frame_snapshot);
        refreshed.display_frame_revision = _prediction_display_frame_revision;
        const double display_window_s = prediction_display_window_s(track.key, now_s, true);
        const double anchored_display_window_s =
                std::max(0.0, display_window_s - std::max(0.0, refreshed.anchor_time_s - now_s));
        const PreviewWindowPolicy window_policy = build_preview_window_policy(now_s,
                                                                              refreshed.anchor_time_s,
                                                                              maneuver_plan_preview_window_s(),
                                                                              anchored_display_window_s,
                                                                              maneuver_post_node_coverage_s(),
                                                                              drag_active);
        refreshed.visual_window_s = window_policy.visual_window_s;
        refreshed.exact_window_s = window_policy.exact_window_s;
        refreshed.pick_window_s = window_policy.pick_window_s;
        refreshed.request_window_s = window_policy.request_window_s;
        refreshed.downstream_maneuver_node_ids.reserve(_maneuver_state.nodes.size());
        const double request_end_s =
                refreshed.anchor_time_s + std::max(refreshed.visual_window_s, refreshed.exact_window_s);
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s) ||
                node.time_s < refreshed.anchor_time_s ||
                node.time_s > request_end_s)
            {
                continue;
            }
            refreshed.downstream_maneuver_node_ids.push_back(node.id);
        }

        if (!track.cache.trajectory_inertial.empty())
        {
            orbitsim::State anchor_state{};
            if (sample_prediction_inertial_state(track.cache.trajectory_inertial, refreshed.anchor_time_s, anchor_state))
            {
                refreshed.anchor_state_inertial = anchor_state;
            }
        }

        const bool anchor_changed = !preview_anchor_matches(track.preview_anchor, refreshed);
        track.preview_anchor = std::move(refreshed);
        if (anchor_changed)
        {
            track.preview_last_anchor_refresh_at_s = now_s;
            if (!std::isfinite(track.preview_entered_at_s))
            {
                track.preview_entered_at_s = now_s;
            }

            if (track.preview_state == PredictionPreviewRuntimeState::DragPreviewPending ||
                track.preview_state == PredictionPreviewRuntimeState::PreviewStreaming)
            {
                track.invalidated_while_pending = track.request_pending || track.derived_request_pending;
            }
            track.preview_state = PredictionPreviewRuntimeState::EnterDrag;
            return;
        }

        if (track.preview_state == PredictionPreviewRuntimeState::Idle ||
            track.preview_state == PredictionPreviewRuntimeState::AwaitFullRefine)
        {
            if (!std::isfinite(track.preview_entered_at_s))
            {
                track.preview_entered_at_s = now_s;
            }
            track.preview_state = PredictionPreviewRuntimeState::EnterDrag;
        }
    }

    void GameplayState::clear_prediction_runtime()
    {
        // Drop every cached artifact when the feature is disabled.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.clear_runtime();
            track.dirty = false;
        }
        _prediction_service.reset();
        _prediction_derived_service.reset();
        _prediction_dirty = false;
        _maneuver_plan_live_preview_active = false;
    }

    void GameplayState::clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        // A service reset invalidates every track, even ones that are currently hidden.
        (void) visible_subjects;
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.clear_runtime();
        }
        _prediction_derived_service.reset();
        _maneuver_plan_live_preview_active = false;
    }

    double GameplayState::prediction_display_window_s(const PredictionSubjectKey key,
                                                      const double now_s,
                                                      const bool with_maneuvers) const
    {
        const double plotted_ahead_s =
                std::max(0.0, _prediction_draw_future_segment ? prediction_future_window_s(key) : 0.0);

        if (!with_maneuvers)
        {
            return plotted_ahead_s;
        }

        double required_ahead_s = plotted_ahead_s;
        double max_node_time_s = now_s;
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s))
            {
                continue;
            }
            max_node_time_s = std::max(max_node_time_s, node.time_s);
        }

        if (max_node_time_s > now_s)
        {
            required_ahead_s = std::max(required_ahead_s, (max_node_time_s - now_s) + maneuver_post_node_coverage_s());
        }

        return required_ahead_s;
    }

    double GameplayState::prediction_preview_exact_window_s(const PredictionTrackState &track,
                                                            const double now_s,
                                                            const bool with_maneuvers) const
    {
        (void) now_s;
        const bool preview_live =
                track.key == _prediction_selection.active_subject &&
                maneuver_fast_preview_active(with_maneuvers);
        if (!preview_live || !track.preview_anchor.valid)
        {
            return 0.0;
        }
        return std::max(0.0, track.preview_anchor.exact_window_s);
    }

    double GameplayState::prediction_required_window_s(const PredictionTrackState &track,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        const double display_window_s = prediction_display_window_s(track.key, now_s, with_maneuvers);
        const bool preview_live =
                track.key == _prediction_selection.active_subject &&
                maneuver_fast_preview_active(with_maneuvers);
        if (!preview_live || !track.preview_anchor.valid)
        {
            return display_window_s;
        }

        if (PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state))
        {
            return std::max(prediction_future_window_s(track.key), std::max(0.0, track.preview_anchor.request_window_s));
        }

        return std::max(display_window_s, std::max(0.0, track.preview_anchor.request_window_s));
    }

    double GameplayState::prediction_required_window_s(const PredictionSubjectKey key,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        if (const PredictionTrackState *track = find_prediction_track(key))
        {
            return prediction_required_window_s(*track, now_s, with_maneuvers);
        }

        const double display_window_s = prediction_display_window_s(key, now_s, with_maneuvers);
        const bool preview_subject_matches =
                !_prediction_selection.active_subject.valid() ||
                key == _prediction_selection.active_subject;
        const bool maneuver_live_preview =
                preview_subject_matches &&
                maneuver_fast_preview_active(with_maneuvers);
        if (!maneuver_live_preview)
        {
            return display_window_s;
        }

        const ManeuverNode *anchor_node = select_preview_anchor_node(_maneuver_state, now_s);
        if (!anchor_node || !std::isfinite(anchor_node->time_s))
        {
            return display_window_s;
        }

        const double anchor_time_s = std::max(now_s, anchor_node->time_s);
        const double anchored_display_window_s =
                std::max(0.0, display_window_s - std::max(0.0, anchor_time_s - now_s));
        const PreviewWindowPolicy window_policy = build_preview_window_policy(now_s,
                                                                              anchor_time_s,
                                                                              maneuver_plan_preview_window_s(),
                                                                              anchored_display_window_s,
                                                                              maneuver_post_node_coverage_s(),
                                                                              PredictionRuntimeDetail::maneuver_drag_active(
                                                                                      _maneuver_gizmo_interaction.state));
        return window_policy.request_window_s;
    }

    bool GameplayState::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                        const double now_s,
                                                        const float fixed_dt,
                                                        const bool thrusting,
                                                        const bool with_maneuvers) const
    {
        // Rebuild when cache state, thrusting, timing, or horizon coverage says we must.
        const bool maneuver_live_preview = maneuver_fast_preview_active(with_maneuvers);
        bool rebuild = track.dirty || !track.cache.valid;
        if (!rebuild && track.preview_state == PredictionPreviewRuntimeState::EnterDrag)
        {
            rebuild = true;
        }
        if (!rebuild &&
            track.request_pending &&
            maneuver_live_preview &&
            (track.invalidated_while_pending ||
             track.pending_solve_quality != OrbitPredictionService::SolveQuality::FastPreview))
        {
            rebuild = true;
        }

        if (!rebuild && thrusting)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_thrust_refresh_s;
        }

        if (!rebuild && maneuver_live_preview)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= OrbitPredictionTuning::kManeuverRefreshS;
        }

        if (!rebuild && _prediction_periodic_refresh_s > 0.0 && (!with_maneuvers || maneuver_live_preview))
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_periodic_refresh_s;
        }

        // Throttle live gizmo drags so the async solver does not thrash every tick.
        if (rebuild &&
            track.cache.valid &&
            maneuver_live_preview &&
            PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state))
        {
            const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
            if (PredictionDragDebugTelemetry::has_time(track.drag_debug.last_preview_request_tp))
            {
                const double dt_since_request_s =
                        std::chrono::duration<double>(now_tp - track.drag_debug.last_preview_request_tp).count();
                if (dt_since_request_s < OrbitPredictionTuning::kDragRebuildMinIntervalS)
                {
                    rebuild = false;
                }
            }
        }

        if (rebuild || !track.cache.valid || track.cache.trajectory_inertial.empty())
        {
            return rebuild;
        }

        double cache_end_s = track.cache.trajectory_inertial.back().t_s;
        if (!track.cache.trajectory_segments_inertial.empty())
        {
            const orbitsim::TrajectorySegment &last_segment = track.cache.trajectory_segments_inertial.back();
            const double segment_end_s = last_segment.t0_s + last_segment.dt_s;
            if (std::isfinite(segment_end_s))
            {
                cache_end_s = segment_end_s;
            }
        }

        const double required_ahead_s = prediction_required_window_s(track, now_s, with_maneuvers);

        // Add a small epsilon so tiny fixed-step jitter does not trigger rebuild churn.
        const double coverage_epsilon_s =
                std::max(1.0e-3, std::min(0.25, std::max(0.0, static_cast<double>(fixed_dt)) * 0.5));
        return (cache_end_s - now_s + coverage_epsilon_s) < required_ahead_s;
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        if (!_prediction_enabled)
        {
            // Fully disable runtime state when the overlay is turned off.
            clear_prediction_runtime();
            return;
        }

        // Keep the subject list aligned with the current gameplay scene.
        rebuild_prediction_subjects();
        rebuild_prediction_frame_options();
        rebuild_prediction_analysis_options();

        const double now_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
        const double interactive_reference_time_s =
                resolve_interactive_prediction_reference_time_s(now_s, _maneuver_gizmo_interaction);

        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        if (!_orbitsim)
        {
            // Without orbit-sim there is no stable frame to predict against.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction_service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            // Bail out when the reference body is not ready for a stable inertial frame.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction_service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        // Rebuild or refresh only the subjects that are actually visible.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            if (!PredictionRuntimeDetail::contains_key(visible_subjects, track.key))
            {
                continue;
            }

            const bool with_maneuvers =
                    prediction_subject_supports_maneuvers(track.key) &&
                    _maneuver_nodes_enabled &&
                    !_maneuver_state.nodes.empty();
            const bool thrusting = prediction_subject_thrust_applied_this_tick(track.key);
            const double track_reference_time_s = with_maneuvers ? interactive_reference_time_s : now_s;
            refresh_prediction_preview_anchor(track, track_reference_time_s, with_maneuvers);
            const bool rebuild =
                    should_rebuild_prediction_track(track, track_reference_time_s, fixed_dt, thrusting, with_maneuvers);
            if (!rebuild)
            {
                continue;
            }

            if (track.key.kind == PredictionSubjectKind::Celestial)
            {
                update_celestial_prediction_track(track, now_s);
                continue;
            }

            update_orbiter_prediction_track(track, track_reference_time_s, thrusting, with_maneuvers);
        }

        // Mirror the active track's last solver time into the shared debug HUD stats.
        PredictionTrackState *active_track = active_prediction_track();
        _orbit_plot_perf.solver_ms_last = active_track ? active_track->solver_ms_last : 0.0;
        sync_prediction_dirty_flag();
    }
} // namespace Game
