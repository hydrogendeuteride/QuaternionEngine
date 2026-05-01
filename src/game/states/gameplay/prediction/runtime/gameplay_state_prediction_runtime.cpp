#include "game/states/gameplay/gameplay_state.h"

#include "core/util/logger.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_invalidation_controller.h"
#include "game/states/gameplay/prediction/runtime/prediction_runtime_controller.h"

#include <cmath>
#include <limits>

namespace Game
{
    namespace
    {
        constexpr double kPredictionTimeEpsilonS = 1.0e-6;

        struct PredictionWindowAnchor
        {
            double time_s{std::numeric_limits<double>::quiet_NaN()};
            PredictionTimeAnchorKind kind{PredictionTimeAnchorKind::None};
            bool is_future{false};
        };

        struct AuthoredPlanWindow
        {
            bool valid{false};
            double t0_s{std::numeric_limits<double>::quiet_NaN()};
            double t1_s{std::numeric_limits<double>::quiet_NaN()};
        };

        double resolve_authored_plan_end_s(const PredictionTimeContext &time_ctx,
                                           const double plan_start_time_s,
                                           const double final_node_time_s,
                                           const double plan_horizon_s,
                                           const double post_node_coverage_s)
        {
            if (!std::isfinite(time_ctx.sim_now_s) ||
                !std::isfinite(plan_start_time_s) ||
                !std::isfinite(final_node_time_s) ||
                !(plan_horizon_s > 0.0))
            {
                return std::numeric_limits<double>::quiet_NaN();
            }

            // The authored horizon is a total plan window, anchored at the first
            // future maneuver, not extra tail time appended after the final node.
            // If a later node lies beyond that window, keep just enough post-node
            // coverage to show/drag it instead of adding another full horizon.
            const double horizon_end_s = plan_start_time_s + plan_horizon_s;
            const double post_node_end_s = final_node_time_s + std::max(0.0, post_node_coverage_s);
            return std::max(horizon_end_s, post_node_end_s);
        }

        bool node_time_in_context_range(const PredictionTimeContext &time_ctx, const double node_time_s)
        {
            if (!std::isfinite(node_time_s))
            {
                return false;
            }

            if (std::isfinite(time_ctx.trajectory_t0_s) && node_time_s < (time_ctx.trajectory_t0_s - kPredictionTimeEpsilonS))
            {
                return false;
            }

            if (std::isfinite(time_ctx.trajectory_t1_s) && node_time_s > (time_ctx.trajectory_t1_s + kPredictionTimeEpsilonS))
            {
                return false;
            }

            return true;
        }

        PredictionWindowAnchor make_window_anchor(const PredictionTimeContext &time_ctx,
                                                  const double time_s,
                                                  const PredictionTimeAnchorKind kind)
        {
            PredictionWindowAnchor out{};
            out.time_s = time_s;
            out.kind = kind;
            out.is_future =
                    std::isfinite(time_s) &&
                    std::isfinite(time_ctx.sim_now_s) &&
                    time_s + kPredictionTimeEpsilonS >= time_ctx.sim_now_s;
            return out;
        }

        AuthoredPlanWindow resolve_authored_plan_window(const PredictionTimeContext &time_ctx,
                                                        const double plan_horizon_s,
                                                        const double post_node_coverage_s)
        {
            AuthoredPlanWindow out{};
            if (!(plan_horizon_s > 0.0) ||
                !time_ctx.has_plan ||
                !std::isfinite(time_ctx.sim_now_s) ||
                !std::isfinite(time_ctx.first_future_node_time_s))
            {
                return out;
            }

            out.t0_s = time_ctx.first_future_node_time_s;
            const double final_node_time_s =
                    std::isfinite(time_ctx.last_future_node_time_s)
                            ? time_ctx.last_future_node_time_s
                            : out.t0_s;
            out.t1_s = resolve_authored_plan_end_s(time_ctx,
                                                   out.t0_s,
                                                   final_node_time_s,
                                                   plan_horizon_s,
                                                   post_node_coverage_s);
            out.valid = std::isfinite(out.t1_s) &&
                        out.t1_s > (out.t0_s + kPredictionTimeEpsilonS);
            return out;
        }

        PredictionWindowAnchor select_visual_anchor(const PredictionTimeContext &time_ctx)
        {
            if (std::isfinite(time_ctx.selected_node_time_s) &&
                time_ctx.selected_node_time_s + kPredictionTimeEpsilonS >= time_ctx.sim_now_s)
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.selected_node_time_s,
                                          PredictionTimeAnchorKind::SelectedNode);
            }
            if (std::isfinite(time_ctx.first_future_node_time_s))
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.first_future_node_time_s,
                                          PredictionTimeAnchorKind::FirstFutureNode);
            }
            return make_window_anchor(time_ctx, time_ctx.sim_now_s, PredictionTimeAnchorKind::SimNow);
        }

        PredictionWindowAnchor select_exact_anchor(const PredictionTimeContext &time_ctx)
        {
            if (std::isfinite(time_ctx.selected_node_time_s))
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.selected_node_time_s,
                                          PredictionTimeAnchorKind::SelectedNode);
            }
            if (std::isfinite(time_ctx.first_future_node_time_s))
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.first_future_node_time_s,
                                          PredictionTimeAnchorKind::FirstFutureNode);
            }
            if (std::isfinite(time_ctx.first_relevant_node_time_s))
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.first_relevant_node_time_s,
                                          PredictionTimeAnchorKind::FirstRelevantNode);
            }
            return make_window_anchor(time_ctx, time_ctx.sim_now_s, PredictionTimeAnchorKind::SimNow);
        }

        PredictionWindowAnchor select_pick_anchor(const PredictionTimeContext &time_ctx)
        {
            if (std::isfinite(time_ctx.selected_node_time_s) &&
                time_ctx.selected_node_time_s + kPredictionTimeEpsilonS >= time_ctx.sim_now_s)
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.selected_node_time_s,
                                          PredictionTimeAnchorKind::SelectedNode);
            }
            if (std::isfinite(time_ctx.first_future_node_time_s))
            {
                return make_window_anchor(time_ctx,
                                          time_ctx.first_future_node_time_s,
                                          PredictionTimeAnchorKind::FirstFutureNode);
            }
            return make_window_anchor(time_ctx, time_ctx.sim_now_s, PredictionTimeAnchorKind::SimNow);
        }

        [[nodiscard]] double resolve_live_preview_visual_window_s(const double request_window_s,
                                                                  const double anchor_offset_s,
                                                                  const double configured_preview_window_s)
        {
            const double available_window_s = std::max(0.0, request_window_s - std::max(0.0, anchor_offset_s));
            const double preview_window_s = std::max(0.0, configured_preview_window_s);
            if (preview_window_s <= 0.0)
            {
                return available_window_s;
            }
            return std::min(available_window_s, preview_window_s);
        }

    } // namespace

    void GameplayState::sync_prediction_dirty_flag()
    {
        // Collapse only visible-track rebuild demand into one cheap UI-facing flag.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        _prediction.dirty = PredictionInvalidationController::any_visible_track_dirty(_prediction.tracks,
                                                                                      visible_subjects);
    }

    void GameplayState::mark_prediction_dirty()
    {
        // Force only the active/overlay-visible tracks to rebuild on the next prediction update.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        PredictionInvalidationController::mark_visible_tracks_dirty(_prediction.tracks, visible_subjects);
        sync_prediction_dirty_flag();
    }

    void GameplayState::mark_maneuver_plan_dirty()
    {
        ++_maneuver_plan_revision;
        Logger::debug("Maneuver plan dirty: revision={} selected_node={} node_count={}",
                      _maneuver_plan_revision,
                      _maneuver_state.selected_node_id,
                      _maneuver_state.nodes.size());

        PredictionInvalidationController::invalidate_maneuver_plan_revision(_prediction.tracks,
                                                                           _prediction.service,
                                                                           _prediction.derived_service,
                                                                           _maneuver_plan_revision);

        mark_prediction_dirty();
    }

    void GameplayState::clear_maneuver_prediction_artifacts()
    {
        PredictionInvalidationController::clear_maneuver_prediction_artifacts(_prediction.tracks);
    }

    void GameplayState::clear_prediction_runtime()
    {
        PredictionRuntimeController::clear_runtime(_prediction);
    }

    void GameplayState::clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects)
    {
        PredictionRuntimeController::clear_visible_runtime(_prediction, visible_subjects);
    }

    double GameplayState::prediction_display_window_s(const PredictionSubjectKey key,
                                                      const double now_s,
                                                      const bool with_maneuvers) const
    {
        const PredictionTimeContext time_ctx = build_prediction_time_context(key, now_s);
        const PredictionWindowPolicyResult policy = resolve_prediction_window_policy(find_prediction_track(key), time_ctx, with_maneuvers);
        return policy.valid ? policy.visual_window_s : policy.request_window_s;
    }

    void GameplayState::refresh_prediction_preview_anchor(PredictionTrackState &track,
                                                          const double now_s,
                                                          const bool with_maneuvers) const
    {
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        const bool preview_active =
                track.supports_maneuvers &&
                maneuver_live_preview_active(with_maneuvers);
        const ManeuverNode *selected = _maneuver_state.find_node(active_maneuver_preview_anchor_node_id());

        if (preview_active && selected && std::isfinite(selected->time_s))
        {
            const double request_window_s = prediction_required_window_s(track.key, now_s, with_maneuvers);
            const double anchor_offset_s = std::max(0.0, selected->time_s - now_s);
            const double visual_window_s = resolve_live_preview_visual_window_s(request_window_s,
                                                                                anchor_offset_s,
                                                                                _maneuver_plan_windows.preview_window_s);
            const double exact_window_s = std::max(0.0, _maneuver_plan_windows.solve_margin_s);

            track.preview_anchor.valid = true;
            track.preview_anchor.anchor_node_id = selected->id;
            track.preview_anchor.anchor_time_s = selected->time_s;
            track.preview_anchor.request_window_s = request_window_s;
            track.preview_anchor.visual_window_s = visual_window_s;
            track.preview_anchor.exact_window_s = exact_window_s;
            track.preview_last_anchor_refresh_at_s = now_s;
            if (PredictionRuntimeDetail::prediction_track_can_enter_preview_drag(lifecycle))
            {
                track.preview_state = PredictionPreviewRuntimeState::EnterDrag;
                track.preview_entered_at_s = now_s;
            }
            return;
        }

        if (track.preview_anchor.valid &&
            PredictionRuntimeDetail::prediction_track_can_transition_to_await_full_refine(lifecycle))
        {
            track.preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
            track.preview_last_anchor_refresh_at_s = now_s;
            return;
        }

        if (lifecycle.preview_state == PredictionPreviewRuntimeState::Idle)
        {
            track.preview_anchor = {};
        }
    }

    double GameplayState::prediction_preview_exact_window_s(const PredictionTrackState &track,
                                                            const double /*now_s*/,
                                                            const bool /*with_maneuvers*/) const
    {
        const double configured_exact_window_s = std::max(0.0, _maneuver_plan_windows.solve_margin_s);
        double preview_exact_cap_s = std::max(OrbitPredictionTuning::kPreviewExactWindowMinS,
                                              _maneuver_plan_windows.preview_window_s);
        if (track.preview_anchor.valid && track.preview_anchor.visual_window_s > 0.0)
        {
            // Keep drag previews centered on the immediately visible patch instead of solving the
            // full post-drag refine window on every gizmo move.
            preview_exact_cap_s =
                    std::max(OrbitPredictionTuning::kPreviewExactWindowMinS,
                             track.preview_anchor.visual_window_s);
        }

        return std::min(configured_exact_window_s, preview_exact_cap_s);
    }

    double GameplayState::prediction_planned_exact_window_s(const PredictionTrackState &track,
                                                            const double now_s,
                                                            const bool with_maneuvers) const
    {
        const PredictionTimeContext time_ctx = build_prediction_time_context(track.key, now_s);
        const PredictionWindowPolicyResult policy = resolve_prediction_window_policy(&track, time_ctx, with_maneuvers);
        return policy.exact_window_s;
    }

    double GameplayState::prediction_required_window_s(const PredictionTrackState &track,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        const PredictionTimeContext time_ctx = build_prediction_time_context(track.key, now_s);
        const PredictionWindowPolicyResult policy = resolve_prediction_window_policy(&track, time_ctx, with_maneuvers);
        return policy.request_window_s;
    }

    double GameplayState::prediction_required_window_s(const PredictionSubjectKey key,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        if (const PredictionTrackState *track = find_prediction_track(key))
        {
            return prediction_required_window_s(*track, now_s, with_maneuvers);
        }
        const PredictionTimeContext time_ctx = build_prediction_time_context(key, now_s);
        const PredictionWindowPolicyResult policy = resolve_prediction_window_policy(nullptr, time_ctx, with_maneuvers);
        return policy.request_window_s;
    }

    PredictionTimeContext GameplayState::build_prediction_time_context(const PredictionSubjectKey /*key*/,
                                                                       const double sim_now_s,
                                                                       const double trajectory_t0_s,
                                                                       const double trajectory_t1_s) const
    {
        // key is unused now but reserved for per-subject node filtering (e.g. multi-ship plans).

        PredictionTimeContext out{};
        out.sim_now_s = sim_now_s;
        out.trajectory_t0_s = trajectory_t0_s;
        out.trajectory_t1_s = trajectory_t1_s;

        if (const ManeuverNode *selected = _maneuver_state.find_node(_maneuver_state.selected_node_id))
        {
            if (node_time_in_context_range(out, selected->time_s))
            {
                out.selected_node_time_s = selected->time_s;
            }
        }

        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!node_time_in_context_range(out, node.time_s))
            {
                continue;
            }

            out.has_plan = true;
            out.first_relevant_node_time_s = std::isfinite(out.first_relevant_node_time_s)
                                                     ? std::min(out.first_relevant_node_time_s, node.time_s)
                                                     : node.time_s;

            if (node.time_s + kPredictionTimeEpsilonS >= sim_now_s)
            {
                out.first_future_node_time_s = std::isfinite(out.first_future_node_time_s)
                                                       ? std::min(out.first_future_node_time_s, node.time_s)
                                                       : node.time_s;
                out.last_future_node_time_s = std::isfinite(out.last_future_node_time_s)
                                                      ? std::max(out.last_future_node_time_s, node.time_s)
                                                      : node.time_s;
            }
        }

        return out;
    }

    PredictionWindowPolicyResult GameplayState::resolve_prediction_window_policy(const PredictionTrackState *track,
                                                                                 const PredictionTimeContext &time_ctx,
                                                                                 const bool with_maneuvers) const
    {
        const PredictionSubjectKey key = track ? track->key : PredictionSubjectKey{};

        PredictionWindowPolicyResult out{};
        out.request_window_s =
                std::max(0.0, _prediction.draw_future_segment ? prediction_future_window_s(key) : 0.0);
        const double solve_margin_s = std::max(0.0, _maneuver_plan_windows.solve_margin_s);
        const double post_node_coverage_s = OrbitPredictionTuning::kPostNodeCoverageMinS;
        const bool supports_maneuver_windows = with_maneuvers && time_ctx.has_plan;
        const double plan_horizon_s = maneuver_plan_horizon_s();
        const AuthoredPlanWindow authored_plan_window =
                supports_maneuver_windows
                        ? resolve_authored_plan_window(time_ctx, plan_horizon_s, post_node_coverage_s)
                        : AuthoredPlanWindow{};

        if (std::isfinite(time_ctx.last_future_node_time_s) &&
            std::isfinite(time_ctx.sim_now_s) &&
            time_ctx.last_future_node_time_s > time_ctx.sim_now_s)
        {
            const double required_plan_end_s =
                    authored_plan_window.valid
                            ? authored_plan_window.t1_s
                            : (time_ctx.last_future_node_time_s + post_node_coverage_s);
            out.request_window_s = std::max(
                    out.request_window_s,
                    std::max(0.0, required_plan_end_s - time_ctx.sim_now_s));
        }

        if (!supports_maneuver_windows)
        {
            return out;
        }

        const PredictionWindowAnchor visual_anchor = select_visual_anchor(time_ctx);
        const PredictionWindowAnchor pick_anchor = select_pick_anchor(time_ctx);
        const PredictionWindowAnchor exact_anchor = select_exact_anchor(time_ctx);

        out.visual_window_s = plan_horizon_s;
        out.pick_window_s = plan_horizon_s;
        out.exact_window_s = plan_horizon_s;
        out.visual_anchor_time_s = visual_anchor.time_s;
        out.pick_anchor_time_s = pick_anchor.time_s;
        out.exact_anchor_time_s = exact_anchor.time_s;
        out.visual_anchor_kind = visual_anchor.kind;
        out.pick_anchor_kind = pick_anchor.kind;
        out.exact_anchor_kind = exact_anchor.kind;
        out.visual_anchor_is_future = visual_anchor.is_future;
        out.pick_anchor_is_future = pick_anchor.is_future;
        out.exact_anchor_is_future = exact_anchor.is_future;

        if (maneuver_live_preview_active(with_maneuvers) &&
            std::isfinite(time_ctx.selected_node_time_s) &&
            std::isfinite(time_ctx.sim_now_s))
        {
            const double anchor_offset_s = std::max(0.0, time_ctx.selected_node_time_s - time_ctx.sim_now_s);
            const double preview_exact_window_s =
                    std::min(solve_margin_s,
                             std::max(OrbitPredictionTuning::kPreviewExactWindowMinS,
                                      std::max(0.0, _maneuver_plan_windows.preview_window_s)));
            if (preview_exact_window_s > 0.0)
            {
                out.request_window_s = std::max(out.request_window_s,
                                                anchor_offset_s + (2.0 * preview_exact_window_s));
            }
            const double anchored_visual_window_s = resolve_live_preview_visual_window_s(out.request_window_s,
                                                                                         anchor_offset_s,
                                                                                         _maneuver_plan_windows.preview_window_s);
            const double exact_window_s = std::max(0.0, _maneuver_plan_windows.solve_margin_s);

            out.visual_window_s = anchored_visual_window_s;
            out.pick_window_s = anchored_visual_window_s;
            out.exact_window_s = exact_window_s;
            out.visual_anchor_time_s = time_ctx.selected_node_time_s;
            out.pick_anchor_time_s = time_ctx.selected_node_time_s;
            out.exact_anchor_time_s = time_ctx.selected_node_time_s;
            out.visual_anchor_kind = PredictionTimeAnchorKind::SelectedNode;
            out.pick_anchor_kind = PredictionTimeAnchorKind::SelectedNode;
            out.exact_anchor_kind = PredictionTimeAnchorKind::SelectedNode;
            out.visual_anchor_is_future = true;
            out.pick_anchor_is_future = true;
            out.exact_anchor_is_future = true;
            out.valid =
                    out.visual_window_s > 0.0 &&
                    out.pick_window_s > 0.0 &&
                    out.exact_window_s > 0.0;
            return out;
        }

        if (authored_plan_window.valid)
        {
            out.visual_window_start_time_s = authored_plan_window.t0_s;
            out.visual_window_end_time_s = authored_plan_window.t1_s;
            out.pick_window_start_time_s = authored_plan_window.t0_s;
            out.pick_window_end_time_s = authored_plan_window.t1_s;
        }

        if (std::isfinite(visual_anchor.time_s))
        {
            if (!authored_plan_window.valid && std::isfinite(time_ctx.last_future_node_time_s))
            {
                const double authored_plan_end_s =
                        resolve_authored_plan_end_s(time_ctx,
                                                    std::isfinite(time_ctx.first_future_node_time_s)
                                                            ? time_ctx.first_future_node_time_s
                                                            : visual_anchor.time_s,
                                                    time_ctx.last_future_node_time_s,
                                                    plan_horizon_s,
                                                    post_node_coverage_s);
                const double authored_plan_span_s = std::max(0.0, authored_plan_end_s - visual_anchor.time_s);
                out.visual_window_s = std::max(out.visual_window_s, authored_plan_span_s);
            }

            if (_prediction.draw_full_orbit)
            {
                double orbit_visual_span_s = 0.0;
                if (track &&
                    std::isfinite(track->cache.analysis.orbital_period_s) &&
                    track->cache.analysis.orbital_period_s > 0.0)
                {
                    orbit_visual_span_s =
                            track->cache.analysis.orbital_period_s * OrbitPredictionTuning::kFullOrbitDrawPeriodScale;
                }
                else if (std::isfinite(time_ctx.trajectory_t1_s) && time_ctx.trajectory_t1_s > visual_anchor.time_s)
                {
                    orbit_visual_span_s = time_ctx.trajectory_t1_s - visual_anchor.time_s;
                }

                out.visual_window_s = std::max(out.visual_window_s, orbit_visual_span_s);
            }
        }

        if (std::isfinite(pick_anchor.time_s) &&
            !authored_plan_window.valid &&
            std::isfinite(time_ctx.last_future_node_time_s))
        {
            const double authored_plan_end_s =
                    resolve_authored_plan_end_s(time_ctx,
                                                std::isfinite(time_ctx.first_future_node_time_s)
                                                        ? time_ctx.first_future_node_time_s
                                                        : pick_anchor.time_s,
                                                time_ctx.last_future_node_time_s,
                                                plan_horizon_s,
                                                post_node_coverage_s);
            const double authored_plan_span_s = std::max(0.0, authored_plan_end_s - pick_anchor.time_s);
            out.pick_window_s = std::max(out.pick_window_s, authored_plan_span_s);
        }

        out.valid =
                std::isfinite(out.visual_anchor_time_s) &&
                std::isfinite(out.pick_anchor_time_s) &&
                std::isfinite(out.exact_anchor_time_s) &&
                out.visual_window_s > 0.0 &&
                out.pick_window_s > 0.0 &&
                out.exact_window_s > 0.0;

        return out;
    }

    bool GameplayState::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                        const double now_s,
                                                        const float fixed_dt,
                                                        const bool thrusting,
                                                        const bool with_maneuvers) const
    {
        return PredictionRuntimeController::should_rebuild_track(_prediction,
                                                                 build_prediction_runtime_context(),
                                                                 track,
                                                                 now_s,
                                                                 fixed_dt,
                                                                 thrusting,
                                                                 with_maneuvers);
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        (void) ctx;

        if (!_prediction.enabled)
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
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        if (!_orbitsim)
        {
            // Without orbit-sim there is no stable frame to predict against.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction.service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            // Bail out when the reference body is not ready for a stable inertial frame.
            clear_visible_prediction_runtime(visible_subjects);
            _prediction.service.reset();
            sync_prediction_dirty_flag();
            return;
        }

        PredictionRuntimeController::update_visible_tracks(_prediction,
                                                          build_prediction_runtime_context(),
                                                          visible_subjects,
                                                          now_s,
                                                          fixed_dt);

        // Mirror the active track's last solver time into the shared debug HUD stats.
        PredictionTrackState *active_track = active_prediction_track();
        _prediction.orbit_plot_perf.solver_ms_last = active_track ? active_track->solver_ms_last : 0.0;
        sync_prediction_dirty_flag();
    }
} // namespace Game
