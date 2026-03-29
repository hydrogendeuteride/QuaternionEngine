#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <cmath>
#include <limits>
#include <utility>

namespace Game
{
    namespace
    {
    } // namespace

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
        mark_prediction_dirty();
    }

    void GameplayState::refresh_prediction_preview_anchor(PredictionTrackState &track,
                                                          const double now_s,
                                                          const bool with_maneuvers)
    {
        (void) now_s;
        (void) with_maneuvers;
        track.preview_state = PredictionPreviewRuntimeState::Idle;
        track.preview_anchor.clear();
        track.preview_entered_at_s = std::numeric_limits<double>::quiet_NaN();
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
        if (!with_maneuvers || !prediction_subject_is_player(track.key) || _maneuver_state.nodes.empty())
        {
            return 0.0;
        }

        double last_future_node_time_s = -std::numeric_limits<double>::infinity();
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (!std::isfinite(node.time_s))
            {
                continue;
            }

            if (node.time_s >= now_s)
            {
                last_future_node_time_s = std::max(last_future_node_time_s, node.time_s);
            }
        }

        if (!std::isfinite(last_future_node_time_s))
        {
            return 0.0;
        }

        // Request enough horizon to keep the preview stable across successive node executions.
        // Rendering still anchors to the first future node, but the solve should already cover
        // the final future node plus the requested preview tail.
        return std::max(0.0, last_future_node_time_s - now_s) + maneuver_plan_preview_window_s();
    }

    double GameplayState::prediction_required_window_s(const PredictionTrackState &track,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        const double display_window_s = prediction_display_window_s(track.key, now_s, with_maneuvers);
        const double preview_exact_window_s = prediction_preview_exact_window_s(track, now_s, with_maneuvers);
        return std::max({display_window_s,
                         preview_exact_window_s,
                         std::max(0.0, track.preview_anchor.request_window_s)});
    }

    double GameplayState::prediction_required_window_s(const PredictionSubjectKey key,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        if (const PredictionTrackState *track = find_prediction_track(key))
        {
            return prediction_required_window_s(*track, now_s, with_maneuvers);
        }
        return prediction_display_window_s(key, now_s, with_maneuvers);
    }

    bool GameplayState::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                        const double now_s,
                                                        const float fixed_dt,
                                                        const bool thrusting,
                                                        const bool with_maneuvers) const
    {
        // Rebuild when cache state, thrusting, timing, or horizon coverage says we must.
        bool rebuild = track.dirty || !track.cache.valid;

        if (!rebuild && thrusting)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_thrust_refresh_s;
        }

        if (!rebuild && _prediction_periodic_refresh_s > 0.0)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            rebuild = dt_since_build_s >= _prediction_periodic_refresh_s;
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
            const double track_reference_time_s = now_s;
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
