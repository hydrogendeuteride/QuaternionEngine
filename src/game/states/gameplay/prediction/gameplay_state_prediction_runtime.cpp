#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_tuning.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    using detail::finite_vec3;

    namespace
    {
        template<typename T>
        bool contains_key(const std::vector<T> &items, const T &needle)
        {
            return std::find(items.begin(), items.end(), needle) != items.end();
        }

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
    } // namespace

    void GameplayState::sync_prediction_dirty_flag()
    {
        // Collapse only visible-track rebuild demand into one cheap UI-facing flag.
        const std::vector<PredictionSubjectKey> visible_subjects = collect_visible_prediction_subjects();
        _prediction_dirty = false;
        for (const PredictionTrackState &track : _prediction_tracks)
        {
            if (!contains_key(visible_subjects, track.key))
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
            if (!contains_key(visible_subjects, track.key))
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
        _maneuver_plan_live_preview_active = true;
        mark_prediction_dirty();
    }

    void GameplayState::poll_completed_prediction_results()
    {
        bool applied_result = false;
        while (auto completed = _prediction_service.poll_completed())
        {
            apply_completed_prediction_result(std::move(*completed));
            applied_result = true;
        }

        while (auto completed = _prediction_derived_service.poll_completed())
        {
            apply_completed_prediction_derived_result(std::move(*completed));
            applied_result = true;
        }

        if (applied_result)
        {
            sync_prediction_dirty_flag();
        }
    }

    void GameplayState::clear_prediction_runtime()
    {
        // Drop every cached artifact when the feature is disabled.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            track.cache.clear();
            track.request_pending = false;
            track.derived_request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track.dirty = false;
            track.invalidated_while_pending = false;
            track.auto_primary_body_id = orbitsim::kInvalidBodyId;
            track.solver_ms_last = 0.0;
            track.solver_diagnostics = {};
            track.derived_diagnostics = {};
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

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        // Queue heavy frame/metrics derivation off-thread, then swap the completed cache on the main thread.
        PredictionTrackState *track = nullptr;
        for (PredictionTrackState &candidate : _prediction_tracks)
        {
            if (candidate.key.track_id() == result.track_id)
            {
                track = &candidate;
                break;
            }
        }

        if (!track)
        {
            return;
        }

        track->solver_ms_last = std::max(0.0, result.compute_time_ms);
        track->solver_diagnostics = result.diagnostics;
        track->derived_diagnostics = {};

        if (!result.valid || result.trajectory_inertial.size() < 2)
        {
            track->request_pending = false;
            track->derived_request_pending = false;
            track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track->dirty = true;
            return;
        }

        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0};
        glm::vec3 build_vel_local{0.0f};
        (void) get_prediction_subject_world_state(track->key, build_pos_world, build_vel_world, build_vel_local);

        OrbitPredictionCache resolve_cache{};
        resolve_cache.build_time_s = result.build_time_s;
        resolve_cache.shared_ephemeris = result.shared_ephemeris;
        resolve_cache.massive_bodies = result.massive_bodies;
        resolve_cache.trajectory_segments_inertial = result.trajectory_segments_inertial;
        const double reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : result.build_time_s;
        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(resolve_cache, reference_time_s);

        resolve_cache.trajectory_inertial = result.trajectory_inertial;
        orbitsim::BodyId analysis_body_id =
                resolve_prediction_analysis_body_id(resolve_cache,
                                                   track->key,
                                                   reference_time_s,
                                                   track->auto_primary_body_id);
        if (analysis_body_id != orbitsim::kInvalidBodyId)
        {
            track->auto_primary_body_id = analysis_body_id;
        }

        std::vector<orbitsim::TrajectorySegment> player_lookup_segments;
        if (prediction_subject_is_player(track->key))
        {
            if (!result.trajectory_segments_inertial_planned.empty())
            {
                player_lookup_segments = result.trajectory_segments_inertial_planned;
            }
            else if (!result.trajectory_segments_inertial.empty())
            {
                player_lookup_segments = result.trajectory_segments_inertial;
            }
        }
        else if (const PredictionTrackState *player_track = player_prediction_track())
        {
            if (!player_track->cache.trajectory_segments_inertial_planned.empty())
            {
                player_lookup_segments = player_track->cache.trajectory_segments_inertial_planned;
            }
            else if (!player_track->cache.trajectory_segments_inertial.empty())
            {
                player_lookup_segments = player_track->cache.trajectory_segments_inertial;
            }
        }

        OrbitPredictionDerivedService::Request derived_request{};
        derived_request.track_id = result.track_id;
        derived_request.generation_id = result.generation_id;
        derived_request.solver_result = std::move(result);
        derived_request.build_pos_world = build_pos_world;
        derived_request.build_vel_world = build_vel_world;
        derived_request.sim_config = _orbitsim ? _orbitsim->sim.config() : orbitsim::GameSimulation::Config{};
        derived_request.resolved_frame_spec = resolved_frame_spec;
        derived_request.analysis_body_id = analysis_body_id;
        derived_request.player_lookup_segments_inertial = std::move(player_lookup_segments);
        _prediction_derived_service.request(std::move(derived_request));
        // Let the solver queue accept a fresher generation while derived work finishes in parallel.
        track->request_pending = false;
        track->derived_request_pending = true;
        track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
    }

    void GameplayState::apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result)
    {
        PredictionTrackState *track = nullptr;
        for (PredictionTrackState &candidate : _prediction_tracks)
        {
            if (candidate.key.track_id() == result.track_id)
            {
                track = &candidate;
                break;
            }
        }

        if (!track)
        {
            return;
        }

        track->derived_request_pending = false;
        track->request_pending = false;
        track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        track->derived_diagnostics = result.diagnostics;
        const bool keep_dirty_for_followup = track->invalidated_while_pending;
        track->invalidated_while_pending = false;

        if (!result.valid || !result.cache.valid || result.cache.trajectory_frame.size() < 2)
        {
            track->dirty = true;
            return;
        }

        track->cache = std::move(result.cache);
        track->pick_cache.clear();
        const bool preview_result = result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
        const bool maneuver_preview_subject =
                prediction_subject_supports_maneuvers(track->key) &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty();
        const bool interaction_idle =
                _maneuver_gizmo_interaction.state != ManeuverGizmoInteraction::State::DragAxis;
        const bool schedule_full_refine =
                preview_result &&
                maneuver_preview_subject &&
                _maneuver_plan_live_preview_active &&
                interaction_idle &&
                !keep_dirty_for_followup;
        track->dirty = keep_dirty_for_followup || schedule_full_refine;

        const bool freeze_maneuver_plan =
                maneuver_preview_subject &&
                _maneuver_plan_live_preview_active &&
                interaction_idle &&
                !preview_result &&
                !keep_dirty_for_followup;
        if (schedule_full_refine)
        {
            // Publish a cheap preview first, then immediately fall through to a full rebuild.
            _maneuver_plan_live_preview_active = false;
        }
        if (freeze_maneuver_plan)
        {
            _maneuver_plan_live_preview_active = false;
        }
    }

    double GameplayState::prediction_required_window_s(const PredictionSubjectKey key,
                                                       const double now_s,
                                                       const bool with_maneuvers) const
    {
        double required_ahead_s =
                std::max(0.0, _prediction_draw_future_segment ? prediction_future_window_s(key) : 0.0);

        if (!with_maneuvers)
        {
            return required_ahead_s;
        }

        const bool maneuver_live_preview =
                _maneuver_plan_live_preview_active ||
                _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis;
        if (maneuver_live_preview)
        {
            const ManeuverNode *anchor_node = select_preview_anchor_node(_maneuver_state, now_s);
            const double anchor_time_s =
                    (anchor_node && std::isfinite(anchor_node->time_s)) ? std::max(now_s, anchor_node->time_s) : now_s;
            const double preview_window_s = std::max(maneuver_plan_preview_window_s(), maneuver_post_node_coverage_s());
            required_ahead_s = std::max(required_ahead_s, (anchor_time_s - now_s) + preview_window_s);
            return required_ahead_s;
        }

        double max_node_time_s = now_s;
        for (const ManeuverNode &node : _maneuver_state.nodes)
        {
            if (std::isfinite(node.time_s))
            {
                max_node_time_s = std::max(max_node_time_s, node.time_s);
            }
        }

        if (max_node_time_s > now_s)
        {
            const double post_node_window_s = maneuver_post_node_coverage_s();
            required_ahead_s = std::max(required_ahead_s, (max_node_time_s - now_s) + post_node_window_s);
        }

        return required_ahead_s;
    }

    bool GameplayState::should_rebuild_prediction_track(const PredictionTrackState &track,
                                                        const double now_s,
                                                        const float fixed_dt,
                                                        const bool thrusting,
                                                        const bool with_maneuvers) const
    {
        // Rebuild when cache state, thrusting, timing, or horizon coverage says we must.
        const bool maneuver_live_preview =
                with_maneuvers &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        bool rebuild = track.dirty || !track.cache.valid;
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
            _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis)
        {
            const double dt_since_build_s = now_s - track.cache.build_time_s;
            if (dt_since_build_s < OrbitPredictionTuning::kDragRebuildMinIntervalS)
            {
                rebuild = false;
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

        const double required_ahead_s = prediction_required_window_s(track.key, now_s, with_maneuvers);

        // Add a small epsilon so tiny fixed-step jitter does not trigger rebuild churn.
        const double coverage_epsilon_s =
                std::max(1.0e-3, std::min(0.25, std::max(0.0, static_cast<double>(fixed_dt)) * 0.5));
        return (cache_end_s - now_s + coverage_epsilon_s) < required_ahead_s;
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers)
    {
        // Package the current spacecraft state into a worker request.
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *ref_sim = _orbitsim->world_reference_sim_body();
        if (!ref_sim || ref_sim->id == orbitsim::kInvalidBodyId)
        {
            return false;
        }

        const WorldVec3 ref_body_world = prediction_world_reference_body_world();
        const glm::dvec3 ship_rel_pos_m = glm::dvec3(subject_pos_world - ref_body_world);
        const glm::dvec3 ship_rel_vel_mps = subject_vel_world;
        const glm::dvec3 ship_bary_pos_m = ref_sim->state.position_m + ship_rel_pos_m;
        const glm::dvec3 ship_bary_vel_mps = ref_sim->state.velocity_mps + ship_rel_vel_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.shared_ephemeris = track.cache.shared_ephemeris;
        request.ship_bary_position_m = ship_bary_pos_m;
        request.ship_bary_velocity_mps = ship_bary_vel_mps;
        request.thrusting = thrusting;
        const bool maneuver_live_preview =
                with_maneuvers &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        request.solve_quality = maneuver_live_preview
                                        ? OrbitPredictionService::SolveQuality::FastPreview
                                        : OrbitPredictionService::SolveQuality::Full;
        request.future_window_s = prediction_required_window_s(track.key, now_s, with_maneuvers);
        const orbitsim::TrajectoryFrameSpec display_frame_spec =
                track.cache.resolved_frame_spec_valid ? track.cache.resolved_frame_spec : _prediction_frame_selection.spec;
        request.lagrange_sensitive = prediction_frame_is_lagrange_sensitive(display_frame_spec);
        request.preferred_primary_body_id = track.auto_primary_body_id;
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            _prediction_analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction_analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = _prediction_analysis_selection.spec.fixed_body_id;
        }
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            display_frame_spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = display_frame_spec.primary_body_id;
        }

        // Copy currently authored maneuver nodes so the worker can include planned burns.
        if (with_maneuvers)
        {
            request.maneuver_impulses.reserve(_maneuver_state.nodes.size());
            const double preview_horizon_end_s = now_s + request.future_window_s;
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s))
                {
                    continue;
                }

                if (request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                    node.time_s > preview_horizon_end_s)
                {
                    continue;
                }

                OrbitPredictionService::ManeuverImpulse impulse{};
                impulse.node_id = node.id;
                impulse.t_s = node.time_s;
                // For auto-primary nodes, let the worker resolve the dominant body at the node time
                // from the propagated state instead of baking in a stale cache-based guess here.
                impulse.primary_body_id = node.primary_body_auto
                                                  ? orbitsim::kInvalidBodyId
                                                  : resolve_maneuver_node_primary_body_id(node, node.time_s);
                impulse.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                request.maneuver_impulses.push_back(impulse);
            }
        }

        _prediction_service.request(std::move(request));
        track.request_pending = true;
        track.derived_request_pending = false;
        track.pending_solve_quality = maneuver_live_preview
                                              ? OrbitPredictionService::SolveQuality::FastPreview
                                              : OrbitPredictionService::SolveQuality::Full;
        track.invalidated_while_pending = false;
        return true;
    }

    bool GameplayState::request_celestial_prediction_async(PredictionTrackState &track, const double now_s)
    {
        if (!_orbitsim)
        {
            return false;
        }

        const orbitsim::MassiveBody *body = _orbitsim->sim.body_by_id(static_cast<orbitsim::BodyId>(track.key.value));
        if (!body)
        {
            return false;
        }

        OrbitPredictionService::Request request{};
        request.kind = OrbitPredictionService::RequestKind::Celestial;
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.shared_ephemeris = track.cache.shared_ephemeris;
        request.subject_body_id = body->id;
        request.future_window_s = prediction_future_window_s(track.key);
        request.lagrange_sensitive = prediction_frame_is_lagrange_sensitive(_prediction_frame_selection.spec);
        if (_prediction_analysis_selection.spec.mode == PredictionAnalysisMode::FixedBodyBCI &&
            _prediction_analysis_selection.spec.fixed_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = _prediction_analysis_selection.spec.fixed_body_id;
        }
        if (request.preferred_primary_body_id == orbitsim::kInvalidBodyId &&
            _prediction_frame_selection.spec.primary_body_id != orbitsim::kInvalidBodyId)
        {
            request.preferred_primary_body_id = _prediction_frame_selection.spec.primary_body_id;
        }

        // Celestial tracks now flow through the same worker queue as spacecraft tracks.
        _prediction_service.request(std::move(request));
        track.request_pending = true;
        track.derived_request_pending = false;
        track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        track.invalidated_while_pending = false;
        return true;
    }

    void GameplayState::update_orbiter_prediction_track(PredictionTrackState &track,
                                                        const double now_s,
                                                        const bool thrusting,
                                                        const bool with_maneuvers)
    {
        // Orbiters rebuild asynchronously from their current world-space snapshot.
        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 subject_vel_world{0.0};
        glm::vec3 subject_vel_local{0.0f};
        if (!get_prediction_subject_world_state(track.key, subject_pos_world, subject_vel_world, subject_vel_local))
        {
            track.cache.clear();
            track.dirty = true;
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            return;
        }

        const bool maneuver_live_preview =
                with_maneuvers &&
                (_maneuver_plan_live_preview_active ||
                 _maneuver_gizmo_interaction.state == ManeuverGizmoInteraction::State::DragAxis);
        const bool supersede_preview_request =
                track.request_pending &&
                maneuver_live_preview &&
                (track.invalidated_while_pending ||
                 track.pending_solve_quality != OrbitPredictionService::SolveQuality::FastPreview);
        if (track.request_pending && !supersede_preview_request)
        {
            return;
        }

        const bool requested =
                request_orbiter_prediction_async(track, subject_pos_world, subject_vel_world, now_s, thrusting, with_maneuvers);
        track.dirty = !requested;
        if (!requested)
        {
            track.cache.clear();
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        }
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        // Celestial predictions rebuild asynchronously so the gameplay thread only packages requests.
        if (!_orbitsim)
        {
            track.cache.clear();
            track.dirty = true;
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            return;
        }

        if (track.request_pending)
        {
            return;
        }

        const bool requested = request_celestial_prediction_async(track, now_s);
        // Keep drawing the previous cache until the worker publishes a replacement.
        track.dirty = !requested;
        if (!requested)
        {
            track.cache.clear();
            track.request_pending = false;
            track.pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
        }
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
            if (!contains_key(visible_subjects, track.key))
            {
                continue;
            }

            const bool with_maneuvers =
                    prediction_subject_supports_maneuvers(track.key) &&
                    _maneuver_nodes_enabled &&
                    !_maneuver_state.nodes.empty();
            const bool thrusting = prediction_subject_thrust_applied_this_tick(track.key);
            const bool rebuild = should_rebuild_prediction_track(track, now_s, fixed_dt, thrusting, with_maneuvers);
            if (!rebuild)
            {
                continue;
            }

            if (track.key.kind == PredictionSubjectKind::Celestial)
            {
                update_celestial_prediction_track(track, now_s);
                continue;
            }

            update_orbiter_prediction_track(track, now_s, thrusting, with_maneuvers);
        }

        // Mirror the active track's last solver time into the shared debug HUD stats.
        PredictionTrackState *active_track = active_prediction_track();
        _orbit_plot_perf.solver_ms_last = active_track ? active_track->solver_ms_last : 0.0;
        sync_prediction_dirty_flag();
    }

} // namespace Game
