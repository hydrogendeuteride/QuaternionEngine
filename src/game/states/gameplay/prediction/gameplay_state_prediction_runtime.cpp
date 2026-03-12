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
    } // namespace

    void GameplayState::sync_prediction_dirty_flag()
    {
        // Collapse only actual rebuild demand into one cheap UI-facing flag.
        _prediction_dirty = false;
        for (const PredictionTrackState &track : _prediction_tracks)
        {
            if (track.dirty)
            {
                _prediction_dirty = true;
                return;
            }
        }
    }

    void GameplayState::mark_prediction_dirty()
    {
        // Force every cached track to rebuild on the next prediction update.
        for (PredictionTrackState &track : _prediction_tracks)
        {
            if (track.request_pending)
            {
                track.invalidated_while_pending = true;
                continue;
            }

            track.dirty = true;
        }
        sync_prediction_dirty_flag();
    }

    void GameplayState::poll_completed_prediction_results()
    {
        bool applied_result = false;
        while (auto completed = _prediction_service.poll_completed())
        {
            apply_completed_prediction_result(std::move(*completed));
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
            track.dirty = false;
            track.invalidated_while_pending = false;
        }
        _prediction_service.reset();
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
    }

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        // Fold worker output back into the owning track cache.
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

        track->request_pending = false;
        const bool keep_dirty_for_followup = track->invalidated_while_pending;
        track->invalidated_while_pending = false;
        track->solver_ms_last = std::max(0.0, result.compute_time_ms);

        if (!result.valid || result.trajectory_inertial.size() < 2)
        {
            track->cache.clear();
            track->dirty = true;
            return;
        }

        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0};
        glm::vec3 build_vel_local{0.0f};
        (void) get_prediction_subject_world_state(track->key, build_pos_world, build_vel_world, build_vel_local);

        track->cache.clear();
        track->cache.build_time_s = result.build_time_s;
        track->cache.build_pos_world = build_pos_world;
        track->cache.build_vel_world = build_vel_world;
        track->cache.shared_ephemeris = std::move(result.shared_ephemeris);
        track->cache.massive_bodies = std::move(result.massive_bodies);
        track->cache.trajectory_inertial = std::move(result.trajectory_inertial);
        track->cache.trajectory_inertial_planned = std::move(result.trajectory_inertial_planned);
        track->cache.trajectory_segments_inertial = std::move(result.trajectory_segments_inertial);
        track->cache.trajectory_segments_inertial_planned = std::move(result.trajectory_segments_inertial_planned);
        track->cache.maneuver_previews = std::move(result.maneuver_previews);
        track->cache.valid = true;
        refresh_prediction_derived_cache(*track);
        track->dirty = keep_dirty_for_followup;
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

        // Throttle live gizmo drags so the async solver does not thrash every tick.
        if (rebuild &&
            track.cache.valid &&
            with_maneuvers &&
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

        double required_ahead_s = std::max(0.0, _prediction_draw_future_segment ? prediction_future_window_s(track.key) : 0.0);
        if (with_maneuvers)
        {
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
                const double future_window_s = prediction_future_window_s(track.key);
                const double post_node_window_s =
                        std::max(OrbitPredictionTuning::kPostNodeCoverageMinS, future_window_s);
                required_ahead_s = std::max(required_ahead_s, (max_node_time_s - now_s) + post_node_window_s);
            }
        }

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
        request.future_window_s = prediction_future_window_s(track.key);
        request.max_maneuver_time_s = now_s;

        // Copy currently authored maneuver nodes so the worker can include planned burns.
        if (with_maneuvers)
        {
            request.maneuver_impulses.reserve(_maneuver_state.nodes.size());
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s))
                {
                    continue;
                }

                request.max_maneuver_time_s = std::max(request.max_maneuver_time_s, node.time_s);

                OrbitPredictionService::ManeuverImpulse impulse{};
                impulse.node_id = node.id;
                impulse.t_s = node.time_s;
                impulse.primary_body_id = node.primary_body_id;
                impulse.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                request.maneuver_impulses.push_back(impulse);
            }
        }

        _prediction_service.request(std::move(request));
        track.request_pending = true;
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

        // Celestial tracks now flow through the same worker queue as spacecraft tracks.
        _prediction_service.request(std::move(request));
        track.request_pending = true;
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
            return;
        }

        if (track.request_pending)
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
