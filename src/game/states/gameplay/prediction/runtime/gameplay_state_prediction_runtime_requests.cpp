#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <cmath>

namespace Game
{
    using detail::finite_vec3;

    namespace
    {
        OrbitPredictionService::RequestPriority classify_prediction_request_priority(
                const PredictionSelectionState &selection,
                const PredictionSubjectKey key,
                const bool is_celestial,
                const bool interactive)
        {
            if (selection.active_subject == key)
            {
                return interactive
                               ? OrbitPredictionService::RequestPriority::ActiveInteractiveTrack
                               : OrbitPredictionService::RequestPriority::ActiveTrack;
            }

            for (const auto &overlay : selection.overlay_subjects)
            {
                if (overlay == key)
                {
                    return OrbitPredictionService::RequestPriority::Overlay;
                }
            }

            return is_celestial
                           ? OrbitPredictionService::RequestPriority::BackgroundCelestial
                           : OrbitPredictionService::RequestPriority::BackgroundOrbiter;
        }

        void mark_prediction_request_submitted(PredictionTrackState &track,
                                               const uint64_t generation_id,
                                               const double now_s,
                                               const OrbitPredictionService::SolveQuality solve_quality)
        {
            track.latest_requested_generation_id = generation_id;
            track.request_pending = true;
            track.derived_request_pending = false;
            track.pending_solve_quality = solve_quality;
            track.invalidated_while_pending = false;
            track.preview_last_request_at_s = now_s;
        }

        void record_preview_request_submission(PredictionTrackState &track,
                                               const uint64_t generation_id,
                                               const double now_s,
                                               const PredictionDragDebugTelemetry::TimePoint &request_tp)
        {
            PredictionDragDebugTelemetry &debug = track.drag_debug;
            debug.last_preview_request_tp = request_tp;
            debug.last_preview_request_generation_id = generation_id;
            ++debug.preview_request_count;
            if (PredictionDragDebugTelemetry::has_time(debug.last_drag_update_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.drag_to_request_ms_last,
                        debug.drag_to_request_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_drag_update_tp, request_tp));
            }

            track.preview_state = PredictionPreviewRuntimeState::DragPreviewPending;
            if (!std::isfinite(track.preview_entered_at_s))
            {
                track.preview_entered_at_s = now_s;
            }
        }
    } // namespace

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers)
    {
        // Package the current spacecraft state into a worker request.
        refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
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

        const bool maneuver_live_preview = PredictionRuntimeDetail::maneuver_live_preview(
                with_maneuvers,
                _maneuver_plan_live_preview_active,
                _maneuver_gizmo_interaction.state);
        const bool interactive_request =
                track.key == _prediction_selection.active_subject && (thrusting || maneuver_live_preview);
        const OrbitPredictionService::SolveQuality solve_quality =
                maneuver_live_preview
                        ? OrbitPredictionService::SolveQuality::FastPreview
                        : OrbitPredictionService::SolveQuality::Full;

        OrbitPredictionService::Request request{};
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.shared_ephemeris = track.cache.shared_ephemeris;
        request.ship_bary_position_m = ship_bary_pos_m;
        request.ship_bary_velocity_mps = ship_bary_vel_mps;
        request.thrusting = thrusting;
        request.solve_quality = solve_quality;
        request.priority = classify_prediction_request_priority(
                _prediction_selection,
                track.key,
                track.is_celestial,
                interactive_request);
        request.future_window_s = prediction_required_window_s(track, now_s, with_maneuvers);
        if (request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
            track.preview_anchor.valid &&
            finite_vec3(track.preview_anchor.anchor_state_inertial.position_m) &&
            finite_vec3(track.preview_anchor.anchor_state_inertial.velocity_mps))
        {
            request.preview_patch.active = true;
            request.preview_patch.anchor_state_valid = true;
            request.preview_patch.baseline_generation_id = track.preview_anchor.baseline_generation_id;
            request.preview_patch.anchor_time_s = track.preview_anchor.anchor_time_s;
            request.preview_patch.patch_window_s = std::max(0.0, track.preview_anchor.patch_window_s);
            request.preview_patch.anchor_state_inertial = track.preview_anchor.anchor_state_inertial;
        }

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
            const double request_horizon_end_s = now_s + request.future_window_s;
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s))
                {
                    continue;
                }

                // FP-0 stays patch-bounded inside the solver. The request still has to carry every
                // downstream maneuver in the requested horizon so FP-1 can refine the remaining tail.
                if (request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                    node.time_s > request_horizon_end_s)
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

        const uint64_t generation_id = _prediction_service.request(std::move(request));
        const auto request_tp = PredictionDragDebugTelemetry::Clock::now();
        mark_prediction_request_submitted(track, generation_id, now_s, solve_quality);
        if (maneuver_live_preview)
        {
            record_preview_request_submission(track, generation_id, now_s, request_tp);
        }
        else if (track.preview_state != PredictionPreviewRuntimeState::Idle)
        {
            track.preview_state = PredictionPreviewRuntimeState::AwaitFullRefine;
        }
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
        request.priority = classify_prediction_request_priority(
                _prediction_selection,
                track.key,
                true,
                false);
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
        const uint64_t generation_id = _prediction_service.request(std::move(request));
        mark_prediction_request_submitted(
                track,
                generation_id,
                now_s,
                OrbitPredictionService::SolveQuality::Full);
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
            track.clear_runtime();
            return;
        }

        const bool maneuver_live_preview = PredictionRuntimeDetail::maneuver_live_preview(
                with_maneuvers,
                _maneuver_plan_live_preview_active,
                _maneuver_gizmo_interaction.state);
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
            track.clear_runtime();
        }
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
        // Celestial predictions rebuild asynchronously so the gameplay thread only packages requests.
        if (!_orbitsim)
        {
            track.clear_runtime();
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
            track.clear_runtime();
        }
    }
} // namespace Game
