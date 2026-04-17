#include "game/states/gameplay/gameplay_state.h"

#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace
    {
        void mark_prediction_request_submitted(PredictionTrackState &track,
                                               const uint64_t generation_id,
                                               const double now_s,
                                               const OrbitPredictionService::SolveQuality solve_quality)
        {
            track.latest_requested_generation_id = generation_id;
            if (solve_quality == OrbitPredictionService::SolveQuality::Full)
            {
                track.latest_requested_authoritative_generation_id = generation_id;
            }
            track.request_pending = true;
            track.derived_request_pending = false;
            track.pending_solve_quality = solve_quality;
            track.invalidated_while_pending = false;
            (void) now_s;

            PredictionDragDebugTelemetry &debug = track.drag_debug;
            const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
            debug.last_request_tp = now_tp;
            debug.last_request_generation_id = generation_id;
            ++debug.request_count;
            if (debug.drag_active && PredictionDragDebugTelemetry::has_time(debug.last_drag_update_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.drag_to_request_ms_last,
                        debug.drag_to_request_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_drag_update_tp, now_tp));
            }
        }

        bool prediction_request_is_interactive(const PredictionSelectionState &selection,
                                               const std::vector<ManeuverNode> &nodes,
                                               const ManeuverGizmoInteraction::State gizmo_state,
                                               const PredictionTrackState &track,
                                               const double now_s,
                                               const bool thrusting,
                                               const bool with_maneuvers)
        {
            const bool dragging_maneuver_axis =
                    with_maneuvers &&
                    PredictionRuntimeDetail::maneuver_drag_active(gizmo_state);
            const bool has_future_maneuver_node =
                    with_maneuvers &&
                    std::any_of(nodes.begin(),
                                nodes.end(),
                                [now_s](const ManeuverNode &node) {
                                    return std::isfinite(node.time_s) && node.time_s >= now_s;
                                });
            const bool active_subject = track.key == selection.active_subject;
            return active_subject &&
                   (thrusting || has_future_maneuver_node || dragging_maneuver_axis);
        }

        bool prediction_request_is_throttled(const PredictionTrackState &track, const bool interactive_request)
        {
            if (!interactive_request)
            {
                return false;
            }

            const PredictionDragDebugTelemetry &debug = track.drag_debug;
            if (!PredictionDragDebugTelemetry::has_time(debug.last_request_tp))
            {
                return false;
            }

            const double elapsed_s = std::chrono::duration<double>(
                                             PredictionDragDebugTelemetry::Clock::now() - debug.last_request_tp)
                                             .count();
            return elapsed_s < OrbitPredictionTuning::kDragRebuildMinIntervalS;
        }

        bool should_defer_solver_request_until_publish(const PredictionTrackState &track)
        {
            if (track.request_pending)
            {
                return true;
            }

            const bool awaiting_latest_publish =
                    !PredictionRuntimeDetail::latest_solver_generation_published(track);
            return (track.derived_request_pending || awaiting_latest_publish) && !track.dirty;
        }

    } // namespace

    bool GameplayState::resolve_prediction_preview_anchor_state(const PredictionTrackState &track,
                                                                orbitsim::State &out_state) const
    {
        out_state = {};
        if (!track.preview_anchor.valid || !std::isfinite(track.preview_anchor.anchor_time_s))
        {
            return false;
        }

        const OrbitPredictionCache &anchor_cache =
                track.authoritative_cache.valid ? track.authoritative_cache : track.cache;

        constexpr double kPreviewAnchorTimeMatchEpsilonS = 1.0e-6;
        const auto preview_it =
                std::find_if(anchor_cache.maneuver_previews.begin(),
                             anchor_cache.maneuver_previews.end(),
                             [&track](const OrbitPredictionService::ManeuverNodePreview &preview) {
                                 return preview.valid &&
                                        preview.node_id == track.preview_anchor.anchor_node_id &&
                                        std::isfinite(preview.t_s) &&
                                        std::abs(preview.t_s - track.preview_anchor.anchor_time_s) <=
                                                kPreviewAnchorTimeMatchEpsilonS &&
                                        detail::finite_vec3(preview.inertial_position_m) &&
                                        detail::finite_vec3(preview.inertial_velocity_mps);
                             });
        if (preview_it != anchor_cache.maneuver_previews.end())
        {
            out_state.position_m = preview_it->inertial_position_m;
            out_state.velocity_mps = preview_it->inertial_velocity_mps;
            return true;
        }

        return PredictionCacheInternal::sample_prediction_inertial_state(anchor_cache.trajectory_segments_inertial_planned,
                                                                         track.preview_anchor.anchor_time_s,
                                                                         out_state) ||
               sample_prediction_inertial_state(anchor_cache.trajectory_inertial_planned,
                                                track.preview_anchor.anchor_time_s,
                                                out_state) ||
               PredictionCacheInternal::sample_prediction_inertial_state(track.cache.trajectory_segments_inertial,
                                                                         track.preview_anchor.anchor_time_s,
                                                                         out_state) ||
               sample_prediction_inertial_state(track.cache.trajectory_inertial,
                                                track.preview_anchor.anchor_time_s,
                                                out_state);
    }

    bool GameplayState::build_orbiter_prediction_request(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers,
                                                         OrbitPredictionService::Request &out_request,
                                                         bool *out_interactive_request,
                                                         bool *out_preview_request_active)
    {
        if (out_interactive_request)
        {
            *out_interactive_request = false;
        }
        if (out_preview_request_active)
        {
            *out_preview_request_active = false;
        }

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
        if (!detail::finite_vec3(ship_bary_pos_m) || !detail::finite_vec3(ship_bary_vel_mps))
        {
            return false;
        }

        refresh_prediction_preview_anchor(track, now_s, with_maneuvers);

        const bool interactive_request =
                prediction_request_is_interactive(
                        _prediction_selection,
                        _maneuver_state.nodes,
                        _maneuver_gizmo_interaction.state,
                        track,
                        now_s,
                        thrusting,
                        with_maneuvers);
        const bool preview_request_active =
                track.preview_anchor.valid &&
                track.supports_maneuvers &&
                with_maneuvers &&
                _maneuver_plan_live_preview_active &&
                PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state);
        if (out_interactive_request)
        {
            *out_interactive_request = interactive_request;
        }
        if (out_preview_request_active)
        {
            *out_preview_request_active = preview_request_active;
        }
        const OrbitPredictionService::SolveQuality solve_quality =
                preview_request_active
                        ? OrbitPredictionService::SolveQuality::FastPreview
                        : OrbitPredictionService::SolveQuality::Full;
        const double preview_exact_window_s =
                preview_request_active ? prediction_preview_exact_window_s(track, now_s, with_maneuvers) : 0.0;

        OrbitPredictionService::Request request{};
        request.track_id = track.key.track_id();
        request.sim_time_s = now_s;
        request.sim_config = _orbitsim->sim.config();
        request.shared_ephemeris = track.cache.shared_ephemeris;
        request.massive_bodies = _orbitsim->sim.massive_bodies();
        request.ship_bary_position_m = ship_bary_pos_m;
        request.ship_bary_velocity_mps = ship_bary_vel_mps;
        request.thrusting = thrusting;
        request.solve_quality = solve_quality;
        request.priority = PredictionRuntimeDetail::classify_prediction_request_priority(
                _prediction_selection,
                track.key,
                track.is_celestial,
                interactive_request);
        if (preview_request_active)
        {
            const double anchor_offset_s = std::max(0.0, track.preview_anchor.anchor_time_s - now_s);
            // Live preview only needs enough horizon to reach the selected node and cover the exact patch window.
            request.future_window_s = std::min(track.preview_anchor.request_window_s,
                                               anchor_offset_s + (2.0 * preview_exact_window_s));
        }
        else
        {
            request.future_window_s = prediction_required_window_s(track, now_s, with_maneuvers);
        }

        if (preview_request_active)
        {
            request.preview_patch.active = true;
            request.preview_patch.anchor_time_s = track.preview_anchor.anchor_time_s;
            request.preview_patch.visual_window_s = track.preview_anchor.visual_window_s;
            request.preview_patch.exact_window_s = preview_exact_window_s;
            orbitsim::State anchor_state{};
            if (resolve_prediction_preview_anchor_state(track, anchor_state))
            {
                request.preview_patch.anchor_state_valid = true;
                request.preview_patch.anchor_state_inertial = anchor_state;
            }
        }

        const orbitsim::TrajectoryFrameSpec display_frame_spec =
                track.cache.resolved_frame_spec_valid ? track.cache.resolved_frame_spec : _prediction_frame_selection.spec;
        request.lagrange_sensitive = prediction_frame_is_lagrange_sensitive(display_frame_spec);
        const bool auto_primary_may_shift_across_plan =
                with_maneuvers && prediction_subject_is_player(track.key);
        request.preferred_primary_body_id =
                auto_primary_may_shift_across_plan ? orbitsim::kInvalidBodyId : track.auto_primary_body_id;
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
            for (const ManeuverNode &node : _maneuver_state.nodes)
            {
                if (!std::isfinite(node.time_s))
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

        request.full_stream_publish.active =
                solve_quality == OrbitPredictionService::SolveQuality::Full &&
                track.key == _prediction_selection.active_subject &&
                prediction_subject_is_player(track.key) &&
                !request.maneuver_impulses.empty();

        out_request = std::move(request);
        return true;
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers,
                                                         bool *out_throttled)
    {
        // Package the current spacecraft state into a worker request.
        if (out_throttled)
        {
            *out_throttled = false;
        }

        OrbitPredictionService::Request request{};
        bool interactive_request = false;
        bool preview_request_active = false;
        if (!build_orbiter_prediction_request(track,
                                              subject_pos_world,
                                              subject_vel_world,
                                              now_s,
                                              thrusting,
                                              with_maneuvers,
                                              request,
                                              &interactive_request,
                                              &preview_request_active))
        {
            return false;
        }

        if (prediction_request_is_throttled(track, interactive_request))
        {
            if (out_throttled)
            {
                *out_throttled = true;
            }
            return false;
        }

        const OrbitPredictionService::SolveQuality submitted_quality = request.solve_quality;
        const uint64_t generation_id = _prediction_service.request(std::move(request));
        mark_prediction_request_submitted(track, generation_id, now_s, submitted_quality);
        if (preview_request_active)
        {
            track.preview_state = PredictionPreviewRuntimeState::DragPreviewPending;
            track.preview_last_request_at_s = now_s;
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
        request.priority = PredictionRuntimeDetail::classify_prediction_request_priority(
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

        const bool live_preview_drag_pending_override =
                (track.dirty || track.invalidated_while_pending) &&
                track.supports_maneuvers &&
                track.key == _prediction_selection.active_subject &&
                with_maneuvers &&
                _maneuver_plan_live_preview_active &&
                PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state);
        if (!live_preview_drag_pending_override &&
            should_defer_solver_request_until_publish(track))
        {
            return;
        }

        bool throttled = false;
        const bool requested = request_orbiter_prediction_async(
                track,
                subject_pos_world,
                subject_vel_world,
                now_s,
                thrusting,
                with_maneuvers,
                &throttled);
        track.dirty = !requested;
        if (!requested && !throttled)
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

        if (should_defer_solver_request_until_publish(track))
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
