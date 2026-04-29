#include "game/states/gameplay/gameplay_state.h"

#include "core/util/logger.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_request_factory.h"

#include <chrono>
#include <utility>

namespace Game
{
    namespace
    {
        void mark_prediction_request_submitted(PredictionTrackState &track,
                                               const uint64_t generation_id,
                                               const double now_s,
                                               const OrbitPredictionService::SolveQuality solve_quality,
                                               const bool request_has_maneuver_plan,
                                               const uint64_t plan_signature)
        {
            track.latest_requested_generation_id = generation_id;
            if (solve_quality == OrbitPredictionService::SolveQuality::Full)
            {
                track.latest_requested_authoritative_generation_id = generation_id;
            }
            track.request_pending = true;
            track.derived_request_pending = false;
            track.pending_solve_quality = solve_quality;
            track.pending_solver_has_maneuver_plan = request_has_maneuver_plan;
            track.pending_solver_plan_signature = request_has_maneuver_plan ? plan_signature : 0u;
            track.pending_derived_has_maneuver_plan = false;
            track.pending_derived_plan_signature = 0u;
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

        bool prediction_request_is_throttled(const PredictionTrackState &track, const bool interactive_request)
        {
            if (!interactive_request)
            {
                return false;
            }

            const PredictionDragDebugTelemetry &debug = track.drag_debug;
            const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
            if (debug.drag_active && PredictionDragDebugTelemetry::has_time(debug.drag_started_tp))
            {
                const double elapsed_drag_s =
                        std::chrono::duration<double>(now_tp - debug.drag_started_tp).count();
                if (elapsed_drag_s < OrbitPredictionTuning::kDragStalePreviewGraceS)
                {
                    return true;
                }
            }

            if (!PredictionDragDebugTelemetry::has_time(debug.last_request_tp))
            {
                return false;
            }

            const double elapsed_s = std::chrono::duration<double>(
                                             now_tp - debug.last_request_tp)
                                             .count();
            return elapsed_s < OrbitPredictionTuning::kDragRebuildMinIntervalS;
        }

        bool should_defer_solver_request_until_publish(const PredictionTrackState &track)
        {
            const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot snapshot =
                    PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
            return PredictionRuntimeDetail::prediction_track_should_defer_solver_request(snapshot);
        }
    } // namespace

    PredictionRuntimeContext GameplayState::build_prediction_runtime_context()
    {
        PredictionRuntimeContext context{};
        context.orbital_scenario = _orbitsim.get();
        context.selection = _prediction.selection;
        context.frame_selection = _prediction.frame_selection;
        context.analysis_selection = _prediction.analysis_selection;
        context.sampling_policy = _prediction.sampling_policy;
        context.maneuver_plan_windows = _maneuver_plan_windows;
        context.maneuver_plan = &_maneuver_state;
        context.maneuver_edit_preview = &_maneuver_node_edit_preview;
        context.maneuver_nodes_enabled = _maneuver_nodes_enabled;
        context.maneuver_edit_in_progress =
                PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state) ||
                _maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::Idle;
        context.maneuver_live_preview_available = maneuver_live_preview_active(true);
        context.maneuver_plan_revision = _maneuver_plan_revision;
        context.maneuver_plan_signature = current_maneuver_plan_signature();
        context.display_frame_revision = _prediction.display_frame_revision;
        context.active_maneuver_preview_anchor_node_id = active_maneuver_preview_anchor_node_id();
        context.current_sim_time_s = current_sim_time_s();
        context.reference_body_world = [this]() { return prediction_world_reference_body_world(); };
        context.get_subject_world_state = [this](const PredictionSubjectKey key,
                                                 WorldVec3 &out_pos_world,
                                                 glm::dvec3 &out_vel_world,
                                                 glm::vec3 &out_vel_local) {
            return get_prediction_subject_world_state(key, out_pos_world, out_vel_world, out_vel_local);
        };
        context.future_window_s = [this](const PredictionSubjectKey key) {
            return prediction_future_window_s(key);
        };
        context.required_window_s = [this](const PredictionTrackState &track,
                                           const double now_s,
                                           const bool with_maneuvers) {
            return prediction_required_window_s(track, now_s, with_maneuvers);
        };
        context.preview_exact_window_s = [this](const PredictionTrackState &track,
                                                const double now_s,
                                                const bool with_maneuvers) {
            return prediction_preview_exact_window_s(track, now_s, with_maneuvers);
        };
        context.refresh_preview_anchor = [this](PredictionTrackState &track,
                                                const double now_s,
                                                const bool with_maneuvers) {
            refresh_prediction_preview_anchor(track, now_s, with_maneuvers);
        };
        context.subject_is_player = [this](const PredictionSubjectKey key) {
            return prediction_subject_is_player(key);
        };
        context.resolve_maneuver_node_primary_body_id = [this](const ManeuverNode &node, const double query_time_s) {
            return resolve_maneuver_node_primary_body_id(node, query_time_s);
        };
        context.resolve_display_frame_spec = [this](const OrbitPredictionCache &cache, const double display_time_s) {
            return resolve_prediction_display_frame_spec(cache, display_time_s);
        };
        context.resolve_analysis_body_id = [this](const OrbitPredictionCache &cache,
                                                  const PredictionSubjectKey key,
                                                  const double query_time_s,
                                                  const orbitsim::BodyId preferred_body_id) {
            return resolve_prediction_analysis_body_id(cache, key, query_time_s, preferred_body_id);
        };
        context.player_effective_cache = [this]() -> const OrbitPredictionCache * {
            return effective_prediction_cache(player_prediction_track());
        };
        return context;
    }

    bool GameplayState::request_orbiter_prediction_async(PredictionTrackState &track,
                                                         const WorldVec3 &subject_pos_world,
                                                         const glm::dvec3 &subject_vel_world,
                                                         const double now_s,
                                                         const bool thrusting,
                                                         const bool with_maneuvers,
                                                         bool *out_throttled)
    {
        if (out_throttled)
        {
            *out_throttled = false;
        }

        PredictionOrbiterRequestBuildResult build =
                PredictionRequestFactory::build_orbiter_request(build_prediction_runtime_context(),
                                                                track,
                                                                subject_pos_world,
                                                                subject_vel_world,
                                                                now_s,
                                                                thrusting,
                                                                with_maneuvers);
        if (!build.built)
        {
            return false;
        }

        if (prediction_request_is_throttled(track, build.interactive_request))
        {
            if (out_throttled)
            {
                *out_throttled = true;
            }
            return false;
        }

        const OrbitPredictionService::SolveQuality submitted_quality = build.request.solve_quality;
        const uint64_t submitted_track_id = build.request.track_id;
        const uint64_t submitted_plan_revision = build.request.maneuver_plan_revision;
        const bool submitted_plan_signature_valid = build.request.maneuver_plan_signature_valid;
        const uint64_t submitted_plan_signature = build.request.maneuver_plan_signature;
        const auto submitted_priority = build.request.priority;
        const bool submitted_preview_patch = build.request.preview_patch.active;
        const bool submitted_anchor_state_valid = build.request.preview_patch.anchor_state_valid;
        const bool submitted_anchor_state_trusted = build.request.preview_patch.anchor_state_trusted;
        const double submitted_anchor_time_s = build.request.preview_patch.anchor_time_s;
        const double submitted_future_window_s = build.request.future_window_s;
        const std::size_t submitted_maneuver_count = build.request.maneuver_impulses.size();
        const uint64_t generation_id = _prediction.service.request(std::move(build.request));
        mark_prediction_request_submitted(track,
                                          generation_id,
                                          now_s,
                                          submitted_quality,
                                          submitted_plan_signature_valid,
                                          submitted_plan_signature);
        if (track.supports_maneuvers)
        {
            Logger::debug("Maneuver prediction request: track={} gen={} plan_rev={} plan_sig={} plan_sig_valid={} "
                          "quality={} priority={} "
                          "preview={} anchor_t={:.6f} anchor_state={} anchor_trusted={} impulses={} "
                          "future_window={:.3f} "
                          "pending_solver={} pending_derived={} dirty={}",
                          submitted_track_id,
                          generation_id,
                          submitted_plan_revision,
                          submitted_plan_signature,
                          submitted_plan_signature_valid,
                          static_cast<int>(submitted_quality),
                          static_cast<int>(submitted_priority),
                          submitted_preview_patch,
                          submitted_anchor_time_s,
                          submitted_anchor_state_valid,
                          submitted_anchor_state_trusted,
                          submitted_maneuver_count,
                          submitted_future_window_s,
                          track.request_pending,
                          track.derived_request_pending,
                          track.dirty);
        }
        if (build.preview_request_active)
        {
            track.preview_state = PredictionPreviewRuntimeState::DragPreviewPending;
            track.preview_last_request_at_s = now_s;
        }
        return true;
    }

    bool GameplayState::request_celestial_prediction_async(PredictionTrackState &track, const double now_s)
    {
        OrbitPredictionService::Request request{};
        if (!PredictionRequestFactory::build_celestial_request(build_prediction_runtime_context(),
                                                               track,
                                                               now_s,
                                                               request))
        {
            return false;
        }

        const uint64_t generation_id = _prediction.service.request(std::move(request));
        mark_prediction_request_submitted(
                track,
                generation_id,
                now_s,
                OrbitPredictionService::SolveQuality::Full,
                false,
                0u);
        return true;
    }

    void GameplayState::update_orbiter_prediction_track(PredictionTrackState &track,
                                                        const double now_s,
                                                        const bool thrusting,
                                                        const bool with_maneuvers)
    {
        WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 subject_vel_world{0.0};
        glm::vec3 subject_vel_local{0.0f};
        if (!get_prediction_subject_world_state(track.key, subject_pos_world, subject_vel_world, subject_vel_local))
        {
            track.clear_runtime();
            return;
        }

        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(track);
        const bool live_preview_pending_override =
                PredictionRuntimeDetail::prediction_track_live_preview_pending_override(lifecycle) &&
                track.supports_maneuvers &&
                track.key == _prediction.selection.active_subject &&
                maneuver_live_preview_active(with_maneuvers);
        if (!live_preview_pending_override &&
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
            const bool has_cache_to_keep = track.cache.identity.valid || track.authoritative_cache.identity.valid;
            if (!has_cache_to_keep)
            {
                track.clear_runtime();
            }
        }
    }

    void GameplayState::update_celestial_prediction_track(PredictionTrackState &track, const double now_s)
    {
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
        track.dirty = !requested;
        if (!requested)
        {
            track.clear_runtime();
        }
    }
} // namespace Game
