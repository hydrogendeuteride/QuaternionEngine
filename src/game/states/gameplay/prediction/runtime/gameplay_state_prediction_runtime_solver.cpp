#include "game/states/gameplay/gameplay_state.h"

#include "core/util/logger.h"
#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"

#include <cmath>
#include <utility>

namespace Game
{
    namespace
    {
        bool frame_specs_match(const orbitsim::TrajectoryFrameSpec &a,
                               const orbitsim::TrajectoryFrameSpec &b)
        {
            return a.type == b.type &&
                   a.primary_body_id == b.primary_body_id &&
                   a.secondary_body_id == b.secondary_body_id &&
                   a.target_spacecraft_id == b.target_spacecraft_id;
        }

        bool frame_supports_live_base_frame_reuse(const orbitsim::TrajectoryFrameSpec &spec)
        {
            switch (spec.type)
            {
                case orbitsim::TrajectoryFrameType::Inertial:
                case orbitsim::TrajectoryFrameType::BodyCenteredInertial:
                case orbitsim::TrajectoryFrameType::BodyFixed:
                case orbitsim::TrajectoryFrameType::Synodic:
                case orbitsim::TrajectoryFrameType::LVLH:
                    return true;
            }
            return false;
        }

        bool base_trajectory_signature_matches(const OrbitPredictionCache &cache,
                                              const OrbitPredictionService::Result &result)
        {
            const std::vector<orbitsim::TrajectorySegment> &cache_segments =
                    cache.resolved_trajectory_segments_inertial();
            const std::vector<orbitsim::TrajectorySample> &cache_samples =
                    cache.resolved_trajectory_inertial();
            const std::vector<orbitsim::TrajectorySegment> &result_segments =
                    result.resolved_trajectory_segments_inertial();
            const std::vector<orbitsim::TrajectorySample> &result_samples =
                    result.resolved_trajectory_inertial();
            if (cache_segments.empty() || result_segments.empty() ||
                cache_samples.size() < 2 || result_samples.size() < 2)
            {
                return false;
            }

            if (cache_segments.size() != result_segments.size() ||
                cache_samples.size() != result_samples.size())
            {
                return false;
            }

            const orbitsim::TrajectorySegment &cache_seg0 = cache_segments.front();
            const orbitsim::TrajectorySegment &cache_seg1 = cache_segments.back();
            const orbitsim::TrajectorySegment &result_seg0 = result_segments.front();
            const orbitsim::TrajectorySegment &result_seg1 = result_segments.back();
            if (cache_seg0.t0_s != result_seg0.t0_s ||
                cache_seg0.dt_s != result_seg0.dt_s ||
                cache_seg1.t0_s != result_seg1.t0_s ||
                cache_seg1.dt_s != result_seg1.dt_s)
            {
                return false;
            }

            const orbitsim::TrajectorySample &cache_sample0 = cache_samples.front();
            const orbitsim::TrajectorySample &cache_sample1 = cache_samples.back();
            const orbitsim::TrajectorySample &result_sample0 = result_samples.front();
            const orbitsim::TrajectorySample &result_sample1 = result_samples.back();
            return cache_sample0.t_s == result_sample0.t_s &&
                   cache_sample1.t_s == result_sample1.t_s;
        }

        bool can_reuse_existing_base_frame_cache(const PredictionTrackState &track,
                                                 const OrbitPredictionService::Result &result,
                                                 const orbitsim::TrajectoryFrameSpec &resolved_frame_spec)
        {
            return result.baseline_reused &&
                   track.cache.valid &&
                   track.cache.resolved_frame_spec_valid &&
                   frame_supports_live_base_frame_reuse(resolved_frame_spec) &&
                   frame_specs_match(track.cache.resolved_frame_spec, resolved_frame_spec) &&
                   track.cache.resolved_shared_ephemeris() == result.resolved_shared_ephemeris() &&
                   track.cache.trajectory_frame.size() >= 2 &&
                   !track.cache.trajectory_segments_frame.empty() &&
                   base_trajectory_signature_matches(track.cache, result);
        }

        PredictionTrackState *find_track_by_id(std::vector<PredictionTrackState> &tracks, const uint64_t track_id)
        {
            for (PredictionTrackState &track : tracks)
            {
                if (track.key.track_id() == track_id)
                {
                    return &track;
                }
            }
            return nullptr;
        }

        void record_solver_result_debug(PredictionTrackState &track,
                                        const OrbitPredictionService::Result &result,
                                        const PredictionDragDebugTelemetry::TimePoint &solver_result_tp)
        {
            PredictionDragDebugTelemetry &debug = track.drag_debug;
            debug.last_result_solve_quality = result.solve_quality;
            debug.last_solver_result_tp = solver_result_tp;
            debug.last_solver_result_generation_id = result.generation_id;
            ++debug.solver_result_count;
            if (debug.last_request_generation_id == result.generation_id &&
                PredictionDragDebugTelemetry::has_time(debug.last_request_tp))
            {
                PredictionRuntimeDetail::update_last_and_peak(
                        debug.request_to_solver_ms_last,
                        debug.request_to_solver_ms_peak,
                        PredictionRuntimeDetail::elapsed_ms(debug.last_request_tp, solver_result_tp));
            }
        }
    } // namespace

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

    void GameplayState::apply_completed_prediction_result(OrbitPredictionService::Result result)
    {
        // Queue heavy frame/metrics derivation off-thread, then swap the completed cache on the main thread.
        const auto solver_result_tp = PredictionDragDebugTelemetry::Clock::now();
        PredictionTrackState *track = find_track_by_id(_prediction_tracks, result.track_id);
        if (!track)
        {
            return;
        }
        const PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle_before_apply =
                PredictionRuntimeDetail::describe_prediction_track_lifecycle(*track);
        const bool live_fast_preview_result =
                track->supports_maneuvers &&
                result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
                maneuver_live_preview_active(true);
        if (track->supports_maneuvers &&
            track->latest_requested_generation_id != 0 &&
            result.generation_id < track->latest_requested_generation_id &&
            !live_fast_preview_result)
        {
            return;
        }
        if (track->supports_maneuvers && result.maneuver_plan_revision != _maneuver_plan_revision)
        {
            Logger::warn("Dropping stale maneuver solver result: track={} gen={} result_plan_rev={} current_plan_rev={} "
                         "latest_request_gen={} request_pending={} derived_pending={} invalidated={} dirty={}",
                         result.track_id,
                         result.generation_id,
                         result.maneuver_plan_revision,
                         _maneuver_plan_revision,
                         track->latest_requested_generation_id,
                         track->request_pending,
                         track->derived_request_pending,
                         track->invalidated_while_pending,
                         track->dirty);
            if (result.generation_id == track->latest_requested_generation_id)
            {
                track->request_pending = false;
                track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
                track->invalidated_while_pending = false;
                track->dirty = true;
            }
            return;
        }

        const OrbitPredictionService::AdaptiveStageDiagnostics previous_frame_base_diagnostics =
                track->derived_diagnostics.frame_base;
        track->solver_ms_last = std::max(0.0, result.compute_time_ms);
        track->solver_diagnostics = result.diagnostics;
        track->derived_diagnostics = {};
        record_solver_result_debug(*track, result, solver_result_tp);

        const bool current_plan_active =
                track->supports_maneuvers &&
                _maneuver_nodes_enabled &&
                !_maneuver_state.nodes.empty();
        const bool active_maneuver_edit =
                current_plan_active &&
                (PredictionRuntimeDetail::maneuver_drag_active(_maneuver_gizmo_interaction.state) ||
                 _maneuver_node_edit_preview.state != ManeuverNodeEditPreview::State::Idle) &&
                track->key == _prediction_selection.active_subject &&
                maneuver_live_preview_active(true);
        const uint64_t current_plan_signature =
                current_plan_active ? current_maneuver_plan_signature() : 0u;
        if (active_maneuver_edit &&
            result.maneuver_plan_signature_valid &&
            result.maneuver_plan_signature != current_plan_signature &&
            result.solve_quality != OrbitPredictionService::SolveQuality::FastPreview)
        {
            track->request_pending = false;
            track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track->invalidated_while_pending = false;
            track->dirty = true;
            return;
        }

        if (!result.valid || result.resolved_trajectory_inertial().size() < 2)
        {
            track->request_pending = false;
            track->derived_request_pending = false;
            track->latest_requested_derived_generation_id = 0;
            track->latest_requested_derived_display_frame_key = 0;
            track->latest_requested_derived_display_frame_revision = 0;
            track->latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
            track->latest_requested_derived_publish_stage = OrbitPredictionService::PublishStage::Final;
            track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track->dirty = true;
            return;
        }

        if (PredictionRuntimeDetail::prediction_track_is_preview_streaming_publish(result.solve_quality,
                                                                                   result.publish_stage))
        {
            track->preview_state = PredictionPreviewRuntimeState::PreviewStreaming;
        }

        WorldVec3 build_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 build_vel_world{0.0};
        glm::vec3 build_vel_local{0.0f};
        (void) get_prediction_subject_world_state(track->key, build_pos_world, build_vel_world, build_vel_local);

        OrbitPredictionCache resolve_cache{};
        resolve_cache.build_time_s = result.build_time_s;
        if (result.has_shared_core_data())
        {
            resolve_cache.set_shared_solver_core_data(result.shared_core_data());
        }
        else
        {
            resolve_cache.shared_ephemeris = result.resolved_shared_ephemeris();
            resolve_cache.massive_bodies = result.resolved_massive_bodies();
            resolve_cache.trajectory_segments_inertial = result.resolved_trajectory_segments_inertial();
            resolve_cache.trajectory_inertial = result.resolved_trajectory_inertial();
        }
        const double reference_time_s = _orbitsim ? _orbitsim->sim.time_s() : result.build_time_s;
        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                resolve_prediction_display_frame_spec(resolve_cache, reference_time_s);

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
            else if (!result.resolved_trajectory_segments_inertial().empty())
            {
                player_lookup_segments = result.resolved_trajectory_segments_inertial();
            }
        }
        else if (const PredictionTrackState *player_track = player_prediction_track())
        {
            if (const OrbitPredictionCache *player_cache = effective_prediction_cache(player_track))
            {
                if (!player_cache->trajectory_segments_inertial_planned.empty())
                {
                    player_lookup_segments = player_cache->trajectory_segments_inertial_planned;
                }
                else if (!player_cache->resolved_trajectory_segments_inertial().empty())
                {
                    player_lookup_segments = player_cache->resolved_trajectory_segments_inertial();
                }
            }
        }

        OrbitPredictionDerivedService::Request derived_request{};
        derived_request.track_id = result.track_id;
        derived_request.generation_id = result.generation_id;
        derived_request.maneuver_plan_revision = result.maneuver_plan_revision;
        derived_request.maneuver_plan_signature_valid = result.maneuver_plan_signature_valid;
        derived_request.maneuver_plan_signature = result.maneuver_plan_signature;
        derived_request.priority = PredictionRuntimeDetail::classify_prediction_subject_priority(
                _prediction_selection,
                track->key,
                track->is_celestial);
        derived_request.solver_result = std::move(result);
        derived_request.reuse_existing_base_frame =
                can_reuse_existing_base_frame_cache(*track,
                                                   derived_request.solver_result,
                                                   resolved_frame_spec);
        derived_request.reused_base_frame_diagnostics = previous_frame_base_diagnostics;
        derived_request.build_pos_world = build_pos_world;
        derived_request.build_vel_world = build_vel_world;
        derived_request.sim_config = _orbitsim ? _orbitsim->sim.config() : orbitsim::GameSimulation::Config{};
        derived_request.resolved_frame_spec = resolved_frame_spec;
        derived_request.display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        derived_request.display_frame_revision = _prediction_display_frame_revision;
        derived_request.analysis_body_id = analysis_body_id;
        derived_request.player_lookup_segments_inertial = std::move(player_lookup_segments);
        _prediction_derived_service.request(derived_request);
        mark_prediction_derived_request_submitted(*track, derived_request);

        // Full-stream publishes are intermediate batches from the same solve generation.
        // Keep the request pending until the final publish so AwaitFullRefine does not
        // immediately enqueue a newer generation and discard the rest of this stream.
        track->request_pending = PredictionRuntimeDetail::prediction_track_should_keep_request_pending_after_solver_publish(
                derived_request.solver_result.solve_quality,
                derived_request.solver_result.publish_stage);

        // If the input changed while this solve was in-flight, promote straight to dirty so the
        // next update tick can submit a fresh solver request without waiting for derived to finish.
        if (PredictionRuntimeDetail::prediction_track_should_promote_dirty_after_solver_publish(lifecycle_before_apply))
        {
            track->dirty = true;
            track->invalidated_while_pending = false;
        }
        track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
    }
} // namespace Game
