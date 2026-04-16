#include "game/states/gameplay/gameplay_state.h"

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
            if (cache.trajectory_segments_inertial.empty() || result.trajectory_segments_inertial.empty() ||
                cache.trajectory_inertial.size() < 2 || result.trajectory_inertial.size() < 2)
            {
                return false;
            }

            if (cache.trajectory_segments_inertial.size() != result.trajectory_segments_inertial.size() ||
                cache.trajectory_inertial.size() != result.trajectory_inertial.size())
            {
                return false;
            }

            const orbitsim::TrajectorySegment &cache_seg0 = cache.trajectory_segments_inertial.front();
            const orbitsim::TrajectorySegment &cache_seg1 = cache.trajectory_segments_inertial.back();
            const orbitsim::TrajectorySegment &result_seg0 = result.trajectory_segments_inertial.front();
            const orbitsim::TrajectorySegment &result_seg1 = result.trajectory_segments_inertial.back();
            if (cache_seg0.t0_s != result_seg0.t0_s ||
                cache_seg0.dt_s != result_seg0.dt_s ||
                cache_seg1.t0_s != result_seg1.t0_s ||
                cache_seg1.dt_s != result_seg1.dt_s)
            {
                return false;
            }

            const orbitsim::TrajectorySample &cache_sample0 = cache.trajectory_inertial.front();
            const orbitsim::TrajectorySample &cache_sample1 = cache.trajectory_inertial.back();
            const orbitsim::TrajectorySample &result_sample0 = result.trajectory_inertial.front();
            const orbitsim::TrajectorySample &result_sample1 = result.trajectory_inertial.back();
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
                   track.cache.shared_ephemeris == result.shared_ephemeris &&
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

        const OrbitPredictionService::AdaptiveStageDiagnostics previous_frame_base_diagnostics =
                track->derived_diagnostics.frame_base;
        track->solver_ms_last = std::max(0.0, result.compute_time_ms);
        track->solver_diagnostics = result.diagnostics;
        track->derived_diagnostics = {};
        record_solver_result_debug(*track, result, solver_result_tp);

        if (!result.valid || result.trajectory_inertial.size() < 2)
        {
            track->request_pending = false;
            track->derived_request_pending = false;
            track->latest_requested_derived_generation_id = 0;
            track->latest_requested_derived_display_frame_key = 0;
            track->latest_requested_derived_display_frame_revision = 0;
            track->latest_requested_derived_analysis_body_id = orbitsim::kInvalidBodyId;
            track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
            track->dirty = true;
            return;
        }

        if (result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview &&
            result.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming)
        {
            track->preview_state = PredictionPreviewRuntimeState::PreviewStreaming;
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
            if (const OrbitPredictionCache *player_cache = effective_prediction_cache(player_track))
            {
                if (!player_cache->trajectory_segments_inertial_planned.empty())
                {
                    player_lookup_segments = player_cache->trajectory_segments_inertial_planned;
                }
                else if (!player_cache->trajectory_segments_inertial.empty())
                {
                    player_lookup_segments = player_cache->trajectory_segments_inertial;
                }
            }
        }

        OrbitPredictionDerivedService::Request derived_request{};
        derived_request.track_id = result.track_id;
        derived_request.generation_id = result.generation_id;
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

        track->request_pending = false;

        // If the input changed while this solve was in-flight, promote straight to dirty so the
        // next update tick can submit a fresh solver request without waiting for derived to finish.
        if (track->invalidated_while_pending)
        {
            track->dirty = true;
            track->invalidated_while_pending = false;
        }
        track->pending_solve_quality = OrbitPredictionService::SolveQuality::Full;
    }
} // namespace Game
