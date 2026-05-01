#include "game/states/gameplay/prediction/prediction_frame_controller.h"

#include "game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_internal.h"
#include "game/states/gameplay/prediction/runtime/prediction_lifecycle_reducer.h"

#include <cmath>
#include <utility>

namespace Game
{
    namespace
    {
        OrbitPredictionService::Result build_prediction_solver_result_from_cache(
                const uint64_t track_id,
                const OrbitPredictionCache &cache)
        {
            OrbitPredictionService::Result result{};
            const auto &base_samples = cache.solver.resolved_trajectory_inertial();
            const auto &base_segments = cache.solver.resolved_trajectory_segments_inertial();
            result.track_id = track_id;
            result.generation_id = cache.identity.generation_id;
            result.maneuver_plan_revision = cache.identity.maneuver_plan_revision;
            result.maneuver_plan_signature_valid = cache.identity.maneuver_plan_signature_valid;
            result.maneuver_plan_signature = cache.identity.maneuver_plan_signature;
            result.valid = base_samples.size() >= 2 && !base_segments.empty();
            result.solve_quality = OrbitPredictionService::SolveQuality::Full;
            result.build_time_s = cache.identity.build_time_s;
            if (cache.solver.shared_solver_core_data)
            {
                result.set_shared_core_data(cache.solver.shared_solver_core_data);
            }
            else
            {
                result.shared_ephemeris = cache.solver.shared_ephemeris;
                result.massive_bodies = cache.solver.massive_bodies;
                result.trajectory_inertial = cache.solver.trajectory_inertial;
                result.trajectory_segments_inertial = cache.solver.trajectory_segments_inertial;
            }
            result.trajectory_inertial_planned = cache.solver.trajectory_inertial_planned;
            result.trajectory_segments_inertial_planned = cache.solver.trajectory_segments_inertial_planned;
            result.maneuver_previews = cache.solver.maneuver_previews;
            return result;
        }
    } // namespace

    void PredictionFrameController::mark_derived_request_submitted(
            PredictionTrackState &track,
            const OrbitPredictionDerivedService::Request &request)
    {
        PredictionLifecycleReducer::mark_derived_request_submitted(track, request);
    }

    bool PredictionFrameController::request_derived_refresh(
            const PredictionFrameControllerContext &context,
            PredictionTrackState &track,
            double display_time_s)
    {
        if (!context.derived_service ||
            !context.resolve_display_frame_spec ||
            !context.resolve_analysis_body_id ||
            !context.get_subject_world_state)
        {
            return false;
        }

        if (track.cache.solver.resolved_trajectory_inertial().size() < 2 ||
            track.cache.solver.resolved_trajectory_segments_inertial().empty())
        {
            return false;
        }

        if (!PredictionRuntimeDetail::latest_solver_generation_published(track))
        {
            return false;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                context.resolve_display_frame_spec(track.cache, display_time_s);
        const uint64_t display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        const uint64_t display_frame_revision = context.display_frame_revision;
        double analysis_time_s = display_time_s;
        if (!std::isfinite(analysis_time_s))
        {
            analysis_time_s = context.current_sim_time_s
                                      ? context.current_sim_time_s()
                                      : track.cache.identity.build_time_s;
        }

        const orbitsim::BodyId analysis_body_id =
                context.resolve_analysis_body_id(track.cache, track.key, analysis_time_s, track.auto_primary_body_id);
        if (track.derived_request_pending &&
            track.latest_requested_derived_generation_id == track.cache.identity.generation_id &&
            track.latest_requested_derived_display_frame_key == display_frame_key &&
            track.latest_requested_derived_display_frame_revision == display_frame_revision &&
            track.latest_requested_derived_analysis_body_id == analysis_body_id)
        {
            return false;
        }

        WorldVec3 build_pos_world = track.cache.identity.build_pos_world;
        glm::dvec3 build_vel_world = track.cache.identity.build_vel_world;
        glm::vec3 build_vel_local{0.0f};
        (void) context.get_subject_world_state(track.key, build_pos_world, build_vel_world, build_vel_local);

        std::vector<orbitsim::TrajectorySegment> player_lookup_segments;
        if (context.subject_is_player && context.subject_is_player(track.key))
        {
            if (!track.cache.solver.trajectory_segments_inertial_planned.empty())
            {
                player_lookup_segments = track.cache.solver.trajectory_segments_inertial_planned;
            }
            else
            {
                player_lookup_segments = track.cache.solver.resolved_trajectory_segments_inertial();
            }
        }
        else if (context.player_track && context.effective_cache)
        {
            if (const PredictionTrackState *player = context.player_track())
            {
                if (const OrbitPredictionCache *player_cache = context.effective_cache(player))
                {
                    if (!player_cache->solver.trajectory_segments_inertial_planned.empty())
                    {
                        player_lookup_segments = player_cache->solver.trajectory_segments_inertial_planned;
                    }
                    else if (!player_cache->solver.resolved_trajectory_segments_inertial().empty())
                    {
                        player_lookup_segments = player_cache->solver.resolved_trajectory_segments_inertial();
                    }
                }
            }
        }

        OrbitPredictionDerivedService::Request derived_request{};
        derived_request.track_id = track.key.track_id();
        derived_request.generation_id = track.cache.identity.generation_id;
        derived_request.maneuver_plan_revision = track.cache.identity.maneuver_plan_revision;
        derived_request.maneuver_plan_signature_valid = track.cache.identity.maneuver_plan_signature_valid;
        derived_request.maneuver_plan_signature = track.cache.identity.maneuver_plan_signature;
        derived_request.priority = PredictionRuntimeDetail::classify_prediction_subject_priority(
                context.selection,
                track.key,
                track.is_celestial);
        derived_request.solver_result = build_prediction_solver_result_from_cache(track.key.track_id(), track.cache);
        derived_request.build_pos_world = build_pos_world;
        derived_request.build_vel_world = build_vel_world;
        derived_request.sim_config = context.sim_config;
        derived_request.resolved_frame_spec = resolved_frame_spec;
        derived_request.display_frame_key = display_frame_key;
        derived_request.display_frame_revision = display_frame_revision;
        derived_request.analysis_body_id = analysis_body_id;
        derived_request.player_lookup_segments_inertial = std::move(player_lookup_segments);
        context.derived_service->request(derived_request);
        mark_derived_request_submitted(track, derived_request);
        return true;
    }

    bool PredictionFrameController::has_current_derived_cache(
            const PredictionFrameControllerContext &context,
            const PredictionTrackState &track,
            double display_time_s)
    {
        if (!context.resolve_display_frame_spec)
        {
            return false;
        }

        if (!track.cache.identity.valid ||
            track.cache.display.trajectory_frame.size() < 2 ||
            track.cache.display.trajectory_segments_frame.empty())
        {
            return false;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                context.resolve_display_frame_spec(track.cache, display_time_s);
        const uint64_t display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        return track.cache.display.resolved_frame_spec_valid &&
               track.cache.display.display_frame_key == display_frame_key &&
               track.cache.display.display_frame_revision == context.display_frame_revision;
    }

    void PredictionFrameController::refresh_derived_cache(
            const PredictionFrameControllerContext &context,
            PredictionTrackState &track,
            double display_time_s)
    {
        if (!context.resolve_display_frame_spec || !context.resolve_analysis_body_id)
        {
            return;
        }

        if (track.cache.solver.resolved_trajectory_inertial().size() < 2)
        {
            return;
        }

        const orbitsim::TrajectoryFrameSpec resolved_frame_spec =
                context.resolve_display_frame_spec(track.cache, display_time_s);
        const uint64_t display_frame_key = prediction_display_frame_key(resolved_frame_spec);
        const uint64_t display_frame_revision = context.display_frame_revision;
        const auto cache_frame_matches = [&](const OrbitPredictionCache &cache) {
            return cache.display.display_frame_key == display_frame_key &&
                   cache.display.display_frame_revision == display_frame_revision;
        };
        if ((track.pick_cache.generation_id != 0u || track.pick_cache.base_valid || track.pick_cache.planned_valid) &&
            !cache_frame_matches(track.cache))
        {
            track.pick_cache.clear();
        }
        if (has_current_derived_cache(context, track, display_time_s))
        {
            return;
        }

        const double analysis_time_s =
                std::isfinite(display_time_s)
                        ? display_time_s
                        : (context.current_sim_time_s ? context.current_sim_time_s() : track.cache.identity.build_time_s);
        const orbitsim::BodyId analysis_body_id =
                context.resolve_analysis_body_id(track.cache, track.key, analysis_time_s, track.auto_primary_body_id);
        if (analysis_body_id != orbitsim::kInvalidBodyId)
        {
            track.auto_primary_body_id = analysis_body_id;
        }

        (void) request_derived_refresh(context, track, display_time_s);
    }

    void PredictionFrameController::reset_track_derived_state(PredictionTrackState &track)
    {
        PredictionLifecycleReducer::reset_derived_request_state(track);
        track.preview_overlay.clear();
        track.full_stream_overlay.clear();
        track.pick_cache.clear();
    }
} // namespace Game
