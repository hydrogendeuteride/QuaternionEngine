#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <iterator>

namespace Game
{
    namespace
    {
        bool validate_segment_boundary(const orbitsim::TrajectorySegment &previous_segment,
                                       const orbitsim::TrajectorySegment &current_segment)
        {
            const double previous_end_time_s = prediction_segment_end_time(previous_segment);
            if (std::abs(current_segment.t0_s - previous_end_time_s) >
                continuity_time_epsilon_s(previous_end_time_s))
            {
                return false;
            }

            if (segment_allows_velocity_discontinuity(current_segment))
            {
                return states_are_position_continuous(previous_segment.end, current_segment.start);
            }

            return states_are_continuous(previous_segment.end, current_segment.start);
        }

        orbitsim::BodyId select_primary_body_id_for_planned(
                const PlannedTrajectoryContext &ctx,
                const orbitsim::State &state,
                const double time_s,
                const orbitsim::BodyId preferred_body_id)
        {
            if (ctx.out.massive_bodies.empty() || !finite_state(state))
            {
                return orbitsim::kInvalidBodyId;
            }

            const orbitsim::CelestialEphemeris &eph = ctx.ephemeris;
            const auto &bodies = ctx.out.massive_bodies;
            const std::size_t primary_index = select_primary_index_with_hysteresis(
                    bodies,
                    state.position_m,
                    [&eph, &bodies, time_s](const std::size_t i) -> orbitsim::Vec3 {
                        std::size_t eph_index = 0u;
                        if (bodies[i].id != orbitsim::kInvalidBodyId &&
                            eph.body_index_for_id(bodies[i].id, &eph_index))
                        {
                            return eph.body_position_at(eph_index, time_s);
                        }
                        return bodies[i].state.position_m;
                    },
                    ctx.request.sim_config.softening_length_m,
                    preferred_body_id);
            return primary_index < bodies.size()
                           ? bodies[primary_index].id
                           : orbitsim::kInvalidBodyId;
        }

        bool validate_chunk_seam(
                const PlannedTrajectoryContext &ctx,
                const OrbitPredictionService::PredictionChunkPlan &current_chunk,
                const std::vector<orbitsim::TrajectorySegment> &previous_seam_segments,
                const std::vector<orbitsim::TrajectorySegment> &current_segments,
                OrbitPredictionService::ChunkSeamDiagnostics &out_diag)
        {
            out_diag = {};
            if (!current_chunk.requires_seam_validation ||
                previous_seam_segments.empty() ||
                current_segments.empty() ||
                current_chunk.profile_id == OrbitPredictionService::PredictionProfileId::Exact ||
                current_chunk.profile_id == OrbitPredictionService::PredictionProfileId::Near ||
                (current_chunk.boundary_flags &
                 static_cast<uint32_t>(OrbitPredictionService::PredictionChunkBoundaryFlags::Maneuver)) != 0u)
            {
                return true;
            }

            const double boundary_time_s = current_chunk.t0_s;
            const double sample_time_s = std::min(prediction_segment_end_time(previous_seam_segments.back()),
                                                  prediction_segment_end_time(current_segments.back()));
            if (!(sample_time_s > boundary_time_s))
            {
                return true;
            }

            orbitsim::State previous_state{};
            orbitsim::State current_state{};
            if (!sample_trajectory_segment_state(previous_seam_segments, sample_time_s, previous_state) ||
                !sample_trajectory_segment_state(current_segments, sample_time_s, current_state))
            {
                return true;
            }

            out_diag.valid = true;
            out_diag.sample_time_s = sample_time_s;
            out_diag.time_error_s = 0.0;
            out_diag.position_error_m =
                    glm::length(glm::dvec3(previous_state.position_m - current_state.position_m));
            out_diag.velocity_error_mps =
                    glm::length(glm::dvec3(previous_state.velocity_mps - current_state.velocity_mps));
            out_diag.previous_primary_body_id = select_primary_body_id_for_planned(
                    ctx,
                    previous_state,
                    sample_time_s,
                    ctx.request.preferred_primary_body_id);
            out_diag.current_primary_body_id = select_primary_body_id_for_planned(
                    ctx,
                    current_state,
                    sample_time_s,
                    out_diag.previous_primary_body_id);
            out_diag.primary_flutter =
                    out_diag.previous_primary_body_id != orbitsim::kInvalidBodyId &&
                    out_diag.current_primary_body_id != orbitsim::kInvalidBodyId &&
                    out_diag.previous_primary_body_id != out_diag.current_primary_body_id;

            const double pos_scale_m = std::max({1.0,
                                                 glm::length(glm::dvec3(previous_state.position_m)),
                                                 glm::length(glm::dvec3(current_state.position_m))});
            const double vel_scale_mps = std::max({1.0,
                                                   glm::length(glm::dvec3(previous_state.velocity_mps)),
                                                   glm::length(glm::dvec3(current_state.velocity_mps))});
            const double pos_tolerance_m =
                    std::max(OrbitPredictionTuning::kPredictionSeamPosToleranceFloorM,
                             pos_scale_m * OrbitPredictionTuning::kPredictionSeamPosToleranceScale);
            const double vel_tolerance_mps =
                    std::max(OrbitPredictionTuning::kPredictionSeamVelToleranceFloorMps,
                             vel_scale_mps * OrbitPredictionTuning::kPredictionSeamVelToleranceScale);
            out_diag.success =
                    out_diag.position_error_m <= pos_tolerance_m &&
                    out_diag.velocity_error_mps <= vel_tolerance_mps &&
                    !out_diag.primary_flutter;
            return out_diag.success;
        }
    } // namespace

    void accumulate_planned_stage_diagnostics(OrbitPredictionService::AdaptiveStageDiagnostics &dst,
                                              const OrbitPredictionService::AdaptiveStageDiagnostics &src,
                                              double &dt_sum_s,
                                              bool &has_dt)
    {
        dst.requested_duration_s += std::max(0.0, src.requested_duration_s);
        dst.covered_duration_s += std::max(0.0, src.covered_duration_s);
        dst.accepted_segments += src.accepted_segments;
        dst.rejected_splits += src.rejected_splits;
        dst.forced_boundary_splits += src.forced_boundary_splits;
        dst.frame_resegmentation_count += src.frame_resegmentation_count;
        dst.hard_cap_hit = dst.hard_cap_hit || src.hard_cap_hit;
        dst.cancelled = dst.cancelled || src.cancelled;
        dst.cache_reused = dst.cache_reused || src.cache_reused;
        dst.maneuver_apply_failed_count += src.maneuver_apply_failed_count;
        if (dst.maneuver_apply_failed_node_id < 0 && src.maneuver_apply_failed_node_id >= 0)
        {
            dst.maneuver_apply_failed_node_id = src.maneuver_apply_failed_node_id;
        }

        if (src.accepted_segments == 0u)
        {
            return;
        }

        if (!has_dt)
        {
            dst.min_dt_s = src.min_dt_s;
            dst.max_dt_s = src.max_dt_s;
            has_dt = true;
        }
        else
        {
            dst.min_dt_s = std::min(dst.min_dt_s, src.min_dt_s);
            dst.max_dt_s = std::max(dst.max_dt_s, src.max_dt_s);
        }

        dt_sum_s += src.avg_dt_s * static_cast<double>(src.accepted_segments);
    }

    void append_planned_chunk_samples(std::vector<orbitsim::TrajectorySample> &dst,
                                      std::vector<orbitsim::TrajectorySample> src)
    {
        if (src.empty())
        {
            return;
        }

        if (!dst.empty())
        {
            const double time_epsilon_s = continuity_time_epsilon_s(src.front().t_s);
            if (std::abs(dst.back().t_s - src.front().t_s) <= time_epsilon_s)
            {
                dst.pop_back();
            }
        }

        dst.insert(dst.end(),
                   std::make_move_iterator(src.begin()),
                   std::make_move_iterator(src.end()));
    }

    void append_planned_chunk_packet(PlannedSolveOutput &planned, PlannedChunkPacket packet)
    {
        planned.chunk_reused.push_back(packet.reused_from_cache);
        planned.previews.insert(planned.previews.end(),
                                std::make_move_iterator(packet.previews.begin()),
                                std::make_move_iterator(packet.previews.end()));
        planned.segments.insert(planned.segments.end(),
                                std::make_move_iterator(packet.segments.begin()),
                                std::make_move_iterator(packet.segments.end()));
        append_planned_chunk_samples(planned.samples, std::move(packet.samples));
    }

    void apply_planned_range_summary(PlannedSolveOutput &planned, const PlannedSolveRangeSummary &summary)
    {
        planned.end_state = summary.end_state;
        planned.diagnostics = summary.diagnostics;
        planned.status = summary.status;
    }

    PlannedSolveRangeSummary solve_planned_chunk_range(
            PlannedTrajectoryContext &ctx,
            const OrbitPredictionService::PredictionSolvePlan &solve_plan,
            const std::size_t chunk_begin_index,
            const std::size_t chunk_end_index,
            const orbitsim::State &range_start_state,
            std::function<bool(PlannedChunkPacket &&)> chunk_sink)
    {
        using Status = OrbitPredictionService::Status;

        PlannedSolveRangeSummary summary{};
        if (!solve_plan.valid ||
            chunk_begin_index >= chunk_end_index ||
            chunk_end_index > solve_plan.chunks.size() ||
            !finite_state(range_start_state))
        {
            summary.status = Status::InvalidInput;
            return summary;
        }

        orbitsim::State chunk_start_state = range_start_state;
        summary.end_state = range_start_state;
        const uint64_t baseline_generation_id =
                planned_baseline_generation_hash(ctx.request, solve_plan.t0_s, range_start_state);
        const uint64_t frame_independent_generation =
                planned_solver_context_hash(ctx.request, ctx.out.massive_bodies);
        double diagnostic_dt_sum_s = 0.0;
        bool have_dt = false;
        std::vector<orbitsim::TrajectorySegment> previous_chunk_seam_segments;
        std::optional<orbitsim::TrajectorySegment> previous_published_segment;
        std::size_t published_sample_count = 0u;
        double previous_sample_time_s = std::numeric_limits<double>::quiet_NaN();

        for (std::size_t chunk_index = chunk_begin_index; chunk_index < chunk_end_index; ++chunk_index)
        {
            if (ctx.cancel_requested())
            {
                summary.status = Status::Cancelled;
                return summary;
            }

            const OrbitPredictionService::PredictionChunkPlan &chunk = solve_plan.chunks[chunk_index];
            if (!(chunk.t1_s > chunk.t0_s))
            {
                summary.status = Status::InvalidInput;
                return summary;
            }

            const bool include_chunk_end_impulse = (chunk_index + 1u) == solve_plan.chunks.size();
            const OrbitPredictionService::ChunkActivityProbe activity_probe =
                    classify_chunk_activity(ctx.request,
                                            chunk,
                                            &ctx.out.trajectory_segments_inertial,
                                            ctx.out.shared_ephemeris);

            OrbitPredictionService::PredictionChunkPlan effective_chunk = chunk;
            effective_chunk.profile_id = activity_probe.recommended_profile_id;
            effective_chunk.allow_reuse = chunk.allow_reuse;

            bool split_chunk = activity_probe.should_split &&
                               (chunk.t1_s - chunk.t0_s) >=
                                       (2.0 * OrbitPredictionTuning::kPredictionActivityProbeMinSplitSpanS);
            ChunkAttemptOutput chunk_attempt{};
            OrbitPredictionService::ChunkSeamDiagnostics seam_diag{};
            bool chunk_solved = false;

            for (std::size_t attempt_index = 0u;
                 attempt_index < OrbitPredictionTuning::kPredictionSeamRetryMaxAttempts;
                 ++attempt_index)
            {
                const PlannedChunkAttemptRequest attempt_request{
                        .chunk = chunk,
                        .effective_chunk = effective_chunk,
                        .split_chunk = split_chunk,
                        .include_chunk_end_impulse = include_chunk_end_impulse,
                        .baseline_generation_id = baseline_generation_id,
                        .frame_independent_generation = frame_independent_generation,
                        .start_state = chunk_start_state,
                };
                chunk_attempt = solve_planned_chunk_attempt(ctx, attempt_request);
                if (chunk_attempt.status != Status::Success)
                {
                    accumulate_planned_stage_diagnostics(summary.diagnostics,
                                                         chunk_attempt.diagnostics,
                                                         diagnostic_dt_sum_s,
                                                         have_dt);
                    summary.status = chunk_attempt.status;
                    return summary;
                }

                if (validate_chunk_seam(ctx,
                                        effective_chunk,
                                        previous_chunk_seam_segments,
                                        chunk_attempt.segments,
                                        seam_diag))
                {
                    seam_diag.retry_count = static_cast<uint32_t>(attempt_index);
                    chunk_solved = true;
                    break;
                }

                seam_diag.retry_count = static_cast<uint32_t>(attempt_index + 1u);
                const bool can_split =
                        !split_chunk &&
                        (chunk.t1_s - chunk.t0_s) >=
                                (2.0 * OrbitPredictionTuning::kPredictionActivityProbeMinSplitSpanS);
                if (can_split)
                {
                    split_chunk = true;
                    continue;
                }

                const auto promoted_profile = promote_prediction_profile(effective_chunk.profile_id, 1u);
                if (promoted_profile == effective_chunk.profile_id)
                {
                    break;
                }

                effective_chunk.profile_id = promoted_profile;
                effective_chunk.allow_reuse = chunk.allow_reuse;
            }

            if (!chunk_solved)
            {
                summary.status = Status::ContinuityFailed;
                return summary;
            }

            if (chunk_attempt.segments.empty())
            {
                summary.status = Status::TrajectorySegmentsUnavailable;
                return summary;
            }
            if (chunk_attempt.samples.size() < 2u)
            {
                summary.status = Status::TrajectorySamplesUnavailable;
                return summary;
            }

            if (previous_published_segment.has_value() &&
                !validate_segment_boundary(*previous_published_segment, chunk_attempt.segments.front()))
            {
                summary.status = Status::ContinuityFailed;
                return summary;
            }

            const orbitsim::TrajectorySegment last_published_segment = chunk_attempt.segments.back();
            if (published_sample_count == 0u)
            {
                published_sample_count = chunk_attempt.samples.size();
            }
            else
            {
                published_sample_count += chunk_attempt.samples.size();
                if (std::isfinite(previous_sample_time_s))
                {
                    const double time_epsilon_s = continuity_time_epsilon_s(chunk_attempt.samples.front().t_s);
                    if (std::abs(previous_sample_time_s - chunk_attempt.samples.front().t_s) <= time_epsilon_s)
                    {
                        --published_sample_count;
                    }
                }
            }
            previous_sample_time_s = chunk_attempt.samples.back().t_s;

            PlannedChunkPacket chunk_packet{};
            chunk_packet.chunk = effective_chunk;
            chunk_packet.start_state = chunk_start_state;
            chunk_packet.end_state = chunk_attempt.end_state;
            chunk_packet.diagnostics = chunk_attempt.diagnostics;
            chunk_packet.reused_from_cache =
                    chunk_attempt.reused_from_cache || chunk_attempt.diagnostics.cache_reused;
            chunk_packet.previews = std::move(chunk_attempt.previews);
            chunk_packet.segments = std::move(chunk_attempt.segments);
            chunk_packet.samples = std::move(chunk_attempt.samples);
            if (!chunk_sink(std::move(chunk_packet)))
            {
                summary.status = Status::Cancelled;
                return summary;
            }

            accumulate_planned_stage_diagnostics(summary.diagnostics,
                                                 chunk_attempt.diagnostics,
                                                 diagnostic_dt_sum_s,
                                                 have_dt);
            chunk_start_state = chunk_attempt.end_state;
            summary.end_state = chunk_start_state;
            previous_chunk_seam_segments = std::move(chunk_attempt.seam_validation_segments);
            previous_published_segment = last_published_segment;
        }

        if (!previous_published_segment.has_value())
        {
            summary.status = Status::TrajectorySegmentsUnavailable;
            return summary;
        }
        if (published_sample_count < 2u)
        {
            summary.status = Status::TrajectorySamplesUnavailable;
            return summary;
        }
        if (summary.diagnostics.accepted_segments > 0u)
        {
            summary.diagnostics.avg_dt_s =
                    diagnostic_dt_sum_s / static_cast<double>(summary.diagnostics.accepted_segments);
        }

        return summary;
    }
} // namespace Game
