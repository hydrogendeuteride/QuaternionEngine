#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/maneuvers.hpp"
#include "orbitsim/maneuvers_types.hpp"
#include "orbitsim/spacecraft_state_cache.hpp"
#include "orbitsim/trajectories.hpp"

#include <iterator>

namespace Game
{
    namespace
    {
        struct SubchunkRange
        {
            double t0_s{0.0};
            double t1_s{0.0};
            double overlap_s{0.0};
            bool include_chunk_end_impulse{false};
        };

        std::vector<SubchunkRange> build_planned_subchunks(
                const OrbitPredictionService::PredictionChunkPlan &chunk,
                const OrbitPredictionService::PredictionProfileDefinition &effective_profile,
                const bool split_chunk,
                const bool include_chunk_end_impulse)
        {
            std::vector<SubchunkRange> subchunks;
            if (split_chunk)
            {
                const double chunk_duration_s = chunk.t1_s - chunk.t0_s;
                const double split_t_s = chunk.t0_s + chunk_duration_s * 0.5;
                subchunks.push_back(SubchunkRange{
                        .t0_s = chunk.t0_s,
                        .t1_s = split_t_s,
                        .overlap_s = 0.0,
                        .include_chunk_end_impulse = false,
                });
                subchunks.push_back(SubchunkRange{
                        .t0_s = split_t_s,
                        .t1_s = chunk.t1_s,
                        .overlap_s = effective_profile.seam_overlap_s,
                        .include_chunk_end_impulse = include_chunk_end_impulse,
                });
            }
            else
            {
                subchunks.push_back(SubchunkRange{
                        .t0_s = chunk.t0_s,
                        .t1_s = chunk.t1_s,
                        .overlap_s = effective_profile.seam_overlap_s,
                        .include_chunk_end_impulse = include_chunk_end_impulse,
                });
            }
            return subchunks;
        }

        bool collect_planned_subchunk_impulses(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::PredictionChunkPlan &chunk,
                const SubchunkRange &subchunk,
                const double extended_t1_s,
                OrbitPredictionService::AdaptiveStageDiagnostics &diagnostics,
                std::vector<OrbitPredictionService::ManeuverImpulse> &out_impulses)
        {
            out_impulses.clear();
            out_impulses.reserve(request.maneuver_impulses.size());
            for (const OrbitPredictionService::ManeuverImpulse &impulse : request.maneuver_impulses)
            {
                if (!planned_maneuver_impulse_input_is_valid(impulse))
                {
                    record_planned_maneuver_apply_failure(diagnostics, impulse.node_id);
                    return false;
                }
                if (impulse.t_s < (subchunk.t0_s - kPlannedChunkBoundaryEpsilonS))
                {
                    continue;
                }
                if (impulse.t_s > (extended_t1_s + kPlannedChunkBoundaryEpsilonS))
                {
                    break;
                }

                const bool before_outer_chunk_end = impulse.t_s < (chunk.t1_s - kPlannedChunkBoundaryEpsilonS);
                const bool at_outer_chunk_end =
                        std::abs(impulse.t_s - chunk.t1_s) <= kPlannedChunkBoundaryEpsilonS;
                const bool inside_overlap = impulse.t_s > (chunk.t1_s + kPlannedChunkBoundaryEpsilonS);
                const bool at_subchunk_end =
                        std::abs(impulse.t_s - subchunk.t1_s) <= kPlannedChunkBoundaryEpsilonS;
                if (at_subchunk_end && !subchunk.include_chunk_end_impulse && !inside_overlap)
                {
                    continue;
                }

                if (before_outer_chunk_end ||
                    (subchunk.include_chunk_end_impulse && at_outer_chunk_end) ||
                    inside_overlap)
                {
                    out_impulses.push_back(impulse);
                }
            }
            return true;
        }

        uint64_t build_upstream_maneuver_hash(
                const OrbitPredictionService::Request &request,
                const OrbitPredictionService::PredictionChunkPlan &chunk,
                const bool include_chunk_end_impulse,
                OrbitPredictionService::AdaptiveStageDiagnostics &diagnostics,
                OrbitPredictionService::Status &out_status)
        {
            uint64_t upstream_maneuver_hash = kPlannedCacheHashSeed;
            out_status = OrbitPredictionService::Status::Success;
            for (const OrbitPredictionService::ManeuverImpulse &upstream_impulse : request.maneuver_impulses)
            {
                if (!planned_maneuver_impulse_input_is_valid(upstream_impulse))
                {
                    record_planned_maneuver_apply_failure(diagnostics, upstream_impulse.node_id);
                    out_status = OrbitPredictionService::Status::InvalidInput;
                    return upstream_maneuver_hash;
                }
                if (upstream_impulse.t_s > (chunk.t1_s + kPlannedChunkBoundaryEpsilonS))
                {
                    break;
                }

                const bool before_chunk_end = upstream_impulse.t_s < (chunk.t1_s - kPlannedChunkBoundaryEpsilonS);
                const bool at_chunk_end =
                        std::abs(upstream_impulse.t_s - chunk.t1_s) <= kPlannedChunkBoundaryEpsilonS;
                if (before_chunk_end || (include_chunk_end_impulse && at_chunk_end))
                {
                    accumulate_planned_maneuver_cache_hash(upstream_maneuver_hash, upstream_impulse);
                }
            }
            return upstream_maneuver_hash;
        }
    } // namespace

    ChunkAttemptOutput solve_planned_chunk_attempt(
            PlannedTrajectoryContext &ctx,
            const PlannedChunkAttemptRequest &request)
    {
        using Status = OrbitPredictionService::Status;

        ChunkAttemptOutput attempt{};

        Status upstream_hash_status = Status::Success;
        const uint64_t upstream_maneuver_hash =
                build_upstream_maneuver_hash(ctx.request,
                                             request.chunk,
                                             request.include_chunk_end_impulse,
                                             attempt.diagnostics,
                                             upstream_hash_status);
        if (upstream_hash_status != Status::Success)
        {
            attempt.status = upstream_hash_status;
            return attempt;
        }

        const OrbitPredictionService::PlannedChunkCacheKey cache_key =
                make_planned_chunk_cache_key(ctx.request,
                                             request.chunk,
                                             request.effective_chunk.profile_id,
                                             request.baseline_generation_id,
                                             request.frame_independent_generation,
                                             upstream_maneuver_hash);
        if (request.effective_chunk.allow_reuse)
        {
            if (const auto cached = ctx.services.find_cached_chunk(cache_key, request.start_state);
                cached.has_value())
            {
                attempt.segments = cached->segments;
                attempt.seam_validation_segments = cached->seam_validation_segments;
                attempt.samples = cached->samples;
                attempt.previews = cached->previews;
                attempt.end_state = cached->end_state;
                attempt.diagnostics = cached->diagnostics;
                attempt.diagnostics.cache_reused = true;
                attempt.reused_from_cache = true;
                return attempt;
            }
        }

        const OrbitPredictionService::PredictionProfileDefinition effective_profile =
                resolve_prediction_profile_definition(ctx.request, request.effective_chunk);
        const std::vector<SubchunkRange> subchunks =
                build_planned_subchunks(request.chunk,
                                        effective_profile,
                                        request.split_chunk,
                                        request.include_chunk_end_impulse);

        const orbitsim::SpacecraftStateLookup empty_lookup{};
        double attempt_dt_sum_s = 0.0;
        bool attempt_have_dt = false;
        orbitsim::State subchunk_start_state = request.start_state;

        for (std::size_t subchunk_index = 0; subchunk_index < subchunks.size(); ++subchunk_index)
        {
            const SubchunkRange &subchunk = subchunks[subchunk_index];
            const double extended_t1_s = std::min(request_end_time_s(ctx.request),
                                                  subchunk.t1_s + std::max(0.0, subchunk.overlap_s));

            std::vector<OrbitPredictionService::ManeuverImpulse> chunk_impulses;
            if (!collect_planned_subchunk_impulses(ctx.request,
                                                   request.chunk,
                                                   subchunk,
                                                   extended_t1_s,
                                                   attempt.diagnostics,
                                                   chunk_impulses))
            {
                attempt.status = Status::InvalidInput;
                return attempt;
            }

            OrbitPredictionService::PredictionChunkPlan subchunk_plan = request.effective_chunk;
            subchunk_plan.t0_s = subchunk.t0_s;
            subchunk_plan.t1_s = extended_t1_s;
            subchunk_plan.requires_seam_validation = false;

            OrbitPredictionService::Request chunk_request = ctx.request;
            chunk_request.sim_time_s = subchunk.t0_s;
            chunk_request.future_window_s = extended_t1_s - subchunk.t0_s;
            chunk_request.maneuver_impulses = chunk_impulses;

            const orbitsim::AdaptiveSegmentOptions chunk_segment_opt =
                    build_spacecraft_adaptive_segment_options_for_chunk(chunk_request,
                                                                       subchunk_plan,
                                                                       ctx.cancel_requested);

            orbitsim::GameSimulation::Config chunk_sim_config = ctx.request.sim_config;
            apply_prediction_integrator_profile(chunk_sim_config,
                                                chunk_request,
                                                chunk_request.future_window_s);
            if (ctx.request.lagrange_sensitive)
            {
                apply_lagrange_integrator_profile(chunk_sim_config);
            }

            std::vector<orbitsim::MassiveBody> chunk_massive_bodies = ctx.out.massive_bodies;
            for (orbitsim::MassiveBody &body_at_start : chunk_massive_bodies)
            {
                std::size_t eph_body_index = 0u;
                if (body_at_start.id != orbitsim::kInvalidBodyId &&
                    ctx.ephemeris.body_index_for_id(body_at_start.id, &eph_body_index))
                {
                    body_at_start.state = ctx.ephemeris.body_state_at(eph_body_index, subchunk.t0_s);
                }
            }

            OrbitPredictionService::EphemerisBuildRequest chunk_ephemeris_request{};
            chunk_ephemeris_request.sim_time_s = subchunk.t0_s;
            chunk_ephemeris_request.sim_config = chunk_request.sim_config;
            chunk_ephemeris_request.massive_bodies = chunk_massive_bodies;
            chunk_ephemeris_request.duration_s = chunk_request.future_window_s;
            chunk_ephemeris_request.adaptive_options =
                    build_adaptive_ephemeris_options_for_chunk(chunk_request,
                                                              subchunk_plan,
                                                              ctx.cancel_requested);

            const OrbitPredictionService::SharedCelestialEphemeris chunk_shared_ephemeris =
                    ctx.services.get_or_build_ephemeris(chunk_ephemeris_request, ctx.cancel_requested);
            if (!chunk_shared_ephemeris || chunk_shared_ephemeris->empty())
            {
                attempt.status = ctx.cancel_requested() ? Status::Cancelled : Status::EphemerisUnavailable;
                return attempt;
            }
            if (!validate_ephemeris_continuity(*chunk_shared_ephemeris))
            {
                attempt.status = Status::ContinuityFailed;
                return attempt;
            }
            const orbitsim::CelestialEphemeris &chunk_eph = *chunk_shared_ephemeris;

            orbitsim::GameSimulation planned_sim(chunk_sim_config);
            if (!planned_sim.set_time_s(subchunk.t0_s))
            {
                attempt.status = Status::InvalidInput;
                return attempt;
            }

            for (const orbitsim::MassiveBody &body_at_start : chunk_massive_bodies)
            {
                const auto body_handle =
                        (body_at_start.id != orbitsim::kInvalidBodyId)
                                ? planned_sim.create_body_with_id(body_at_start.id, body_at_start)
                                : planned_sim.create_body(body_at_start);
                if (!body_handle.valid())
                {
                    attempt.status = Status::InvalidSubject;
                    return attempt;
                }
            }

            orbitsim::Spacecraft planned_ship{};
            planned_ship.state = subchunk_start_state;
            planned_ship.dry_mass_kg = 1.0;
            const auto planned_ship_h = planned_sim.create_spacecraft(planned_ship);
            if (!planned_ship_h.valid())
            {
                attempt.status = Status::InvalidSubject;
                return attempt;
            }

            orbitsim::ManeuverPlan chunk_plan{};
            chunk_plan.impulses.reserve(chunk_impulses.size());
            std::vector<OrbitPredictionService::ManeuverNodePreview> chunk_previews;
            chunk_previews.reserve(chunk_impulses.size());
            std::vector<PlannedSegmentBoundaryState> planned_boundary_states;
            planned_boundary_states.reserve(chunk_impulses.size());
            orbitsim::Spacecraft preview_spacecraft = planned_ship;
            preview_spacecraft.id = planned_ship_h.id;
            double preview_time_s = subchunk.t0_s;

            for (const OrbitPredictionService::ManeuverImpulse &src : chunk_impulses)
            {
                if (ctx.cancel_requested())
                {
                    attempt.status = Status::Cancelled;
                    return attempt;
                }

                if (src.t_s > preview_time_s)
                {
                    preview_spacecraft = orbitsim::detail::propagate_spacecraft_in_ephemeris(
                            preview_spacecraft,
                            planned_sim.massive_bodies(),
                            chunk_eph,
                            orbitsim::ManeuverPlan{},
                            planned_sim.config().gravitational_constant,
                            planned_sim.config().softening_length_m,
                            planned_sim.config().spacecraft_integrator,
                            preview_time_s,
                            src.t_s - preview_time_s,
                            empty_lookup);
                    preview_time_s = src.t_s;
                }

                orbitsim::ImpulseSegment impulse{};
                impulse.t_s = src.t_s;
                impulse.primary_body_id = src.primary_body_id;
                impulse.dv_rtn_mps = src.dv_rtn_mps;
                impulse.spacecraft_id = planned_ship_h.id;
                chunk_plan.impulses.push_back(impulse);

                const bool contributes_to_core_chunk =
                        src.t_s <= (subchunk.t1_s + kPlannedChunkBoundaryEpsilonS);
                std::vector<OrbitPredictionService::ManeuverNodePreview> *preview_sink =
                        contributes_to_core_chunk ? &chunk_previews : nullptr;
                std::vector<PlannedSegmentBoundaryState> *boundary_sink =
                        contributes_to_core_chunk ? &planned_boundary_states : nullptr;

                if (!apply_planned_maneuver_impulse_to_spacecraft(preview_spacecraft,
                                                                  src,
                                                                  chunk_massive_bodies,
                                                                  chunk_eph,
                                                                  planned_sim.config().softening_length_m,
                                                                  preview_sink,
                                                                  boundary_sink,
                                                                  attempt.diagnostics))
                {
                    attempt.status = Status::InvalidInput;
                    return attempt;
                }
            }

            orbitsim::sort_impulses_by_time(chunk_plan);
            planned_sim.maneuver_plan() = std::move(chunk_plan);

            orbitsim::AdaptiveSegmentDiagnostics chunk_diag{};
            std::vector<orbitsim::TrajectorySegment> chunk_segments =
                    orbitsim::predict_spacecraft_trajectory_segments_adaptive(
                            planned_sim,
                            chunk_eph,
                            planned_ship_h.id,
                            chunk_segment_opt,
                            &chunk_diag);
            if (!planned_boundary_states.empty())
            {
                std::vector<orbitsim::TrajectorySegment> split_planned =
                        split_trajectory_segments_at_known_boundaries(chunk_segments,
                                                                     planned_boundary_states);
                if (!split_planned.empty() && validate_trajectory_segment_continuity(split_planned))
                {
                    chunk_segments = std::move(split_planned);
                }
            }

            if (chunk_segments.empty() || !validate_trajectory_segment_continuity(chunk_segments))
            {
                attempt.status = chunk_segments.empty() ? Status::TrajectorySegmentsUnavailable
                                                        : Status::ContinuityFailed;
                return attempt;
            }

            std::vector<orbitsim::TrajectorySegment> core_segments =
                    slice_trajectory_segments(chunk_segments, subchunk.t0_s, subchunk.t1_s);
            if (core_segments.empty() || !validate_trajectory_segment_continuity(core_segments))
            {
                attempt.status = Status::ContinuityFailed;
                return attempt;
            }

            const OrbitPredictionService::PredictionChunkPlan sample_plan{
                    .chunk_id = request.effective_chunk.chunk_id,
                    .t0_s = subchunk.t0_s,
                    .t1_s = subchunk.t1_s,
                    .profile_id = request.effective_chunk.profile_id,
                    .boundary_flags = request.effective_chunk.boundary_flags,
                    .priority = request.effective_chunk.priority,
                    .allow_reuse = request.effective_chunk.allow_reuse,
                    .requires_seam_validation = false,
            };
            const std::size_t chunk_sample_count =
                    prediction_sample_budget_for_chunk(chunk_request,
                                                      sample_plan,
                                                      core_segments.size());
            std::vector<orbitsim::TrajectorySample> core_samples =
                    resample_segments_uniform(core_segments, chunk_sample_count);
            if (core_samples.size() < 2)
            {
                attempt.status = Status::TrajectorySamplesUnavailable;
                return attempt;
            }

            OrbitPredictionService::AdaptiveStageDiagnostics subchunk_diag =
                    make_stage_diagnostics_from_segments(core_segments, subchunk.t1_s - subchunk.t0_s);
            subchunk_diag.rejected_splits = chunk_diag.rejected_splits;
            subchunk_diag.forced_boundary_splits = chunk_diag.forced_boundary_splits;
            subchunk_diag.hard_cap_hit = chunk_diag.hard_cap_hit;
            subchunk_diag.cancelled = chunk_diag.cancelled;
            accumulate_planned_stage_diagnostics(attempt.diagnostics,
                                                 subchunk_diag,
                                                 attempt_dt_sum_s,
                                                 attempt_have_dt);

            attempt.previews.insert(attempt.previews.end(),
                                    chunk_previews.begin(),
                                    chunk_previews.end());
            attempt.segments.insert(attempt.segments.end(),
                                    std::make_move_iterator(core_segments.begin()),
                                    std::make_move_iterator(core_segments.end()));
            append_planned_chunk_samples(attempt.samples, std::move(core_samples));
            subchunk_start_state = attempt.segments.back().end;

            if (subchunk_index + 1u == subchunks.size() &&
                extended_t1_s > (subchunk.t1_s + kPlannedChunkBoundaryEpsilonS))
            {
                attempt.seam_validation_segments =
                        slice_trajectory_segments(chunk_segments, subchunk.t1_s, extended_t1_s);
            }
        }

        if (attempt.segments.empty() || attempt.samples.size() < 2)
        {
            attempt.status = attempt.segments.empty() ? Status::TrajectorySegmentsUnavailable
                                                      : Status::TrajectorySamplesUnavailable;
            return attempt;
        }

        if (attempt.diagnostics.accepted_segments > 0u)
        {
            attempt.diagnostics.avg_dt_s =
                    attempt_dt_sum_s / static_cast<double>(attempt.diagnostics.accepted_segments);
        }
        attempt.end_state = attempt.segments.back().end;

        if (request.effective_chunk.allow_reuse)
        {
            OrbitPredictionService::PlannedChunkCacheEntry cache_entry{};
            cache_entry.key = cache_key;
            cache_entry.start_state = request.start_state;
            cache_entry.end_state = attempt.end_state;
            cache_entry.diagnostics = attempt.diagnostics;
            cache_entry.segments = attempt.segments;
            cache_entry.seam_validation_segments = attempt.seam_validation_segments;
            cache_entry.samples = attempt.samples;
            cache_entry.previews = attempt.previews;
            ctx.services.store_cached_chunk(std::move(cache_entry));
        }

        return attempt;
    }
} // namespace Game
