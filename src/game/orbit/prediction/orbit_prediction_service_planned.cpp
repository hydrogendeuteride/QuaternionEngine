#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/detail/spacecraft_propagation.hpp"
#include "orbitsim/frame_utils.hpp"
#include "orbitsim/maneuvers.hpp"
#include "orbitsim/maneuvers_types.hpp"
#include "orbitsim/spacecraft_state_cache.hpp"
#include "orbitsim/trajectories.hpp"
#include "orbitsim/trajectory_transforms.hpp"

#include <chrono>
#include <iterator>

namespace Game
{
    namespace
    {
        constexpr std::size_t kMaxCachedPlannedChunks = 512u;
        // Cache keys only select candidates; find_cached_chunk validates state continuity before reuse.
        constexpr double kBaselineCacheStartTimeQuantumS = 0.001;
        constexpr double kBaselineCachePositionQuantumM = 1.0;
        constexpr double kBaselineCacheVelocityQuantumMps = 1.0e-3;
        constexpr double kBaselineCacheSpinAxisQuantum = 1.0e-6;
        constexpr double kBaselineCacheSpinAngleQuantumRad = 1.0e-6;
        constexpr double kBaselineCacheSpinRateQuantumRadPerS = 1.0e-6;
        constexpr double kManeuverCacheTimeQuantumS = 0.001;
        constexpr double kManeuverCacheDvQuantumMps = 1.0e-6;

        template<typename T>
        void hash_combine(uint64_t &seed, const T &value)
        {
            seed ^= static_cast<uint64_t>(std::hash<T>{}(value)) + 0x9e3779b97f4a7c15ULL + (seed << 6u) + (seed >> 2u);
        }

        uint64_t hash_vec3(const orbitsim::Vec3 &v)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            hash_combine(seed, v.x);
            hash_combine(seed, v.y);
            hash_combine(seed, v.z);
            return seed;
        }

        int64_t quantized_cache_tick(const double value, const double quantum)
        {
            if (!std::isfinite(value) || !(quantum > 0.0))
            {
                return 0;
            }

            const long double scaled = static_cast<long double>(value) / static_cast<long double>(quantum);
            const long double rounded = std::round(scaled);
            constexpr long double min_tick =
                    static_cast<long double>(std::numeric_limits<int64_t>::min());
            constexpr long double max_tick =
                    static_cast<long double>(std::numeric_limits<int64_t>::max());
            if (rounded <= min_tick)
            {
                return std::numeric_limits<int64_t>::min();
            }
            if (rounded >= max_tick)
            {
                return std::numeric_limits<int64_t>::max();
            }
            return static_cast<int64_t>(rounded);
        }

        double planned_chunk_cache_time_quantum_s(const OrbitPredictionService::PredictionProfileId profile_id)
        {
            switch (profile_id)
            {
            case OrbitPredictionService::PredictionProfileId::Exact:
                return 0.001;
            case OrbitPredictionService::PredictionProfileId::Near:
                return 0.1;
            case OrbitPredictionService::PredictionProfileId::Tail:
                return 1.0;
            }
            return 0.1;
        }

        uint64_t hash_quantized_vec3(const orbitsim::Vec3 &v, const double quantum)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            hash_combine(seed, quantized_cache_tick(v.x, quantum));
            hash_combine(seed, quantized_cache_tick(v.y, quantum));
            hash_combine(seed, quantized_cache_tick(v.z, quantum));
            return seed;
        }

        uint64_t hash_quantized_state(const orbitsim::State &state)
        {
            uint64_t seed = hash_quantized_vec3(state.position_m, kBaselineCachePositionQuantumM);
            hash_combine(seed, hash_quantized_vec3(state.velocity_mps, kBaselineCacheVelocityQuantumMps));
            hash_combine(seed, quantized_cache_tick(state.spin.axis.x, kBaselineCacheSpinAxisQuantum));
            hash_combine(seed, quantized_cache_tick(state.spin.axis.y, kBaselineCacheSpinAxisQuantum));
            hash_combine(seed, quantized_cache_tick(state.spin.axis.z, kBaselineCacheSpinAxisQuantum));
            hash_combine(seed, quantized_cache_tick(state.spin.angle_rad, kBaselineCacheSpinAngleQuantumRad));
            hash_combine(seed, quantized_cache_tick(state.spin.rate_rad_per_s, kBaselineCacheSpinRateQuantumRadPerS));
            return seed;
        }

        uint64_t hash_state(const orbitsim::State &state)
        {
            uint64_t seed = hash_vec3(state.position_m);
            hash_combine(seed, hash_vec3(state.velocity_mps));
            hash_combine(seed, state.spin.axis.x);
            hash_combine(seed, state.spin.axis.y);
            hash_combine(seed, state.spin.axis.z);
            hash_combine(seed, state.spin.angle_rad);
            hash_combine(seed, state.spin.rate_rad_per_s);
            return seed;
        }

        uint64_t hash_massive_body_set(const std::vector<orbitsim::MassiveBody> &bodies)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            for (const orbitsim::MassiveBody &body : bodies)
            {
                hash_combine(seed, body.id);
                hash_combine(seed, body.mass_kg);
                hash_combine(seed, hash_state(body.state));
            }
            return seed;
        }

        uint64_t hash_solver_context(const OrbitPredictionService::Request &request,
                                     const std::vector<orbitsim::MassiveBody> &massive_bodies)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            hash_combine(seed, static_cast<uint8_t>(request.solve_quality));
            hash_combine(seed, request.thrusting);
            hash_combine(seed, request.lagrange_sensitive);
            hash_combine(seed, request.sim_config.gravitational_constant);
            hash_combine(seed, request.sim_config.softening_length_m);
            hash_combine(seed, request.sim_config.spacecraft_integrator.adaptive);
            hash_combine(seed, request.sim_config.spacecraft_integrator.max_step_s);
            hash_combine(seed, request.sim_config.spacecraft_integrator.abs_tol);
            hash_combine(seed, request.sim_config.spacecraft_integrator.rel_tol);
            hash_combine(seed, request.sim_config.spacecraft_integrator.max_substeps);
            hash_combine(seed, request.sim_config.spacecraft_integrator.max_substeps_hard);
            hash_combine(seed, request.sim_config.spacecraft_integrator.max_interval_splits);
            hash_combine(seed, request.preferred_primary_body_id);
            hash_combine(seed, hash_massive_body_set(massive_bodies));
            return seed;
        }

        uint64_t hash_baseline_generation(const OrbitPredictionService::Request &request,
                                          const double start_time_s,
                                          const orbitsim::State &start_state)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            hash_combine(seed, request.track_id);
            hash_combine(seed, quantized_cache_tick(start_time_s, kBaselineCacheStartTimeQuantumS));
            hash_combine(seed, hash_quantized_state(start_state));
            return seed;
        }

        uint64_t hash_maneuver_impulse_for_cache(const OrbitPredictionService::ManeuverImpulse &impulse)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            hash_combine(seed, impulse.node_id);
            hash_combine(seed, quantized_cache_tick(impulse.t_s, kManeuverCacheTimeQuantumS));
            hash_combine(seed, impulse.primary_body_id);
            hash_combine(seed, quantized_cache_tick(impulse.dv_rtn_mps.x, kManeuverCacheDvQuantumMps));
            hash_combine(seed, quantized_cache_tick(impulse.dv_rtn_mps.y, kManeuverCacheDvQuantumMps));
            hash_combine(seed, quantized_cache_tick(impulse.dv_rtn_mps.z, kManeuverCacheDvQuantumMps));
            return seed;
        }

        void accumulate_stage_diagnostics(OrbitPredictionService::AdaptiveStageDiagnostics &dst,
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

        void append_chunk_samples(std::vector<orbitsim::TrajectorySample> &dst,
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

        bool planned_chunk_cache_key_matches(const OrbitPredictionService::PlannedChunkCacheKey &a,
                                             const OrbitPredictionService::PlannedChunkCacheKey &b)
        {
            return a.track_id == b.track_id &&
                   a.baseline_generation_id == b.baseline_generation_id &&
                   a.upstream_maneuver_hash == b.upstream_maneuver_hash &&
                   a.frame_independent_generation == b.frame_independent_generation &&
                   a.chunk_t0_tick == b.chunk_t0_tick &&
                   a.chunk_t1_tick == b.chunk_t1_tick &&
                   a.profile_id == b.profile_id;
        }

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

        bool apply_maneuver_impulse_to_spacecraft(orbitsim::Spacecraft &spacecraft,
                                                  const OrbitPredictionService::ManeuverImpulse &src,
                                                  const std::vector<orbitsim::MassiveBody> &massive_bodies,
                                                  const orbitsim::CelestialEphemeris &ephemeris,
                                                  const double softening_length_m,
                                                  std::vector<OrbitPredictionService::ManeuverNodePreview> *out_previews,
                                                  std::vector<PlannedSegmentBoundaryState> *out_boundaries)
        {
            if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
            {
                return true;
            }

            OrbitPredictionService::ManeuverNodePreview preview{};
            preview.node_id = src.node_id;
            preview.t_s = src.t_s;

            const orbitsim::State pre_burn_state = spacecraft.state;
            const bool have_preview = build_maneuver_preview(spacecraft.state, src.t_s, preview);
            if (have_preview && out_previews)
            {
                out_previews->push_back(preview);
            }

            if (have_preview)
            {
                std::optional<std::size_t> primary_index = orbitsim::body_index_for_id(massive_bodies, src.primary_body_id);
                if (!primary_index.has_value())
                {
                    primary_index = orbitsim::auto_select_primary_index(
                            massive_bodies,
                            preview.inertial_position_m,
                            [&ephemeris, time_s = src.t_s](const std::size_t i) -> orbitsim::Vec3 {
                                return ephemeris.body_position_at(i, time_s);
                            },
                            softening_length_m);
                }

                if (primary_index.has_value() && *primary_index < massive_bodies.size())
                {
                    const orbitsim::Vec3 dv_inertial_mps = orbitsim::rtn_vector_to_inertial(
                            ephemeris,
                            massive_bodies,
                            *primary_index,
                            src.t_s,
                            preview.inertial_position_m,
                            preview.inertial_velocity_mps,
                            src.dv_rtn_mps);
                    if (finite_vec3(dv_inertial_mps))
                    {
                        spacecraft.state.velocity_mps += dv_inertial_mps;
                    }
                }
            }

            if (out_boundaries)
            {
                append_or_merge_planned_boundary_state(
                        *out_boundaries,
                        src.t_s,
                        pre_burn_state,
                        spacecraft.state);
            }
            return true;
        }
    } // namespace

    void append_planned_chunk_packet(PlannedSolveOutput &planned, PlannedChunkPacket packet)
    {
        planned.chunk_reused.push_back(packet.reused_from_cache);
        planned.previews.insert(planned.previews.end(),
                                std::make_move_iterator(packet.previews.begin()),
                                std::make_move_iterator(packet.previews.end()));
        planned.segments.insert(planned.segments.end(),
                                std::make_move_iterator(packet.segments.begin()),
                                std::make_move_iterator(packet.segments.end()));
        append_chunk_samples(planned.samples, std::move(packet.samples));
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

        constexpr double kChunkBoundaryEpsilonS = 1.0e-9;
        orbitsim::State chunk_start_state = range_start_state;
        summary.end_state = range_start_state;
        const uint64_t baseline_generation_id =
                hash_baseline_generation(ctx.request, solve_plan.t0_s, range_start_state);
        const uint64_t frame_independent_generation =
                hash_solver_context(ctx.request, ctx.out.massive_bodies);
        double diagnostic_dt_sum_s = 0.0;
        bool have_dt = false;
        const orbitsim::SpacecraftStateLookup empty_lookup{};
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

            const auto solve_chunk_attempt =
                    [&](const OrbitPredictionService::PredictionChunkPlan &effective_chunk,
                        const bool split_chunk,
                        const orbitsim::State &attempt_start_state) -> ChunkAttemptOutput {
                ChunkAttemptOutput attempt{};

                uint64_t upstream_maneuver_hash = 0xcbf29ce484222325ULL;
                for (const OrbitPredictionService::ManeuverImpulse &upstream_impulse : ctx.request.maneuver_impulses)
                {
                    if (!std::isfinite(upstream_impulse.t_s) || !finite_vec3(upstream_impulse.dv_rtn_mps))
                    {
                        continue;
                    }
                    if (upstream_impulse.t_s > (chunk.t1_s + kChunkBoundaryEpsilonS))
                    {
                        break;
                    }

                    const bool before_chunk_end = upstream_impulse.t_s < (chunk.t1_s - kChunkBoundaryEpsilonS);
                    const bool at_chunk_end =
                            std::abs(upstream_impulse.t_s - chunk.t1_s) <= kChunkBoundaryEpsilonS;
                    if (before_chunk_end || (include_chunk_end_impulse && at_chunk_end))
                    {
                        hash_combine(upstream_maneuver_hash, hash_maneuver_impulse_for_cache(upstream_impulse));
                    }
                }

                const double chunk_cache_time_quantum_s =
                        planned_chunk_cache_time_quantum_s(effective_chunk.profile_id);
                const OrbitPredictionService::PlannedChunkCacheKey cache_key{
                        .track_id = ctx.request.track_id,
                        .baseline_generation_id = baseline_generation_id,
                        .upstream_maneuver_hash = upstream_maneuver_hash,
                        .frame_independent_generation = frame_independent_generation,
                        .chunk_t0_s = chunk.t0_s,
                        .chunk_t1_s = chunk.t1_s,
                        .chunk_t0_tick = quantized_cache_tick(chunk.t0_s, chunk_cache_time_quantum_s),
                        .chunk_t1_tick = quantized_cache_tick(chunk.t1_s, chunk_cache_time_quantum_s),
                        .profile_id = effective_chunk.profile_id,
                };
                if (effective_chunk.allow_reuse)
                {
                    if (const auto cached = ctx.find_cached_chunk(cache_key, attempt_start_state); cached.has_value())
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
                        resolve_prediction_profile_definition(ctx.request, effective_chunk);

                struct SubchunkRange
                {
                    double t0_s{0.0};
                    double t1_s{0.0};
                    double overlap_s{0.0};
                    bool include_chunk_end_impulse{false};
                };

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

                double attempt_dt_sum_s = 0.0;
                bool attempt_have_dt = false;
                orbitsim::State subchunk_start_state = attempt_start_state;

                for (std::size_t subchunk_index = 0; subchunk_index < subchunks.size(); ++subchunk_index)
                {
                    const SubchunkRange &subchunk = subchunks[subchunk_index];
                    const double extended_t1_s = std::min(request_end_time_s(ctx.request),
                                                          subchunk.t1_s + std::max(0.0, subchunk.overlap_s));

                    std::vector<OrbitPredictionService::ManeuverImpulse> chunk_impulses;
                    chunk_impulses.reserve(ctx.request.maneuver_impulses.size());
                    for (const OrbitPredictionService::ManeuverImpulse &impulse : ctx.request.maneuver_impulses)
                    {
                        if (!std::isfinite(impulse.t_s) || !finite_vec3(impulse.dv_rtn_mps))
                        {
                            continue;
                        }
                        if (impulse.t_s < (subchunk.t0_s - kChunkBoundaryEpsilonS))
                        {
                            continue;
                        }
                        if (impulse.t_s > (extended_t1_s + kChunkBoundaryEpsilonS))
                        {
                            break;
                        }

                        const bool before_outer_chunk_end = impulse.t_s < (chunk.t1_s - kChunkBoundaryEpsilonS);
                        const bool at_outer_chunk_end =
                                std::abs(impulse.t_s - chunk.t1_s) <= kChunkBoundaryEpsilonS;
                        const bool inside_overlap = impulse.t_s > (chunk.t1_s + kChunkBoundaryEpsilonS);
                        const bool at_subchunk_end =
                                std::abs(impulse.t_s - subchunk.t1_s) <= kChunkBoundaryEpsilonS;
                        if (at_subchunk_end && !subchunk.include_chunk_end_impulse && !inside_overlap)
                        {
                            continue;
                        }

                        if (before_outer_chunk_end ||
                            (subchunk.include_chunk_end_impulse && at_outer_chunk_end) ||
                            inside_overlap)
                        {
                            chunk_impulses.push_back(impulse);
                        }
                    }

                    OrbitPredictionService::PredictionChunkPlan subchunk_plan = effective_chunk;
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
                            ctx.get_or_build_ephemeris(chunk_ephemeris_request, ctx.cancel_requested);
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
                                src.t_s <= (subchunk.t1_s + kChunkBoundaryEpsilonS);
                        std::vector<OrbitPredictionService::ManeuverNodePreview> *preview_sink =
                                contributes_to_core_chunk ? &chunk_previews : nullptr;
                        std::vector<PlannedSegmentBoundaryState> *boundary_sink =
                                contributes_to_core_chunk ? &planned_boundary_states : nullptr;

                        if (!apply_maneuver_impulse_to_spacecraft(preview_spacecraft,
                                                                  src,
                                                                  chunk_massive_bodies,
                                                                  chunk_eph,
                                                                  planned_sim.config().softening_length_m,
                                                                  preview_sink,
                                                                  boundary_sink))
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
                            .chunk_id = effective_chunk.chunk_id,
                            .t0_s = subchunk.t0_s,
                            .t1_s = subchunk.t1_s,
                            .profile_id = effective_chunk.profile_id,
                            .boundary_flags = effective_chunk.boundary_flags,
                            .priority = effective_chunk.priority,
                            .allow_reuse = effective_chunk.allow_reuse,
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
                    accumulate_stage_diagnostics(attempt.diagnostics,
                                                 subchunk_diag,
                                                 attempt_dt_sum_s,
                                                 attempt_have_dt);

                    attempt.previews.insert(attempt.previews.end(),
                                            chunk_previews.begin(),
                                            chunk_previews.end());
                    attempt.segments.insert(attempt.segments.end(),
                                            std::make_move_iterator(core_segments.begin()),
                                            std::make_move_iterator(core_segments.end()));
                    append_chunk_samples(attempt.samples, std::move(core_samples));
                    subchunk_start_state = attempt.segments.back().end;

                    if (subchunk_index + 1u == subchunks.size() &&
                        extended_t1_s > (subchunk.t1_s + kChunkBoundaryEpsilonS))
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

                if (effective_chunk.allow_reuse)
                {
                    OrbitPredictionService::PlannedChunkCacheEntry cache_entry{};
                    cache_entry.key = cache_key;
                    cache_entry.start_state = attempt_start_state;
                    cache_entry.end_state = attempt.end_state;
                    cache_entry.diagnostics = attempt.diagnostics;
                    cache_entry.segments = attempt.segments;
                    cache_entry.seam_validation_segments = attempt.seam_validation_segments;
                    cache_entry.samples = attempt.samples;
                    cache_entry.previews = attempt.previews;
                    ctx.store_cached_chunk(std::move(cache_entry));
                }

                return attempt;
            };

            OrbitPredictionService::PredictionChunkPlan effective_chunk = chunk;
            effective_chunk.profile_id = activity_probe.recommended_profile_id;
            effective_chunk.allow_reuse = chunk.allow_reuse &&
                                          effective_chunk.profile_id !=
                                                  OrbitPredictionService::PredictionProfileId::Exact;

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
                chunk_attempt = solve_chunk_attempt(effective_chunk, split_chunk, chunk_start_state);
                if (chunk_attempt.status != Status::Success)
                {
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
                effective_chunk.allow_reuse = chunk.allow_reuse &&
                                              effective_chunk.profile_id !=
                                                      OrbitPredictionService::PredictionProfileId::Exact;
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
            chunk_packet.reused_from_cache = chunk_attempt.reused_from_cache;
            chunk_packet.previews = std::move(chunk_attempt.previews);
            chunk_packet.segments = std::move(chunk_attempt.segments);
            chunk_packet.samples = std::move(chunk_attempt.samples);
            if (!chunk_sink(std::move(chunk_packet)))
            {
                summary.status = Status::Cancelled;
                return summary;
            }

            accumulate_stage_diagnostics(summary.diagnostics,
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
