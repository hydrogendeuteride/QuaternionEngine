#include "game/orbit/orbit_prediction_service.h"
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

namespace Game
{
    namespace
    {
        bool planned_chunk_cache_key_matches(const OrbitPredictionService::PlannedChunkCacheKey &a,
                                             const OrbitPredictionService::PlannedChunkCacheKey &b)
        {
            return a.track_id == b.track_id &&
                   a.baseline_generation_id == b.baseline_generation_id &&
                   a.upstream_maneuver_hash == b.upstream_maneuver_hash &&
                   a.frame_independent_generation == b.frame_independent_generation &&
                   a.chunk_t0_s == b.chunk_t0_s &&
                   a.chunk_t1_s == b.chunk_t1_s &&
                   a.profile_id == b.profile_id;
        }
    } // namespace

    void OrbitPredictionService::compute_prediction(const PendingJob &job)
    {
        const uint64_t generation_id = job.generation_id;
        const Request &request = job.request;
        const uint64_t request_epoch = job.request_epoch;

        const std::chrono::steady_clock::time_point compute_start = std::chrono::steady_clock::now();
        Result out{};
        out.generation_id = generation_id;
        out.track_id = request.track_id;
        out.build_time_s = request.sim_time_s;
        out.solve_quality = request.solve_quality;
        const auto cancel_requested = [this,
                                       track_id = request.track_id,
                                       generation_id,
                                       request_epoch]() {
            return !should_continue_job(track_id, generation_id, request_epoch);
        };
        const auto elapsed_ms = [&compute_start]() {
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compute_start).count();
        };
        const auto publish = [this, &job, &elapsed_ms](Result result) {
            result.compute_time_ms = elapsed_ms();
            return publish_completed_result(job, std::move(result));
        };
        const auto fail = [&out, &publish](const Status status) {
            out.diagnostics.status = status;
            publish(out);
            return;
        };
        const auto cancel = [&out, &publish]() {
            out.diagnostics.cancelled = true;
            out.diagnostics.status = Status::Cancelled;
            publish(out);
            return;
        };
        const auto sync_stage_counts = [&out]() {
            out.diagnostics.ephemeris_segment_count = out.diagnostics.ephemeris.accepted_segments;
            out.diagnostics.trajectory_segment_count = out.diagnostics.trajectory_base.accepted_segments;
            out.diagnostics.trajectory_segment_count_planned = out.diagnostics.trajectory_planned.accepted_segments;
        };

        // Invalid inputs produce an invalid result rather than throwing on the worker thread.
        if (!std::isfinite(request.sim_time_s))
        {
            fail(Status::InvalidInput);
            return;
        }

        std::optional<EphemerisSamplingSpec> spacecraft_sampling_spec{};
        if (request.kind == RequestKind::Spacecraft)
        {
            spacecraft_sampling_spec = build_ephemeris_sampling_spec(request);
            if (!spacecraft_sampling_spec->valid)
            {
                fail(Status::InvalidSamplingSpec);
                return;
            }
        }

        orbitsim::GameSimulation::Config sim_config = request.sim_config;
        const double resolved_integrator_horizon_s =
                spacecraft_sampling_spec.has_value()
                        ? spacecraft_sampling_spec->horizon_s
                        : std::max(OrbitPredictionTuning::kMinHorizonS,
                                   std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s)
                                                                          : 0.0);
        apply_prediction_integrator_profile(sim_config, request, resolved_integrator_horizon_s);
        if (request.lagrange_sensitive)
        {
            apply_lagrange_integrator_profile(sim_config);
        }

        orbitsim::GameSimulation sim(sim_config);
        if (!sim.set_time_s(request.sim_time_s))
        {
            fail(Status::InvalidInput);
            return;
        }

        if (cancel_requested())
        {
            cancel();
            return;
        }

        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            const auto body_handle =
                    (body.id != orbitsim::kInvalidBodyId)
                        ? sim.create_body_with_id(body.id, body)
                        : sim.create_body(body);
            if (!body_handle.valid())
            {
                fail(Status::InvalidSubject);
                return;
            }
        }

        if (cancel_requested())
        {
            cancel();
            return;
        }

        out.massive_bodies = sim.massive_bodies();

        // Celestial route: predict the body's inertial motion.
        if (request.kind == RequestKind::Celestial)
        {
            if (request.subject_body_id == orbitsim::kInvalidBodyId)
            {
                fail(Status::InvalidSubject);
                return;
            }

            const orbitsim::MassiveBody *subject_sim = sim.body_by_id(request.subject_body_id);
            if (!subject_sim)
            {
                fail(Status::InvalidSubject);
                return;
            }

            const CelestialPredictionSamplingSpec sampling_spec =
                    build_celestial_prediction_sampling_spec(request, *subject_sim);
            if (!sampling_spec.valid)
            {
                fail(Status::InvalidSamplingSpec);
                return;
            }

            SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
            AdaptiveStageDiagnostics ephemeris_diag{};
            if (!ephemeris_covers_horizon(shared_ephemeris, request.sim_time_s, sampling_spec.horizon_s))
            {
                // Keep ephemeris construction on the worker so gameplay only enqueues work.
                EphemerisSamplingSpec ephemeris_sampling_spec{};
                ephemeris_sampling_spec.valid = true;
                ephemeris_sampling_spec.horizon_s = sampling_spec.horizon_s;
                shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, ephemeris_sampling_spec),
                                                          cancel_requested,
                                                          &ephemeris_diag);
            }
            else
            {
                ephemeris_diag = make_stage_diagnostics_from_ephemeris(shared_ephemeris, sampling_spec.horizon_s, true);
            }
            if (!shared_ephemeris || shared_ephemeris->empty())
            {
                fail(Status::EphemerisUnavailable);
                return;
            }
            if (!validate_ephemeris_continuity(*shared_ephemeris))
            {
                fail(Status::ContinuityFailed);
                return;
            }
            out.diagnostics.ephemeris = ephemeris_diag;
            sync_stage_counts();

            if (cancel_requested())
            {
                cancel();
                return;
            }

            constexpr std::size_t kCelestialUISampleCount = 2'000;
            const std::size_t celestial_sample_count = kCelestialUISampleCount;
            std::vector<orbitsim::TrajectorySample> traj_inertial =
                    resample_ephemeris_uniform(*shared_ephemeris, subject_sim->id,
                                              request.sim_time_s,
                                              request.sim_time_s + sampling_spec.horizon_s,
                                              celestial_sample_count);
            if (traj_inertial.size() < 2)
            {
                fail(Status::TrajectorySamplesUnavailable);
                return;
            }

            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial =
                    trajectory_segments_from_body_ephemeris(*shared_ephemeris, subject_sim->id);
            if (traj_segments_inertial.empty())
            {
                fail(Status::TrajectorySegmentsUnavailable);
                return;
            }
            if (!validate_trajectory_segment_continuity(traj_segments_inertial))
            {
                fail(Status::ContinuityFailed);
                return;
            }

            out.shared_ephemeris = shared_ephemeris;
            out.diagnostics.trajectory_base =
                    make_stage_diagnostics_from_segments(traj_segments_inertial, sampling_spec.horizon_s);
            out.diagnostics.trajectory_sample_count = traj_inertial.size();
            sync_stage_counts();
            out.trajectory_inertial = std::move(traj_inertial);
            out.trajectory_segments_inertial = std::move(traj_segments_inertial);
            out.valid = true;
            out.diagnostics.status = Status::Success;
            publish(std::move(out));
            return;
        }

        // Spacecraft route: build a transient ship state, then predict inertial baseline and planned trajectories.
        const glm::dvec3 ship_bary_pos_m = request.ship_bary_position_m;
        const glm::dvec3 ship_bary_vel_mps = request.ship_bary_velocity_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            fail(Status::InvalidInput);
            return;
        }

        const EphemerisSamplingSpec sampling_spec = *spacecraft_sampling_spec;
        const double horizon_s = sampling_spec.horizon_s;
        orbitsim::Spacecraft ship_sc{};
        ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
        ship_sc.dry_mass_kg = 1.0;

        const auto ship_h = sim.create_spacecraft(ship_sc);
        if (!ship_h.valid())
        {
            fail(Status::InvalidSubject);
            return;
        }

        sim.maneuver_plan() = orbitsim::ManeuverPlan{};

        const orbitsim::AdaptiveSegmentOptions segment_opt =
                build_spacecraft_adaptive_segment_options(request, sampling_spec, cancel_requested);

        SharedCelestialEphemeris shared_ephemeris = request.shared_ephemeris;
        AdaptiveStageDiagnostics ephemeris_diag{};
        if (!ephemeris_covers_horizon(shared_ephemeris, request.sim_time_s, horizon_s))
        {
            // Spacecraft requests follow the same rule: never build ephemeris on the gameplay thread.
            shared_ephemeris = get_or_build_ephemeris(build_ephemeris_build_request(request, sampling_spec),
                                                      cancel_requested,
                                                      &ephemeris_diag);
        }
        else
        {
            ephemeris_diag = make_stage_diagnostics_from_ephemeris(shared_ephemeris, horizon_s, true);
        }
        if (!shared_ephemeris || shared_ephemeris->empty())
        {
            fail(Status::EphemerisUnavailable);
            return;
        }
        if (!validate_ephemeris_continuity(*shared_ephemeris))
        {
            fail(Status::ContinuityFailed);
            return;
        }
        out.diagnostics.ephemeris = ephemeris_diag;
        sync_stage_counts();

        if (cancel_requested())
        {
            cancel();
            return;
        }
        const orbitsim::CelestialEphemeris &eph = *shared_ephemeris;
        const std::optional<ReusableBaselineCacheEntry> reusable_baseline =
                request_can_reuse_spacecraft_baseline(request)
                        ? find_reusable_baseline(request.track_id, request_epoch)
                        : std::nullopt;
        const double reusable_window_s =
                std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s) : 0.0;

        bool reused_baseline = false;
        if (reusable_baseline.has_value() &&
            reusable_baseline->trajectory_inertial.size() >= 2 &&
            validate_trajectory_segment_continuity(reusable_baseline->trajectory_segments_inertial) &&
            trajectory_segments_cover_window(reusable_baseline->trajectory_segments_inertial,
                                             request.sim_time_s,
                                             reusable_window_s))
        {
            out.diagnostics.trajectory_base =
                    make_stage_diagnostics_from_segments(reusable_baseline->trajectory_segments_inertial, horizon_s, true);
            out.diagnostics.trajectory_sample_count = reusable_baseline->trajectory_inertial.size();
            sync_stage_counts();
            out.shared_ephemeris = shared_ephemeris;
            out.trajectory_segments_inertial = reusable_baseline->trajectory_segments_inertial;
            out.trajectory_inertial = reusable_baseline->trajectory_inertial;
            reused_baseline = true;
            out.baseline_reused = true;
        }

        if (!reused_baseline)
        {
            orbitsim::AdaptiveSegmentDiagnostics base_diag{};
            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_baseline =
                    orbitsim::predict_spacecraft_trajectory_segments_adaptive(sim, eph, ship_h.id, segment_opt, &base_diag);
            if (traj_segments_inertial_baseline.empty())
            {
                fail(Status::TrajectorySegmentsUnavailable);
                return;
            }
            if (!validate_trajectory_segment_continuity(traj_segments_inertial_baseline))
            {
                fail(Status::ContinuityFailed);
                return;
            }
            out.diagnostics.trajectory_base = make_stage_diagnostics_from_adaptive(base_diag, horizon_s);
            out.diagnostics.trajectory_base.accepted_segments = traj_segments_inertial_baseline.size();
            out.diagnostics.trajectory_base.covered_duration_s = prediction_segment_span_s(traj_segments_inertial_baseline);
            sync_stage_counts();

            if (cancel_requested())
            {
                cancel();
                return;
            }

            const std::size_t baseline_sample_count =
                    prediction_sample_budget(request, traj_segments_inertial_baseline.size());
            std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                    resample_segments_uniform(traj_segments_inertial_baseline, baseline_sample_count);
            if (traj_inertial_baseline.size() < 2)
            {
                fail(Status::TrajectorySamplesUnavailable);
                return;
            }
            out.diagnostics.trajectory_sample_count = traj_inertial_baseline.size();
            sync_stage_counts();

            if (cancel_requested())
            {
                cancel();
                return;
            }

            out.shared_ephemeris = shared_ephemeris;
            out.trajectory_segments_inertial = std::move(traj_segments_inertial_baseline);
            out.trajectory_inertial = std::move(traj_inertial_baseline);
        }

        if (!request.maneuver_impulses.empty())
        {
            std::vector<ManeuverImpulse> maneuver_impulses = request.maneuver_impulses;
            std::stable_sort(maneuver_impulses.begin(),
                             maneuver_impulses.end(),
                             [](const ManeuverImpulse &a, const ManeuverImpulse &b) { return a.t_s < b.t_s; });

            // Create a modified request with sorted impulses for planned trajectory solving.
            Request planned_request = request;
            planned_request.maneuver_impulses = std::move(maneuver_impulses);

            PlannedTrajectoryContext ctx{
                    planned_request,
                    cancel_requested,
                    out,
                    eph,
                    publish,
                    // find_cached_chunk closure
                    [this](const PlannedChunkCacheKey &key,
                           const orbitsim::State &expected_start_state)
                            -> std::optional<PlannedChunkCacheEntry> {
                        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
                        for (PlannedChunkCacheEntry &entry : _planned_chunk_cache)
                        {
                            if (!planned_chunk_cache_key_matches(entry.key, key) ||
                                !states_are_continuous(entry.start_state, expected_start_state) ||
                                entry.samples.size() < 2u ||
                                entry.segments.empty() ||
                                !validate_trajectory_segment_continuity(entry.segments) ||
                                (!entry.seam_validation_segments.empty() &&
                                 !validate_trajectory_segment_continuity(entry.seam_validation_segments)))
                            {
                                continue;
                            }

                            entry.last_use_serial = _next_planned_chunk_cache_use_serial++;
                            return entry;
                        }
                        return std::nullopt;
                    },
                    // store_cached_chunk closure
                    [this](PlannedChunkCacheEntry entry) {
                        constexpr std::size_t kMaxCachedPlannedChunks = 512u;
                        if (entry.samples.size() < 2u ||
                            entry.segments.empty() ||
                            !validate_trajectory_segment_continuity(entry.segments) ||
                            (!entry.seam_validation_segments.empty() &&
                             !validate_trajectory_segment_continuity(entry.seam_validation_segments)) ||
                            !finite_state(entry.start_state) ||
                            !finite_state(entry.end_state))
                        {
                            return;
                        }

                        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
                        entry.last_use_serial = _next_planned_chunk_cache_use_serial++;
                        const auto existing_it = std::find_if(
                                _planned_chunk_cache.begin(),
                                _planned_chunk_cache.end(),
                                [&entry](const PlannedChunkCacheEntry &cached) {
                                    return planned_chunk_cache_key_matches(cached.key, entry.key);
                                });
                        if (existing_it != _planned_chunk_cache.end())
                        {
                            *existing_it = std::move(entry);
                        }
                        else
                        {
                            _planned_chunk_cache.push_back(std::move(entry));
                        }

                        if (_planned_chunk_cache.size() > kMaxCachedPlannedChunks)
                        {
                            const auto lru_it = std::min_element(
                                    _planned_chunk_cache.begin(),
                                    _planned_chunk_cache.end(),
                                    [](const PlannedChunkCacheEntry &a,
                                       const PlannedChunkCacheEntry &b) {
                                        return a.last_use_serial < b.last_use_serial;
                                    });
                            if (lru_it != _planned_chunk_cache.end())
                            {
                                _planned_chunk_cache.erase(lru_it);
                            }
                        }
                    },
                    // get_or_build_ephemeris closure
                    [this](const EphemerisBuildRequest &req, const CancelCheck &cancel_check) {
                        return get_or_build_ephemeris(req, cancel_check);
                    },
            };

            const PredictionSolvePlan solve_plan = build_prediction_solve_plan(planned_request);
            if (!solve_plan.valid)
            {
                fail(Status::InvalidInput);
                return;
            }

            PlannedSolveOutput planned{};
            const PlannedSolveRangeSummary planned_summary =
                    solve_planned_chunk_range(ctx,
                                              solve_plan,
                                              0u,
                                              solve_plan.chunks.size(),
                                              ship_sc.state,
                                              [&planned](PlannedChunkPacket &&chunk_packet) {
                                                  append_planned_chunk_packet(planned, std::move(chunk_packet));
                                                  return true;
                                              });
            apply_planned_range_summary(planned, planned_summary);
            if (planned.status != Status::Success)
            {
                fail(planned.status);
                return;
            }

            out.diagnostics.trajectory_planned = planned.diagnostics;
            out.diagnostics.trajectory_sample_count_planned = planned.samples.size();
            sync_stage_counts();
            out.trajectory_segments_inertial_planned = planned.segments;
            out.trajectory_inertial_planned = planned.samples;
            out.maneuver_previews = planned.previews;
        }

        store_reusable_baseline(request.track_id,
                                generation_id,
                                request_epoch,
                                out.shared_ephemeris,
                                out.trajectory_inertial,
                                out.trajectory_segments_inertial);
        out.valid = true;
        out.diagnostics.status = Status::Success;
        publish(std::move(out));
        return;
    }
} // namespace Game
