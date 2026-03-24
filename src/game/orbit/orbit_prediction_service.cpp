#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <algorithm>

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
    OrbitPredictionService::OrbitPredictionService()
    {
        // Use a small worker pool so independent visible tracks can solve in parallel.
        const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
        const std::size_t worker_count = std::clamp<std::size_t>(static_cast<std::size_t>(hw_threads > 1 ? hw_threads - 1 : 1),
                                                                 1,
                                                                 4);
        _workers.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i)
        {
            _workers.emplace_back(&OrbitPredictionService::worker_loop, this);
        }
    }

    OrbitPredictionService::~OrbitPredictionService()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // Stop accepting/publishing work before waking the worker for shutdown.
            _running = false;
            _pending_jobs.clear();
            _completed.clear();
        }
        _cv.notify_all();

        for (std::thread &worker : _workers)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
    }

    void OrbitPredictionService::request(Request request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const uint64_t generation_id = _next_generation_id++;
            const uint64_t track_id = request.track_id;
            const uint64_t request_epoch = _request_epoch;
            _latest_requested_generation_by_track[track_id] = generation_id;

            // Keep only the newest queued request per track to avoid backlogging stale previews.
            auto existing = std::find_if(_pending_jobs.begin(),
                                         _pending_jobs.end(),
                                         [track_id](const PendingJob &job) { return job.track_id == track_id; });
            PendingJob job{};
            if (existing != _pending_jobs.end())
            {
                job = std::move(*existing);
                _pending_jobs.erase(existing);
            }

            job.track_id = track_id;
            job.request_epoch = request_epoch;
            job.generation_id = generation_id;
            job.request = std::move(request);

            auto insert_it = std::find_if(
                    _pending_jobs.begin(),
                    _pending_jobs.end(),
                    [&job](const PendingJob &queued) {
                        return static_cast<uint8_t>(job.request.priority) >
                               static_cast<uint8_t>(queued.request.priority);
                    });
            _pending_jobs.insert(insert_it, std::move(job));
        }
        _cv.notify_one();
    }

    std::optional<OrbitPredictionService::Result> OrbitPredictionService::poll_completed()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_completed.empty())
        {
            return std::nullopt;
        }

        Result out = std::move(_completed.front());
        _completed.pop_front();
        return out;
    }

    void OrbitPredictionService::reset()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // Bump the epoch so any in-flight jobs become stale even if they finish later.
            ++_request_epoch;
            _pending_jobs.clear();
            _completed.clear();
            _latest_requested_generation_by_track.clear();
        }

        {
            std::lock_guard<std::mutex> lock(_baseline_cache_mutex);
            _reusable_baseline_by_track.clear();
        }

        {
            std::lock_guard<std::mutex> cache_lock(_ephemeris_mutex);
            _ephemeris_cache.clear();
            _next_ephemeris_use_serial = 1;
        }
    }

    std::optional<OrbitPredictionService::ReusableBaselineCacheEntry> OrbitPredictionService::find_reusable_baseline(
            const uint64_t track_id,
            const uint64_t request_epoch) const
    {
        std::lock_guard<std::mutex> lock(_baseline_cache_mutex);
        const auto it = _reusable_baseline_by_track.find(track_id);
        if (it == _reusable_baseline_by_track.end() || it->second.request_epoch != request_epoch)
        {
            return std::nullopt;
        }
        return it->second;
    }

    void OrbitPredictionService::store_reusable_baseline(
            const uint64_t track_id,
            const uint64_t generation_id,
            const uint64_t request_epoch,
            SharedCelestialEphemeris shared_ephemeris,
            std::vector<orbitsim::TrajectorySample> trajectory_inertial,
            std::vector<orbitsim::TrajectorySegment> trajectory_segments_inertial)
    {
        if (!shared_ephemeris || trajectory_inertial.size() < 2 || !validate_trajectory_segment_continuity(trajectory_segments_inertial))
        {
            return;
        }

        std::lock_guard<std::mutex> state_lock(_mutex);
        if (!_running || request_epoch != _request_epoch)
        {
            return;
        }
        const auto latest_it = _latest_requested_generation_by_track.find(track_id);
        if (latest_it != _latest_requested_generation_by_track.end() && generation_id < latest_it->second)
        {
            return;
        }

        std::lock_guard<std::mutex> baseline_lock(_baseline_cache_mutex);
        ReusableBaselineCacheEntry &entry = _reusable_baseline_by_track[track_id];
        if (entry.request_epoch == request_epoch && entry.generation_id > generation_id)
        {
            return;
        }

        entry.generation_id = generation_id;
        entry.request_epoch = request_epoch;
        entry.shared_ephemeris = std::move(shared_ephemeris);
        entry.trajectory_inertial = std::move(trajectory_inertial);
        entry.trajectory_segments_inertial = std::move(trajectory_segments_inertial);
    }

    OrbitPredictionService::SharedCelestialEphemeris OrbitPredictionService::get_or_build_ephemeris(
            const EphemerisBuildRequest &request)
    {
        return get_or_build_ephemeris(request, {});
    }

    OrbitPredictionService::SharedCelestialEphemeris OrbitPredictionService::get_or_build_ephemeris(
            const EphemerisBuildRequest &request,
            const std::function<bool()> &cancel_requested,
            AdaptiveStageDiagnostics *out_diagnostics,
            bool *out_cache_reused)
    {
        if (out_diagnostics)
        {
            *out_diagnostics = {};
            out_diagnostics->requested_duration_s = std::max(0.0, request.duration_s);
        }
        if (out_cache_reused)
        {
            *out_cache_reused = false;
        }

        // Reuse an equal-or-better ephemeris when another prediction already paid the build cost.
        {
            std::lock_guard<std::mutex> lock(_ephemeris_mutex);
            for (CachedEphemerisEntry &entry : _ephemeris_cache)
            {
                if (!compatible_cached_ephemeris(entry, request))
                {
                    continue;
                }

                entry.last_use_serial = _next_ephemeris_use_serial++;
                if (out_diagnostics)
                {
                    *out_diagnostics = entry.diagnostics;
                    out_diagnostics->cache_reused = true;
                }
                if (out_cache_reused)
                {
                    *out_cache_reused = true;
                }
                return entry.ephemeris;
            }
        }

        orbitsim::AdaptiveEphemerisDiagnostics build_diag{};
        SharedCelestialEphemeris built = build_ephemeris_from_request(request, cancel_requested, &build_diag);
        if (!built)
        {
            return {};
        }

        std::lock_guard<std::mutex> lock(_ephemeris_mutex);
        // Re-check after the build in case another worker filled the cache first.
        for (CachedEphemerisEntry &entry : _ephemeris_cache)
        {
            if (!compatible_cached_ephemeris(entry, request))
            {
                continue;
            }

            entry.last_use_serial = _next_ephemeris_use_serial++;
            if (out_diagnostics)
            {
                *out_diagnostics = entry.diagnostics;
                out_diagnostics->cache_reused = true;
            }
            if (out_cache_reused)
            {
                *out_cache_reused = true;
            }
            return entry.ephemeris;
        }

        CachedEphemerisEntry entry{};
        entry.sim_time_s = request.sim_time_s;
        entry.duration_s = request.duration_s;
        entry.sim_config = request.sim_config;
        entry.massive_bodies = request.massive_bodies;
        entry.adaptive_options = request.adaptive_options;
        entry.adaptive_options.cancel_requested = {};
        entry.ephemeris = std::move(built);
        entry.diagnostics = make_stage_diagnostics_from_adaptive(build_diag, request.duration_s);
        entry.last_use_serial = _next_ephemeris_use_serial++;
        _ephemeris_cache.push_back(std::move(entry));

        if (_ephemeris_cache.size() > kMaxCachedEphemerides)
        {
            // Bound memory usage with a simple LRU eviction policy.
            auto lru_it = std::min_element(_ephemeris_cache.begin(),
                                           _ephemeris_cache.end(),
                                           [](const CachedEphemerisEntry &a, const CachedEphemerisEntry &b) {
                                               return a.last_use_serial < b.last_use_serial;
                                           });
            if (lru_it != _ephemeris_cache.end())
            {
                _ephemeris_cache.erase(lru_it);
            }
        }

        if (out_diagnostics)
        {
            *out_diagnostics = _ephemeris_cache.back().diagnostics;
        }
        return _ephemeris_cache.back().ephemeris;
    }

    OrbitPredictionService::EphemerisSamplingSpec OrbitPredictionService::build_ephemeris_sampling_spec(const Request &request)
    {
        EphemerisSamplingSpec out{};
        if (!std::isfinite(request.sim_time_s) || request.massive_bodies.empty())
        {
            return out;
        }

        const std::size_t primary_index = select_primary_index_with_hysteresis(
                request.massive_bodies,
                request.ship_bary_position_m,
                [&request](const std::size_t i) -> orbitsim::Vec3 { return request.massive_bodies[i].state.position_m; },
                request.sim_config.softening_length_m,
                request.preferred_primary_body_id);
        if (primary_index >= request.massive_bodies.size())
        {
            return out;
        }

        const orbitsim::MassiveBody &primary_body = request.massive_bodies[primary_index];
        const double mu_ref_m3_s2 = request.sim_config.gravitational_constant * primary_body.mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return out;
        }

        const glm::dvec3 ship_rel_pos_m = request.ship_bary_position_m - glm::dvec3(primary_body.state.position_m);
        const glm::dvec3 ship_rel_vel_mps = request.ship_bary_velocity_mps - glm::dvec3(primary_body.state.velocity_mps);
        if (!finite_vec3(ship_rel_pos_m) || !finite_vec3(ship_rel_vel_mps))
        {
            return out;
        }

        const double horizon_s_auto =
                OrbitPredictionMath::select_prediction_horizon(mu_ref_m3_s2, ship_rel_pos_m, ship_rel_vel_mps);

        double horizon_s = std::max(OrbitPredictionTuning::kMinHorizonS, horizon_s_auto);
        const double requested_window_s =
                std::isfinite(request.future_window_s) ? std::max(0.0, request.future_window_s) : 0.0;
        horizon_s = std::max(horizon_s, std::max(1.0, requested_window_s));

        if (request.thrusting)
        {
            const double thrust_horizon_cap_s =
                    std::clamp(std::max(OrbitPredictionTuning::kThrustHorizonMinS,
                                        requested_window_s * OrbitPredictionTuning::kThrustHorizonWindowScale),
                               OrbitPredictionTuning::kThrustHorizonMinS,
                               OrbitPredictionTuning::kThrustHorizonMaxS);
            horizon_s = std::min(horizon_s, thrust_horizon_cap_s);
            horizon_s = std::max(horizon_s, std::max(1.0, requested_window_s));
        }

        out.valid = (horizon_s > 0.0);
        out.horizon_s = horizon_s;
        return out;
    }

    bool OrbitPredictionService::should_publish_result(
            const PendingJob &job,
            const uint64_t current_request_epoch,
            const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track)
    {
        // reset() invalidates every queued and in-flight job by moving to a new epoch.
        if (job.request_epoch != current_request_epoch)
        {
            return false;
        }

        // Only publish the newest generation for each track.
        const auto latest_it = latest_requested_generation_by_track.find(job.track_id);
        return latest_it == latest_requested_generation_by_track.end() ||
               job.generation_id >= latest_it->second;
    }

    bool OrbitPredictionService::should_continue_job(const uint64_t track_id,
                                                     const uint64_t generation_id,
                                                     const uint64_t request_epoch) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_running || request_epoch != _request_epoch)
        {
            return false;
        }

        const auto latest_it = _latest_requested_generation_by_track.find(track_id);
        return latest_it == _latest_requested_generation_by_track.end() ||
               generation_id >= latest_it->second;
    }

    void OrbitPredictionService::worker_loop()
    {
        while (true)
        {
            PendingJob job{};
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this]() {
                    return !_running || !_pending_jobs.empty();
                });

                if (!_running && _pending_jobs.empty())
                {
                    return;
                }

                if (_pending_jobs.empty())
                {
                    continue;
                }

                job = std::move(_pending_jobs.front());
                _pending_jobs.pop_front();
            }

            if (!should_continue_job(job.track_id, job.generation_id, job.request_epoch))
            {
                continue;
            }

            // Run the expensive simulation outside the lock so request() stays responsive.
            Result result = compute_prediction(job.generation_id, job.request, job.request_epoch);

            {
                std::lock_guard<std::mutex> lock(_mutex);
                if (!_running)
                {
                    return;
                }

                if (!should_publish_result(job, _request_epoch, _latest_requested_generation_by_track))
                {
                    continue;
                }

                _completed.push_back(std::move(result));
            }
        }
    }

    OrbitPredictionService::Result OrbitPredictionService::compute_prediction(const uint64_t generation_id,
                                                                              const Request &request,
                                                                              const uint64_t request_epoch)
    {
        Result out{};
        struct ScopedComputeTimer
        {
            Result *result{nullptr};
            std::chrono::steady_clock::time_point start{};
            ~ScopedComputeTimer()
            {
                if (!result)
                {
                    return;
                }
                result->compute_time_ms =
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            }
        };
        ScopedComputeTimer timer{};
        timer.result = &out;
        timer.start = std::chrono::steady_clock::now();

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
        const auto fail = [&out](const Status status) -> Result {
            out.diagnostics.status = status;
            return out;
        };
        const auto cancel = [&out]() -> Result {
            out.diagnostics.cancelled = true;
            out.diagnostics.status = Status::Cancelled;
            return out;
        };
        const auto sync_stage_counts = [&out]() {
            out.diagnostics.ephemeris_segment_count = out.diagnostics.ephemeris.accepted_segments;
            out.diagnostics.trajectory_segment_count = out.diagnostics.trajectory_base.accepted_segments;
            out.diagnostics.trajectory_segment_count_planned = out.diagnostics.trajectory_planned.accepted_segments;
        };

        // Invalid inputs produce an invalid result rather than throwing on the worker thread.
        if (!std::isfinite(request.sim_time_s))
        {
            return fail(Status::InvalidInput);
        }

        std::optional<EphemerisSamplingSpec> spacecraft_sampling_spec{};
        if (request.kind == RequestKind::Spacecraft)
        {
            spacecraft_sampling_spec = build_ephemeris_sampling_spec(request);
            if (!spacecraft_sampling_spec->valid)
            {
                return fail(Status::InvalidSamplingSpec);
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
            return fail(Status::InvalidInput);
        }

        if (cancel_requested())
        {
            return cancel();
        }

        for (const orbitsim::MassiveBody &body : request.massive_bodies)
        {
            const auto body_handle =
                    (body.id != orbitsim::kInvalidBodyId)
                        ? sim.create_body_with_id(body.id, body)
                        : sim.create_body(body);
            if (!body_handle.valid())
            {
                return fail(Status::InvalidSubject);
            }
        }

        if (cancel_requested())
        {
            return cancel();
        }

        out.massive_bodies = sim.massive_bodies();

        // Celestial route: predict the body's inertial motion.
        if (request.kind == RequestKind::Celestial)
        {
            if (request.subject_body_id == orbitsim::kInvalidBodyId)
            {
                return fail(Status::InvalidSubject);
            }

            const orbitsim::MassiveBody *subject_sim = sim.body_by_id(request.subject_body_id);
            if (!subject_sim)
            {
                return fail(Status::InvalidSubject);
            }

            const CelestialPredictionSamplingSpec sampling_spec =
                    build_celestial_prediction_sampling_spec(request, *subject_sim);
            if (!sampling_spec.valid)
            {
                return fail(Status::InvalidSamplingSpec);
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
                return fail(Status::EphemerisUnavailable);
            }
            if (!validate_ephemeris_continuity(*shared_ephemeris))
            {
                return fail(Status::ContinuityFailed);
            }
            out.diagnostics.ephemeris = ephemeris_diag;
            sync_stage_counts();

            if (cancel_requested())
            {
                return cancel();
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
                return fail(Status::TrajectorySamplesUnavailable);
            }

            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial =
                    trajectory_segments_from_body_ephemeris(*shared_ephemeris, subject_sim->id);
            if (traj_segments_inertial.empty())
            {
                return fail(Status::TrajectorySegmentsUnavailable);
            }
            if (!validate_trajectory_segment_continuity(traj_segments_inertial))
            {
                return fail(Status::ContinuityFailed);
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
            return out;
        }

        // Spacecraft route: build a transient ship state, then predict inertial baseline and planned trajectories.
        const glm::dvec3 ship_bary_pos_m = request.ship_bary_position_m;
        const glm::dvec3 ship_bary_vel_mps = request.ship_bary_velocity_mps;
        if (!finite_vec3(ship_bary_pos_m) || !finite_vec3(ship_bary_vel_mps))
        {
            return fail(Status::InvalidInput);
        }

        const EphemerisSamplingSpec sampling_spec = *spacecraft_sampling_spec;
        const double horizon_s = sampling_spec.horizon_s;
        orbitsim::Spacecraft ship_sc{};
        ship_sc.state = orbitsim::make_state(ship_bary_pos_m, ship_bary_vel_mps);
        ship_sc.dry_mass_kg = 1.0;

        const auto ship_h = sim.create_spacecraft(ship_sc);
        if (!ship_h.valid())
        {
            return fail(Status::InvalidSubject);
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
            return fail(Status::EphemerisUnavailable);
        }
        if (!validate_ephemeris_continuity(*shared_ephemeris))
        {
            return fail(Status::ContinuityFailed);
        }
        out.diagnostics.ephemeris = ephemeris_diag;
        sync_stage_counts();

        if (cancel_requested())
        {
            return cancel();
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
                return fail(Status::TrajectorySegmentsUnavailable);
            }
            if (!validate_trajectory_segment_continuity(traj_segments_inertial_baseline))
            {
                return fail(Status::ContinuityFailed);
            }
            out.diagnostics.trajectory_base = make_stage_diagnostics_from_adaptive(base_diag, horizon_s);
            out.diagnostics.trajectory_base.accepted_segments = traj_segments_inertial_baseline.size();
            out.diagnostics.trajectory_base.covered_duration_s = prediction_segment_span_s(traj_segments_inertial_baseline);
            sync_stage_counts();

            if (cancel_requested())
            {
                return cancel();
            }

            const std::size_t baseline_sample_count =
                    prediction_sample_budget(request, traj_segments_inertial_baseline.size());
            std::vector<orbitsim::TrajectorySample> traj_inertial_baseline =
                    resample_segments_uniform(traj_segments_inertial_baseline, baseline_sample_count);
            if (traj_inertial_baseline.size() < 2)
            {
                return fail(Status::TrajectorySamplesUnavailable);
            }
            out.diagnostics.trajectory_sample_count = traj_inertial_baseline.size();
            sync_stage_counts();

            if (cancel_requested())
            {
                return cancel();
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

            orbitsim::ManeuverPlan plan{};
            plan.impulses.reserve(maneuver_impulses.size());
            out.maneuver_previews.reserve(maneuver_impulses.size());
            std::vector<PlannedSegmentBoundaryState> planned_boundary_states;
            planned_boundary_states.reserve(maneuver_impulses.size());
            orbitsim::Spacecraft preview_spacecraft = ship_sc;
            preview_spacecraft.id = ship_h.id;
            double preview_time_s = request.sim_time_s;
            const orbitsim::SpacecraftStateLookup empty_lookup{};

            // Propagate once from node-to-node so long plans do not repeatedly restart from t0.
            for (const ManeuverImpulse &src : maneuver_impulses)
            {
                if (cancel_requested())
                {
                    return cancel();
                }

                if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                {
                    continue;
                }

                if (src.t_s > preview_time_s)
                {
                    preview_spacecraft = orbitsim::detail::propagate_spacecraft_in_ephemeris(
                            preview_spacecraft,
                            out.massive_bodies,
                            eph,
                            orbitsim::ManeuverPlan{},
                            sim.config().gravitational_constant,
                            sim.config().softening_length_m,
                            sim.config().spacecraft_integrator,
                            preview_time_s,
                            src.t_s - preview_time_s,
                            empty_lookup);
                    preview_time_s = src.t_s;
                }

                ManeuverNodePreview preview{};
                preview.node_id = src.node_id;
                preview.t_s = src.t_s;

                const orbitsim::State pre_burn_state = preview_spacecraft.state;
                if (build_maneuver_preview(preview_spacecraft.state, src.t_s, preview))
                {
                    out.maneuver_previews.push_back(preview);
                }

                orbitsim::ImpulseSegment impulse{};
                impulse.t_s = src.t_s;
                impulse.primary_body_id = src.primary_body_id;
                impulse.dv_rtn_mps = src.dv_rtn_mps;
                impulse.spacecraft_id = ship_h.id;
                plan.impulses.push_back(impulse);

                if (preview.valid)
                {
                    std::optional<std::size_t> primary_index = orbitsim::body_index_for_id(out.massive_bodies, src.primary_body_id);
                    if (!primary_index.has_value())
                    {
                        primary_index = orbitsim::auto_select_primary_index(
                                out.massive_bodies,
                                preview.inertial_position_m,
                                [&eph, time_s = src.t_s](const std::size_t i) -> orbitsim::Vec3 {
                                    return eph.body_position_at(i, time_s);
                                },
                                sim.config().softening_length_m);
                    }

                    if (primary_index.has_value() && *primary_index < out.massive_bodies.size())
                    {
                        const orbitsim::Vec3 dv_inertial_mps = orbitsim::rtn_vector_to_inertial(
                                eph,
                                out.massive_bodies,
                                *primary_index,
                                src.t_s,
                                preview.inertial_position_m,
                                preview.inertial_velocity_mps,
                                src.dv_rtn_mps);
                        if (finite_vec3(dv_inertial_mps))
                        {
                            preview_spacecraft.state.velocity_mps += dv_inertial_mps;
                        }
                    }
                }

                append_or_merge_planned_boundary_state(
                        planned_boundary_states,
                        src.t_s,
                        pre_burn_state,
                        preview_spacecraft.state);
            }

            orbitsim::sort_impulses_by_time(plan);
            sim.maneuver_plan() = std::move(plan);

            // Planned output shows the same prediction window with all maneuver nodes applied.
            orbitsim::AdaptiveSegmentDiagnostics planned_diag{};
            std::vector<orbitsim::TrajectorySegment> traj_segments_inertial_planned =
                    orbitsim::predict_spacecraft_trajectory_segments_adaptive(sim, eph, ship_h.id, segment_opt, &planned_diag);
            if (!planned_boundary_states.empty())
            {
                std::vector<orbitsim::TrajectorySegment> split_planned =
                        split_trajectory_segments_at_known_boundaries(traj_segments_inertial_planned, planned_boundary_states);
                if (!split_planned.empty() && validate_trajectory_segment_continuity(split_planned))
                {
                    traj_segments_inertial_planned = std::move(split_planned);
                }
            }
            if (!traj_segments_inertial_planned.empty())
            {
                if (cancel_requested())
                {
                    return cancel();
                }

                if (!validate_trajectory_segment_continuity(traj_segments_inertial_planned))
                {
                    return fail(Status::ContinuityFailed);
                }
                out.diagnostics.trajectory_planned = make_stage_diagnostics_from_adaptive(planned_diag, horizon_s);
                out.diagnostics.trajectory_planned.accepted_segments = traj_segments_inertial_planned.size();
                out.diagnostics.trajectory_planned.covered_duration_s =
                        prediction_segment_span_s(traj_segments_inertial_planned);
                sync_stage_counts();
                out.trajectory_segments_inertial_planned = std::move(traj_segments_inertial_planned);

                const std::size_t planned_sample_count =
                        prediction_sample_budget(request, out.trajectory_segments_inertial_planned.size());
                std::vector<orbitsim::TrajectorySample> traj_inertial_planned =
                        resample_segments_uniform(out.trajectory_segments_inertial_planned, planned_sample_count);
                if (traj_inertial_planned.size() >= 2)
                {
                    out.diagnostics.trajectory_sample_count_planned = traj_inertial_planned.size();
                    sync_stage_counts();
                    out.trajectory_inertial_planned = std::move(traj_inertial_planned);
                }
            }
        }

        store_reusable_baseline(request.track_id,
                                generation_id,
                                request_epoch,
                                out.shared_ephemeris,
                                out.trajectory_inertial,
                                out.trajectory_segments_inertial);
        out.valid = true;
        out.diagnostics.status = Status::Success;
        return out;
    }
} // namespace Game
