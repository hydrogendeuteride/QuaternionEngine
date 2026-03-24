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
#include <iterator>

namespace Game
{
    namespace
    {
        constexpr std::size_t kMaxCachedPlannedChunks = 512u;

        OrbitPredictionService::PublishedChunk make_published_chunk(const uint32_t chunk_id,
                                                                    const OrbitPredictionService::ChunkQualityState quality_state,
                                                                    const double t0_s,
                                                                    const double t1_s,
                                                                    const bool includes_planned_path,
                                                                    const bool reused_from_cache = false)
        {
            OrbitPredictionService::PublishedChunk chunk{};
            chunk.chunk_id = chunk_id;
            chunk.quality_state = quality_state;
            chunk.t0_s = t0_s;
            chunk.t1_s = t1_s;
            chunk.includes_planned_path = includes_planned_path;
            chunk.reused_from_cache = reused_from_cache;
            return chunk;
        }

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
            hash_combine(seed, start_time_s);
            hash_combine(seed, hash_state(start_state));
            return seed;
        }

        uint64_t hash_maneuver_impulse(const OrbitPredictionService::ManeuverImpulse &impulse)
        {
            uint64_t seed = 0xcbf29ce484222325ULL;
            hash_combine(seed, impulse.node_id);
            hash_combine(seed, impulse.t_s);
            hash_combine(seed, impulse.primary_body_id);
            hash_combine(seed, impulse.dv_rtn_mps.x);
            hash_combine(seed, impulse.dv_rtn_mps.y);
            hash_combine(seed, impulse.dv_rtn_mps.z);
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
    } // namespace

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

    uint64_t OrbitPredictionService::request(Request request)
    {
        uint64_t generation_id = 0;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            generation_id = _next_generation_id++;
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
        return generation_id;
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
            std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
            _planned_chunk_cache.clear();
            _next_planned_chunk_cache_use_serial = 1;
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

    bool OrbitPredictionService::publish_completed_result(const PendingJob &job, Result result)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_running)
        {
            return false;
        }

        if (!should_publish_result(job, _request_epoch, _latest_requested_generation_by_track))
        {
            return false;
        }

        _completed.push_back(std::move(result));
        return true;
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
            compute_prediction(job);
        }
    }

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

            const auto apply_maneuver_impulse_to_spacecraft = [&](orbitsim::Spacecraft &spacecraft,
                                                                  const ManeuverImpulse &src,
                                                                  const std::vector<orbitsim::MassiveBody> &massive_bodies,
                                                                  const orbitsim::CelestialEphemeris &ephemeris,
                                                                  const double softening_length_m,
                                                                  std::vector<ManeuverNodePreview> *out_previews,
                                                                  std::vector<PlannedSegmentBoundaryState> *out_boundaries)
                    -> bool {
                if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                {
                    return true;
                }

                ManeuverNodePreview preview{};
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
            };

            struct PlannedSolveOutput
            {
                std::vector<orbitsim::TrajectorySegment> segments{};
                std::vector<orbitsim::TrajectorySample> samples{};
                std::vector<ManeuverNodePreview> previews{};
                std::vector<bool> chunk_reused{};
                orbitsim::State end_state{};
                AdaptiveStageDiagnostics diagnostics{};
                Status status{Status::Success};
            };

            const auto resolve_anchor_state = [&](const double base_time_s,
                                                  const orbitsim::State &base_state,
                                                  const std::vector<ManeuverImpulse> &source_impulses,
                                                  const double anchor_time_s,
                                                  orbitsim::State &out_anchor_state) -> bool {
                out_anchor_state = base_state;
                if (!std::isfinite(base_time_s) || !std::isfinite(anchor_time_s) || !finite_state(base_state))
                {
                    return false;
                }

                constexpr double kAnchorTimeEpsilonS = 1.0e-9;
                if (anchor_time_s < (base_time_s - kAnchorTimeEpsilonS))
                {
                    return false;
                }

                orbitsim::Spacecraft preview_spacecraft{};
                preview_spacecraft.id = ship_h.id;
                preview_spacecraft.state = base_state;
                preview_spacecraft.dry_mass_kg = 1.0;
                double preview_time_s = base_time_s;
                const orbitsim::SpacecraftStateLookup empty_lookup{};

                for (const ManeuverImpulse &src : source_impulses)
                {
                    if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                    {
                        continue;
                    }

                    if ((src.t_s + kAnchorTimeEpsilonS) >= anchor_time_s)
                    {
                        break;
                    }
                    if (src.t_s < (preview_time_s - kAnchorTimeEpsilonS))
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

                        if (!apply_maneuver_impulse_to_spacecraft(preview_spacecraft,
                                                                  src,
                                                                  out.massive_bodies,
                                                                  eph,
                                                                  sim.config().softening_length_m,
                                                                  nullptr,
                                                                  nullptr))
                        {
                            return false;
                        }
                }

                if (anchor_time_s > preview_time_s)
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
                            anchor_time_s - preview_time_s,
                            empty_lookup);
                }

                out_anchor_state = preview_spacecraft.state;
                return finite_state(out_anchor_state);
            };

            const auto planned_chunk_cache_key_matches =
                    [](const OrbitPredictionService::PlannedChunkCacheKey &a,
                       const OrbitPredictionService::PlannedChunkCacheKey &b) {
                return a.track_id == b.track_id &&
                       a.baseline_generation_id == b.baseline_generation_id &&
                       a.upstream_maneuver_hash == b.upstream_maneuver_hash &&
                       a.frame_independent_generation == b.frame_independent_generation &&
                       a.chunk_t0_s == b.chunk_t0_s &&
                       a.chunk_t1_s == b.chunk_t1_s &&
                       a.profile_id == b.profile_id;
            };

            const auto find_cached_planned_chunk =
                    [this, &planned_chunk_cache_key_matches](const OrbitPredictionService::PlannedChunkCacheKey &key,
                                                             const orbitsim::State &expected_start_state)
                    -> std::optional<OrbitPredictionService::PlannedChunkCacheEntry> {
                std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
                for (OrbitPredictionService::PlannedChunkCacheEntry &entry : _planned_chunk_cache)
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
            };

            const auto store_cached_planned_chunk =
                    [this, &planned_chunk_cache_key_matches](OrbitPredictionService::PlannedChunkCacheEntry entry) {
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
                        [&entry, &planned_chunk_cache_key_matches](
                                const OrbitPredictionService::PlannedChunkCacheEntry &cached) {
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
                            [](const OrbitPredictionService::PlannedChunkCacheEntry &a,
                               const OrbitPredictionService::PlannedChunkCacheEntry &b) {
                                return a.last_use_serial < b.last_use_serial;
                            });
                    if (lru_it != _planned_chunk_cache.end())
                    {
                        _planned_chunk_cache.erase(lru_it);
                    }
                }
            };

            struct ChunkAttemptOutput
            {
                std::vector<orbitsim::TrajectorySegment> segments{};
                std::vector<orbitsim::TrajectorySegment> seam_validation_segments{};
                std::vector<orbitsim::TrajectorySample> samples{};
                std::vector<ManeuverNodePreview> previews{};
                orbitsim::State end_state{};
                AdaptiveStageDiagnostics diagnostics{};
                Status status{Status::Success};
                bool reused_from_cache{false};
            };

            const auto select_primary_body_id_for_state =
                    [&](const orbitsim::State &state,
                        const double time_s,
                        const orbitsim::BodyId preferred_body_id) -> orbitsim::BodyId {
                if (out.massive_bodies.empty() || !finite_state(state))
                {
                    return orbitsim::kInvalidBodyId;
                }

                const std::size_t primary_index = select_primary_index_with_hysteresis(
                        out.massive_bodies,
                        state.position_m,
                        [&eph, bodies = &out.massive_bodies, time_s](const std::size_t i) -> orbitsim::Vec3 {
                            std::size_t eph_index = 0u;
                            if ((*bodies)[i].id != orbitsim::kInvalidBodyId &&
                                eph.body_index_for_id((*bodies)[i].id, &eph_index))
                            {
                                return eph.body_position_at(eph_index, time_s);
                            }
                            return (*bodies)[i].state.position_m;
                        },
                        request.sim_config.softening_length_m,
                        preferred_body_id);
                return primary_index < out.massive_bodies.size()
                               ? out.massive_bodies[primary_index].id
                               : orbitsim::kInvalidBodyId;
            };

            const auto validate_chunk_seam =
                    [&](const OrbitPredictionService::PredictionChunkPlan &current_chunk,
                        const std::vector<orbitsim::TrajectorySegment> &previous_seam_segments,
                        const std::vector<orbitsim::TrajectorySegment> &current_segments,
                        OrbitPredictionService::ChunkSeamDiagnostics &out_diag) -> bool {
                out_diag = {};
                if (!current_chunk.requires_seam_validation ||
                    previous_seam_segments.empty() ||
                    current_segments.empty() ||
                    current_chunk.profile_id == OrbitPredictionService::PredictionProfileId::InteractiveExact ||
                    current_chunk.profile_id == OrbitPredictionService::PredictionProfileId::NearBody ||
                    current_chunk.profile_id == OrbitPredictionService::PredictionProfileId::Transfer ||
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
                out_diag.previous_primary_body_id = select_primary_body_id_for_state(
                        previous_state,
                        sample_time_s,
                        request.preferred_primary_body_id);
                out_diag.current_primary_body_id = select_primary_body_id_for_state(
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
            };

            const auto solve_planned_chunk_range =
                    [&](const OrbitPredictionService::Request &planner_request,
                        const OrbitPredictionService::PredictionSolvePlan &solve_plan,
                        const std::size_t chunk_begin_index,
                        const std::size_t chunk_end_index,
                        const orbitsim::State &range_start_state) -> PlannedSolveOutput {
                PlannedSolveOutput planned{};
                if (!solve_plan.valid ||
                    chunk_begin_index >= chunk_end_index ||
                    chunk_end_index > solve_plan.chunks.size() ||
                    !finite_state(range_start_state))
                {
                    planned.status = Status::InvalidInput;
                    return planned;
                }

                constexpr double kChunkBoundaryEpsilonS = 1.0e-9;
                orbitsim::State chunk_start_state = range_start_state;
                planned.end_state = range_start_state;
                const uint64_t baseline_generation_id =
                        hash_baseline_generation(planner_request, solve_plan.t0_s, range_start_state);
                const uint64_t frame_independent_generation =
                        hash_solver_context(planner_request, out.massive_bodies);
                double diagnostic_dt_sum_s = 0.0;
                bool have_dt = false;
                const orbitsim::SpacecraftStateLookup empty_lookup{};
                std::vector<orbitsim::TrajectorySegment> previous_chunk_seam_segments;

                for (std::size_t chunk_index = chunk_begin_index; chunk_index < chunk_end_index; ++chunk_index)
                {
                    if (cancel_requested())
                    {
                        planned.status = Status::Cancelled;
                        return planned;
                    }

                    const OrbitPredictionService::PredictionChunkPlan &chunk = solve_plan.chunks[chunk_index];
                    if (!(chunk.t1_s > chunk.t0_s))
                    {
                        planned.status = Status::InvalidInput;
                        return planned;
                    }

                    const bool include_chunk_end_impulse = (chunk_index + 1u) == solve_plan.chunks.size();
                    const OrbitPredictionService::ChunkActivityProbe activity_probe =
                            classify_chunk_activity(planner_request,
                                                    chunk,
                                                    &out.trajectory_segments_inertial,
                                                    out.shared_ephemeris);

                    const auto solve_chunk_attempt =
                            [&](const OrbitPredictionService::PredictionChunkPlan &effective_chunk,
                                const bool split_chunk,
                                const orbitsim::State &attempt_start_state) -> ChunkAttemptOutput {
                        ChunkAttemptOutput attempt{};

                        uint64_t upstream_maneuver_hash = 0xcbf29ce484222325ULL;
                        for (const ManeuverImpulse &upstream_impulse : planner_request.maneuver_impulses)
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
                                hash_combine(upstream_maneuver_hash, hash_maneuver_impulse(upstream_impulse));
                            }
                        }

                        const OrbitPredictionService::PlannedChunkCacheKey cache_key{
                                .track_id = planner_request.track_id,
                                .baseline_generation_id = baseline_generation_id,
                                .upstream_maneuver_hash = upstream_maneuver_hash,
                                .frame_independent_generation = frame_independent_generation,
                                .chunk_t0_s = chunk.t0_s,
                                .chunk_t1_s = chunk.t1_s,
                                .profile_id = effective_chunk.profile_id,
                        };
                        if (effective_chunk.allow_reuse)
                        {
                            if (const auto cached = find_cached_planned_chunk(cache_key, attempt_start_state); cached.has_value())
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
                                resolve_prediction_profile_definition(planner_request, effective_chunk);

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
                            const double extended_t1_s = std::min(request_end_time_s(planner_request),
                                                                  subchunk.t1_s + std::max(0.0, subchunk.overlap_s));

                            std::vector<ManeuverImpulse> chunk_impulses;
                            chunk_impulses.reserve(planner_request.maneuver_impulses.size());
                            for (const ManeuverImpulse &impulse : planner_request.maneuver_impulses)
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

                            OrbitPredictionService::Request chunk_request = planner_request;
                            chunk_request.sim_time_s = subchunk.t0_s;
                            chunk_request.future_window_s = extended_t1_s - subchunk.t0_s;
                            chunk_request.preview_patch = {};
                            chunk_request.maneuver_impulses = chunk_impulses;

                            const orbitsim::AdaptiveSegmentOptions chunk_segment_opt =
                                    build_spacecraft_adaptive_segment_options_for_chunk(chunk_request,
                                                                                       subchunk_plan,
                                                                                       cancel_requested);

                            orbitsim::GameSimulation::Config chunk_sim_config = request.sim_config;
                            apply_prediction_integrator_profile(chunk_sim_config,
                                                                chunk_request,
                                                                chunk_request.future_window_s);
                            if (request.lagrange_sensitive)
                            {
                                apply_lagrange_integrator_profile(chunk_sim_config);
                            }

                            std::vector<orbitsim::MassiveBody> chunk_massive_bodies = out.massive_bodies;
                            for (orbitsim::MassiveBody &body_at_start : chunk_massive_bodies)
                            {
                                std::size_t eph_body_index = 0u;
                                if (body_at_start.id != orbitsim::kInvalidBodyId &&
                                    eph.body_index_for_id(body_at_start.id, &eph_body_index))
                                {
                                    body_at_start.state = eph.body_state_at(eph_body_index, subchunk.t0_s);
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
                                                                              cancel_requested);

                            const OrbitPredictionService::SharedCelestialEphemeris chunk_shared_ephemeris =
                                    get_or_build_ephemeris(chunk_ephemeris_request, cancel_requested);
                            if (!chunk_shared_ephemeris || chunk_shared_ephemeris->empty())
                            {
                                attempt.status = cancel_requested() ? Status::Cancelled : Status::EphemerisUnavailable;
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
                            std::vector<ManeuverNodePreview> chunk_previews;
                            chunk_previews.reserve(chunk_impulses.size());
                            std::vector<PlannedSegmentBoundaryState> planned_boundary_states;
                            planned_boundary_states.reserve(chunk_impulses.size());
                            orbitsim::Spacecraft preview_spacecraft = planned_ship;
                            preview_spacecraft.id = planned_ship_h.id;
                            double preview_time_s = subchunk.t0_s;

                            for (const ManeuverImpulse &src : chunk_impulses)
                            {
                                if (cancel_requested())
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
                                std::vector<ManeuverNodePreview> *preview_sink =
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
                            store_cached_planned_chunk(std::move(cache_entry));
                        }

                        return attempt;
                    };

                    OrbitPredictionService::PredictionChunkPlan effective_chunk = chunk;
                    effective_chunk.profile_id = activity_probe.recommended_profile_id;
                    effective_chunk.allow_reuse = chunk.allow_reuse &&
                                                  effective_chunk.profile_id !=
                                                          OrbitPredictionService::PredictionProfileId::InteractiveExact;

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
                            planned.status = chunk_attempt.status;
                            return planned;
                        }

                        if (validate_chunk_seam(effective_chunk,
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
                                                              OrbitPredictionService::PredictionProfileId::InteractiveExact;
                    }

                    if (!chunk_solved)
                    {
                        planned.status = Status::ContinuityFailed;
                        return planned;
                    }

                    accumulate_stage_diagnostics(planned.diagnostics,
                                                 chunk_attempt.diagnostics,
                                                 diagnostic_dt_sum_s,
                                                 have_dt);
                    planned.chunk_reused.push_back(chunk_attempt.reused_from_cache);
                    planned.previews.insert(planned.previews.end(),
                                            chunk_attempt.previews.begin(),
                                            chunk_attempt.previews.end());
                    planned.segments.insert(planned.segments.end(),
                                            std::make_move_iterator(chunk_attempt.segments.begin()),
                                            std::make_move_iterator(chunk_attempt.segments.end()));
                    append_chunk_samples(planned.samples, std::move(chunk_attempt.samples));
                    chunk_start_state = chunk_attempt.end_state;
                    previous_chunk_seam_segments = std::move(chunk_attempt.seam_validation_segments);
                }

                if (planned.segments.empty())
                {
                    planned.status = Status::TrajectorySegmentsUnavailable;
                    return planned;
                }
                if (!validate_trajectory_segment_continuity(planned.segments))
                {
                    planned.status = Status::ContinuityFailed;
                    return planned;
                }
                if (planned.samples.size() < 2)
                {
                    planned.status = Status::TrajectorySamplesUnavailable;
                    return planned;
                }
                if (planned.diagnostics.accepted_segments > 0u)
                {
                    planned.diagnostics.avg_dt_s =
                            diagnostic_dt_sum_s / static_cast<double>(planned.diagnostics.accepted_segments);
                }
                planned.end_state = chunk_start_state;

                return planned;
            };

            const auto collect_published_chunks =
                    [](const OrbitPredictionService::PredictionSolvePlan &solve_plan,
                       const std::size_t chunk_begin_index,
                       const std::size_t chunk_end_index,
                       const OrbitPredictionService::ChunkQualityState quality_state,
                       const std::vector<bool> *chunk_reused = nullptr) {
                        std::vector<OrbitPredictionService::PublishedChunk> chunks;
                        if (!solve_plan.valid || chunk_begin_index >= chunk_end_index ||
                            chunk_end_index > solve_plan.chunks.size())
                        {
                            return chunks;
                        }

                        chunks.reserve(chunk_end_index - chunk_begin_index);
                        for (std::size_t chunk_index = chunk_begin_index; chunk_index < chunk_end_index; ++chunk_index)
                        {
                            const OrbitPredictionService::PredictionChunkPlan &chunk = solve_plan.chunks[chunk_index];
                            bool reused_from_cache = false;
                            if (chunk_reused)
                            {
                                const std::size_t local_index = chunk_index - chunk_begin_index;
                                if (local_index < chunk_reused->size())
                                {
                                    reused_from_cache = (*chunk_reused)[local_index];
                                }
                            }
                            chunks.push_back(make_published_chunk(chunk.chunk_id,
                                                                  quality_state,
                                                                  chunk.t0_s,
                                                                  chunk.t1_s,
                                                                  true,
                                                                  reused_from_cache));
                        }
                        return chunks;
                    };

            const bool use_preview_patch = request_uses_preview_patch(request);
            if (use_preview_patch)
            {
                const double full_window_s = preview_patch_remaining_window_s(request);
                const double fp0_window_s = preview_fp0_window_s(request);
                if (full_window_s > 0.0)
                {
                    orbitsim::State preview_patch_anchor_state = request.preview_patch.anchor_state_inertial;
                    if (!resolve_anchor_state(request.sim_time_s,
                                              ship_sc.state,
                                              maneuver_impulses,
                                              request.preview_patch.anchor_time_s,
                                              preview_patch_anchor_state))
                    {
                        fail(Status::InvalidInput);
                        return;
                    }

                    std::vector<ManeuverImpulse> patch_maneuver_impulses;
                    patch_maneuver_impulses.reserve(maneuver_impulses.size());
                    constexpr double kPatchAnchorTimeEpsilonS = 1.0e-9;
                    for (const ManeuverImpulse &src : maneuver_impulses)
                    {
                        if (!std::isfinite(src.t_s) || !finite_vec3(src.dv_rtn_mps))
                        {
                            continue;
                        }
                        if ((src.t_s + kPatchAnchorTimeEpsilonS) < request.preview_patch.anchor_time_s)
                        {
                            continue;
                        }
                        patch_maneuver_impulses.push_back(src);
                    }

                    OrbitPredictionService::Request planner_request = request;
                    planner_request.sim_time_s = request.preview_patch.anchor_time_s;
                    planner_request.future_window_s = full_window_s;
                    planner_request.preview_patch.active = true;
                    planner_request.preview_patch.anchor_state_valid = true;
                    planner_request.preview_patch.anchor_time_s = planner_request.sim_time_s;
                    planner_request.preview_patch.anchor_state_inertial = preview_patch_anchor_state;
                    planner_request.maneuver_impulses = patch_maneuver_impulses;

                    const OrbitPredictionService::PredictionSolvePlan solve_plan =
                            build_prediction_solve_plan(planner_request);
                    if (!solve_plan.valid)
                    {
                        fail(Status::InvalidInput);
                        return;
                    }

                    const double fp0_end_s = planner_request.sim_time_s + (fp0_window_s > 0.0 ? fp0_window_s : full_window_s);
                    std::size_t fp0_chunk_end_index = 0u;
                    while (fp0_chunk_end_index < solve_plan.chunks.size() &&
                           solve_plan.chunks[fp0_chunk_end_index].t0_s < (fp0_end_s - kContinuityMinTimeEpsilonS))
                    {
                        ++fp0_chunk_end_index;
                    }
                    if (fp0_chunk_end_index == 0u)
                    {
                        fail(Status::InvalidInput);
                        return;
                    }

                    const PlannedSolveOutput fp0_planned =
                            solve_planned_chunk_range(planner_request,
                                                      solve_plan,
                                                      0u,
                                                      fp0_chunk_end_index,
                                                      preview_patch_anchor_state);
                    if (fp0_planned.status != Status::Success)
                    {
                        fail(fp0_planned.status);
                        return;
                    }

                    Result fp0_result = out;
                    fp0_result.generation_complete = full_window_s <= (fp0_window_s + kContinuityMinTimeEpsilonS);
                    fp0_result.publish_stage = fp0_result.generation_complete ? PublishStage::FastPreviewFP1
                                                                              : PublishStage::FastPreviewFP0;
                    fp0_result.diagnostics.trajectory_planned = fp0_planned.diagnostics;
                    fp0_result.diagnostics.trajectory_sample_count_planned = fp0_planned.samples.size();
                    fp0_result.trajectory_segments_inertial_planned = fp0_planned.segments;
                    fp0_result.trajectory_inertial_planned = fp0_planned.samples;
                    fp0_result.maneuver_previews = fp0_planned.previews;
                    fp0_result.published_chunks = collect_published_chunks(
                            solve_plan,
                            0u,
                            fp0_chunk_end_index,
                            fp0_result.generation_complete ? ChunkQualityState::Final : ChunkQualityState::PreviewPatch,
                            &fp0_planned.chunk_reused);

                    fp0_result.valid = true;
                    fp0_result.diagnostics.status = Status::Success;
                    fp0_result.diagnostics.trajectory_segment_count_planned =
                            fp0_result.diagnostics.trajectory_planned.accepted_segments;
                    if (!publish(std::move(fp0_result)))
                    {
                        return;
                    }

                    if (cancel_requested() || full_window_s <= (fp0_window_s + kContinuityMinTimeEpsilonS))
                    {
                        return;
                    }

                    if (fp0_chunk_end_index >= solve_plan.chunks.size())
                    {
                        return;
                    }

                    const double tail_start_time_s = solve_plan.chunks[fp0_chunk_end_index].t0_s;
                    const double fp0_actual_end_s =
                            prediction_segment_end_time(fp0_planned.segments.back());
                    if (std::abs(fp0_actual_end_s - tail_start_time_s) >
                            continuity_time_epsilon_s(tail_start_time_s) ||
                        !finite_state(fp0_planned.end_state))
                    {
                        fail(Status::ContinuityFailed);
                        return;
                    }
                    const orbitsim::State tail_start_state = fp0_planned.end_state;

                    const PlannedSolveOutput final_planned =
                            solve_planned_chunk_range(planner_request,
                                                      solve_plan,
                                                      fp0_chunk_end_index,
                                                      solve_plan.chunks.size(),
                                                      tail_start_state);
                    if (final_planned.status != Status::Success)
                    {
                        fail(final_planned.status);
                        return;
                    }

                    out.generation_complete = true;
                    out.publish_stage = PublishStage::FastPreviewFP1;
                    out.diagnostics.trajectory_planned = final_planned.diagnostics;
                    out.diagnostics.trajectory_sample_count_planned = final_planned.samples.size();
                    out.trajectory_segments_inertial_planned = final_planned.segments;
                    out.trajectory_inertial_planned = final_planned.samples;
                    out.maneuver_previews = final_planned.previews;
                    out.published_chunks =
                            collect_published_chunks(solve_plan,
                                                     fp0_chunk_end_index,
                                                     solve_plan.chunks.size(),
                                                     ChunkQualityState::Final,
                                                     &final_planned.chunk_reused);
                    out.diagnostics.trajectory_segment_count_planned = out.diagnostics.trajectory_planned.accepted_segments;
                    out.valid = true;
                    out.diagnostics.status = Status::Success;
                    publish(std::move(out));
                    return;
                }
            }

            const OrbitPredictionService::PredictionSolvePlan solve_plan = build_prediction_solve_plan(request);
            if (!solve_plan.valid)
            {
                fail(Status::InvalidInput);
                return;
            }

            const PlannedSolveOutput planned =
                    solve_planned_chunk_range(request,
                                              solve_plan,
                                              0u,
                                              solve_plan.chunks.size(),
                                              ship_sc.state);
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
            out.published_chunks =
                    collect_published_chunks(solve_plan,
                                             0u,
                                             solve_plan.chunks.size(),
                                             ChunkQualityState::Final,
                                             &planned.chunk_reused);
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
