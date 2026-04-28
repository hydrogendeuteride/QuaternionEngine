#include "game/orbit/orbit_prediction_service.h"
#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <algorithm>

#include "orbitsim/trajectories.hpp"

namespace Game
{
    namespace
    {
        constexpr std::size_t kMaxPendingCompletedResults = 64;

        bool prediction_result_is_streaming(const OrbitPredictionService::Result &result)
        {
            return result.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming ||
                   result.publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
        }

        bool should_drop_completed_result_for_incoming(
                const OrbitPredictionService::Result &queued,
                const OrbitPredictionService::Result &incoming)
        {
            if (queued.track_id != incoming.track_id || !prediction_result_is_streaming(queued))
            {
                return false;
            }

            if (!prediction_result_is_streaming(incoming))
            {
                return incoming.generation_id > queued.generation_id;
            }

            if (incoming.generation_id > queued.generation_id)
            {
                return true;
            }

            // Preview publishes are visual snapshots. Keep only the newest queued
            // preview for a track, while preserving full-stream chunk batches.
            return incoming.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming &&
                   queued.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming;
        }

        void coalesce_completed_results(
                std::deque<OrbitPredictionService::Result> &completed,
                const OrbitPredictionService::Result &incoming)
        {
            completed.erase(std::remove_if(completed.begin(),
                                           completed.end(),
                                           [&incoming](const OrbitPredictionService::Result &queued) {
                                               return should_drop_completed_result_for_incoming(queued, incoming);
                                           }),
                            completed.end());
        }

        void trim_completed_results(std::deque<OrbitPredictionService::Result> &completed)
        {
            while (completed.size() > kMaxPendingCompletedResults)
            {
                auto streaming_it = std::find_if(completed.begin(),
                                                 completed.end(),
                                                 [](const OrbitPredictionService::Result &queued) {
                                                     return prediction_result_is_streaming(queued);
                                                 });
                if (streaming_it == completed.end())
                {
                    return;
                }
                completed.erase(streaming_it);
            }
        }

        void enqueue_completed_result(std::deque<OrbitPredictionService::Result> &completed,
                                      OrbitPredictionService::Result result)
        {
            coalesce_completed_results(completed, result);
            completed.push_back(std::move(result));
            trim_completed_results(completed);
        }

        void discard_completed_results_before_generation(
                std::deque<OrbitPredictionService::Result> &completed,
                const uint64_t track_id,
                const uint64_t generation_id,
                const bool preserve_fast_preview)
        {
            completed.erase(std::remove_if(completed.begin(),
                                           completed.end(),
                                           [track_id, generation_id, preserve_fast_preview](
                                                   const OrbitPredictionService::Result &queued) {
                                                return queued.track_id == track_id &&
                                                       queued.generation_id < generation_id &&
                                                       !(preserve_fast_preview &&
                                                         queued.solve_quality ==
                                                                 OrbitPredictionService::SolveQuality::FastPreview);
                                           }),
                            completed.end());
        }

        void discard_stale_maneuver_completed_results(
                std::deque<OrbitPredictionService::Result> &completed,
                const uint64_t track_id,
                const uint64_t maneuver_plan_revision)
        {
            completed.erase(std::remove_if(completed.begin(),
                                           completed.end(),
                                           [track_id, maneuver_plan_revision](const OrbitPredictionService::Result &queued) {
                                               return queued.track_id == track_id &&
                                                      queued.maneuver_plan_revision < maneuver_plan_revision;
                                           }),
                            completed.end());
        }

        bool maneuver_revision_is_current(
                const uint64_t track_id,
                const uint64_t maneuver_plan_revision,
                const std::unordered_map<uint64_t, uint64_t> &latest_maneuver_plan_revision_by_track)
        {
            const auto latest_it = latest_maneuver_plan_revision_by_track.find(track_id);
            return latest_it == latest_maneuver_plan_revision_by_track.end() ||
                   maneuver_plan_revision >= latest_it->second;
        }

        OrbitPredictionService::AdaptiveStageDiagnostics make_reused_ephemeris_diagnostics(
                const OrbitPredictionService::CachedEphemerisEntry &entry,
                const OrbitPredictionService::EphemerisBuildRequest &request)
        {
            OrbitPredictionService::AdaptiveStageDiagnostics out = entry.diagnostics;
            out.requested_duration_s = std::max(0.0, request.duration_s);
            out.cache_reused = true;

            if (entry.ephemeris && !entry.ephemeris->empty())
            {
                const double request_start_s = request.sim_time_s;
                const double request_end_s = request.sim_time_s + request.duration_s;
                const double covered_start_s = std::max(entry.ephemeris->t0_s(), request_start_s);
                const double covered_end_s = std::min(entry.ephemeris->t_end_s(), request_end_s);
                if (std::isfinite(covered_start_s) &&
                    std::isfinite(covered_end_s) &&
                    covered_end_s > covered_start_s)
                {
                    out.covered_duration_s = covered_end_s - covered_start_s;
                }
            }

            return out;
        }
    } // namespace

    // ── Lifecycle ────────────────────────────────────────────────────────────

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
            _latest_maneuver_plan_revision_by_track[track_id] =
                    std::max(_latest_maneuver_plan_revision_by_track[track_id],
                             request.maneuver_plan_revision);
            discard_completed_results_before_generation(
                    _completed,
                    track_id,
                    generation_id,
                    request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview);
            discard_stale_maneuver_completed_results(_completed,
                                                     track_id,
                                                     _latest_maneuver_plan_revision_by_track[track_id]);

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

    void OrbitPredictionService::invalidate_maneuver_plan_revision(
            const uint64_t track_id,
            const uint64_t maneuver_plan_revision)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            uint64_t &latest_revision = _latest_maneuver_plan_revision_by_track[track_id];
            if (maneuver_plan_revision > latest_revision)
            {
                latest_revision = maneuver_plan_revision;
            }

            discard_stale_maneuver_completed_results(_completed, track_id, latest_revision);
        }
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
            _latest_maneuver_plan_revision_by_track.clear();
        }

        {
            std::lock_guard<std::mutex> lock(_baseline_cache_mutex);
            _reusable_baseline_by_track.clear();
        }

        {
            std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
            _planned_chunk_cache_by_key.clear();
            _planned_chunk_cache.clear();
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

    std::optional<OrbitPredictionService::PlannedChunkCacheEntry>
    OrbitPredictionService::find_cached_planned_chunk(
            const PlannedChunkCacheKey &key,
            const orbitsim::State &expected_start_state)
    {
        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
        const auto cache_it = _planned_chunk_cache_by_key.find(key);
        if (cache_it == _planned_chunk_cache_by_key.end())
        {
            return std::nullopt;
        }

        auto entry_it = cache_it->second;
        PlannedChunkCacheEntry &entry = *entry_it;
        if (entry.samples.size() < 2u ||
            entry.segments.empty() ||
            !validate_trajectory_segment_continuity(entry.segments) ||
            (!entry.seam_validation_segments.empty() &&
             !validate_trajectory_segment_continuity(entry.seam_validation_segments)))
        {
            _planned_chunk_cache_by_key.erase(cache_it);
            _planned_chunk_cache.erase(entry_it);
            return std::nullopt;
        }

        if (!states_are_continuous(entry.start_state, expected_start_state))
        {
            return std::nullopt;
        }

        _planned_chunk_cache.splice(_planned_chunk_cache.begin(),
                                    _planned_chunk_cache,
                                    entry_it);
        cache_it->second = _planned_chunk_cache.begin();
        return *_planned_chunk_cache.begin();
    }

    void OrbitPredictionService::store_cached_planned_chunk(PlannedChunkCacheEntry entry)
    {
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
        const auto existing_it = _planned_chunk_cache_by_key.find(entry.key);
        if (existing_it != _planned_chunk_cache_by_key.end())
        {
            auto lru_it = existing_it->second;
            *lru_it = std::move(entry);
            _planned_chunk_cache.splice(_planned_chunk_cache.begin(),
                                        _planned_chunk_cache,
                                        lru_it);
            existing_it->second = _planned_chunk_cache.begin();
        }
        else
        {
            _planned_chunk_cache.push_front(std::move(entry));
            _planned_chunk_cache_by_key.emplace(_planned_chunk_cache.front().key,
                                                _planned_chunk_cache.begin());
        }

        if (_planned_chunk_cache.size() > kMaxCachedPlannedChunks)
        {
            _planned_chunk_cache_by_key.erase(_planned_chunk_cache.back().key);
            _planned_chunk_cache.pop_back();
        }
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
                    *out_diagnostics = make_reused_ephemeris_diagnostics(entry, request);
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
                *out_diagnostics = make_reused_ephemeris_diagnostics(entry, request);
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
               job.generation_id >= latest_it->second ||
               job.request.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
    }

    bool OrbitPredictionService::should_continue_job(const uint64_t track_id,
                                                     const uint64_t generation_id,
                                                     const uint64_t request_epoch,
                                                     const uint64_t maneuver_plan_revision,
                                                     const SolveQuality solve_quality) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_running || request_epoch != _request_epoch)
        {
            return false;
        }

        const auto latest_it = _latest_requested_generation_by_track.find(track_id);
        if (latest_it != _latest_requested_generation_by_track.end() &&
            generation_id < latest_it->second &&
            solve_quality != OrbitPredictionService::SolveQuality::FastPreview)
        {
            return false;
        }

        const auto latest_revision_it = _latest_maneuver_plan_revision_by_track.find(track_id);
        if (latest_revision_it != _latest_maneuver_plan_revision_by_track.end() &&
            maneuver_plan_revision < latest_revision_it->second)
        {
            return false;
        }

        return true;
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
        if (!maneuver_revision_is_current(job.track_id,
                                          job.request.maneuver_plan_revision,
                                          _latest_maneuver_plan_revision_by_track))
        {
            return false;
        }

        enqueue_completed_result(_completed, std::move(result));
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

            if (!should_continue_job(job.track_id,
                                     job.generation_id,
                                     job.request_epoch,
                                     job.request.maneuver_plan_revision,
                                     job.request.solve_quality))
            {
                continue;
            }

            // Run the expensive simulation outside the lock so request() stays responsive.
            compute_prediction(job);
        }
    }
} // namespace Game
