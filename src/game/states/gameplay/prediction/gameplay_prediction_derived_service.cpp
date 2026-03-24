#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include <algorithm>

namespace Game
{
    OrbitPredictionDerivedService::OrbitPredictionDerivedService()
    {
        const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
        const std::size_t worker_count = std::clamp<std::size_t>(static_cast<std::size_t>(hw_threads > 2 ? 2 : 1), 1, 2);
        _workers.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i)
        {
            _workers.emplace_back(&OrbitPredictionDerivedService::worker_loop, this);
        }
    }

    OrbitPredictionDerivedService::~OrbitPredictionDerivedService()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
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

    void OrbitPredictionDerivedService::request(Request request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _latest_requested_generation_by_track[request.track_id] = request.generation_id;

            auto existing = std::find_if(_pending_jobs.begin(),
                                         _pending_jobs.end(),
                                         [track_id = request.track_id](const PendingJob &job) {
                                             return job.track_id == track_id;
                                         });
            if (existing != _pending_jobs.end())
            {
                existing->request_epoch = _request_epoch;
                existing->generation_id = request.generation_id;
                existing->request = std::move(request);
            }
            else
            {
                PendingJob job{};
                job.track_id = request.track_id;
                job.request_epoch = _request_epoch;
                job.generation_id = request.generation_id;
                job.request = std::move(request);
                _pending_jobs.push_back(std::move(job));
            }
        }
        _cv.notify_one();
    }

    std::optional<OrbitPredictionDerivedService::Result> OrbitPredictionDerivedService::poll_completed()
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

    void OrbitPredictionDerivedService::reset()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        ++_request_epoch;
        _pending_jobs.clear();
        _completed.clear();
        _latest_requested_generation_by_track.clear();
    }

    bool OrbitPredictionDerivedService::should_publish_result(
            const PendingJob &job,
            const uint64_t current_request_epoch,
            const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track)
    {
        if (job.request_epoch != current_request_epoch)
        {
            return false;
        }

        const auto latest_it = latest_requested_generation_by_track.find(job.track_id);
        return latest_it == latest_requested_generation_by_track.end() ||
               job.generation_id >= latest_it->second;
    }

    bool OrbitPredictionDerivedService::should_continue_job(const uint64_t track_id,
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

    OrbitPredictionDerivedService::Result OrbitPredictionDerivedService::build_cache(PendingJob job) const
    {
        Result out{};
        out.track_id = job.track_id;
        out.generation_id = job.generation_id;

        const auto cancel_requested = [this,
                                       track_id = job.track_id,
                                       generation_id = job.generation_id,
                                       request_epoch = job.request_epoch]() {
            return !should_continue_job(track_id, generation_id, request_epoch);
        };

        Request &request = job.request;
        OrbitPredictionService::Result &solver = request.solver_result;
        out.solve_quality = solver.solve_quality;
        out.publish_stage = solver.publish_stage;
        out.generation_complete = solver.generation_complete;
        if (!solver.valid || solver.trajectory_inertial.size() < 2 || solver.trajectory_segments_inertial.empty())
        {
            out.diagnostics.status = PredictionDerivedStatus::MissingSolverData;
            return out;
        }

        OrbitPredictionCache cache{};
        cache.generation_id = job.generation_id;
        cache.build_time_s = solver.build_time_s;
        cache.build_pos_world = request.build_pos_world;
        cache.build_vel_world = request.build_vel_world;
        cache.shared_ephemeris = solver.shared_ephemeris;
        cache.massive_bodies = std::move(solver.massive_bodies);
        cache.trajectory_inertial = std::move(solver.trajectory_inertial);
        cache.trajectory_inertial_planned = std::move(solver.trajectory_inertial_planned);
        cache.trajectory_segments_inertial = std::move(solver.trajectory_segments_inertial);
        cache.trajectory_segments_inertial_planned = std::move(solver.trajectory_segments_inertial_planned);
        cache.maneuver_previews = std::move(solver.maneuver_previews);
        cache.valid = true;

        const orbitsim::TrajectoryFrameSpec &resolved_frame_spec = request.resolved_frame_spec;
        const bool reuse_existing_base_frame =
                request.reuse_existing_base_frame &&
                solver.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
        if (reuse_existing_base_frame)
        {
            cache.analysis_cache_body_id = request.analysis_body_id;
            cache.metrics_body_id = request.analysis_body_id;
        }

        const bool use_chunk_path =
                reuse_existing_base_frame && !solver.published_chunks.empty();
        bool frame_cache_built = false;
        if (use_chunk_path)
        {
            frame_cache_built = PredictionCacheInternal::rebuild_prediction_patch_chunks(
                    out.chunk_assembly,
                    cache,
                    solver.published_chunks,
                    job.generation_id,
                    resolved_frame_spec,
                    request.player_lookup_segments_inertial,
                    cancel_requested,
                    &out.diagnostics);
            if (frame_cache_built)
            {
                PredictionCacheInternal::flatten_chunk_assembly_to_cache(cache, out.chunk_assembly);
                cache.resolved_frame_spec = resolved_frame_spec;
                cache.resolved_frame_spec_valid = true;
                out.diagnostics.status = PredictionDerivedStatus::Success;
            }
        }
        else if (reuse_existing_base_frame)
        {
            frame_cache_built = PredictionCacheInternal::rebuild_prediction_planned_frame_cache(
                    cache,
                    resolved_frame_spec,
                    request.player_lookup_segments_inertial,
                    cancel_requested,
                    &out.diagnostics);
        }
        else
        {
            frame_cache_built = PredictionCacheInternal::rebuild_prediction_frame_cache(
                    cache,
                    resolved_frame_spec,
                    request.player_lookup_segments_inertial,
                    cancel_requested,
                    &out.diagnostics);
        }

        if (!frame_cache_built)
        {
            return out;
        }
        out.base_frame_reused = reuse_existing_base_frame;
        if (reuse_existing_base_frame)
        {
            out.diagnostics.frame_base = request.reused_base_frame_diagnostics;
        }

        if (solver.solve_quality != OrbitPredictionService::SolveQuality::FastPreview)
        {
            PredictionCacheInternal::rebuild_prediction_metrics(
                    cache,
                    request.sim_config,
                    request.analysis_body_id,
                    cancel_requested);
        }

        if (cancel_requested())
        {
            out.diagnostics.status = PredictionDerivedStatus::Cancelled;
            return out;
        }

        out.cache = std::move(cache);
        out.valid = out.cache.valid;
        return out;
    }

    void OrbitPredictionDerivedService::worker_loop()
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

            Result result = build_cache(std::move(job));

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
} // namespace Game
