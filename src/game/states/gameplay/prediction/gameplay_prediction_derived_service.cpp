#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace
    {
        bool derived_requests_share_stage(const OrbitPredictionDerivedService::Request &a,
                                          const OrbitPredictionDerivedService::Request &b)
        {
            return a.solver_result.publish_stage == b.solver_result.publish_stage;
        }

        bool request_is_full_streaming_stage(const OrbitPredictionDerivedService::Request &request)
        {
            return request.solver_result.publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
        }

        bool derived_requests_share_context(const OrbitPredictionDerivedService::Request &a,
                                            const OrbitPredictionDerivedService::Request &b)
        {
            return a.display_frame_key == b.display_frame_key &&
                   a.display_frame_revision == b.display_frame_revision &&
                   a.analysis_body_id == b.analysis_body_id &&
                   a.reuse_existing_base_frame == b.reuse_existing_base_frame &&
                   a.resolved_frame_spec.type == b.resolved_frame_spec.type &&
                   a.resolved_frame_spec.primary_body_id == b.resolved_frame_spec.primary_body_id &&
                   a.resolved_frame_spec.secondary_body_id == b.resolved_frame_spec.secondary_body_id &&
                   a.resolved_frame_spec.target_spacecraft_id == b.resolved_frame_spec.target_spacecraft_id;
        }

        void merge_published_chunks(std::vector<OrbitPredictionService::PublishedChunk> &dst,
                                    const std::vector<OrbitPredictionService::PublishedChunk> &src)
        {
            for (const OrbitPredictionService::PublishedChunk &chunk : src)
            {
                auto existing = std::find_if(dst.begin(),
                                             dst.end(),
                                             [&chunk](const OrbitPredictionService::PublishedChunk &candidate) {
                                                 return candidate.chunk_id == chunk.chunk_id;
                                             });
                if (existing != dst.end())
                {
                    *existing = chunk;
                    continue;
                }
                dst.push_back(chunk);
            }

            std::sort(dst.begin(),
                      dst.end(),
                      [](const OrbitPredictionService::PublishedChunk &a,
                         const OrbitPredictionService::PublishedChunk &b) { return a.chunk_id < b.chunk_id; });
        }

        void merge_streamed_planned_chunks(std::vector<OrbitPredictionService::StreamedPlannedChunk> &dst,
                                           const std::vector<OrbitPredictionService::StreamedPlannedChunk> &src)
        {
            for (const OrbitPredictionService::StreamedPlannedChunk &chunk : src)
            {
                auto existing = std::find_if(dst.begin(),
                                             dst.end(),
                                             [&chunk](const OrbitPredictionService::StreamedPlannedChunk &candidate) {
                                                 return candidate.published_chunk.chunk_id ==
                                                        chunk.published_chunk.chunk_id;
                                             });
                if (existing != dst.end())
                {
                    *existing = chunk;
                    continue;
                }
                dst.push_back(chunk);
            }

            std::sort(dst.begin(),
                      dst.end(),
                      [](const OrbitPredictionService::StreamedPlannedChunk &a,
                         const OrbitPredictionService::StreamedPlannedChunk &b) {
                          return a.published_chunk.chunk_id < b.published_chunk.chunk_id;
                      });
        }

        void merge_pending_full_stream_request(OrbitPredictionDerivedService::Request &dst,
                                               OrbitPredictionDerivedService::Request src)
        {
            std::vector<OrbitPredictionService::PublishedChunk> merged_published_chunks =
                    dst.solver_result.published_chunks;
            merge_published_chunks(merged_published_chunks, src.solver_result.published_chunks);

            std::vector<OrbitPredictionService::StreamedPlannedChunk> merged_streamed_chunks =
                    dst.solver_result.streamed_planned_chunks;
            merge_streamed_planned_chunks(merged_streamed_chunks, src.solver_result.streamed_planned_chunks);

            src.solver_result.published_chunks = std::move(merged_published_chunks);
            src.solver_result.streamed_planned_chunks = std::move(merged_streamed_chunks);
            dst = std::move(src);
        }
    } // namespace

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

            auto it = _pending_jobs.begin();
            while (it != _pending_jobs.end())
            {
                if (it->track_id != request.track_id)
                {
                    ++it;
                    continue;
                }

                if (it->generation_id < request.generation_id)
                {
                    it = _pending_jobs.erase(it);
                    continue;
                }

                if (it->generation_id == request.generation_id)
                {
                    if (!derived_requests_share_stage(it->request, request))
                    {
                        ++it;
                        continue;
                    }

                    it->request_epoch = _request_epoch;
                    it->generation_id = request.generation_id;
                    if (request_is_full_streaming_stage(it->request) &&
                        request_is_full_streaming_stage(request) &&
                        derived_requests_share_context(it->request, request))
                    {
                        merge_pending_full_stream_request(it->request, std::move(request));
                    }
                    else
                    {
                        it->request = std::move(request);
                    }
                    _cv.notify_one();
                    return;
                }

                ++it;
            }

            PendingJob job{};
            job.track_id = request.track_id;
            job.request_epoch = _request_epoch;
            job.generation_id = request.generation_id;
            job.request = std::move(request);
            _pending_jobs.push_back(std::move(job));
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
        _tracks_in_flight.clear();
    }

    bool OrbitPredictionDerivedService::should_publish_result(
            const uint64_t track_id,
            const uint64_t generation_id,
            const uint64_t request_epoch,
            const uint64_t current_request_epoch,
            const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track)
    {
        if (request_epoch != current_request_epoch)
        {
            return false;
        }

        const auto latest_it = latest_requested_generation_by_track.find(track_id);
        return latest_it == latest_requested_generation_by_track.end() ||
               generation_id >= latest_it->second;
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
        const auto build_start_tp = std::chrono::steady_clock::now();
        Result out{};
        out.track_id = job.track_id;
        out.generation_id = job.generation_id;
        out.display_frame_key = job.request.display_frame_key;
        out.display_frame_revision = job.request.display_frame_revision;
        out.analysis_body_id = job.request.analysis_body_id;

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
        const bool preview_stage = solver.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
        const bool preview_streaming_stage =
                preview_stage &&
                solver.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming;
        const bool full_streaming_stage =
                !preview_stage &&
                solver.publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
        const bool rebuild_metrics = !preview_stage && !full_streaming_stage;
        const bool build_planned_render_curve = !preview_streaming_stage && !full_streaming_stage;
        const bool build_chunk_render_curves = full_streaming_stage;
        const bool use_dense_chunk_samples = !preview_stage && !full_streaming_stage;
        if (!solver.valid ||
            solver.resolved_trajectory_inertial().size() < 2 ||
            solver.resolved_trajectory_segments_inertial().empty())
        {
            out.diagnostics.status = PredictionDerivedStatus::MissingSolverData;
            out.timings.total_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start_tp).count();
            return out;
        }

        OrbitPredictionCache cache{};
        cache.generation_id = job.generation_id;
        cache.build_time_s = solver.build_time_s;
        cache.build_pos_world = request.build_pos_world;
        cache.build_vel_world = request.build_vel_world;
        cache.shared_ephemeris = solver.resolved_shared_ephemeris();
        cache.massive_bodies = solver.take_massive_bodies();
        cache.trajectory_inertial = solver.take_trajectory_inertial();
        cache.trajectory_inertial_planned = std::move(solver.trajectory_inertial_planned);
        cache.trajectory_segments_inertial = solver.take_trajectory_segments_inertial();
        cache.trajectory_segments_inertial_planned = std::move(solver.trajectory_segments_inertial_planned);
        cache.maneuver_previews = std::move(solver.maneuver_previews);
        cache.valid = true;

        const orbitsim::TrajectoryFrameSpec &resolved_frame_spec = request.resolved_frame_spec;
        cache.display_frame_key = request.display_frame_key;
        cache.display_frame_revision = request.display_frame_revision;
        const bool reuse_existing_base_frame = request.reuse_existing_base_frame;
        if (reuse_existing_base_frame)
        {
            cache.analysis_cache_body_id = request.analysis_body_id;
            cache.metrics_body_id = request.analysis_body_id;
        }

        bool frame_cache_built = false;
        const auto frame_build_start_tp = std::chrono::steady_clock::now();
        if (reuse_existing_base_frame)
        {
            frame_cache_built = PredictionCacheInternal::rebuild_prediction_planned_frame_cache(
                    cache,
                    resolved_frame_spec,
                    request.player_lookup_segments_inertial,
                    cancel_requested,
                    &out.diagnostics,
                    build_planned_render_curve);
            out.timings.frame_build_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame_build_start_tp)
                            .count();
        }
        else
        {
            frame_cache_built = PredictionCacheInternal::rebuild_prediction_frame_cache(
                    cache,
                    resolved_frame_spec,
                    request.player_lookup_segments_inertial,
                    cancel_requested,
                    &out.diagnostics,
                    build_planned_render_curve);
            out.timings.frame_build_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame_build_start_tp)
                            .count();
        }

        if (!frame_cache_built)
        {
            out.timings.total_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start_tp).count();
            return out;
        }
        out.base_frame_reused = reuse_existing_base_frame;
        if (reuse_existing_base_frame)
        {
            out.diagnostics.frame_base = request.reused_base_frame_diagnostics;
        }

        if (rebuild_metrics)
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
            out.timings.total_ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start_tp).count();
            return out;
        }

        OrbitPredictionDerivedDiagnostics chunk_diagnostics{};
        if (full_streaming_stage)
        {
            if (!solver.streamed_planned_chunks.empty())
            {
                PredictionChunkAssembly chunk_assembly{};
                if (PredictionCacheInternal::rebuild_prediction_streamed_chunk_assembly(
                            chunk_assembly,
                            cache,
                            solver.streamed_planned_chunks,
                            job.generation_id,
                            resolved_frame_spec,
                            request.player_lookup_segments_inertial,
                            cancel_requested,
                            &chunk_diagnostics,
                            build_chunk_render_curves,
                            use_dense_chunk_samples))
                {
                    out.chunk_assembly = std::move(chunk_assembly);
                }
            }
        }
        else if (!solver.published_chunks.empty())
        {
            std::vector<double> node_times_s;
            node_times_s.reserve(solver.maneuver_previews.size());
            for (const OrbitPredictionService::ManeuverNodePreview &preview : solver.maneuver_previews)
            {
                if (std::isfinite(preview.t_s))
                {
                    node_times_s.push_back(preview.t_s);
                }
            }

            PredictionChunkAssembly chunk_assembly{};
            if (PredictionCacheInternal::rebuild_prediction_patch_chunks(chunk_assembly,
                                                                         cache,
                                                                         solver.published_chunks,
                                                                         job.generation_id,
                                                                         resolved_frame_spec,
                                                                         request.display_frame_key,
                                                                         request.display_frame_revision,
                                                                         cancel_requested,
                                                                         node_times_s,
                                                                         &chunk_diagnostics,
                                                                         build_chunk_render_curves,
                                                                         use_dense_chunk_samples))
            {
                out.chunk_assembly = std::move(chunk_assembly);
                if (!preview_stage)
                {
                    const auto flatten_start_tp = std::chrono::steady_clock::now();
                    PredictionCacheInternal::flatten_chunk_assembly_to_cache(
                            cache,
                            out.chunk_assembly,
                            build_planned_render_curve);
                    out.timings.flatten_ms =
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                                      flatten_start_tp)
                                    .count();
                }
            }
        }

        if (chunk_diagnostics.status != PredictionDerivedStatus::None)
        {
            out.diagnostics.status = chunk_diagnostics.status;
            out.diagnostics.frame_planned = chunk_diagnostics.frame_planned;
            out.diagnostics.frame_segment_count_planned = chunk_diagnostics.frame_segment_count_planned;
            out.diagnostics.frame_sample_count_planned = chunk_diagnostics.frame_sample_count_planned;
        }

        out.cache = std::move(cache);
        out.valid = out.cache.valid;
        out.timings.total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start_tp).count();
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
                    if (!_running)
                    {
                        return true;
                    }

                    return std::any_of(_pending_jobs.begin(),
                                       _pending_jobs.end(),
                                       [this](const PendingJob &pending) {
                                           return !_tracks_in_flight.contains(pending.track_id);
                                       });
                });

                if (!_running && _pending_jobs.empty())
                {
                    return;
                }

                auto job_it = std::find_if(_pending_jobs.begin(),
                                           _pending_jobs.end(),
                                           [this](const PendingJob &pending) {
                                               return !_tracks_in_flight.contains(pending.track_id);
                                           });
                if (job_it == _pending_jobs.end())
                {
                    continue;
                }

                job = std::move(*job_it);
                _pending_jobs.erase(job_it);
                _tracks_in_flight.insert(job.track_id);
            }

            if (!should_continue_job(job.track_id, job.generation_id, job.request_epoch))
            {
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _tracks_in_flight.erase(job.track_id);
                }
                _cv.notify_all();
                continue;
            }

            const uint64_t completed_track_id = job.track_id;
            const uint64_t completed_generation_id = job.generation_id;
            const uint64_t completed_request_epoch = job.request_epoch;
            Result result = build_cache(std::move(job));

            bool should_enqueue_result = false;
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _tracks_in_flight.erase(completed_track_id);
                if (!_running)
                {
                    break;
                }

                should_enqueue_result = should_publish_result(completed_track_id,
                                                             completed_generation_id,
                                                             completed_request_epoch,
                                                             _request_epoch,
                                                             _latest_requested_generation_by_track);
                if (should_enqueue_result)
                {
                    _completed.push_back(std::move(result));
                }
            }
            _cv.notify_all();
        }
    }
} // namespace Game
