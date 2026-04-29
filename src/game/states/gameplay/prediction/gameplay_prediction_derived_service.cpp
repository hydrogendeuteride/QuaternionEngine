#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/prediction_frame_cache_builder.h"
#include "game/states/gameplay/prediction/prediction_metrics_builder.h"
#include "game/states/gameplay/prediction/streamed_chunk_assembly_builder.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace Game
{
    namespace
    {
        constexpr std::size_t kMaxPendingDerivedCompletedResults = 64;

        bool request_is_full_streaming_stage(const OrbitPredictionDerivedService::Request &request)
        {
            return request.solver_result.publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
        }

        bool request_is_preview_streaming_stage(const OrbitPredictionDerivedService::Request &request)
        {
            return request.solver_result.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming;
        }

        uint8_t derived_request_track_priority_rank(const OrbitPredictionService::RequestPriority priority)
        {
            switch (priority)
            {
                case OrbitPredictionService::RequestPriority::ActiveInteractiveTrack:
                case OrbitPredictionService::RequestPriority::ActiveTrack:
                    return 3u;
                case OrbitPredictionService::RequestPriority::Overlay:
                    return 2u;
                case OrbitPredictionService::RequestPriority::BackgroundOrbiter:
                    return 1u;
                case OrbitPredictionService::RequestPriority::BackgroundCelestial:
                    return 0u;
            }
            return 0u;
        }

        bool derived_result_is_streaming(const OrbitPredictionDerivedService::Result &result)
        {
            return result.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming ||
                   result.publish_stage == OrbitPredictionService::PublishStage::FullStreaming;
        }

        bool should_drop_derived_completed_result_for_incoming(
                const OrbitPredictionDerivedService::Result &queued,
                const OrbitPredictionDerivedService::Result &incoming)
        {
            if (queued.track_id != incoming.track_id || !derived_result_is_streaming(queued))
            {
                return false;
            }

            if (!derived_result_is_streaming(incoming))
            {
                return true;
            }

            if (incoming.generation_id > queued.generation_id)
            {
                return true;
            }

            return incoming.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming &&
                   queued.publish_stage == OrbitPredictionService::PublishStage::PreviewStreaming;
        }

        void coalesce_derived_completed_results(
                std::deque<OrbitPredictionDerivedService::Result> &completed,
                const OrbitPredictionDerivedService::Result &incoming)
        {
            completed.erase(std::remove_if(completed.begin(),
                                           completed.end(),
                                           [&incoming](const OrbitPredictionDerivedService::Result &queued) {
                                               return should_drop_derived_completed_result_for_incoming(queued, incoming);
                                           }),
                            completed.end());
        }

        void trim_derived_completed_results(std::deque<OrbitPredictionDerivedService::Result> &completed)
        {
            while (completed.size() > kMaxPendingDerivedCompletedResults)
            {
                auto streaming_it = std::find_if(completed.begin(),
                                                 completed.end(),
                                                 [](const OrbitPredictionDerivedService::Result &queued) {
                                                     return derived_result_is_streaming(queued);
                                                 });
                if (streaming_it == completed.end())
                {
                    return;
                }
                completed.erase(streaming_it);
            }
        }

        void enqueue_derived_completed_result(std::deque<OrbitPredictionDerivedService::Result> &completed,
                                              OrbitPredictionDerivedService::Result result)
        {
            coalesce_derived_completed_results(completed, result);
            completed.push_back(std::move(result));
            trim_derived_completed_results(completed);
        }

        void discard_derived_completed_results_before_generation(
                std::deque<OrbitPredictionDerivedService::Result> &completed,
                const uint64_t track_id,
                const uint64_t generation_id,
                const bool preserve_fast_preview)
        {
            completed.erase(std::remove_if(completed.begin(),
                                           completed.end(),
                                           [track_id, generation_id, preserve_fast_preview](
                                                   const OrbitPredictionDerivedService::Result &queued) {
                                                return queued.track_id == track_id &&
                                                       queued.generation_id < generation_id &&
                                                       !(preserve_fast_preview &&
                                                         queued.solve_quality ==
                                                                 OrbitPredictionService::SolveQuality::FastPreview);
                                           }),
                            completed.end());
        }

        void discard_stale_maneuver_derived_completed_results(
                std::deque<OrbitPredictionDerivedService::Result> &completed,
                const uint64_t track_id,
                const uint64_t maneuver_plan_revision)
        {
            completed.erase(std::remove_if(completed.begin(),
                                           completed.end(),
                                           [track_id, maneuver_plan_revision](const OrbitPredictionDerivedService::Result &queued) {
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

        uint8_t derived_request_stage_priority_rank(const OrbitPredictionDerivedService::Request &request)
        {
            if (request_is_preview_streaming_stage(request))
            {
                return 2u;
            }
            if (request_is_full_streaming_stage(request))
            {
                return 0u;
            }
            return 1u;
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

        void populate_prediction_cache_identity(OrbitPredictionCache &cache,
                                                const uint64_t generation_id,
                                                const OrbitPredictionDerivedService::Request &request,
                                                const OrbitPredictionService::Result &solver)
        {
            cache.identity.generation_id = generation_id;
            cache.identity.maneuver_plan_revision = request.maneuver_plan_revision;
            cache.identity.maneuver_plan_signature_valid = request.maneuver_plan_signature_valid;
            cache.identity.maneuver_plan_signature = request.maneuver_plan_signature;
            cache.identity.build_time_s = solver.build_time_s;
            cache.identity.build_pos_world = request.build_pos_world;
            cache.identity.build_vel_world = request.build_vel_world;
        }

        void populate_prediction_cache_solver(OrbitPredictionCache &cache,
                                              OrbitPredictionService::Result &solver)
        {
            if (solver.has_shared_core_data())
            {
                cache.set_shared_solver_core_data(solver.shared_core_data());
            }
            else
            {
                cache.solver.shared_ephemeris = std::move(solver.shared_ephemeris);
                cache.solver.massive_bodies = solver.take_massive_bodies();
                cache.solver.trajectory_inertial = solver.take_trajectory_inertial();
                cache.solver.trajectory_segments_inertial = solver.take_trajectory_segments_inertial();
            }
            cache.solver.trajectory_inertial_planned = std::move(solver.trajectory_inertial_planned);
            cache.solver.trajectory_segments_inertial_planned = std::move(solver.trajectory_segments_inertial_planned);
            cache.solver.maneuver_previews = std::move(solver.maneuver_previews);
            cache.identity.valid = true;
        }

        std::vector<double> finite_maneuver_preview_times(
                const std::vector<OrbitPredictionService::ManeuverNodePreview> &previews)
        {
            std::vector<double> node_times_s;
            node_times_s.reserve(previews.size());
            for (const OrbitPredictionService::ManeuverNodePreview &preview : previews)
            {
                if (std::isfinite(preview.t_s))
                {
                    node_times_s.push_back(preview.t_s);
                }
            }
            return node_times_s;
        }

        template<typename CancelRequestedFn>
        void build_prediction_chunk_assembly_for_derived_request(
                OrbitPredictionDerivedService::Result &out,
                OrbitPredictionCache &cache,
                const OrbitPredictionDerivedService::Request &request,
                const OrbitPredictionService::Result &solver,
                const uint64_t generation_id,
                const bool preview_stage,
                const bool full_streaming_stage,
                const bool build_chunk_render_curves,
                const bool build_planned_render_curve,
                const bool use_dense_chunk_samples,
                CancelRequestedFn &&cancel_requested,
                OrbitPredictionDerivedDiagnostics &chunk_diagnostics)
        {
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec = request.resolved_frame_spec;
            if (full_streaming_stage)
            {
                if (solver.streamed_planned_chunks.empty())
                {
                    return;
                }

                PredictionChunkAssembly chunk_assembly{};
                if (StreamedChunkAssemblyBuilder::rebuild_from_streamed(
                            chunk_assembly,
                            cache.solver,
                            cache.display,
                            solver.streamed_planned_chunks,
                            generation_id,
                            resolved_frame_spec,
                            request.player_lookup_segments_inertial,
                            cancel_requested,
                            &chunk_diagnostics,
                            build_chunk_render_curves,
                            use_dense_chunk_samples))
                {
                    out.chunk_assembly = std::move(chunk_assembly);
                }
                return;
            }

            if (solver.published_chunks.empty())
            {
                return;
            }

            const std::vector<double> node_times_s = finite_maneuver_preview_times(cache.solver.maneuver_previews);
            PredictionChunkAssembly chunk_assembly{};
            if (StreamedChunkAssemblyBuilder::rebuild_from_published(chunk_assembly,
                                                                      cache.display,
                                                                      solver.published_chunks,
                                                                      generation_id,
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
                    StreamedChunkAssemblyBuilder::flatten(
                            cache.display,
                            out.chunk_assembly,
                            build_planned_render_curve);
                    out.timings.flatten_ms =
                            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                                      flatten_start_tp)
                                    .count();
                }
            }
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
            if (!maneuver_revision_is_current(request.track_id,
                                              request.maneuver_plan_revision,
                                              _latest_maneuver_plan_revision_by_track))
            {
                return;
            }

            const auto latest_it = _latest_requested_generation_by_track.find(request.track_id);
            const bool fast_preview_request =
                    request.solver_result.solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
            if (latest_it != _latest_requested_generation_by_track.end() &&
                request.generation_id < latest_it->second &&
                !fast_preview_request)
            {
                return;
            }
            _latest_requested_generation_by_track[request.track_id] =
                    std::max(_latest_requested_generation_by_track[request.track_id],
                             request.generation_id);
            _latest_maneuver_plan_revision_by_track[request.track_id] =
                    std::max(_latest_maneuver_plan_revision_by_track[request.track_id],
                             request.maneuver_plan_revision);
            discard_derived_completed_results_before_generation(_completed,
                                                                request.track_id,
                                                                request.generation_id,
                                                                fast_preview_request);
            discard_stale_maneuver_derived_completed_results(_completed,
                                                             request.track_id,
                                                             _latest_maneuver_plan_revision_by_track[request.track_id]);

            PendingJob job{};
            job.track_id = request.track_id;
            job.request_epoch = _request_epoch;
            job.generation_id = request.generation_id;
            job.enqueue_serial = _next_enqueue_serial++;
            job.request = std::move(request);

            const auto job_precedes = [](const PendingJob &candidate, const PendingJob &queued) {
                const uint8_t candidate_track_priority =
                        derived_request_track_priority_rank(candidate.request.priority);
                const uint8_t queued_track_priority =
                        derived_request_track_priority_rank(queued.request.priority);
                if (candidate_track_priority != queued_track_priority)
                {
                    return candidate_track_priority > queued_track_priority;
                }

                const uint8_t candidate_stage_priority =
                        derived_request_stage_priority_rank(candidate.request);
                const uint8_t queued_stage_priority =
                        derived_request_stage_priority_rank(queued.request);
                if (candidate_stage_priority != queued_stage_priority)
                {
                    return candidate_stage_priority > queued_stage_priority;
                }

                if (candidate.generation_id != queued.generation_id)
                {
                    return candidate.generation_id > queued.generation_id;
                }

                return candidate.enqueue_serial > queued.enqueue_serial;
            };

            auto it = _pending_jobs.begin();
            while (it != _pending_jobs.end())
            {
                if (it->track_id != job.track_id)
                {
                    ++it;
                    continue;
                }

                if (it->generation_id > job.generation_id)
                {
                    if (!fast_preview_request)
                    {
                        return;
                    }
                    ++it;
                    continue;
                }

                if (it->generation_id < job.generation_id)
                {
                    it = _pending_jobs.erase(it);
                    continue;
                }

                if (it->generation_id == job.generation_id)
                {
                    if (request_is_full_streaming_stage(it->request) &&
                        request_is_full_streaming_stage(job.request) &&
                        derived_requests_share_context(it->request, job.request))
                    {
                        Request merged_request = std::move(it->request);
                        merge_pending_full_stream_request(merged_request, std::move(job.request));
                        job.request = std::move(merged_request);
                    }

                    // Same-generation derived requests target the same authoritative solver result.
                    // Keep only the newest pending job so full-stream backlogs do not delay final refreshes.
                    it = _pending_jobs.erase(it);
                    continue;
                }

                ++it;
            }

            const auto insert_it =
                    std::find_if(_pending_jobs.begin(),
                                 _pending_jobs.end(),
                                 [&job_precedes, &job](const PendingJob &queued) {
                                     return job_precedes(job, queued);
                                 });
            _pending_jobs.insert(insert_it, std::move(job));
        }
        _cv.notify_one();
    }

    void OrbitPredictionDerivedService::invalidate_maneuver_plan_revision(
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

            discard_stale_maneuver_derived_completed_results(_completed, track_id, latest_revision);
        }
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
        _next_enqueue_serial = 1;
        _pending_jobs.clear();
        _completed.clear();
        _latest_requested_generation_by_track.clear();
        _latest_maneuver_plan_revision_by_track.clear();
        _tracks_in_flight.clear();
    }

    bool OrbitPredictionDerivedService::should_publish_result(
            const uint64_t track_id,
            const uint64_t generation_id,
            const uint64_t request_epoch,
            const uint64_t current_request_epoch,
            const OrbitPredictionService::SolveQuality solve_quality,
            const std::unordered_map<uint64_t, uint64_t> &latest_requested_generation_by_track)
    {
        if (request_epoch != current_request_epoch)
        {
            return false;
        }

        const auto latest_it = latest_requested_generation_by_track.find(track_id);
        return latest_it == latest_requested_generation_by_track.end() ||
               generation_id >= latest_it->second ||
               solve_quality == OrbitPredictionService::SolveQuality::FastPreview;
    }

    bool OrbitPredictionDerivedService::should_continue_job(const uint64_t track_id,
                                                            const uint64_t generation_id,
                                                            const uint64_t request_epoch,
                                                            const uint64_t maneuver_plan_revision,
                                                            const OrbitPredictionService::SolveQuality solve_quality) const
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

    OrbitPredictionDerivedService::Result OrbitPredictionDerivedService::build_cache(PendingJob job) const
    {
        const auto build_start_tp = std::chrono::steady_clock::now();
        Result out{};
        out.track_id = job.track_id;
        out.generation_id = job.generation_id;
        out.maneuver_plan_revision = job.request.maneuver_plan_revision;
        out.maneuver_plan_signature_valid = job.request.maneuver_plan_signature_valid;
        out.maneuver_plan_signature = job.request.maneuver_plan_signature;
        out.display_frame_key = job.request.display_frame_key;
        out.display_frame_revision = job.request.display_frame_revision;
        out.analysis_body_id = job.request.analysis_body_id;

        const auto cancel_requested = [this,
                                        track_id = job.track_id,
                                        generation_id = job.generation_id,
                                        request_epoch = job.request_epoch,
                                        maneuver_plan_revision = job.request.maneuver_plan_revision,
                                        solve_quality = job.request.solver_result.solve_quality]() {
            return !should_continue_job(track_id, generation_id, request_epoch, maneuver_plan_revision, solve_quality);
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
        populate_prediction_cache_identity(cache, job.generation_id, request, solver);
        populate_prediction_cache_solver(cache, solver);

        const orbitsim::TrajectoryFrameSpec &resolved_frame_spec = request.resolved_frame_spec;
        cache.display.display_frame_key = request.display_frame_key;
        cache.display.display_frame_revision = request.display_frame_revision;
        const bool reuse_existing_base_frame = request.reuse_existing_base_frame;
        if (reuse_existing_base_frame)
        {
            cache.analysis.analysis_cache_body_id = request.analysis_body_id;
            cache.analysis.metrics_body_id = request.analysis_body_id;
        }

        bool frame_cache_built = false;
        const auto frame_build_start_tp = std::chrono::steady_clock::now();
        if (reuse_existing_base_frame)
        {
            frame_cache_built = PredictionFrameCacheBuilder::rebuild_planned(
                    cache.solver,
                    cache.display,
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
            frame_cache_built = PredictionFrameCacheBuilder::rebuild(
                    cache.solver,
                    cache.display,
                    cache.analysis,
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
            PredictionMetricsBuilder::rebuild(
                    cache.solver,
                    cache.display,
                    cache.analysis,
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
        build_prediction_chunk_assembly_for_derived_request(out,
                                                            cache,
                                                            request,
                                                            solver,
                                                            job.generation_id,
                                                            preview_stage,
                                                            full_streaming_stage,
                                                            build_chunk_render_curves,
                                                            build_planned_render_curve,
                                                            use_dense_chunk_samples,
                                                            cancel_requested,
                                                            chunk_diagnostics);

        if (chunk_diagnostics.status != PredictionDerivedStatus::None)
        {
            out.diagnostics.status = chunk_diagnostics.status;
            out.diagnostics.frame_planned = chunk_diagnostics.frame_planned;
            out.diagnostics.frame_segment_count_planned = chunk_diagnostics.frame_segment_count_planned;
            out.diagnostics.frame_sample_count_planned = chunk_diagnostics.frame_sample_count_planned;
        }

        out.cache = std::move(cache);
        out.valid = out.cache.identity.valid;
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

            if (!should_continue_job(job.track_id,
                                     job.generation_id,
                                     job.request_epoch,
                                     job.request.maneuver_plan_revision,
                                     job.request.solver_result.solve_quality))
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
                                                              result.solve_quality,
                                                              _latest_requested_generation_by_track);
                should_enqueue_result = should_enqueue_result &&
                                        maneuver_revision_is_current(completed_track_id,
                                                                     result.maneuver_plan_revision,
                                                                     _latest_maneuver_plan_revision_by_track);
                if (should_enqueue_result)
                {
                    enqueue_derived_completed_result(_completed, std::move(result));
                }
            }
            _cv.notify_all();
        }
    }
} // namespace Game
