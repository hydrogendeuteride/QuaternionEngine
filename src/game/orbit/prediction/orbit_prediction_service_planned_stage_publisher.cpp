#include "game/orbit/prediction/orbit_prediction_service_internal.h"

#include <chrono>

namespace Game
{
    namespace
    {
        void sync_prediction_stage_counts(OrbitPredictionService::Result &result)
        {
            result.diagnostics.ephemeris_segment_count = result.diagnostics.ephemeris.accepted_segments;
            result.diagnostics.trajectory_segment_count = result.diagnostics.trajectory_base.accepted_segments;
            result.diagnostics.trajectory_segment_count_planned = result.diagnostics.trajectory_planned.accepted_segments;
        }

        OrbitPredictionService::StreamedPlannedChunk make_streamed_planned_chunk(
                const PlannedChunkPacket &packet,
                const OrbitPredictionService::PublishedChunk &published_chunk)
        {
            OrbitPredictionService::StreamedPlannedChunk streamed_chunk{};
            streamed_chunk.published_chunk = published_chunk;
            streamed_chunk.trajectory_segments_inertial = packet.segments;
            streamed_chunk.trajectory_inertial = packet.samples;
            streamed_chunk.maneuver_previews = packet.previews;
            streamed_chunk.diagnostics = packet.diagnostics;
            streamed_chunk.start_state = packet.start_state;
            streamed_chunk.end_state = packet.end_state;
            return streamed_chunk;
        }

        std::vector<OrbitPredictionService::ManeuverNodePreview> collect_prefix_chunk_previews(
                const std::vector<OrbitPredictionService::ManeuverNodePreview> &previews,
                const double chunk_t0_s,
                const double chunk_t1_s)
        {
            std::vector<OrbitPredictionService::ManeuverNodePreview> out;
            for (const OrbitPredictionService::ManeuverNodePreview &preview : previews)
            {
                if (!preview.valid || !std::isfinite(preview.t_s))
                {
                    continue;
                }

                const double start_epsilon_s = continuity_time_epsilon_s(chunk_t0_s);
                const double end_epsilon_s = continuity_time_epsilon_s(chunk_t1_s);
                if (preview.t_s >= (chunk_t0_s - start_epsilon_s) &&
                    preview.t_s < (chunk_t1_s - end_epsilon_s))
                {
                    out.push_back(preview);
                }
            }
            return out;
        }
    } // namespace

    OrbitPredictionService::PublishedChunk make_published_chunk(
            const PlannedChunkPacket &packet,
            const uint32_t chunk_id,
            const OrbitPredictionService::ChunkQualityState quality_state)
    {
        return OrbitPredictionService::PublishedChunk{
                .chunk_id = chunk_id,
                .quality_state = quality_state,
                .t0_s = packet.chunk.t0_s,
                .t1_s = packet.chunk.t1_s,
                .includes_planned_path = !packet.segments.empty(),
                .reused_from_cache = packet.reused_from_cache,
        };
    }

    void append_published_chunk(std::vector<OrbitPredictionService::PublishedChunk> &dst,
                                const PlannedChunkPacket &packet,
                                const uint32_t chunk_id,
                                const OrbitPredictionService::ChunkQualityState quality_state)
    {
        dst.push_back(make_published_chunk(packet, chunk_id, quality_state));
    }

    void append_published_chunks_as_final(std::vector<OrbitPredictionService::PublishedChunk> &dst,
                                          const std::vector<OrbitPredictionService::PublishedChunk> &src)
    {
        for (const OrbitPredictionService::PublishedChunk &chunk : src)
        {
            OrbitPredictionService::PublishedChunk final_chunk = chunk;
            final_chunk.chunk_id = static_cast<uint32_t>(dst.size());
            final_chunk.quality_state = OrbitPredictionService::ChunkQualityState::Final;
            dst.push_back(final_chunk);
        }
    }

    OrbitPredictionService::PublishedChunk make_published_chunk_from_plan(
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const uint32_t chunk_id,
            const bool reused_from_cache)
    {
        return OrbitPredictionService::PublishedChunk{
                .chunk_id = chunk_id,
                .quality_state = OrbitPredictionService::ChunkQualityState::Final,
                .t0_s = chunk.t0_s,
                .t1_s = chunk.t1_s,
                .includes_planned_path = true,
                .reused_from_cache = reused_from_cache,
        };
    }

    std::shared_ptr<const OrbitPredictionService::Result::CoreData> StagedCoreDataBuilder::get()
    {
        if (staged_core_data)
        {
            return staged_core_data;
        }

        auto core_data = std::make_shared<OrbitPredictionService::Result::CoreData>();
        core_data->shared_ephemeris = source.shared_ephemeris;
        core_data->massive_bodies = source.massive_bodies;
        core_data->trajectory_inertial = source.trajectory_inertial;
        core_data->trajectory_segments_inertial = source.trajectory_segments_inertial;
        staged_core_data = std::move(core_data);
        return staged_core_data;
    }

    OrbitPredictionService::Result make_planned_stage_result(
            const OrbitPredictionService::Result &base_result,
            const PlannedSolveOutput &stage_output,
            const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
            const OrbitPredictionService::PublishStage publish_stage,
            std::shared_ptr<const OrbitPredictionService::Result::CoreData> shared_core_data,
            std::vector<OrbitPredictionService::StreamedPlannedChunk> streamed_planned_chunks,
            const bool include_cumulative_planned)
    {
        OrbitPredictionService::Result stage_result{};
        stage_result.track_id = base_result.track_id;
        stage_result.generation_id = base_result.generation_id;
        stage_result.maneuver_plan_revision = base_result.maneuver_plan_revision;
        stage_result.maneuver_plan_signature_valid = base_result.maneuver_plan_signature_valid;
        stage_result.maneuver_plan_signature = base_result.maneuver_plan_signature;
        stage_result.valid = true;
        stage_result.baseline_reused = base_result.baseline_reused;
        stage_result.solve_quality = base_result.solve_quality;
        stage_result.publish_stage = publish_stage;
        stage_result.diagnostics = base_result.diagnostics;
        stage_result.build_time_s = base_result.build_time_s;
        stage_result.set_shared_core_data(std::move(shared_core_data));
        stage_result.diagnostics.trajectory_planned = stage_output.diagnostics;
        stage_result.diagnostics.trajectory_sample_count_planned = stage_output.samples.size();
        stage_result.diagnostics.status = OrbitPredictionService::Status::Success;
        sync_prediction_stage_counts(stage_result);
        if (include_cumulative_planned)
        {
            stage_result.trajectory_segments_inertial_planned = stage_output.segments;
            stage_result.trajectory_inertial_planned = stage_output.samples;
            stage_result.maneuver_previews = stage_output.previews;
        }
        stage_result.published_chunks = published_chunks;
        stage_result.streamed_planned_chunks = std::move(streamed_planned_chunks);
        return stage_result;
    }

    bool build_cached_prefix_stream_chunks(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionSolvePlan &solve_plan,
            const std::size_t suffix_begin_index,
            const PlannedSolveOutput &prefix_output,
            std::vector<OrbitPredictionService::PublishedChunk> &out_published_chunks,
            std::vector<OrbitPredictionService::StreamedPlannedChunk> &out_streamed_chunks)
    {
        out_published_chunks.clear();
        out_streamed_chunks.clear();
        if (suffix_begin_index == 0u || suffix_begin_index > solve_plan.chunks.size())
        {
            return true;
        }

        out_published_chunks.reserve(suffix_begin_index);
        out_streamed_chunks.reserve(suffix_begin_index);
        for (std::size_t chunk_index = 0u; chunk_index < suffix_begin_index; ++chunk_index)
        {
            const OrbitPredictionService::PredictionChunkPlan &chunk = solve_plan.chunks[chunk_index];
            std::vector<orbitsim::TrajectorySegment> chunk_segments =
                    slice_trajectory_segments(prefix_output.segments, chunk.t0_s, chunk.t1_s);
            if (chunk_segments.empty() || !validate_trajectory_segment_continuity(chunk_segments))
            {
                out_published_chunks.clear();
                out_streamed_chunks.clear();
                return false;
            }

            std::vector<orbitsim::TrajectorySample> chunk_samples =
                    resample_segments_uniform(chunk_segments,
                                              prediction_sample_budget(request, chunk_segments.size()));
            if (chunk_samples.size() < 2u)
            {
                out_published_chunks.clear();
                out_streamed_chunks.clear();
                return false;
            }

            OrbitPredictionService::PublishedChunk published_chunk =
                    make_published_chunk_from_plan(chunk, chunk.chunk_id, true);
            OrbitPredictionService::StreamedPlannedChunk streamed_chunk{};
            streamed_chunk.published_chunk = published_chunk;
            streamed_chunk.trajectory_segments_inertial = std::move(chunk_segments);
            streamed_chunk.trajectory_inertial = std::move(chunk_samples);
            streamed_chunk.maneuver_previews =
                    collect_prefix_chunk_previews(prefix_output.previews, chunk.t0_s, chunk.t1_s);
            streamed_chunk.diagnostics =
                    make_stage_diagnostics_from_segments(streamed_chunk.trajectory_segments_inertial,
                                                         chunk.t1_s - chunk.t0_s,
                                                         true);
            streamed_chunk.start_state = streamed_chunk.trajectory_segments_inertial.front().start;
            streamed_chunk.end_state = streamed_chunk.trajectory_segments_inertial.back().end;

            out_published_chunks.push_back(std::move(published_chunk));
            out_streamed_chunks.push_back(std::move(streamed_chunk));
        }

        return true;
    }

    void FullStreamBatchPublisher::append(const OrbitPredictionService::PublishedChunk &published_chunk,
                                          const PlannedChunkPacket &packet)
    {
        if (!active)
        {
            return;
        }

        pending_published_chunks.push_back(published_chunk);
        pending_streamed_chunks.push_back(make_streamed_planned_chunk(packet, published_chunk));
    }

    bool FullStreamBatchPublisher::flush(const PlannedSolveOutput &stage_output,
                                         const MakeStageResultFn &make_stage_result,
                                         const PublishFn &publish,
                                         const bool force_publish)
    {
        if (!active || pending_published_chunks.empty())
        {
            return true;
        }

        const auto now_tp = std::chrono::steady_clock::now();
        const double elapsed_since_last_publish_s =
                std::chrono::duration<double>(now_tp - last_publish_tp).count();
        if (!force_publish &&
            published_any_batch &&
            elapsed_since_last_publish_s < min_publish_interval_s)
        {
            return true;
        }

        if (!publish(make_stage_result(stage_output,
                                       pending_published_chunks,
                                       OrbitPredictionService::PublishStage::FullStreaming,
                                       std::move(pending_streamed_chunks),
                                       false)))
        {
            pending_published_chunks.clear();
            pending_streamed_chunks.clear();
            return false;
        }

        pending_published_chunks.clear();
        pending_streamed_chunks.clear();
        last_publish_tp = now_tp;
        published_any_batch = true;
        return true;
    }

    FullStreamBatchPublisher make_full_stream_batch_publisher(
            const OrbitPredictionService::Request &request,
            const std::chrono::steady_clock::time_point compute_start)
    {
        return FullStreamBatchPublisher{
                .active = request.solve_quality == OrbitPredictionService::SolveQuality::Full &&
                          request.full_stream_publish.active,
                .min_publish_interval_s =
                        (request.full_stream_publish.min_publish_interval_s > 0.0)
                                ? request.full_stream_publish.min_publish_interval_s
                                : OrbitPredictionTuning::kFullStreamPublishMinIntervalS,
                .last_publish_tp = compute_start,
        };
    }
} // namespace Game
