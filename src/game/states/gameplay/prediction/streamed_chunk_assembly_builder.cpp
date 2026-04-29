#include "game/states/gameplay/prediction/streamed_chunk_assembly_builder.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

namespace Game
{
    bool StreamedChunkAssemblyBuilder::rebuild_from_streamed(
            PredictionChunkAssembly &out_assembly,
            const PredictionSolverTrajectoryCache &solver,
            const PredictionDisplayFrameCache &display,
            const std::vector<OrbitPredictionService::StreamedPlannedChunk> &streamed_chunks,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_chunk_render_curves,
            const bool use_dense_chunk_samples)
    {
        return PredictionCacheInternal::rebuild_prediction_streamed_chunk_assembly(out_assembly,
                                                                                   solver,
                                                                                   display,
                                                                                   streamed_chunks,
                                                                                   generation_id,
                                                                                   resolved_frame_spec,
                                                                                   player_lookup_segments_inertial,
                                                                                   cancel_requested,
                                                                                   diagnostics,
                                                                                   build_chunk_render_curves,
                                                                                   use_dense_chunk_samples);
    }

    bool StreamedChunkAssemblyBuilder::rebuild_from_streamed(
            PredictionChunkAssembly &out_assembly,
            const OrbitPredictionCache &cache,
            const std::vector<OrbitPredictionService::StreamedPlannedChunk> &streamed_chunks,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_chunk_render_curves,
            const bool use_dense_chunk_samples)
    {
        return PredictionCacheInternal::rebuild_prediction_streamed_chunk_assembly(out_assembly,
                                                                                   cache,
                                                                                   streamed_chunks,
                                                                                   generation_id,
                                                                                   resolved_frame_spec,
                                                                                   player_lookup_segments_inertial,
                                                                                   cancel_requested,
                                                                                   diagnostics,
                                                                                   build_chunk_render_curves,
                                                                                   use_dense_chunk_samples);
    }

    bool StreamedChunkAssemblyBuilder::rebuild_from_published(
            PredictionChunkAssembly &out_assembly,
            const PredictionDisplayFrameCache &display,
            const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const uint64_t display_frame_key,
            const uint64_t display_frame_revision,
            const CancelCheck &cancel_requested,
            const std::vector<double> &node_times_s,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_chunk_render_curves,
            const bool use_dense_chunk_samples)
    {
        return PredictionCacheInternal::rebuild_prediction_patch_chunks(out_assembly,
                                                                        display,
                                                                        published_chunks,
                                                                        generation_id,
                                                                        resolved_frame_spec,
                                                                        display_frame_key,
                                                                        display_frame_revision,
                                                                        cancel_requested,
                                                                        node_times_s,
                                                                        diagnostics,
                                                                        build_chunk_render_curves,
                                                                        use_dense_chunk_samples);
    }

    bool StreamedChunkAssemblyBuilder::rebuild_from_published(
            PredictionChunkAssembly &out_assembly,
            const OrbitPredictionCache &cache,
            const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const uint64_t display_frame_key,
            const uint64_t display_frame_revision,
            const CancelCheck &cancel_requested,
            const std::vector<double> &node_times_s,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_chunk_render_curves,
            const bool use_dense_chunk_samples)
    {
        return PredictionCacheInternal::rebuild_prediction_patch_chunks(out_assembly,
                                                                        cache,
                                                                        published_chunks,
                                                                        generation_id,
                                                                        resolved_frame_spec,
                                                                        display_frame_key,
                                                                        display_frame_revision,
                                                                        cancel_requested,
                                                                        node_times_s,
                                                                        diagnostics,
                                                                        build_chunk_render_curves,
                                                                        use_dense_chunk_samples);
    }

    void StreamedChunkAssemblyBuilder::flatten(PredictionDisplayFrameCache &display,
                                               const PredictionChunkAssembly &assembly,
                                               const bool build_render_curve)
    {
        PredictionCacheInternal::flatten_chunk_assembly_to_cache(display, assembly, build_render_curve);
    }

    void StreamedChunkAssemblyBuilder::flatten(OrbitPredictionCache &cache,
                                               const PredictionChunkAssembly &assembly,
                                               const bool build_render_curve)
    {
        PredictionCacheInternal::flatten_chunk_assembly_to_cache(cache, assembly, build_render_curve);
    }
} // namespace Game
