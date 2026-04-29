#pragma once

#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace Game
{
    struct StreamedChunkAssemblyBuilder
    {
        using CancelCheck = std::function<bool()>;

        static bool rebuild_from_streamed(
                PredictionChunkAssembly &out_assembly,
                const PredictionSolverTrajectoryCache &solver,
                const PredictionDisplayFrameCache &display,
                const std::vector<OrbitPredictionService::StreamedPlannedChunk> &streamed_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static bool rebuild_from_streamed(
                PredictionChunkAssembly &out_assembly,
                const OrbitPredictionCache &cache,
                const std::vector<OrbitPredictionService::StreamedPlannedChunk> &streamed_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static bool rebuild_from_published(
                PredictionChunkAssembly &out_assembly,
                const PredictionDisplayFrameCache &display,
                const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                uint64_t display_frame_key,
                uint64_t display_frame_revision,
                const CancelCheck &cancel_requested = {},
                const std::vector<double> &node_times_s = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static bool rebuild_from_published(
                PredictionChunkAssembly &out_assembly,
                const OrbitPredictionCache &cache,
                const std::vector<OrbitPredictionService::PublishedChunk> &published_chunks,
                uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                uint64_t display_frame_key,
                uint64_t display_frame_revision,
                const CancelCheck &cancel_requested = {},
                const std::vector<double> &node_times_s = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_chunk_render_curves = false,
                bool use_dense_chunk_samples = true);

        static void flatten(PredictionDisplayFrameCache &display,
                            const PredictionChunkAssembly &assembly,
                            bool build_render_curve = true);

        static void flatten(OrbitPredictionCache &cache,
                            const PredictionChunkAssembly &assembly,
                            bool build_render_curve = true);
    };
} // namespace Game
