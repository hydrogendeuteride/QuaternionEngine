#include "game/states/gameplay/prediction/streamed_chunk_assembly_builder.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Game
{
    using namespace PredictionCacheInternal;

    namespace
    {
        bool build_prediction_streamed_planned_chunk(
                OrbitChunk &out_chunk,
                OrbitPredictionService::AdaptiveStageDiagnostics *out_stage_diagnostics,
                const PredictionSolverTrajectoryCache &solver,
                const PredictionDisplayFrameCache &display,
                const OrbitPredictionService::StreamedPlannedChunk &streamed_chunk,
                const uint64_t generation_id,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const StreamedChunkAssemblyBuilder::CancelCheck &cancel_requested,
                OrbitPredictionDerivedDiagnostics *diagnostics,
                const bool build_chunk_render_curve,
                const bool use_dense_chunk_samples)
        {
            out_chunk = {};
            if (out_stage_diagnostics)
            {
                *out_stage_diagnostics = {};
            }

            const OrbitPredictionService::PublishedChunk &published_chunk = streamed_chunk.published_chunk;
            if (!published_chunk.includes_planned_path ||
                streamed_chunk.trajectory_segments_inertial.empty() ||
                !std::isfinite(published_chunk.t0_s) ||
                !std::isfinite(published_chunk.t1_s) ||
                !(published_chunk.t1_s > published_chunk.t0_s))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::MissingSolverData);
                return false;
            }

            std::vector<orbitsim::TrajectorySegment> frame_segments{};
            std::vector<orbitsim::TrajectorySample> frame_samples{};
            const std::size_t sample_budget =
                    std::max<std::size_t>(streamed_chunk.trajectory_inertial.size(),
                                          std::max<std::size_t>(2u, streamed_chunk.trajectory_segments_inertial.size() + 1u));
            std::vector<double> node_times_s;
            node_times_s.reserve(streamed_chunk.maneuver_previews.size());
            for (const OrbitPredictionService::ManeuverNodePreview &preview : streamed_chunk.maneuver_previews)
            {
                if (std::isfinite(preview.t_s))
                {
                    node_times_s.push_back(preview.t_s);
                }
            }

            if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
            {
                frame_segments = streamed_chunk.trajectory_segments_inertial;
                if (!validate_trajectory_segment_continuity(frame_segments))
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                    return false;
                }

                frame_samples =
                        use_dense_chunk_samples && streamed_chunk.trajectory_inertial.size() >= 2
                                ? streamed_chunk.trajectory_inertial
                                : (use_dense_chunk_samples
                                           ? sample_prediction_segments(frame_segments, sample_budget, node_times_s)
                                           : collect_segment_boundary_samples(frame_segments));
                if (out_stage_diagnostics)
                {
                    *out_stage_diagnostics = make_stage_diagnostics_from_segments(
                            frame_segments,
                            prediction_segment_span_s(streamed_chunk.trajectory_segments_inertial));
                }
            }
            else
            {
                const auto &base_ephemeris = solver.resolved_shared_ephemeris();
                const auto &base_bodies = solver.resolved_massive_bodies();
                if (!base_ephemeris || base_ephemeris->empty())
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::MissingEphemeris);
                    return false;
                }

                const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
                const orbitsim::FrameSegmentTransformOptions frame_opt =
                        PredictionCacheInternal::build_frame_segment_transform_options(
                                resolved_frame_spec,
                                streamed_chunk.trajectory_segments_inertial,
                                cancel_requested);
                orbitsim::FrameSegmentTransformDiagnostics frame_diag{};
                frame_segments = orbitsim::transform_trajectory_segments_to_frame_spec(
                        streamed_chunk.trajectory_segments_inertial,
                        *base_ephemeris,
                        base_bodies,
                        resolved_frame_spec,
                        frame_opt,
                        player_lookup,
                        &frame_diag);
                if (cancel_requested && cancel_requested())
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Cancelled);
                    return false;
                }
                if (frame_segments.empty())
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::FrameTransformFailed);
                    return false;
                }
                if (!validate_trajectory_segment_continuity(frame_segments))
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                    return false;
                }

                frame_samples =
                        use_dense_chunk_samples
                                ? sample_prediction_segments(frame_segments, sample_budget, node_times_s)
                                : collect_segment_boundary_samples(frame_segments);
                if (out_stage_diagnostics)
                {
                    *out_stage_diagnostics = make_stage_diagnostics_from_adaptive(
                            frame_diag,
                            prediction_segment_span_s(streamed_chunk.trajectory_segments_inertial));
                    out_stage_diagnostics->accepted_segments = frame_segments.size();
                    out_stage_diagnostics->covered_duration_s = prediction_segment_span_s(frame_segments);
                    out_stage_diagnostics->frame_resegmentation_count = frame_diag.frame_resegmentation_count;
                }
            }

            if (frame_samples.size() < 2)
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::FrameSamplesUnavailable);
                return false;
            }

            out_chunk.chunk_id = published_chunk.chunk_id;
            out_chunk.generation_id = generation_id;
            out_chunk.quality_state = published_chunk.quality_state;
            out_chunk.t0_s = published_chunk.t0_s;
            out_chunk.t1_s = published_chunk.t1_s;
            out_chunk.frame_samples = std::move(frame_samples);
            out_chunk.frame_segments = std::move(frame_segments);
            if (build_chunk_render_curve && !out_chunk.frame_segments.empty())
            {
                out_chunk.render_curve = OrbitRenderCurve::build(out_chunk.frame_segments);
            }
            out_chunk.valid = !out_chunk.frame_segments.empty();
            return out_chunk.valid;
        }
    } // namespace

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
        out_assembly.clear();
        if (diagnostics)
        {
            *diagnostics = {};
        }

        out_assembly.chunks.reserve(streamed_chunks.size());
        OrbitPredictionService::AdaptiveStageDiagnostics planned_stage_diagnostics{};
        bool planned_stage_diagnostics_valid = false;
        std::size_t total_chunk_segment_count = 0;
        std::size_t total_chunk_sample_count = 0;

        for (const OrbitPredictionService::StreamedPlannedChunk &streamed_chunk : streamed_chunks)
        {
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (!streamed_chunk.published_chunk.includes_planned_path)
            {
                continue;
            }

            OrbitChunk chunk{};
            OrbitPredictionService::AdaptiveStageDiagnostics chunk_diagnostics{};
            if (!build_prediction_streamed_planned_chunk(chunk,
                                                         &chunk_diagnostics,
                                                         solver,
                                                         display,
                                                         streamed_chunk,
                                                         generation_id,
                                                         resolved_frame_spec,
                                                         player_lookup_segments_inertial,
                                                         cancel_requested,
                                                         diagnostics,
                                                         build_chunk_render_curves,
                                                         use_dense_chunk_samples))
            {
                return false;
            }

            total_chunk_segment_count += chunk.frame_segments.size();
            total_chunk_sample_count += chunk.frame_samples.size();
            if (!planned_stage_diagnostics_valid)
            {
                planned_stage_diagnostics = chunk_diagnostics;
                planned_stage_diagnostics_valid = true;
            }
            else
            {
                accumulate_stage_diagnostics(planned_stage_diagnostics, chunk_diagnostics);
            }
            out_assembly.chunks.push_back(std::move(chunk));
        }

        std::sort(out_assembly.chunks.begin(),
                  out_assembly.chunks.end(),
                  [](const OrbitChunk &a, const OrbitChunk &b) { return a.chunk_id < b.chunk_id; });
        out_assembly.generation_id = generation_id;
        out_assembly.valid = !out_assembly.chunks.empty();
        if (diagnostics)
        {
            diagnostics->status =
                    out_assembly.valid ? PredictionDerivedStatus::Success : PredictionDerivedStatus::MissingSolverData;
            diagnostics->frame_segment_count = display.trajectory_segments_frame.size();
            diagnostics->frame_segment_count_planned = total_chunk_segment_count;
            diagnostics->frame_sample_count = display.trajectory_frame.size();
            diagnostics->frame_sample_count_planned = total_chunk_sample_count;
            if (planned_stage_diagnostics_valid)
            {
                diagnostics->frame_planned = planned_stage_diagnostics;
                diagnostics->frame_planned.accepted_segments = total_chunk_segment_count;
            }
        }
        return out_assembly.valid;
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
        return rebuild_from_streamed(out_assembly,
                                     cache.solver,
                                     cache.display,
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
            const uint64_t /*display_frame_key*/,
            const uint64_t /*display_frame_revision*/,
            const CancelCheck &cancel_requested,
            const std::vector<double> &node_times_s,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_chunk_render_curves,
            const bool use_dense_chunk_samples)
    {
        out_assembly.clear();
        if (diagnostics)
        {
            *diagnostics = {};
        }
        (void) resolved_frame_spec;

        out_assembly.chunks.reserve(published_chunks.size());
        std::size_t segment_cursor = 0;
        double previous_chunk_t0_s = -std::numeric_limits<double>::infinity();
        std::size_t total_chunk_segment_count = 0;
        std::size_t total_chunk_sample_count = 0;
        double total_chunk_duration_s = 0.0;

        for (const OrbitPredictionService::PublishedChunk &published_chunk : published_chunks)
        {
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Cancelled);
                return false;
            }

            if (!published_chunk.includes_planned_path ||
                !std::isfinite(published_chunk.t0_s) ||
                !std::isfinite(published_chunk.t1_s) ||
                !(published_chunk.t1_s > published_chunk.t0_s))
            {
                continue;
            }

            const double chunk_order_epsilon_s = continuity_time_epsilon_s(published_chunk.t0_s);
            if (published_chunk.t0_s < (previous_chunk_t0_s - chunk_order_epsilon_s))
            {
                segment_cursor = 0;
            }
            previous_chunk_t0_s = published_chunk.t0_s;

            std::vector<orbitsim::TrajectorySegment> clipped_segments{};
            if (!slice_trajectory_segments_from_cursor(display.trajectory_segments_frame_planned,
                                                       published_chunk.t0_s,
                                                       published_chunk.t1_s,
                                                       segment_cursor,
                                                       clipped_segments))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (clipped_segments.empty() || !validate_trajectory_segment_continuity(clipped_segments))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }

            std::vector<orbitsim::TrajectorySample> clipped_samples =
                    use_dense_chunk_samples
                            ? sample_prediction_segments(clipped_segments,
                                                         std::max<std::size_t>(2u, clipped_segments.size() + 1u),
                                                         node_times_s)
                            : collect_segment_boundary_samples(clipped_segments);

            OrbitChunk chunk{};
            chunk.chunk_id = published_chunk.chunk_id;
            chunk.generation_id = generation_id;
            chunk.quality_state = published_chunk.quality_state;
            chunk.t0_s = published_chunk.t0_s;
            chunk.t1_s = published_chunk.t1_s;
            chunk.frame_samples = std::move(clipped_samples);
            chunk.frame_segments = std::move(clipped_segments);
            if (build_chunk_render_curves && !chunk.frame_segments.empty())
            {
                chunk.render_curve = OrbitRenderCurve::build(chunk.frame_segments);
            }
            else
            {
                chunk.render_curve.clear();
            }
            chunk.valid = !chunk.frame_segments.empty();
            total_chunk_segment_count += chunk.frame_segments.size();
            total_chunk_sample_count += chunk.frame_samples.size();
            total_chunk_duration_s += prediction_segment_span_s(chunk.frame_segments);
            out_assembly.chunks.push_back(std::move(chunk));
        }

        out_assembly.generation_id = generation_id;
        out_assembly.valid = !out_assembly.chunks.empty();
        if (diagnostics && out_assembly.valid)
        {
            diagnostics->status = PredictionDerivedStatus::Success;
            diagnostics->frame_segment_count = display.trajectory_segments_frame.size();
            diagnostics->frame_segment_count_planned = total_chunk_segment_count;
            diagnostics->frame_sample_count = display.trajectory_frame.size();
            diagnostics->frame_sample_count_planned = total_chunk_sample_count;
            diagnostics->frame_planned.accepted_segments = total_chunk_segment_count;
            diagnostics->frame_planned.covered_duration_s = total_chunk_duration_s;
        }
        return out_assembly.valid;
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
        return rebuild_from_published(out_assembly,
                                      cache.display,
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
        display.clear_planned();

        if (!assembly.valid)
        {
            return;
        }

        std::size_t total_segment_count = 0;
        std::size_t total_sample_count = 0;
        for (const OrbitChunk &chunk : assembly.chunks)
        {
            total_segment_count += chunk.frame_segments.size();
            total_sample_count += chunk.frame_samples.size();
        }
        display.trajectory_segments_frame_planned.reserve(total_segment_count);
        display.trajectory_frame_planned.reserve(total_sample_count);

        for (const OrbitChunk &chunk : assembly.chunks)
        {
            display.trajectory_segments_frame_planned.insert(display.trajectory_segments_frame_planned.end(),
                                                           chunk.frame_segments.begin(),
                                                           chunk.frame_segments.end());

            for (const orbitsim::TrajectorySample &sample : chunk.frame_samples)
            {
                append_unique_prediction_sample(display.trajectory_frame_planned, sample);
            }
        }

        if (build_render_curve && !display.trajectory_segments_frame_planned.empty())
        {
            display.render_curve_frame_planned = OrbitRenderCurve::build(display.trajectory_segments_frame_planned);
        }
    }

    void StreamedChunkAssemblyBuilder::flatten(OrbitPredictionCache &cache,
                                               const PredictionChunkAssembly &assembly,
                                               const bool build_render_curve)
    {
        flatten(cache.display, assembly, build_render_curve);
    }
} // namespace Game
