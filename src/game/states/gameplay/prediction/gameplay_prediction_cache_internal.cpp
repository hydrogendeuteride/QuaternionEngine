#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

namespace Game::PredictionCacheInternal
{
    orbitsim::FrameSegmentTransformOptions build_frame_segment_transform_options(
            const orbitsim::TrajectoryFrameSpec &frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &inertial_segments,
            const CancelCheck &cancel_requested)
    {
        orbitsim::FrameSegmentTransformOptions out{};
        const double start_time_s = inertial_segments.empty() ? 0.0 : inertial_segments.front().t0_s;
        const double end_time_s =
                inertial_segments.empty() ? 0.0 : prediction_segment_end_time(inertial_segments.back());
        const double horizon_s = std::max(0.0, end_time_s - start_time_s);
        const bool sensitive = frame_spec.type == orbitsim::TrajectoryFrameType::Synodic ||
                               frame_spec.type == orbitsim::TrajectoryFrameType::LVLH;

        out.min_dt_s = OrbitPredictionTuning::kAdaptiveFrameTransformMinDtS;
        out.max_dt_s = sensitive
                               ? std::min(OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtSensitiveS,
                                          OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtS)
                               : OrbitPredictionTuning::kAdaptiveFrameTransformMaxDtS;
        out.soft_max_segments =
                std::max<std::size_t>(OrbitPredictionTuning::kAdaptiveFrameTransformSoftMaxSegments,
                                      inertial_segments.size());
        out.hard_max_segments =
                std::max<std::size_t>(OrbitPredictionTuning::kAdaptiveFrameTransformHardMaxSegments,
                                      out.soft_max_segments);
        out.tolerance.pos_near_m = OrbitPredictionTuning::kAdaptiveFrameTransformPosTolNearM;
        out.tolerance.pos_far_m = OrbitPredictionTuning::kAdaptiveFrameTransformPosTolFarM;
        out.tolerance.vel_near_mps = OrbitPredictionTuning::kAdaptiveFrameTransformVelTolNearMps;
        out.tolerance.vel_far_mps = OrbitPredictionTuning::kAdaptiveFrameTransformVelTolFarMps;
        out.tolerance.rel_pos_floor = OrbitPredictionTuning::kAdaptiveFrameTransformRelPosFloor;
        out.tolerance.rel_vel_floor = OrbitPredictionTuning::kAdaptiveFrameTransformRelVelFloor;
        if (sensitive)
        {
            out.tolerance.pos_near_m *= 0.5;
            out.tolerance.pos_far_m *= 0.25;
            out.tolerance.vel_near_mps *= 0.5;
            out.tolerance.vel_far_mps *= 0.25;
        }
        out.cancel_requested = cancel_requested;
        return out;
    }

    bool rebuild_prediction_frame_cache(
            const PredictionSolverTrajectoryCache &solver,
            PredictionDisplayFrameCache &display,
            PredictionAnalysisCache &analysis,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        if (diagnostics)
        {
            *diagnostics = {};
        }

        display.trajectory_frame.clear();
        display.trajectory_frame_planned.clear();
        display.trajectory_segments_frame.clear();
        display.trajectory_segments_frame_planned.clear();
        display.render_curve_frame.clear();
        display.render_curve_frame_planned.clear();
        display.resolved_frame_spec = {};
        display.resolved_frame_spec_valid = false;
        analysis.trajectory_analysis_bci.clear();
        analysis.trajectory_segments_analysis_bci.clear();
        analysis.analysis_cache_body_id = orbitsim::kInvalidBodyId;
        analysis.analysis_cache_valid = false;
        analysis.metrics_valid = false;

        const auto &base_ephemeris = solver.resolved_shared_ephemeris();
        const auto &base_bodies = solver.resolved_massive_bodies();
        const auto &base_samples = solver.resolved_trajectory_inertial();
        const auto &base_segments = solver.resolved_trajectory_segments_inertial();

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            display.trajectory_frame = base_samples;
            display.trajectory_frame_planned = solver.trajectory_inertial_planned;
            display.trajectory_segments_frame = base_segments;
            display.trajectory_segments_frame_planned = solver.trajectory_segments_inertial_planned;
            if (!validate_trajectory_segment_continuity(display.trajectory_segments_frame))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (!display.trajectory_segments_frame_planned.empty() &&
                !validate_trajectory_segment_continuity(display.trajectory_segments_frame_planned))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (diagnostics)
            {
                diagnostics->frame_base = make_stage_diagnostics_from_segments(
                        display.trajectory_segments_frame,
                        prediction_segment_span_s(base_segments));
                diagnostics->frame_planned = make_stage_diagnostics_from_segments(
                        display.trajectory_segments_frame_planned,
                        prediction_segment_span_s(solver.trajectory_segments_inertial_planned));
            }
        }
        else
        {
            if (!base_ephemeris || base_ephemeris->empty())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::MissingEphemeris);
                return false;
            }

            const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
            const std::size_t base_sample_budget = std::max<std::size_t>(base_samples.size(), 2);
            const std::size_t planned_sample_budget =
                    solver.trajectory_inertial_planned.size() >= 2 ? solver.trajectory_inertial_planned.size()
                                                                   : base_sample_budget;
            const std::vector<double> node_times = collect_maneuver_node_times(solver);

            const orbitsim::FrameSegmentTransformOptions base_opt =
                    build_frame_segment_transform_options(
                            resolved_frame_spec,
                            base_segments,
                            cancel_requested);
            orbitsim::FrameSegmentTransformDiagnostics base_frame_diag{};
            display.trajectory_segments_frame = orbitsim::transform_trajectory_segments_to_frame_spec(
                    base_segments,
                    *base_ephemeris,
                    base_bodies,
                    resolved_frame_spec,
                    base_opt,
                    player_lookup,
                    &base_frame_diag);
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (display.trajectory_segments_frame.empty())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::FrameTransformFailed);
                return false;
            }
            if (!validate_trajectory_segment_continuity(display.trajectory_segments_frame))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }
            if (diagnostics)
            {
                diagnostics->frame_base = make_stage_diagnostics_from_adaptive(
                        base_frame_diag,
                        prediction_segment_span_s(base_segments));
                diagnostics->frame_base.accepted_segments = display.trajectory_segments_frame.size();
                diagnostics->frame_base.covered_duration_s = prediction_segment_span_s(display.trajectory_segments_frame);
                diagnostics->frame_base.frame_resegmentation_count = base_frame_diag.frame_resegmentation_count;
            }
            display.trajectory_frame = sample_prediction_segments(display.trajectory_segments_frame, base_sample_budget);
            if (display.trajectory_frame.size() < 2)
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::FrameSamplesUnavailable);
                return false;
            }

            if (!solver.trajectory_segments_inertial_planned.empty())
            {
                const orbitsim::FrameSegmentTransformOptions planned_opt =
                        build_frame_segment_transform_options(
                                resolved_frame_spec,
                                solver.trajectory_segments_inertial_planned,
                                cancel_requested);
                orbitsim::FrameSegmentTransformDiagnostics planned_frame_diag{};
                display.trajectory_segments_frame_planned = orbitsim::transform_trajectory_segments_to_frame_spec(
                        solver.trajectory_segments_inertial_planned,
                        *base_ephemeris,
                        base_bodies,
                        resolved_frame_spec,
                        planned_opt,
                        player_lookup,
                        &planned_frame_diag);
                if (cancel_requested && cancel_requested())
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Cancelled);
                    return false;
                }
                if (!display.trajectory_segments_frame_planned.empty())
                {
                    if (!validate_trajectory_segment_continuity(display.trajectory_segments_frame_planned))
                    {
                        update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                        return false;
                    }
                    if (diagnostics)
                    {
                        diagnostics->frame_planned = make_stage_diagnostics_from_adaptive(
                                planned_frame_diag,
                                prediction_segment_span_s(solver.trajectory_segments_inertial_planned));
                        diagnostics->frame_planned.accepted_segments = display.trajectory_segments_frame_planned.size();
                        diagnostics->frame_planned.covered_duration_s =
                                prediction_segment_span_s(display.trajectory_segments_frame_planned);
                        diagnostics->frame_planned.frame_resegmentation_count =
                                planned_frame_diag.frame_resegmentation_count;
                    }
                    display.trajectory_frame_planned = sample_prediction_segments(
                            display.trajectory_segments_frame_planned,
                            planned_sample_budget,
                            node_times);
                }
            }
        }

        if (display.trajectory_frame.size() < 2 || display.trajectory_segments_frame.empty())
        {
            update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::MissingSolverData);
            return false;
        }

        display.render_curve_frame = OrbitRenderCurve::build(display.trajectory_segments_frame);
        display.render_curve_frame_planned =
                (build_planned_render_curve && !display.trajectory_segments_frame_planned.empty())
                        ? OrbitRenderCurve::build(display.trajectory_segments_frame_planned)
                        : OrbitRenderCurve{};
        display.resolved_frame_spec = resolved_frame_spec;
        display.resolved_frame_spec_valid = true;
        update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Success);
        return true;
    }

    bool rebuild_prediction_frame_cache(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        return rebuild_prediction_frame_cache(cache.solver,
                                              cache.display,
                                              cache.analysis,
                                              resolved_frame_spec,
                                              player_lookup_segments_inertial,
                                              cancel_requested,
                                              diagnostics,
                                              build_planned_render_curve);
    }

    bool rebuild_prediction_planned_frame_cache(
            const PredictionSolverTrajectoryCache &solver,
            PredictionDisplayFrameCache &display,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        if (diagnostics)
        {
            *diagnostics = {};
        }

        display.clear_planned();
        display.resolved_frame_spec = {};
        display.resolved_frame_spec_valid = false;

        if (solver.trajectory_segments_inertial_planned.empty())
        {
            display.resolved_frame_spec = resolved_frame_spec;
            display.resolved_frame_spec_valid = true;
            update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Success);
            return true;
        }

        const auto &base_ephemeris = solver.resolved_shared_ephemeris();
        const auto &base_bodies = solver.resolved_massive_bodies();
        const auto &base_samples = solver.resolved_trajectory_inertial();
        const std::size_t base_sample_budget = std::max<std::size_t>(base_samples.size(), 2);
        const std::size_t planned_sample_budget =
                solver.trajectory_inertial_planned.size() >= 2 ? solver.trajectory_inertial_planned.size()
                                                               : base_sample_budget;
        const std::vector<double> node_times = collect_maneuver_node_times(solver);

        if (resolved_frame_spec.type == orbitsim::TrajectoryFrameType::Inertial)
        {
            display.trajectory_segments_frame_planned = solver.trajectory_segments_inertial_planned;
            if (!validate_trajectory_segment_continuity(display.trajectory_segments_frame_planned))
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                return false;
            }

            display.trajectory_frame_planned =
                    solver.trajectory_inertial_planned.size() >= 2
                            ? solver.trajectory_inertial_planned
                            : sample_prediction_segments(display.trajectory_segments_frame_planned,
                                                          planned_sample_budget,
                                                          node_times);
            if (diagnostics)
            {
                diagnostics->frame_planned = make_stage_diagnostics_from_segments(
                        display.trajectory_segments_frame_planned,
                        prediction_segment_span_s(solver.trajectory_segments_inertial_planned));
            }
        }
        else
        {
            if (!base_ephemeris || base_ephemeris->empty())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::MissingEphemeris);
                return false;
            }

            const auto player_lookup = build_player_lookup(player_lookup_segments_inertial);
            const orbitsim::FrameSegmentTransformOptions planned_opt =
                    build_frame_segment_transform_options(
                            resolved_frame_spec,
                            solver.trajectory_segments_inertial_planned,
                            cancel_requested);
            orbitsim::FrameSegmentTransformDiagnostics planned_frame_diag{};
            display.trajectory_segments_frame_planned = orbitsim::transform_trajectory_segments_to_frame_spec(
                    solver.trajectory_segments_inertial_planned,
                    *base_ephemeris,
                    base_bodies,
                    resolved_frame_spec,
                    planned_opt,
                    player_lookup,
                    &planned_frame_diag);
            if (cancel_requested && cancel_requested())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Cancelled);
                return false;
            }
            if (display.trajectory_segments_frame_planned.empty())
            {
                update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::FrameTransformFailed);
                return false;
            }

            if (!display.trajectory_segments_frame_planned.empty())
            {
                if (!validate_trajectory_segment_continuity(display.trajectory_segments_frame_planned))
                {
                    update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::ContinuityFailed);
                    return false;
                }
                if (diagnostics)
                {
                    diagnostics->frame_planned = make_stage_diagnostics_from_adaptive(
                            planned_frame_diag,
                            prediction_segment_span_s(solver.trajectory_segments_inertial_planned));
                    diagnostics->frame_planned.accepted_segments = display.trajectory_segments_frame_planned.size();
                    diagnostics->frame_planned.covered_duration_s =
                            prediction_segment_span_s(display.trajectory_segments_frame_planned);
                    diagnostics->frame_planned.frame_resegmentation_count = planned_frame_diag.frame_resegmentation_count;
                }
                display.trajectory_frame_planned = sample_prediction_segments(
                        display.trajectory_segments_frame_planned,
                        planned_sample_budget,
                        node_times);
            }
        }

        display.render_curve_frame_planned =
                (build_planned_render_curve && !display.trajectory_segments_frame_planned.empty())
                        ? OrbitRenderCurve::build(display.trajectory_segments_frame_planned)
                        : OrbitRenderCurve{};
        display.resolved_frame_spec = resolved_frame_spec;
        display.resolved_frame_spec_valid = true;
        update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Success);
        return true;
    }

    bool rebuild_prediction_planned_frame_cache(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        return rebuild_prediction_planned_frame_cache(cache.solver,
                                                      cache.display,
                                                      resolved_frame_spec,
                                                      player_lookup_segments_inertial,
                                                      cancel_requested,
                                                      diagnostics,
                                                      build_planned_render_curve);
    }

    bool build_prediction_streamed_planned_chunk(
            OrbitChunk &out_chunk,
            OrbitPredictionService::AdaptiveStageDiagnostics *out_stage_diagnostics,
            const PredictionSolverTrajectoryCache &solver,
            const PredictionDisplayFrameCache &display,
            const OrbitPredictionService::StreamedPlannedChunk &streamed_chunk,
            const uint64_t generation_id,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
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
                    build_frame_segment_transform_options(
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

    bool rebuild_prediction_streamed_chunk_assembly(
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

    void clear_prediction_metrics(OrbitPredictionCache &cache, const orbitsim::BodyId analysis_body_id)
    {
        cache.analysis.clear_metrics(analysis_body_id);
    }

    struct PredictionRadialExtrema
    {
        bool periapsis_valid{false};
        bool apoapsis_valid{false};
        double periapsis_radius_m{std::numeric_limits<double>::infinity()};
        double apoapsis_radius_m{0.0};
    };

    enum class PredictionRadialExtremumKind : uint8_t
    {
        None = 0,
        Periapsis,
        Apoapsis,
    };

    bool evaluate_radial_motion(const orbitsim::TrajectorySegment &segment,
                                       const double t_s,
                                       double &out_r_dot_v,
                                       double *out_radius_m = nullptr)
    {
        orbitsim::State state{};
        if (!eval_segment_state(segment, t_s, state))
        {
            return false;
        }

        const double radius_m = glm::length(state.position_m);
        const double r_dot_v = glm::dot(state.position_m, state.velocity_mps);
        if (!std::isfinite(radius_m) || !std::isfinite(r_dot_v))
        {
            return false;
        }

        out_r_dot_v = r_dot_v;
        if (out_radius_m)
        {
            *out_radius_m = radius_m;
        }
        return true;
    }

    bool sample_prediction_radius_m(const std::vector<orbitsim::TrajectorySegment> &segments,
                                           const double t_s,
                                           double &out_radius_m)
    {
        orbitsim::State state{};
        if (!sample_trajectory_segment_state(segments,
                                             t_s,
                                             state,
                                             TrajectoryBoundarySide::ContinuousPositionOnly))
        {
            return false;
        }

        const double radius_m = glm::length(state.position_m);
        if (!(radius_m > 0.0) || !std::isfinite(radius_m))
        {
            return false;
        }

        out_radius_m = radius_m;
        return true;
    }

    PredictionRadialExtremumKind classify_radial_extremum_by_radius(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const orbitsim::TrajectorySegment &reference_segment,
            const double t_s,
            const double radius_m)
    {
        if (segments.empty() || !(radius_m > 0.0) || !std::isfinite(radius_m))
        {
            return PredictionRadialExtremumKind::None;
        }

        const double trajectory_t0_s = segments.front().t0_s;
        const double trajectory_t1_s = prediction_segment_end_time(segments.back());
        if (!std::isfinite(trajectory_t0_s) || !std::isfinite(trajectory_t1_s) || !(trajectory_t1_s > trajectory_t0_s))
        {
            return PredictionRadialExtremumKind::None;
        }

        const double probe_dt_s = std::clamp(reference_segment.dt_s * 1.0e-3, 1.0e-6, 1.0);
        const double tolerance_m = std::max(1.0e-6, radius_m * 1.0e-9);

        double before_radius_m = 0.0;
        double after_radius_m = 0.0;
        const bool have_before =
                (t_s - probe_dt_s) >= trajectory_t0_s &&
                sample_prediction_radius_m(segments, t_s - probe_dt_s, before_radius_m);
        const bool have_after =
                (t_s + probe_dt_s) <= trajectory_t1_s &&
                sample_prediction_radius_m(segments, t_s + probe_dt_s, after_radius_m);

        if (!have_before && !have_after)
        {
            return PredictionRadialExtremumKind::None;
        }

        const bool lower_than_before = !have_before || radius_m <= (before_radius_m + tolerance_m);
        const bool lower_than_after = !have_after || radius_m <= (after_radius_m + tolerance_m);
        const bool strictly_lower =
                (have_before && radius_m < (before_radius_m - tolerance_m)) ||
                (have_after && radius_m < (after_radius_m - tolerance_m));
        if (lower_than_before && lower_than_after && strictly_lower)
        {
            return PredictionRadialExtremumKind::Periapsis;
        }

        const bool higher_than_before = !have_before || radius_m >= (before_radius_m - tolerance_m);
        const bool higher_than_after = !have_after || radius_m >= (after_radius_m - tolerance_m);
        const bool strictly_higher =
                (have_before && radius_m > (before_radius_m + tolerance_m)) ||
                (have_after && radius_m > (after_radius_m + tolerance_m));
        if (higher_than_before && higher_than_after && strictly_higher)
        {
            return PredictionRadialExtremumKind::Apoapsis;
        }

        return PredictionRadialExtremumKind::None;
    }

    int radial_motion_sign(const double r_dot_v)
    {
        if (!std::isfinite(r_dot_v))
        {
            return 0;
        }

        constexpr double kRelativeZero = 1.0e-12;
        const double tolerance = std::max(1.0e-12, std::abs(r_dot_v) * kRelativeZero);
        if (r_dot_v > tolerance)
        {
            return 1;
        }
        if (r_dot_v < -tolerance)
        {
            return -1;
        }
        return 0;
    }

    double refine_radial_extremum_time(const orbitsim::TrajectorySegment &segment,
                                              const double t0_s,
                                              const double t1_s,
                                              const double f0)
    {
        if (!(t1_s > t0_s) || !std::isfinite(t0_s) || !std::isfinite(t1_s))
        {
            return t0_s;
        }

        double a = t0_s;
        double b = t1_s;
        double fa = f0;
        const double time_tolerance_s = std::max(1.0e-9, (t1_s - t0_s) * 1.0e-12);

        for (int i = 0; i < 64 && (b - a) > time_tolerance_s; ++i)
        {
            const double m = 0.5 * (a + b);
            double fm = 0.0;
            if (!evaluate_radial_motion(segment, m, fm))
            {
                break;
            }

            const int sign_a = radial_motion_sign(fa);
            const int sign_m = radial_motion_sign(fm);
            if (sign_m == 0)
            {
                return m;
            }
            if (sign_a == 0 || sign_a != sign_m)
            {
                b = m;
            }
            else
            {
                a = m;
                fa = fm;
            }
        }

        return 0.5 * (a + b);
    }

    void record_radial_extremum(PredictionRadialExtrema &out,
                                       const std::vector<orbitsim::TrajectorySegment> &segments,
                                       const orbitsim::TrajectorySegment &segment,
                                       const double t_s,
                                       const int sign_before,
                                       const int sign_after,
                                       double &last_recorded_t_s)
    {
        if (!std::isfinite(t_s) || sign_before == sign_after)
        {
            return;
        }

        constexpr double kDuplicateRootToleranceS = 1.0e-6;
        if (std::isfinite(last_recorded_t_s) && std::abs(t_s - last_recorded_t_s) <= kDuplicateRootToleranceS)
        {
            return;
        }

        double r_dot_v = 0.0;
        double radius_m = 0.0;
        if (!evaluate_radial_motion(segment, t_s, r_dot_v, &radius_m) || !(radius_m > 0.0))
        {
            return;
        }

        const PredictionRadialExtremumKind kind =
                classify_radial_extremum_by_radius(segments, segment, t_s, radius_m);

        if (kind == PredictionRadialExtremumKind::Periapsis)
        {
            out.periapsis_radius_m = std::min(out.periapsis_radius_m, radius_m);
            out.periapsis_valid = true;
        }
        else if (kind == PredictionRadialExtremumKind::Apoapsis)
        {
            out.apoapsis_radius_m = std::max(out.apoapsis_radius_m, radius_m);
            out.apoapsis_valid = true;
        }
        else
        {
            return;
        }

        last_recorded_t_s = t_s;
    }

    PredictionRadialExtrema find_prediction_radial_extrema(
            const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        PredictionRadialExtrema out{};
        if (segments.empty())
        {
            return out;
        }

        constexpr int kRootScanSubstepsPerSegment = 8;
        double last_recorded_t_s = std::numeric_limits<double>::quiet_NaN();

        for (const orbitsim::TrajectorySegment &segment : segments)
        {
            const double seg_t0_s = segment.t0_s;
            const double seg_t1_s = prediction_segment_end_time(segment);
            if (!(segment.dt_s > 0.0) || !std::isfinite(seg_t0_s) || !std::isfinite(seg_t1_s) ||
                !(seg_t1_s > seg_t0_s))
            {
                continue;
            }

            double prev_t_s = seg_t0_s;
            double prev_f = 0.0;
            if (!evaluate_radial_motion(segment, prev_t_s, prev_f))
            {
                continue;
            }

            for (int i = 1; i <= kRootScanSubstepsPerSegment; ++i)
            {
                const double curr_t_s =
                        (i == kRootScanSubstepsPerSegment)
                                ? seg_t1_s
                                : (seg_t0_s + (segment.dt_s * static_cast<double>(i)) /
                                                    static_cast<double>(kRootScanSubstepsPerSegment));
                double curr_f = 0.0;
                if (!evaluate_radial_motion(segment, curr_t_s, curr_f))
                {
                    continue;
                }

                int prev_sign = radial_motion_sign(prev_f);
                int curr_sign = radial_motion_sign(curr_f);
                if (prev_sign == 0 && curr_sign != 0)
                {
                    record_radial_extremum(out, segments, segment, prev_t_s, -curr_sign, curr_sign, last_recorded_t_s);
                }
                else if (prev_sign != 0 && curr_sign == 0)
                {
                    record_radial_extremum(out, segments, segment, curr_t_s, prev_sign, -prev_sign, last_recorded_t_s);
                }
                else if (prev_sign != 0 && curr_sign != 0 && prev_sign != curr_sign)
                {
                    const double root_t_s = refine_radial_extremum_time(segment, prev_t_s, curr_t_s, prev_f);
                    record_radial_extremum(out, segments, segment, root_t_s, prev_sign, curr_sign, last_recorded_t_s);
                }

                prev_t_s = curr_t_s;
                prev_f = curr_f;
            }
        }

        return out;
    }

    void rebuild_prediction_metrics(
            const PredictionSolverTrajectoryCache &solver,
            const PredictionDisplayFrameCache &display,
            PredictionAnalysisCache &analysis,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested)
    {
        analysis.clear_metrics(analysis_body_id);

        const auto &base_ephemeris = solver.resolved_shared_ephemeris();
        const auto &base_bodies = solver.resolved_massive_bodies();
        const auto &base_samples = solver.resolved_trajectory_inertial();
        const auto &base_segments = solver.resolved_trajectory_segments_inertial();
        const auto analysis_it = std::find_if(base_bodies.begin(),
                                              base_bodies.end(),
                                              [analysis_body_id](const orbitsim::MassiveBody &body) {
                                                  return body.id == analysis_body_id;
                                              });
        if (analysis_it == base_bodies.end() || !(analysis_it->mass_kg > 0.0))
        {
            return;
        }

        const double mu_ref_m3_s2 = sim_config.gravitational_constant * analysis_it->mass_kg;
        if (!(mu_ref_m3_s2 > 0.0) || !std::isfinite(mu_ref_m3_s2))
        {
            return;
        }

        std::vector<orbitsim::TrajectorySample> rel_samples;
        if (display.resolved_frame_spec_valid &&
            display.resolved_frame_spec.type == orbitsim::TrajectoryFrameType::BodyCenteredInertial &&
            display.resolved_frame_spec.primary_body_id == analysis_body_id)
        {
            rel_samples = display.trajectory_frame;
            analysis.trajectory_segments_analysis_bci = display.trajectory_segments_frame;
            analysis.trajectory_analysis_bci = rel_samples;
            analysis.analysis_cache_body_id = analysis_body_id;
            analysis.analysis_cache_valid = true;
        }
        else if (analysis.analysis_cache_valid && analysis.analysis_cache_body_id == analysis_body_id)
        {
            rel_samples = analysis.trajectory_analysis_bci;
        }
        else if (base_ephemeris && !base_ephemeris->empty())
        {
            const std::size_t sample_budget = std::max<std::size_t>(base_samples.size(), 2);
            const orbitsim::TrajectoryFrameSpec analysis_frame =
                    orbitsim::TrajectoryFrameSpec::body_centered_inertial(analysis_body_id);
            const orbitsim::FrameSegmentTransformOptions frame_opt =
                    build_frame_segment_transform_options(
                            analysis_frame,
                            base_segments,
                            cancel_requested);
            std::vector<orbitsim::TrajectorySegment> rel_segments =
                    orbitsim::transform_trajectory_segments_to_frame_spec(
                            base_segments,
                            *base_ephemeris,
                            base_bodies,
                            analysis_frame,
                            frame_opt);
            if (cancel_requested && cancel_requested())
            {
                return;
            }
            rel_samples = sample_prediction_segments(rel_segments, sample_budget);
            analysis.trajectory_segments_analysis_bci = std::move(rel_segments);
            analysis.trajectory_analysis_bci = rel_samples;
            analysis.analysis_cache_body_id = analysis_body_id;
            analysis.analysis_cache_valid = !analysis.trajectory_analysis_bci.empty();
        }

        if (rel_samples.size() < 2)
        {
            return;
        }

        analysis.altitude_km.reserve(rel_samples.size());
        analysis.speed_kmps.reserve(rel_samples.size());
        for (const orbitsim::TrajectorySample &sample : rel_samples)
        {
            const double r_m = glm::length(sample.position_m);
            const double alt_km = (r_m - analysis_it->radius_m) * 1.0e-3;
            const double spd_kmps = glm::length(sample.velocity_mps) * 1.0e-3;
            analysis.altitude_km.push_back(static_cast<float>(alt_km));
            analysis.speed_kmps.push_back(static_cast<float>(spd_kmps));
        }

        const OrbitPredictionMath::OrbitalElementsEstimate elements =
                OrbitPredictionMath::compute_orbital_elements(mu_ref_m3_s2,
                                                              rel_samples.front().position_m,
                                                              rel_samples.front().velocity_mps);
        if (!elements.valid)
        {
            return;
        }

        analysis.semi_major_axis_m = elements.semi_major_axis_m;
        analysis.eccentricity = elements.eccentricity;
        analysis.orbital_period_s = elements.orbital_period_s;
        analysis.periapsis_alt_km = (elements.periapsis_m - analysis_it->radius_m) * 1.0e-3;
        analysis.apoapsis_alt_km = std::isfinite(elements.apoapsis_m)
                                           ? (elements.apoapsis_m - analysis_it->radius_m) * 1.0e-3
                                           : std::numeric_limits<double>::infinity();

        const PredictionRadialExtrema radial_extrema =
                find_prediction_radial_extrema(analysis.trajectory_segments_analysis_bci);
        if (radial_extrema.periapsis_valid)
        {
            analysis.periapsis_alt_km = (radial_extrema.periapsis_radius_m - analysis_it->radius_m) * 1.0e-3;
        }
        if (radial_extrema.apoapsis_valid)
        {
            analysis.apoapsis_alt_km = (radial_extrema.apoapsis_radius_m - analysis_it->radius_m) * 1.0e-3;
        }
    }

    void rebuild_prediction_metrics(
            OrbitPredictionCache &cache,
            const orbitsim::GameSimulation::Config &sim_config,
            const orbitsim::BodyId analysis_body_id,
            const CancelCheck &cancel_requested)
    {
        rebuild_prediction_metrics(cache.solver,
                                   cache.display,
                                   cache.analysis,
                                   sim_config,
                                   analysis_body_id,
                                   cancel_requested);
    }

    bool rebuild_prediction_patch_chunks(
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

    bool rebuild_prediction_streamed_chunk_assembly(
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
        return rebuild_prediction_streamed_chunk_assembly(out_assembly,
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

    bool rebuild_prediction_patch_chunks(
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
        return rebuild_prediction_patch_chunks(out_assembly,
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

    void flatten_chunk_assembly_to_cache(PredictionDisplayFrameCache &display,
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

    void flatten_chunk_assembly_to_cache(OrbitPredictionCache &cache,
                                         const PredictionChunkAssembly &assembly,
                                         const bool build_render_curve)
    {
        flatten_chunk_assembly_to_cache(cache.display, assembly, build_render_curve);
    }
} // namespace Game::PredictionCacheInternal
