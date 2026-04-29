#include "game/states/gameplay/prediction/prediction_frame_cache_builder.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

namespace Game
{
    using namespace PredictionCacheInternal;

    orbitsim::FrameSegmentTransformOptions PredictionFrameCacheBuilder::build_frame_segment_transform_options(
            const orbitsim::TrajectoryFrameSpec &frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &inertial_segments,
            const CancelCheck &cancel_requested)
    {
        return PredictionCacheInternal::build_frame_segment_transform_options(frame_spec,
                                                                              inertial_segments,
                                                                              cancel_requested);
    }

    bool PredictionFrameCacheBuilder::rebuild(
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
                    PredictionCacheInternal::build_frame_segment_transform_options(
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
                        PredictionCacheInternal::build_frame_segment_transform_options(
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

    bool PredictionFrameCacheBuilder::rebuild(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        return rebuild(cache.solver,
                       cache.display,
                       cache.analysis,
                       resolved_frame_spec,
                       player_lookup_segments_inertial,
                       cancel_requested,
                       diagnostics,
                       build_planned_render_curve);
    }

    bool PredictionFrameCacheBuilder::rebuild_planned(
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
                    PredictionCacheInternal::build_frame_segment_transform_options(
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

        display.render_curve_frame_planned =
                (build_planned_render_curve && !display.trajectory_segments_frame_planned.empty())
                        ? OrbitRenderCurve::build(display.trajectory_segments_frame_planned)
                        : OrbitRenderCurve{};
        display.resolved_frame_spec = resolved_frame_spec;
        display.resolved_frame_spec_valid = true;
        update_derived_diagnostics(diagnostics, display, PredictionDerivedStatus::Success);
        return true;
    }

    bool PredictionFrameCacheBuilder::rebuild_planned(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        return rebuild_planned(cache.solver,
                               cache.display,
                               resolved_frame_spec,
                               player_lookup_segments_inertial,
                               cancel_requested,
                               diagnostics,
                               build_planned_render_curve);
    }
} // namespace Game
