#pragma once

#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace Game
{
    struct PredictionFrameCacheBuilder
    {
        using CancelCheck = std::function<bool()>;

        static orbitsim::FrameSegmentTransformOptions build_frame_segment_transform_options(
                const orbitsim::TrajectoryFrameSpec &frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &inertial_segments,
                const CancelCheck &cancel_requested = {});

        static bool rebuild(
                const PredictionSolverTrajectoryCache &solver,
                PredictionDisplayFrameCache &display,
                PredictionAnalysisCache &analysis,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_planned_render_curve = true);

        static bool rebuild(
                OrbitPredictionCache &cache,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_planned_render_curve = true);

        static bool rebuild_planned(
                const PredictionSolverTrajectoryCache &solver,
                PredictionDisplayFrameCache &display,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_planned_render_curve = true);

        static bool rebuild_planned(
                OrbitPredictionCache &cache,
                const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
                const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
                const CancelCheck &cancel_requested = {},
                OrbitPredictionDerivedDiagnostics *diagnostics = nullptr,
                bool build_planned_render_curve = true);
    };
} // namespace Game
