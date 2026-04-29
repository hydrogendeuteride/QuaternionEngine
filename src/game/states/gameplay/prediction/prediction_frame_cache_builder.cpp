#include "game/states/gameplay/prediction/prediction_frame_cache_builder.h"
#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

namespace Game
{
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
        return PredictionCacheInternal::rebuild_prediction_frame_cache(solver,
                                                                       display,
                                                                       analysis,
                                                                       resolved_frame_spec,
                                                                       player_lookup_segments_inertial,
                                                                       cancel_requested,
                                                                       diagnostics,
                                                                       build_planned_render_curve);
    }

    bool PredictionFrameCacheBuilder::rebuild(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        return PredictionCacheInternal::rebuild_prediction_frame_cache(cache,
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
        return PredictionCacheInternal::rebuild_prediction_planned_frame_cache(solver,
                                                                               display,
                                                                               resolved_frame_spec,
                                                                               player_lookup_segments_inertial,
                                                                               cancel_requested,
                                                                               diagnostics,
                                                                               build_planned_render_curve);
    }

    bool PredictionFrameCacheBuilder::rebuild_planned(
            OrbitPredictionCache &cache,
            const orbitsim::TrajectoryFrameSpec &resolved_frame_spec,
            const std::vector<orbitsim::TrajectorySegment> &player_lookup_segments_inertial,
            const CancelCheck &cancel_requested,
            OrbitPredictionDerivedDiagnostics *diagnostics,
            const bool build_planned_render_curve)
    {
        return PredictionCacheInternal::rebuild_prediction_planned_frame_cache(cache,
                                                                               resolved_frame_spec,
                                                                               player_lookup_segments_inertial,
                                                                               cancel_requested,
                                                                               diagnostics,
                                                                               build_planned_render_curve);
    }
} // namespace Game
