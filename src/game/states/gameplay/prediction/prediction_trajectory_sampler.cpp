#include "game/states/gameplay/prediction/prediction_trajectory_sampler.h"

#include "game/states/gameplay/prediction/gameplay_prediction_cache_internal.h"

namespace Game
{
    bool PredictionTrajectorySampler::sample_inertial_state(
            const std::vector<orbitsim::TrajectorySample> &trajectory,
            const double query_time_s,
            orbitsim::State &out_state)
    {
        return PredictionCacheInternal::sample_prediction_inertial_state(trajectory, query_time_s, out_state);
    }

    bool PredictionTrajectorySampler::sample_inertial_state(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const double query_time_s,
            orbitsim::State &out_state,
            const TrajectoryBoundarySide boundary_side)
    {
        return PredictionCacheInternal::sample_prediction_inertial_state(segments,
                                                                         query_time_s,
                                                                         out_state,
                                                                         boundary_side);
    }

    orbitsim::SpacecraftStateLookup PredictionTrajectorySampler::build_player_lookup(
            const std::vector<orbitsim::TrajectorySample> &trajectory)
    {
        return PredictionCacheInternal::build_player_lookup(trajectory);
    }

    orbitsim::SpacecraftStateLookup PredictionTrajectorySampler::build_player_lookup(
            const std::vector<orbitsim::TrajectorySegment> &segments)
    {
        return PredictionCacheInternal::build_player_lookup(segments);
    }

    bool PredictionTrajectorySampler::slice_segments_from_cursor(
            const std::vector<orbitsim::TrajectorySegment> &segments,
            const double t0_s,
            const double t1_s,
            std::size_t &cursor,
            std::vector<orbitsim::TrajectorySegment> &out_segments)
    {
        return PredictionCacheInternal::slice_trajectory_segments_from_cursor(segments,
                                                                              t0_s,
                                                                              t1_s,
                                                                              cursor,
                                                                              out_segments);
    }
} // namespace Game
