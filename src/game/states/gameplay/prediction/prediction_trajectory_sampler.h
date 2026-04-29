#pragma once

#include "game/orbit/trajectory/trajectory_utils.h"

#include "orbitsim/spacecraft_lookup.hpp"
#include "orbitsim/trajectory_segments.hpp"
#include "orbitsim/trajectory_types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace Game
{
    struct PredictionTrajectorySampler
    {
        static constexpr orbitsim::SpacecraftId kPlayerDisplayTargetSpacecraftId =
                static_cast<orbitsim::SpacecraftId>(0x7000'0001u);

        static bool sample_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
                                          double query_time_s,
                                          orbitsim::State &out_state);

        static bool sample_inertial_state(const std::vector<orbitsim::TrajectorySegment> &segments,
                                          double query_time_s,
                                          orbitsim::State &out_state,
                                          TrajectoryBoundarySide boundary_side = TrajectoryBoundarySide::Before);

        static orbitsim::SpacecraftStateLookup build_player_lookup(
                const std::vector<orbitsim::TrajectorySample> &trajectory);

        static orbitsim::SpacecraftStateLookup build_player_lookup(
                const std::vector<orbitsim::TrajectorySegment> &segments);

        static bool slice_segments_from_cursor(const std::vector<orbitsim::TrajectorySegment> &segments,
                                               double t0_s,
                                               double t1_s,
                                               std::size_t &cursor,
                                               std::vector<orbitsim::TrajectorySegment> &out_segments);
    };
} // namespace Game
