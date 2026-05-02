#pragma once

#include "core/world.h"
#include "orbitsim/types.hpp"

#include <vector>

namespace orbitsim
{
    struct TrajectorySample;
} // namespace orbitsim

namespace Game
{
    class GameplayState;
    struct GameStateContext;
    struct ManeuverNode;
    struct OrbitPredictionCache;

    class ManeuverPredictionBridge
    {
    public:
        static orbitsim::BodyId resolve_node_primary_body_id(const GameplayState &state,
                                                             const ManeuverNode &node,
                                                             double query_time_s);
        static WorldVec3 compute_align_delta(GameplayState &state,
                                             GameStateContext &ctx,
                                             const OrbitPredictionCache &cache,
                                             const std::vector<orbitsim::TrajectorySample> &traj_base);
    };
} // namespace Game
