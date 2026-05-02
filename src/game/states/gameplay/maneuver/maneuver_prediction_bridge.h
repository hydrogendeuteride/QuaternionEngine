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
        static void begin_node_dv_edit_preview(GameplayState &state, int node_id);
        static void update_node_dv_edit_preview(GameplayState &state, int node_id);
        static void finish_node_dv_edit_preview(GameplayState &state, bool changed);
        static void begin_node_time_edit_preview(GameplayState &state, int node_id, double previous_time_s);
        static void update_node_time_edit_preview(GameplayState &state, int node_id, double previous_time_s);
        static void finish_node_time_edit_preview(GameplayState &state, bool changed);

        static orbitsim::BodyId resolve_node_primary_body_id(const GameplayState &state,
                                                             const ManeuverNode &node,
                                                             double query_time_s);
        static WorldVec3 compute_align_delta(GameplayState &state,
                                             GameStateContext &ctx,
                                             const OrbitPredictionCache &cache,
                                             const std::vector<orbitsim::TrajectorySample> &traj_base);
    };
} // namespace Game
