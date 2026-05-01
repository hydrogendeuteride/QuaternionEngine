#pragma once

#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"
#include "game/states/gameplay/prediction/prediction_frame_resolver.h"
#include "game/states/gameplay/prediction/runtime/prediction_track_lifecycle.h"

#include <cstddef>
#include <functional>
#include <limits>

namespace Game
{
    struct ManeuverRuntimeCacheInput
    {
        ManeuverPlanState &plan;
        const PredictionTrackState *player_track{nullptr};
        const OrbitPredictionCache *active_cache{nullptr};
        const OrbitPredictionCache *stable_cache{nullptr};
        PredictionFrameResolverContext frame_context{};
        PredictionRuntimeDetail::PredictionTrackLifecycleSnapshot lifecycle{};
        const ManeuverGizmoInteraction &gizmo_interaction;
        const ManeuverNodeEditPreview &edit_preview;
        ManeuverGizmoBasisMode basis_mode{ManeuverGizmoBasisMode::ProgradeOutwardNormal};
        double display_time_s{std::numeric_limits<double>::quiet_NaN()};
        double current_sim_time_s{std::numeric_limits<double>::quiet_NaN()};
        WorldVec3 align_delta{0.0, 0.0, 0.0};
        int active_preview_anchor_node_id{-1};
        bool hold_cached_release_state{false};
        std::function<orbitsim::BodyId(const ManeuverNode &, double)> resolve_primary_body_id{};
    };

    struct ManeuverRuntimeCacheResult
    {
        std::size_t node_count{0};
        std::size_t valid_node_count{0};
        bool active_prediction_cache_valid{false};
    };

    class ManeuverRuntimeCacheBuilder
    {
    public:
        static ManeuverRuntimeCacheResult rebuild(const ManeuverRuntimeCacheInput &input);
    };
} // namespace Game
