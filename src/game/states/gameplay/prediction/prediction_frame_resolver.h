#pragma once

#include "game/states/gameplay/orbit_helpers.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"

#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/spacecraft_lookup.hpp"

#include <limits>
#include <vector>

namespace Game
{
    struct PredictionFrameResolverContext
    {
        PredictionFrameSelectionState frame_selection{};
        WorldVec3 system_center{0.0, 0.0, 0.0};
        WorldVec3 world_reference_body_world{0.0, 0.0, 0.0};
        double current_sim_time_s{std::numeric_limits<double>::quiet_NaN()};
        double softening_length_m{0.0};
        bool maneuver_nodes_enabled{false};
        bool maneuver_plan_has_nodes{false};
        bool maneuver_axis_drag_active{false};
        double maneuver_drag_display_reference_time_s{std::numeric_limits<double>::quiet_NaN()};
        orbitsim::BodyId world_reference_body_id{orbitsim::kInvalidBodyId};
        const orbitsim::MassiveBody *world_reference_sim_body{nullptr};
        orbitsim::SpacecraftStateLookup player_lookup{};
    };

    class PredictionFrameResolver
    {
    public:
        static bool same_frame_spec(const orbitsim::TrajectoryFrameSpec &a,
                                    const orbitsim::TrajectoryFrameSpec &b);

        static bool is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec);

        static const orbitsim::MassiveBody *find_massive_body(
                const std::vector<orbitsim::MassiveBody> &bodies,
                orbitsim::BodyId body_id);

        static orbitsim::BodyId select_primary_body_id(
                const PredictionFrameResolverContext &context,
                const std::vector<orbitsim::MassiveBody> &bodies,
                const OrbitPredictionCache *cache,
                const orbitsim::Vec3 &query_pos_m,
                double query_time_s,
                orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId);

        static orbitsim::TrajectoryFrameSpec resolve_display_frame_spec(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static double resolve_display_reference_time_s(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static bool build_display_frame(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                orbitsim::RotatingFrame &out_frame,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static bool build_display_transform(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                WorldVec3 &out_origin_world,
                glm::dmat3 &out_frame_to_world,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static WorldVec3 sample_position_world(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                const orbitsim::TrajectorySample &sample,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static WorldVec3 sample_hermite_world(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                const orbitsim::TrajectorySample &a,
                const orbitsim::TrajectorySample &b,
                double t_s,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());

        static WorldVec3 frame_origin_world(
                const PredictionFrameResolverContext &context,
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());
    };
} // namespace Game
