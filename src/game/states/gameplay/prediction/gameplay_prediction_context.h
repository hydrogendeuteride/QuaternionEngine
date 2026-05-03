#pragma once

#include "orbitsim/types.hpp"

#include <functional>

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameWorld;
    class ManeuverSystem;
    class OrbitalPhysicsSystem;
    class OrbitalRuntimeSystem;
    class PredictionSystem;
    struct ManeuverNode;
    struct ScenarioConfig;
    struct TimeWarpState;

    struct GameplayPredictionContext
    {
        const OrbitalRuntimeSystem &orbit;
        const GameWorld &world;
        const Physics::PhysicsWorld *physics{nullptr};
        const Physics::PhysicsContext *physics_context{nullptr};
        const ScenarioConfig &scenario_config;
        const OrbitalPhysicsSystem &orbital_physics;
        const TimeWarpState &time_warp;
        const ManeuverSystem &maneuver;
        const PredictionSystem *prediction{nullptr};
        double fixed_time_s{0.0};
        double current_sim_time_s{0.0};
        bool debug_draw_enabled{false};
        std::function<orbitsim::BodyId(const ManeuverNode &, double)> resolve_maneuver_node_primary_body_id{};
    };
} // namespace Game
