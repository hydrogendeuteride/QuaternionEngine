#pragma once

#include "core/world.h"

#include <glm/glm.hpp>

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameWorld;
    class OrbitalRuntimeSystem;
    struct OrbiterInfo;
    struct ScenarioConfig;

    class OrbiterWorldStateProvider
    {
    public:
        struct Context
        {
            const OrbitalRuntimeSystem &orbit;
            const GameWorld &world;
            const Physics::PhysicsWorld *physics{nullptr};
            const Physics::PhysicsContext *physics_context{nullptr};
            const ScenarioConfig &scenario_config;
        };

        explicit OrbiterWorldStateProvider(Context context);

        [[nodiscard]] bool get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                   WorldVec3 &out_pos_world,
                                                   glm::dvec3 &out_vel_world,
                                                   glm::vec3 &out_vel_local) const;

    private:
        Context _context;
    };
} // namespace Game
