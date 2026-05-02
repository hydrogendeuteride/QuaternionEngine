#pragma once

#include "game/states/gameplay/orbit_runtime_types.h"

#include <functional>

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameWorld;
    class OrbitalRuntimeSystem;

    class FormationHoldSystem
    {
    public:
        using OrbiterWorldStateSampler =
                std::function<bool(const OrbiterInfo &, WorldVec3 &, glm::dvec3 &, glm::vec3 &)>;

        struct Context
        {
            OrbitalRuntimeSystem &orbit;
            GameWorld &world;
            Physics::PhysicsWorld *physics{nullptr};
            Physics::PhysicsContext *physics_context{nullptr};
            WorldVec3 system_center{0.0, 0.0, 0.0};
            double omega{0.5};
            double max_dv_per_step_mps{20.0};
            OrbiterWorldStateSampler world_state_sampler{};
        };

        static void update(const Context &context, double dt_s);
    };
} // namespace Game
