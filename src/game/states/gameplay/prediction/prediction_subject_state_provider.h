#pragma once

#include "core/world.h"
#include "game/states/gameplay/prediction/prediction_host_context.h"

#include <glm/glm.hpp>

#include <vector>

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameWorld;
    class OrbitalPhysicsSystem;
    class OrbitalRuntimeSystem;
    struct OrbiterInfo;
    struct ScenarioConfig;
    struct TimeWarpState;

    class PredictionSubjectStateProvider
    {
    public:
        struct Context
        {
            const OrbitalRuntimeSystem &orbit;
            const GameWorld &world;
            const Physics::PhysicsWorld *physics{nullptr};
            const Physics::PhysicsContext *physics_context{nullptr};
            const ScenarioConfig &scenario_config;
            const OrbitalPhysicsSystem &orbital_physics;
            const TimeWarpState &time_warp;
        };

        explicit PredictionSubjectStateProvider(Context context);

        [[nodiscard]] PredictionSubjectKey player_subject_key() const;
        [[nodiscard]] std::vector<PredictionSubjectDescriptor> build_subject_descriptors() const;

        [[nodiscard]] bool get_player_world_state(WorldVec3 &out_pos_world,
                                                  glm::dvec3 &out_vel_world,
                                                  glm::vec3 &out_vel_local) const;
        [[nodiscard]] bool get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                   WorldVec3 &out_pos_world,
                                                   glm::dvec3 &out_vel_world,
                                                   glm::vec3 &out_vel_local) const;
        [[nodiscard]] bool get_subject_world_state(PredictionSubjectKey key,
                                                   WorldVec3 &out_pos_world,
                                                   glm::dvec3 &out_vel_world,
                                                   glm::vec3 &out_vel_local) const;
        [[nodiscard]] WorldVec3 render_subject_position_world(PredictionSubjectKey key, float alpha) const;
        [[nodiscard]] WorldVec3 reference_body_world() const;

        [[nodiscard]] bool subject_is_player(PredictionSubjectKey key) const;
        [[nodiscard]] bool subject_supports_maneuvers(PredictionSubjectKey key) const;
        [[nodiscard]] glm::vec3 subject_orbit_rgb(PredictionSubjectKey key) const;
        [[nodiscard]] bool subject_thrust_applied_this_tick(PredictionSubjectKey key) const;

    private:
        Context _context;
    };
} // namespace Game
