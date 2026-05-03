#include "game/states/gameplay/orbiter_world_state_provider.h"

#include "game/game_world.h"
#include "game/states/gameplay/orbit_helpers.h"
#include "game/states/gameplay/orbital_runtime_system.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

namespace Game
{
    OrbiterWorldStateProvider::OrbiterWorldStateProvider(Context context)
        : _context(context)
    {
    }

    bool OrbiterWorldStateProvider::get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                            WorldVec3 &out_pos_world,
                                                            glm::dvec3 &out_vel_world,
                                                            glm::vec3 &out_vel_local) const
    {
        if (!orbiter.entity.is_valid())
        {
            return false;
        }

        const Entity *entity = _context.world.entities().find(orbiter.entity);
        if (!entity)
        {
            return false;
        }

        out_pos_world = entity->physics_center_of_mass_world();
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

        if (const OrbitalScenario *scenario = _context.orbit.scenario())
        {
            const orbitsim::MassiveBody *ref_sim = scenario->world_reference_sim_body();
            if (orbiter.rails.active() && ref_sim)
            {
                if (const orbitsim::Spacecraft *sc = scenario->sim.spacecraft_by_id(orbiter.rails.sc_id))
                {
                    out_pos_world = _context.scenario_config.system_center +
                            WorldVec3(sc->state.position_m - ref_sim->state.position_m);
                    out_vel_world = sc->state.velocity_mps - ref_sim->state.velocity_mps;
                    out_vel_local = glm::vec3(0.0f);
                    return true;
                }
            }
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (_context.physics && _context.physics_context && entity->has_physics())
        {
            const Physics::BodyId body_id{entity->physics_body_value()};
            if (_context.physics->is_body_valid(body_id))
            {
                const glm::quat rotation = _context.physics->get_rotation(body_id);
                const WorldVec3 body_origin_world =
                        local_to_world_d(_context.physics->get_position(body_id),
                                         _context.physics_context->origin_world());
                out_pos_world = entity->physics_center_of_mass_world(body_origin_world, rotation);
                out_vel_local = _context.physics->get_linear_velocity(body_id);
                out_vel_world = _context.physics_context->velocity_origin_world() + glm::dvec3(out_vel_local);
            }
        }
#endif

        return true;
    }
} // namespace Game
