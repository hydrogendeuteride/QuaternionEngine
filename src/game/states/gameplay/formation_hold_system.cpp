#include "game/states/gameplay/formation_hold_system.h"

#include "game/game_world.h"
#include "game/states/gameplay/orbit_helpers.h"
#include "game/states/gameplay/orbital_runtime_system.h"
#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/frame_utils.hpp"
#include "physics/physics_world.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string_view>

namespace Game
{
    namespace
    {
        using detail::finite_vec3;

        bool sample_inertial_state(const OrbitalRuntimeSystem &orbit,
                                   const OrbiterInfo &orbiter,
                                   const WorldVec3 &system_center,
                                   const FormationHoldSystem::OrbiterWorldStateSampler &world_state_sampler,
                                   orbitsim::State &out_state)
        {
            out_state = {};
            if (!orbit.scenario_owner() || !world_state_sampler)
            {
                return false;
            }

            const orbitsim::MassiveBody *ref_sim = orbit.scenario_owner()->world_reference_sim_body();
            if (!ref_sim)
            {
                return false;
            }

            WorldVec3 pos_world{0.0};
            glm::dvec3 vel_world{0.0};
            glm::vec3 vel_local{0.0f};
            if (!world_state_sampler(orbiter, pos_world, vel_world, vel_local))
            {
                return false;
            }

            out_state = orbitsim::make_state(
                    ref_sim->state.position_m + glm::dvec3(pos_world - system_center),
                    ref_sim->state.velocity_mps + vel_world);
            return true;
        }

        const orbitsim::MassiveBody *select_primary_body_for_state(const OrbitalRuntimeSystem &orbit,
                                                                   const orbitsim::State &state)
        {
            if (!orbit.scenario_owner() || orbit.scenario_owner()->sim.massive_bodies().empty())
            {
                return nullptr;
            }

            const auto body_position_at = [&orbit](const std::size_t i) -> orbitsim::Vec3 {
                return orbit.scenario_owner()->sim.massive_bodies()[i].state.position_m;
            };

            const std::size_t primary_index = orbitsim::auto_select_primary_index(
                    orbit.scenario_owner()->sim.massive_bodies(),
                    state.position_m,
                    body_position_at,
                    orbit.scenario_owner()->sim.config().softening_length_m);
            if (primary_index >= orbit.scenario_owner()->sim.massive_bodies().size())
            {
                return nullptr;
            }

            return &orbit.scenario_owner()->sim.massive_bodies()[primary_index];
        }

        bool build_formation_leader_lvlh_frame(
                const OrbitalRuntimeSystem &orbit,
                const OrbiterInfo &leader,
                const WorldVec3 &system_center,
                const FormationHoldSystem::OrbiterWorldStateSampler &world_state_sampler,
                orbitsim::RotatingFrame &out_frame)
        {
            out_frame = {};

            orbitsim::State leader_state{};
            if (!sample_inertial_state(orbit, leader, system_center, world_state_sampler, leader_state))
            {
                return false;
            }

            const orbitsim::MassiveBody *primary = select_primary_body_for_state(orbit, leader_state);
            if (!primary)
            {
                return false;
            }

            const std::optional<orbitsim::RotatingFrame> lvlh =
                    orbitsim::make_lvlh_frame(primary->state, leader_state);
            if (!lvlh.has_value() || !lvlh->valid())
            {
                return false;
            }

            out_frame = *lvlh;
            return true;
        }
    } // namespace

    void FormationHoldSystem::update(const Context &context, const double dt_s)
    {
        if (!(dt_s > 0.0) || !std::isfinite(dt_s) || !context.orbit.scenario_owner() ||
            !context.world_state_sampler)
        {
            return;
        }

        const double omega = std::max(0.0, context.omega);
        if (!(omega > 0.0) || !std::isfinite(omega))
        {
            return;
        }

        const double max_dv_per_step_mps = std::max(0.0, context.max_dv_per_step_mps);

        for (OrbiterInfo &orbiter : context.orbit.orbiters())
        {
            if (!orbiter.formation_hold_enabled || orbiter.formation_leader_name.empty() || orbiter.is_player)
            {
                continue;
            }

            const OrbiterInfo *leader = context.orbit.find_orbiter(std::string_view(orbiter.formation_leader_name));
            if (!leader || leader == &orbiter)
            {
                continue;
            }

            orbitsim::RotatingFrame leader_lvlh{};
            if (!build_formation_leader_lvlh_frame(context.orbit,
                                                   *leader,
                                                   context.system_center,
                                                   context.world_state_sampler,
                                                   leader_lvlh))
            {
                continue;
            }

            orbitsim::State follower_state_inertial{};
            if (!sample_inertial_state(context.orbit,
                                       orbiter,
                                       context.system_center,
                                       context.world_state_sampler,
                                       follower_state_inertial))
            {
                continue;
            }

            const orbitsim::State follower_state_lvlh =
                    orbitsim::inertial_state_to_frame(follower_state_inertial, leader_lvlh);

            const glm::dvec3 pos_error =
                    glm::dvec3(follower_state_lvlh.position_m) - orbiter.formation_slot_lvlh_m;
            const glm::dvec3 vel = glm::dvec3(follower_state_lvlh.velocity_mps);

            const double odt = omega * dt_s;
            const double exp_decay = std::exp(-odt);
            const glm::dvec3 vel_after =
                    (vel * (1.0 - odt) - (omega * omega * pos_error * dt_s)) * exp_decay;
            glm::dvec3 dv_lvlh = vel_after - vel;

            if (!finite_vec3(dv_lvlh))
            {
                continue;
            }

            const double dv_len = glm::length(dv_lvlh);
            if (std::isfinite(dv_len) && dv_len > max_dv_per_step_mps && dv_len > 0.0)
            {
                dv_lvlh *= (max_dv_per_step_mps / dv_len);
            }

            const glm::dvec3 dv_world =
                    orbitsim::frame_vector_to_inertial(leader_lvlh, dv_lvlh);
            if (!finite_vec3(dv_world))
            {
                continue;
            }

            if (orbiter.rails.active())
            {
                orbitsim::Spacecraft *sc =
                        context.orbit.scenario_owner()->sim.spacecraft_by_id(orbiter.rails.sc_id);
                if (!sc)
                {
                    continue;
                }

                sc->state.velocity_mps += dv_world;
                continue;
            }

            if (!context.physics || !context.physics_context || !orbiter.entity.is_valid())
            {
                continue;
            }

            Entity *entity = context.world.entities().find(orbiter.entity);
            if (!entity || !entity->has_physics())
            {
                continue;
            }

            const Physics::BodyId body_id{entity->physics_body_value()};
            if (!context.physics->is_body_valid(body_id))
            {
                continue;
            }

            const double mass_kg = std::max(1.0, orbiter.mass_kg);
            const glm::vec3 force = glm::vec3(dv_world * mass_kg / dt_s);
            context.physics->add_force(body_id, force);
            context.physics->activate(body_id);
        }
    }
} // namespace Game
