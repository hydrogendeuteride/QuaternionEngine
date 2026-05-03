#include "game/states/gameplay/orbital_runtime_system.h"

#include "game/states/gameplay/orbit_helpers.h"
#include "game/states/gameplay/scenario/scenario_config.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Game
{
    namespace
    {
        using detail::circular_orbit_relative_state_xz;
        using detail::two_body_circular_barycentric_xz;
    } // namespace

    void OrbitalRuntimeSystem::reset()
    {
        _orbiters.clear();
        _scenario.reset();
    }

    void OrbitalRuntimeSystem::set_scenario(std::unique_ptr<OrbitalScenario> scenario)
    {
        _scenario = std::move(scenario);
    }

    void OrbitalRuntimeSystem::initialize_scenario(const ScenarioConfig &config,
                                                   WorldVec3 &player_pos_world,
                                                   glm::dvec3 &player_vel_world)
    {
        if (config.celestials.empty())
        {
            return;
        }

        auto scenario = std::make_unique<OrbitalScenario>();

        const double speed_scale = std::max(0.0, config.speed_scale);

        orbitsim::GameSimulation::Config sim_config;
        sim_config.gravitational_constant = orbitsim::kGravitationalConstant_SI * speed_scale * speed_scale;
        sim_config.softening_length_m = 0.0;
        sim_config.enable_events = false;

        scenario->sim = orbitsim::GameSimulation(sim_config);
        scenario->world_reference_body_index = 0;

        const auto &reference_def = config.celestials[0];

        orbitsim::MassiveBody reference_body{};
        reference_body.mass_kg = reference_def.mass_kg;
        reference_body.radius_m = reference_def.radius_m;
        reference_body.atmosphere_top_height_m = reference_def.atmosphere_top_m;
        reference_body.terrain_max_height_m = reference_def.terrain_max_m;
        reference_body.soi_radius_m = reference_def.soi_radius_m;

        std::vector<orbitsim::MassiveBody> sim_bodies;
        sim_bodies.push_back(reference_body);

        for (std::size_t i = 1; i < config.celestials.size(); ++i)
        {
            const auto &body_def = config.celestials[i];

            orbitsim::MassiveBody body{};
            body.mass_kg = body_def.mass_kg;
            body.radius_m = body_def.radius_m;
            body.atmosphere_top_height_m = body_def.atmosphere_top_m;
            body.terrain_max_height_m = body_def.terrain_max_m;
            body.soi_radius_m = body_def.soi_radius_m;
            sim_bodies.push_back(body);
        }

        if (sim_bodies.size() >= 2)
        {
            for (std::size_t i = 1; i < sim_bodies.size(); ++i)
            {
                const double separation_m = std::max(reference_def.radius_m * 2.0,
                                                     config.celestials[i].orbit_distance_m);
                const auto barycentric_init = two_body_circular_barycentric_xz(
                        sim_config.gravitational_constant,
                        sim_bodies[0].mass_kg,
                        sim_bodies[i].mass_kg,
                        separation_m,
                        0.0);

                sim_bodies[0].state.position_m += barycentric_init.state_a.position_m;
                sim_bodies[0].state.velocity_mps += barycentric_init.state_a.velocity_mps;
                sim_bodies[i].state = barycentric_init.state_b;
            }
        }

        bool all_valid = true;
        for (std::size_t i = 0; i < sim_bodies.size(); ++i)
        {
            const auto handle = scenario->sim.create_body(sim_bodies[i]);
            if (!handle.valid())
            {
                all_valid = false;
                break;
            }

            CelestialBodyInfo info{};
            info.sim_id = handle.id;
            info.name = config.celestials[i].name;
            info.radius_m = config.celestials[i].radius_m;
            info.mass_kg = config.celestials[i].mass_kg;
            info.has_terrain = config.celestials[i].has_terrain;
            scenario->bodies.push_back(std::move(info));
        }

        if (!all_valid || scenario->bodies.empty())
        {
            set_scenario(std::move(scenario));
            return;
        }

        const CelestialBodyInfo *reference_info = scenario->world_reference_body();
        const orbitsim::MassiveBody *reference_sim = scenario->world_reference_sim_body();
        if (reference_info && reference_sim)
        {
            double player_altitude_m = 400'000.0;
            for (const auto &orbiter_def : config.orbiters)
            {
                if (orbiter_def.is_player)
                {
                    player_altitude_m = orbiter_def.orbit_altitude_m;
                    break;
                }
            }

            const double orbit_radius_m = reference_info->radius_m + player_altitude_m;
            const detail::OrbitRelativeState ship_relative =
                    circular_orbit_relative_state_xz(sim_config.gravitational_constant,
                                                     reference_info->mass_kg,
                                                     std::max(1.0, orbit_radius_m),
                                                     0.0);

            player_pos_world = config.system_center + WorldVec3(ship_relative.position_m);
            player_vel_world = glm::dvec3(ship_relative.velocity_mps);
        }

        set_scenario(std::move(scenario));
    }

    double OrbitalRuntimeSystem::sim_time_or(const double fallback_s) const
    {
        return _scenario ? _scenario->sim.time_s() : fallback_s;
    }

    OrbiterInfo *OrbitalRuntimeSystem::find_player_orbiter()
    {
        for (OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.is_player)
            {
                return &orbiter;
            }
        }
        return nullptr;
    }

    const OrbiterInfo *OrbitalRuntimeSystem::find_player_orbiter() const
    {
        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.is_player)
            {
                return &orbiter;
            }
        }
        return nullptr;
    }

    OrbiterInfo *OrbitalRuntimeSystem::find_orbiter(const EntityId entity)
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }
        return nullptr;
    }

    const OrbiterInfo *OrbitalRuntimeSystem::find_orbiter(const EntityId entity) const
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }
        return nullptr;
    }

    OrbiterInfo *OrbitalRuntimeSystem::find_orbiter(const std::string_view name)
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }
        return nullptr;
    }

    const OrbiterInfo *OrbitalRuntimeSystem::find_orbiter(const std::string_view name) const
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }
        return nullptr;
    }

    EntityId OrbitalRuntimeSystem::player_entity() const
    {
        const OrbiterInfo *player = find_player_orbiter();
        return player ? player->entity : EntityId{};
    }

    EntityId OrbitalRuntimeSystem::select_rebase_anchor_entity() const
    {
        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.is_rebase_anchor && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.is_player && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        return EntityId{};
    }

} // namespace Game
