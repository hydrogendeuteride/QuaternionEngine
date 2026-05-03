#include "game/states/gameplay/prediction/prediction_subject_state_provider.h"

#include "game/component/ship_controller.h"
#include "game/game_world.h"
#include "game/states/gameplay/orbiter_world_state_provider.h"
#include "game/states/gameplay/orbital_physics_system.h"
#include "game/states/gameplay/orbital_runtime_system.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "game/states/gameplay/time_warp_state.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <array>

namespace Game
{
    PredictionSubjectStateProvider::PredictionSubjectStateProvider(Context context)
        : _context(context)
    {
    }

    PredictionSubjectKey PredictionSubjectStateProvider::player_subject_key() const
    {
        const EntityId player_eid = _context.orbit.player_entity();
        return player_eid.is_valid()
                   ? PredictionSubjectKey{PredictionSubjectKind::Orbiter, player_eid.value}
                   : PredictionSubjectKey{};
    }

    std::vector<PredictionSubjectDescriptor> PredictionSubjectStateProvider::build_subject_descriptors() const
    {
        const OrbitalScenario *scenario = _context.orbit.scenario();
        std::vector<PredictionSubjectDescriptor> subjects;
        subjects.reserve(_context.orbit.orbiters().size() + (scenario ? scenario->bodies.size() : 0u));

        for (const OrbiterInfo &orbiter : _context.orbit.orbiters())
        {
            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            PredictionSubjectKey key{};
            key.kind = PredictionSubjectKind::Orbiter;
            key.value = orbiter.entity.value;

            subjects.push_back(PredictionSubjectDescriptor{
                    .key = key,
                    .label = orbiter.name,
                    .supports_maneuvers = subject_supports_maneuvers(key),
                    .is_celestial = false,
                    .orbit_rgb = subject_orbit_rgb(key),
            });
        }

        if (scenario)
        {
            const CelestialBodyInfo *ref_info = scenario->world_reference_body();
            for (const CelestialBodyInfo &body : scenario->bodies)
            {
                if (ref_info && body.sim_id == ref_info->sim_id)
                {
                    continue;
                }

                PredictionSubjectKey key{};
                key.kind = PredictionSubjectKind::Celestial;
                key.value = static_cast<uint32_t>(body.sim_id);

                subjects.push_back(PredictionSubjectDescriptor{
                        .key = key,
                        .label = body.name,
                        .supports_maneuvers = false,
                        .is_celestial = true,
                        .orbit_rgb = subject_orbit_rgb(key),
                });
            }
        }

        return subjects;
    }

    bool PredictionSubjectStateProvider::get_player_world_state(WorldVec3 &out_pos_world,
                                                                glm::dvec3 &out_vel_world,
                                                                glm::vec3 &out_vel_local) const
    {
        const OrbiterInfo *player = _context.orbit.find_player_orbiter();
        return player && get_orbiter_world_state(*player, out_pos_world, out_vel_world, out_vel_local);
    }

    bool PredictionSubjectStateProvider::get_orbiter_world_state(const OrbiterInfo &orbiter,
                                                                 WorldVec3 &out_pos_world,
                                                                 glm::dvec3 &out_vel_world,
                                                                 glm::vec3 &out_vel_local) const
    {
        return OrbiterWorldStateProvider(OrbiterWorldStateProvider::Context{
                .orbit = _context.orbit,
                .world = _context.world,
                .physics = _context.physics,
                .physics_context = _context.physics_context,
                .scenario_config = _context.scenario_config,
        }).get_orbiter_world_state(
                orbiter,
                out_pos_world,
                out_vel_world,
                out_vel_local);
    }

    bool PredictionSubjectStateProvider::get_subject_world_state(PredictionSubjectKey key,
                                                                 WorldVec3 &out_pos_world,
                                                                 glm::dvec3 &out_vel_world,
                                                                 glm::vec3 &out_vel_local) const
    {
        out_pos_world = WorldVec3(0.0);
        out_vel_world = glm::dvec3(0.0);
        out_vel_local = glm::vec3(0.0f);

        if (!key.valid())
        {
            return false;
        }

        if (key.kind == PredictionSubjectKind::Orbiter)
        {
            const OrbiterInfo *orbiter = _context.orbit.find_orbiter(EntityId{key.value});
            return orbiter && get_orbiter_world_state(*orbiter, out_pos_world, out_vel_world, out_vel_local);
        }

        const OrbitalScenario *scenario = _context.orbit.scenario();
        if (!scenario)
        {
            return false;
        }

        const CelestialBodyInfo *body_info = nullptr;
        for (const CelestialBodyInfo &candidate : scenario->bodies)
        {
            if (static_cast<uint32_t>(candidate.sim_id) == key.value)
            {
                body_info = &candidate;
                break;
            }
        }

        const orbitsim::MassiveBody *body = body_info ? scenario->sim.body_by_id(body_info->sim_id) : nullptr;
        const orbitsim::MassiveBody *ref_sim = scenario->world_reference_sim_body();
        if (!body || !ref_sim)
        {
            return false;
        }

        out_pos_world = _context.scenario_config.system_center +
                WorldVec3(body->state.position_m - ref_sim->state.position_m);
        out_vel_world = body->state.velocity_mps - ref_sim->state.velocity_mps;
        if (body_info && body_info->render_entity.is_valid())
        {
            if (const Entity *render_entity = _context.world.entities().find(body_info->render_entity))
            {
                out_pos_world = render_entity->position_world();
            }
        }

        return true;
    }

    WorldVec3 PredictionSubjectStateProvider::render_subject_position_world(const PredictionSubjectKey key,
                                                                            const float alpha) const
    {
        if (key.kind == PredictionSubjectKind::Orbiter)
        {
            if (const Entity *entity = _context.world.entities().find(EntityId{key.value}))
            {
                return entity->get_render_physics_center_of_mass_world(alpha);
            }
        }

        WorldVec3 pos_world{0.0, 0.0, 0.0};
        glm::dvec3 vel_world{0.0};
        glm::vec3 vel_local{0.0f};
        (void) get_subject_world_state(key, pos_world, vel_world, vel_local);
        return pos_world;
    }

    WorldVec3 PredictionSubjectStateProvider::reference_body_world() const
    {
        const OrbitalScenario *scenario = _context.orbit.scenario();
        if (!scenario)
        {
            return _context.scenario_config.system_center;
        }

        const CelestialBodyInfo *ref_info = scenario->world_reference_body();
        if (ref_info && ref_info->render_entity.is_valid())
        {
            if (const Entity *ref_entity = _context.world.entities().find(ref_info->render_entity))
            {
                return ref_entity->position_world();
            }
        }

        return _context.scenario_config.system_center;
    }

    bool PredictionSubjectStateProvider::subject_is_player(const PredictionSubjectKey key) const
    {
        return key.kind == PredictionSubjectKind::Orbiter && key.value == _context.orbit.player_entity().value;
    }

    bool PredictionSubjectStateProvider::subject_supports_maneuvers(const PredictionSubjectKey key) const
    {
        if (subject_is_player(key))
        {
            return true;
        }
        if (key.kind == PredictionSubjectKind::Orbiter)
        {
            const OrbiterInfo *orbiter = _context.orbit.find_orbiter(EntityId(key.value));
            return orbiter && orbiter->formation_hold_enabled && !orbiter->formation_leader_name.empty();
        }
        return false;
    }

    glm::vec3 PredictionSubjectStateProvider::subject_orbit_rgb(const PredictionSubjectKey key) const
    {
        static constexpr std::array<glm::vec3, 4> kOrbiterPalette{{
                {1.00f, 0.25f, 0.25f},
                {0.25f, 0.52f, 1.00f},
                {0.62f, 0.65f, 0.70f},
                {0.28f, 0.88f, 0.38f},
        }};
        static constexpr glm::vec3 kAnchorOrbiterColor{0.70f, 0.35f, 1.00f};
        static constexpr glm::vec3 kDefaultCelestialColor{0.80f, 0.82f, 0.86f};

        if (!key.valid())
        {
            return kDefaultCelestialColor;
        }

        if (key.kind == PredictionSubjectKind::Celestial)
        {
            const OrbitalScenario *scenario = _context.orbit.scenario();
            const CelestialBodyInfo *body_info = nullptr;
            if (scenario)
            {
                for (const CelestialBodyInfo &body : scenario->bodies)
                {
                    if (body.sim_id == static_cast<orbitsim::BodyId>(key.value))
                    {
                        body_info = &body;
                        break;
                    }
                }
            }

            if (!body_info)
            {
                return kDefaultCelestialColor;
            }

            for (const ScenarioConfig::CelestialDef &body_def : _context.scenario_config.celestials)
            {
                if (body_def.name == body_info->name)
                {
                    return body_def.has_prediction_orbit_color
                               ? body_def.prediction_orbit_color
                               : kDefaultCelestialColor;
                }
            }

            return kDefaultCelestialColor;
        }

        if (subject_is_player(key))
        {
            return kAnchorOrbiterColor;
        }

        const OrbiterInfo *orbiter = _context.orbit.find_orbiter(EntityId{key.value});
        if (orbiter)
        {
            for (const ScenarioConfig::OrbiterDef &orbiter_def : _context.scenario_config.orbiters)
            {
                if (orbiter_def.name == orbiter->name && orbiter_def.has_prediction_orbit_color)
                {
                    return orbiter_def.prediction_orbit_color;
                }
            }
        }

        std::size_t orbiter_palette_index = 0;
        for (const OrbiterInfo &candidate : _context.orbit.orbiters())
        {
            if (!candidate.entity.is_valid() || candidate.is_player)
            {
                continue;
            }

            if (candidate.entity.value == key.value)
            {
                return kOrbiterPalette[orbiter_palette_index % kOrbiterPalette.size()];
            }
            ++orbiter_palette_index;
        }

        return kOrbiterPalette[0];
    }

    bool PredictionSubjectStateProvider::subject_thrust_applied_this_tick(const PredictionSubjectKey key) const
    {
        if (!subject_is_player(key))
        {
            return false;
        }

        if (_context.orbital_physics.rails_warp_active() && _context.time_warp.mode == TimeWarpState::Mode::RailsWarp)
        {
            return _context.orbital_physics.rails_thrust_applied_this_tick();
        }

        const EntityId player_eid = _context.orbit.player_entity();
        if (!player_eid.is_valid())
        {
            return false;
        }

        const Entity *player = _context.world.entities().find(player_eid);
        if (!player)
        {
            return false;
        }

        const ShipController *sc = player->get_component<ShipController>();
        return sc && sc->thrust_applied_this_tick();
    }
} // namespace Game
