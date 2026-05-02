#include "game/states/gameplay/orbital_runtime_system.h"

#include <utility>

namespace Game
{
    void OrbitalRuntimeSystem::reset()
    {
        _orbiters.clear();
        _scenario.reset();
    }

    void OrbitalRuntimeSystem::set_scenario(std::unique_ptr<OrbitalScenario> scenario)
    {
        _scenario = std::move(scenario);
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
