#pragma once

#include "game/entity.h"
#include "physics/body_settings.h"

#include "orbitsim/game_sim.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace Game
{
    // ============================================================================
    // CelestialBodyInfo -- runtime state for one massive body in the scenario.
    // ============================================================================

    struct CelestialBodyInfo
    {
        orbitsim::BodyId sim_id{orbitsim::kInvalidBodyId};
        EntityId render_entity;
        std::string name;
        double radius_m{0.0};
        double mass_kg{0.0};
        bool has_terrain{false};
    };

    // ============================================================================
    // OrbitalScenario -- owns the N-body simulation + celestial body registry.
    // Replaces the old OrbitsimDemo which only supported Earth + Moon.
    // ============================================================================

    struct OrbitalScenario
    {
        orbitsim::GameSimulation sim{};
        std::vector<CelestialBodyInfo> bodies;
        size_t world_reference_body_index{0}; // index into bodies[] for the world/render frame center (e.g. earth)

        const CelestialBodyInfo *world_reference_body() const
        {
            if (world_reference_body_index < bodies.size())
            {
                return &bodies[world_reference_body_index];
            }
            return nullptr;
        }

        CelestialBodyInfo *world_reference_body()
        {
            if (world_reference_body_index < bodies.size())
            {
                return &bodies[world_reference_body_index];
            }
            return nullptr;
        }

        const CelestialBodyInfo *find_body(const std::string &name) const
        {
            for (const auto &b : bodies)
            {
                if (b.name == name)
                {
                    return &b;
                }
            }
            return nullptr;
        }

        const orbitsim::MassiveBody *world_reference_sim_body() const
        {
            const CelestialBodyInfo *ref = world_reference_body();
            if (!ref || ref->sim_id == orbitsim::kInvalidBodyId)
            {
                return nullptr;
            }
            return sim.body_by_id(ref->sim_id);
        }

        orbitsim::MassiveBody *world_reference_sim_body()
        {
            CelestialBodyInfo *ref = world_reference_body();
            if (!ref || ref->sim_id == orbitsim::kInvalidBodyId)
            {
                return nullptr;
            }
            return sim.body_by_id(ref->sim_id);
        }
    };

    // ============================================================================
    // OrbiterInfo -- runtime state for one orbiting entity (ship, probe, etc.)
    // ============================================================================

    struct OrbiterInfo
    {
        EntityId entity;
        std::string name;
        bool apply_gravity{true};
        bool is_player{false}; // HUD/camera/prediction subject candidates
        bool is_rebase_anchor{false};
        bool render_is_gltf{false};
        double mass_kg{0.0};
        Physics::BodySettings physics_settings{};
        bool use_physics_interpolation{true};
        bool formation_hold_enabled{false};
        std::string formation_leader_name;
        glm::dvec3 formation_slot_lvlh_m{0.0};

        struct RailsState
        {
            orbitsim::SpacecraftId sc_id{orbitsim::kInvalidSpacecraftId};
            glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
            glm::vec3 angular_velocity_radps{0.0f};
            bool sas_enabled{false};
            bool sas_toggle_prev_down{false};

            bool active() const { return sc_id != orbitsim::kInvalidSpacecraftId; }

            void clear()
            {
                sc_id = orbitsim::kInvalidSpacecraftId;
                rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                angular_velocity_radps = glm::vec3(0.0f);
                sas_enabled = false;
                sas_toggle_prev_down = false;
            }
        };

        RailsState rails{};
    };
} // namespace Game
