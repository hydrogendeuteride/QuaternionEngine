#pragma once

#include "core/game_api.h"
#include "core/world.h"
#include "physics/body_settings.h"

#include <glm/vec3.hpp>

#include <string>
#include <vector>

namespace Game
{
    // ScenarioConfig defines a gameplay scenario as serializable data:
    // massive bodies plus orbiting entities spawned into the scene.
    struct ScenarioConfig
    {
        struct CelestialDef
        {
            std::string name;
            double mass_kg{0.0};
            double radius_m{0.0};
            double atmosphere_top_m{0.0};
            double terrain_max_m{0.0};
            double soi_radius_m{0.0};
            double orbit_distance_m{0.0}; // 0 = this body is the scenario reference body
            bool has_terrain{false};

            // Terrain rendering assets (only used when has_terrain=true).
            std::string albedo_dir;
            std::string height_dir;
            double height_max_m{0.0};
            std::string emission_dir;
            glm::vec3 emission_factor{0.0f};
            float render_scale{1.0f}; // legacy fallback scale for non-planet-system celestial rendering
            glm::vec3 prediction_orbit_color{0.75f, 0.75f, 0.75f};
            bool has_prediction_orbit_color{false};
        };

        struct OrbiterDef
        {
            std::string name;
            double orbit_altitude_m{0.0};      // altitude above reference body surface
            glm::dvec3 offset_from_player{0.0}; // relative spawn offset for non-player orbiters
            glm::dvec3 relative_velocity{0.0};  // initial velocity relative to player
            bool formation_hold_enabled{false};
            std::string formation_leader;
            glm::dvec3 formation_slot_lvlh_m{0.0}; // slot in leader-centered LVLH frame
            std::string prediction_group;       // optional authored orbit overlay group
            glm::vec3 prediction_orbit_color{1.0f, 0.25f, 0.25f};
            bool has_prediction_orbit_color{false};
            GameAPI::PrimitiveType primitive{GameAPI::PrimitiveType::Capsule};
            glm::vec3 render_scale{1.0f};
            Physics::BodySettings body_settings{};
            bool is_player{false};
            bool is_rebase_anchor{false}; // explicit floating-origin anchor candidate
        };

        std::vector<CelestialDef> celestials; // [0] = reference body
        std::vector<OrbiterDef> orbiters;
        double speed_scale{1.0};
        double mu_base{3.986004418e14}; // gravitational parameter (m^3/s^2), scaled by speed_scale^2
        WorldVec3 system_center{1.0e12, 0.0, 0.0};
    };

    ScenarioConfig default_earth_moon_config();
} // namespace Game
