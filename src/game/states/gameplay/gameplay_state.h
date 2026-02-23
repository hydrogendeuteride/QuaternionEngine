#pragma once

#include "game/state/game_state.h"
#include "game/game_world.h"
#include "game/component/component.h"
#include "core/game_api.h"
#include "physics/physics_world.h"
#include "physics/physics_context.h"
#include "orbit_helpers.h"

#include <memory>
#include <deque>
#include <string>
#include <vector>

namespace Game
{
    // ============================================================================
    // ScenarioConfig — data-driven definition of a scenario (celestials + orbiters).
    // ============================================================================

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
            double orbit_distance_m{0.0}; // distance from reference body; 0 = this IS the reference body
            bool has_terrain{false};
            // Planet terrain rendering assets (only when has_terrain=true)
            std::string albedo_dir;
            std::string height_dir;
            double height_max_m{0.0};
            std::string emission_dir;
            glm::vec3 emission_factor{0.0f};
            float render_scale{1.0f}; // visual scale for non-terrain celestials
        };

        struct OrbiterDef
        {
            std::string name;
            double orbit_altitude_m{0.0};       // altitude above reference body surface
            glm::dvec3 offset_from_player{0.0}; // offset relative to player (non-player orbiters)
            glm::dvec3 relative_velocity{0.0};  // velocity relative to player
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

    // ============================================================================
    // GameplayState: Main gameplay — orbital mechanics, combat, ship control
    //
    // This is where the actual game simulation lives.
    // Owns GameWorld, physics, and orbital simulation.
    // ============================================================================

    class GameplayState : public IGameState
    {
    public:
        GameplayState();

        ~GameplayState() override;

        void on_enter(GameStateContext &ctx) override;

        void on_exit(GameStateContext &ctx) override;

        void on_update(GameStateContext &ctx, float dt) override;

        void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;

        void on_draw_ui(GameStateContext &ctx) override;

        bool wants_fixed_update() const override { return true; }
        const char *name() const override { return "Gameplay"; }

    private:
        void setup_scene(GameStateContext &ctx);
        void setup_environment(GameStateContext &ctx);
        void init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world);

        void step_physics(GameStateContext &ctx, float fixed_dt);
        void update_prediction(GameStateContext &ctx, float fixed_dt);

        ComponentContext build_component_context(GameStateContext &ctx, float alpha = 0.0f);

        bool get_player_world_state(WorldVec3 &out_pos_world,
                                    glm::dvec3 &out_vel_world,
                                    glm::vec3 &out_vel_local) const;

        void update_orbit_prediction_cache(const WorldVec3 &ship_pos_world, const glm::dvec3 &ship_vel_world);

        void emit_orbit_prediction_debug(GameStateContext &ctx);

        float effective_prediction_interval() const;

        // Orbiter helpers
        // player = first `is_player` orbiter (HUD/camera/prediction subject)
        const OrbiterInfo *find_player_orbiter() const;
        EntityId player_entity() const;
        EntityId select_rebase_anchor_entity() const;
        void update_rebase_anchor();

        // Game world (entities + resource lifetime)
        GameWorld _world;

        // Physics
        std::unique_ptr<Physics::PhysicsWorld> _physics;
        std::unique_ptr<Physics::PhysicsContext> _physics_context;
        enum class VelocityOriginMode
        {
            // Every fixed step: absorb anchor local velocity into velocity-origin.
            PerStepAnchorSync,

            // Integrate velocity-origin from anchor acceleration and run in anchor free-fall frame.
            FreeFallAnchorFrame
        };

        VelocityOriginMode _velocity_origin_mode{VelocityOriginMode::FreeFallAnchorFrame};

        // Scenario configuration
        ScenarioConfig _scenario_config;

        // Runtime entities
        std::vector<OrbiterInfo> _orbiters;

        // Orbital simulation
        std::unique_ptr<OrbitalScenario> _orbitsim;

        struct ContactLogEntry
        {
            float time_s{0.0f};
            Physics::ContactEventType type{Physics::ContactEventType::Begin};
            uint32_t self_body{0};
            uint32_t other_body{0};
            uint64_t self_user_data{0};
            uint64_t other_user_data{0};
            glm::vec3 point{0.0f};
            glm::vec3 normal{0.0f, 1.0f, 0.0f};
            float penetration_depth{0.0f};
        };

        std::deque<ContactLogEntry> _contact_log;
        size_t _contact_log_capacity{64};
        bool _contact_log_enabled{true};
        bool _contact_log_print_console{false};

        bool _debug_draw_enabled{true};
        bool _reset_requested{false};

        // Orbit prediction (UI + debug draw)
        bool _prediction_enabled{true};
        double _prediction_dt_s{1.0};
        double _prediction_horizon_s{180.0};
        float _prediction_update_interval_s{0.25f};
        float _prediction_update_accum_s{0.0f};
        float _prediction_debug_ttl_s{0.35f};

        std::vector<float> _prediction_altitude_km;
        std::vector<float> _prediction_speed_kmps;
        std::vector<WorldVec3> _prediction_points_world;

        // Timing
        float _elapsed{0.0f};
        double _fixed_time_s{0.0};
    };
} // namespace Game
