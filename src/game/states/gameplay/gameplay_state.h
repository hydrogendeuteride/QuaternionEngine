#pragma once

#include "game/state/game_state.h"
#include "game/game_world.h"
#include "game/component/component.h"
#include "physics/physics_world.h"
#include "physics/physics_context.h"

#include <memory>
#include <deque>
#include <string>
#include <vector>

namespace Game
{
    struct OrbitsimDemo;

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
        void init_orbitsim(double orbit_radius_m, double speed_scale,
                           WorldVec3 &ship_pos_world, glm::dvec3 &ship_vel_world,
                           WorldVec3 &moon_pos_world, bool &have_moon);

        void step_physics(GameStateContext &ctx, float fixed_dt);
        void update_prediction(GameStateContext &ctx, float fixed_dt);

        ComponentContext build_component_context(GameStateContext &ctx, float alpha = 0.0f);

        bool get_ship_world_state(WorldVec3 &out_pos_world,
                                  glm::dvec3 &out_vel_world,
                                  glm::vec3 &out_vel_local) const;

        void update_orbit_prediction_cache(const WorldVec3 &ship_pos_world, const glm::dvec3 &ship_vel_world);

        void emit_orbit_prediction_debug(GameStateContext &ctx);

        float effective_prediction_interval() const;

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

        // Scenario entities (early gameplay prototype)
        EntityId _ship_entity;
        EntityId _probe_entity;
        EntityId _moon_entity;

        // orbitsim (Earth-Moon)
        std::unique_ptr<OrbitsimDemo> _orbitsim;

        // Planet configuration (world-space, meters)
        std::string _planet_name{"earth"};
        WorldVec3 _planet_center_world{1.0e12, 0.0, 0.0};
        double _planet_radius_m{6'371'000.0};

        // Orbit configuration (meters / seconds)
        double _orbit_altitude_m{400'000.0};
        double _orbit_speed_scale{1.0}; // scales circular speed via mu *= scale^2
        double _moon_distance_m{384'400'000.0};

        // Earth gravitational parameter (m^3/s^2). We scale this to get faster orbits.
        double _mu_base_m3ps2{3.986004418e14};

        // Initial offset for the "probe" relative to the ship (meters, world axes).
        glm::dvec3 _probe_offset_world{0.0, -2.0, 30.0};

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
