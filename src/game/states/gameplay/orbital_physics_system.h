#pragma once

#include "game/states/gameplay/formation_hold_system.h"

#include <glm/glm.hpp>

#include <functional>

class VulkanEngine;

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameWorld;
    class OrbitalRuntimeSystem;
    struct GameStateContext;
    struct Keybinds;
    struct OrbiterInfo;
    struct ScenarioConfig;

    class OrbitalPhysicsSystem
    {
    public:
        enum class VelocityOriginMode
        {
            PerStepAnchorSync,
            FreeFallAnchorFrame
        };

        struct Context
        {
            VulkanEngine *renderer{nullptr};
            GameWorld &world;
            OrbitalRuntimeSystem &orbit;
            Physics::PhysicsWorld *physics{nullptr};
            Physics::PhysicsContext *physics_context{nullptr};
            const ScenarioConfig &scenario_config;
            const Keybinds *keybinds{nullptr};
            FormationHoldSystem::OrbiterWorldStateSampler orbiter_world_state_sampler{};
            std::function<bool(const GameStateContext &)> ui_capture_keyboard{};
            std::function<void()> mark_prediction_dirty{};
        };

        void reset();

        [[nodiscard]] bool rails_warp_active() const { return _rails_warp_active; }
        [[nodiscard]] bool rails_thrust_applied_this_tick() const { return _rails_thrust_applied_this_tick; }
        [[nodiscard]] const glm::vec3 &rails_last_thrust_dir_local() const { return _rails_last_thrust_dir_local; }
        [[nodiscard]] const glm::vec3 &rails_last_torque_dir_local() const { return _rails_last_torque_dir_local; }

        [[nodiscard]] double last_sim_step_dt_s() const { return _last_sim_step_dt_s; }
        void set_last_sim_step_dt_s(double dt_s) { _last_sim_step_dt_s = dt_s; }

        [[nodiscard]] VelocityOriginMode velocity_origin_mode() const { return _velocity_origin_mode; }
        void set_velocity_origin_mode(VelocityOriginMode mode) { _velocity_origin_mode = mode; }

        void sync_celestial_render_entities(const Context &context, GameStateContext &ctx);
        void update_runtime_orbiter_rails(const Context &context);
        void sync_runtime_orbiter_rails(const Context &context, double dt_s);
        bool promote_orbiter_to_rails(const Context &context, OrbiterInfo &orbiter);
        bool demote_orbiter_from_rails(const Context &context, OrbiterInfo &orbiter);
        void update_formation_hold(const Context &context, double dt_s);
        void step_physics(const Context &context, GameStateContext &ctx, float fixed_dt);
        bool enter_rails_warp(const Context &context, GameStateContext &ctx);
        void exit_rails_warp(const Context &context, GameStateContext &ctx);
        void rails_warp_step(const Context &context, GameStateContext &ctx, double dt_s);

#if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
        void set_rails_warp_active_for_test(bool active) { _rails_warp_active = active; }
#endif

    private:
        static constexpr double kRuntimeOrbiterRailsReturnDistanceRatio = 0.85;
        static constexpr double kFormationHoldOmega = 0.5;
        static constexpr double kFormationHoldMaxDvPerStepMps = 20.0;

        VelocityOriginMode _velocity_origin_mode{VelocityOriginMode::FreeFallAnchorFrame};
        bool _rails_warp_active{false};
        double _last_sim_step_dt_s{0.0};
        bool _rails_thrust_applied_this_tick{false};
        glm::vec3 _rails_last_thrust_dir_local{0.0f};
        glm::vec3 _rails_last_torque_dir_local{0.0f};
    };
} // namespace Game
