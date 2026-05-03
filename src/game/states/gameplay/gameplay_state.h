#pragma once

#include "game/game_world.h"
#include "game/state/game_state.h"
#include "core/game_api.h"
#include "game/states/gameplay/maneuver/maneuver_system.h"
#include "game/states/gameplay/gameplay_settings.h"
#include "game/input/keybinds.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "orbital_runtime_system.h"
#include "orbital_physics_system.h"
#include "frame_monitor.h"
#include "time_warp_state.h"
#include "physics/physics_body.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class GameplayPredictionAdapter;
    class ManeuverPredictionBridge;
    class ManeuverUiController;
    class PredictionSystem;
    struct GameplayPredictionState;
    struct ManeuverCommand;
    struct ManeuverCommandResult;

    // ============================================================================
    // GameplayState: Main gameplay — orbital mechanics, combat, ship control
    //
    // This is where the actual game simulation lives.
    // Owns GameWorld, physics, and orbital simulation.
    // ============================================================================

    #if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
    #define VULKAN_ENGINE_GAMEPLAY_STATE_PRIVATE public
    #else
    #define VULKAN_ENGINE_GAMEPLAY_STATE_PRIVATE private
    #endif

    class GameplayState : public IGameState
    {
        friend class GameplayPredictionAdapter;
        friend class ManeuverPredictionBridge;
        friend class ManeuverUiController;

    public:
        GameplayState();
        explicit GameplayState(ScenarioConfig scenario_config);

        ~GameplayState() override;

        void on_enter(GameStateContext &ctx) override;

        void on_exit(GameStateContext &ctx) override;

        void on_update(GameStateContext &ctx, float dt) override;

        void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;

        void on_draw_ui(GameStateContext &ctx) override;

        bool wants_fixed_update() const override { return true; }
        const char *name() const override { return "Gameplay"; }

    VULKAN_ENGINE_GAMEPLAY_STATE_PRIVATE:
#if defined(VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS)
        GameplayPredictionState &prediction_for_test();
        const GameplayPredictionState &prediction_for_test() const;
        PredictionSystem &prediction_system_for_test();
        const PredictionSystem &prediction_system_for_test() const;
#endif

        // Settings and scene lifecycle
        GameplaySettings extract_settings() const;
        void apply_settings(const GameplaySettings &s);
        void setup_scene(GameStateContext &ctx);
        void setup_environment(GameStateContext &ctx);

        // Simulation and time warp
        double current_sim_time_s() const;
        ComponentContext build_component_context(GameStateContext &ctx, float alpha = 0.0f);
        void reset_time_warp_state();
        void handle_time_warp_input(GameStateContext &ctx);
        void set_time_warp_level(GameStateContext &ctx, int level);
        bool ui_capture_keyboard(const GameStateContext &ctx) const;

        // Orbiter adapters
        void update_rebase_anchor();
        bool set_active_player_orbiter(GameStateContext &ctx, EntityId entity);
        bool cycle_player_orbiter(GameStateContext &ctx, int direction);
        void sync_player_camera_target(GameStateContext &ctx) const;
        void sync_player_collision_callbacks();

        // Prediction entry points
        void update_prediction(GameStateContext &ctx, float fixed_dt);
        void draw_prediction(GameStateContext &ctx);
        void mark_prediction_dirty();
        void clear_prediction_runtime();

        // Maneuver adapters
        void draw_orbit_drag_debug_window(GameStateContext &ctx);
        void refresh_maneuver_node_runtime_cache(GameStateContext &ctx);
        void update_maneuver_nodes_time_warp(GameStateContext &ctx, float fixed_dt);
        void update_maneuver_nodes_execution(GameStateContext &ctx);
        ManeuverCommandResult apply_maneuver_command(const ManeuverCommand &command);

        // Owned state
        GameWorld _world;
        VulkanEngine *_renderer{nullptr};

        std::unique_ptr<Physics::PhysicsWorld> _physics;
        std::unique_ptr<Physics::PhysicsContext> _physics_context;

        bool _scenario_preloaded{false};
        ScenarioConfig _scenario_config;
        std::string _scenario_slot_rel_path{"scenarios/user_gameplay.json"};
        std::string _settings_rel_path{"settings/gameplay.json"};
        std::string _settings_io_status{};
        bool _settings_io_status_ok{true};

        // Outline settings stash: GameplayState applies its own scope on enter, restores on exit.
        GameAPI::Engine::OutlineSettings _saved_outline_settings{};
        bool _outline_settings_saved{false};

        // Input keybindings (TOML). Loaded on_enter; starts from struct defaults.
        Keybinds _keybinds{};
        std::string _keybinds_rel_path{"settings/keybinds.toml"};
        std::string _scenario_io_status{};
        bool _scenario_io_status_ok{true};

        OrbitalRuntimeSystem _orbit;
        OrbitalPhysicsSystem _orbital_physics;

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
        bool _show_orbit_hud{true};
        bool _show_orbit_drag_debug{true};
        bool _show_frame_view{true};
        bool _show_maneuver_nodes_panel{false};
        bool _reset_requested{false};
        std::unique_ptr<PredictionSystem> _prediction;
        ManeuverSystem _maneuver{};

        float _elapsed{0.0f};
        double _fixed_time_s{0.0};

        TimeWarpState _time_warp{};
        FrameMonitor _frame_monitor{};
    };

    #undef VULKAN_ENGINE_GAMEPLAY_STATE_PRIVATE

} // namespace Game
