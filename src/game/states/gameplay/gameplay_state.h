#pragma once

#include "game/game_world.h"
#include "game/state/game_state.h"
#include "core/game_api.h"
#include "game/states/gameplay/maneuver/maneuver_system.h"
#include "game/states/gameplay/prediction/prediction_host_context.h"
#include "game/states/gameplay/prediction/prediction_system.h"
#include "game/states/gameplay/gameplay_settings.h"
#include "game/input/keybinds.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "orbital_runtime_system.h"
#include "frame_monitor.h"
#include "time_warp_state.h"
#include "physics/physics_body.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

struct ImDrawList;

namespace Physics
{
    class PhysicsContext;
    class PhysicsWorld;
    struct BodySettings;
} // namespace Physics

namespace orbitsim
{
    struct RotatingFrame;
} // namespace orbitsim

namespace Game
{
    class GameplayPredictionAdapter;
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

    public:
        static constexpr double kDefaultRuntimeOrbiterRailsDistanceM = 10'000.0;
        static constexpr double kRuntimeOrbiterRailsReturnDistanceRatio = 0.85;
        // Critically-damped spring controller (exact integration, stable at any dt).
        static constexpr double kFormationHoldOmega = 0.5;          // natural frequency [rad/s]
        static constexpr double kFormationHoldMaxDvPerStepMps = 20.0; // safety clamp on dv per tick

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
        GameplayPredictionState &prediction_for_test() { return _prediction->state(); }
        const GameplayPredictionState &prediction_for_test() const { return _prediction->state(); }
        PredictionSystem &prediction_system_for_test() { return *_prediction; }
        const PredictionSystem &prediction_system_for_test() const { return *_prediction; }
#endif

        // Settings and scene lifecycle
        GameplaySettings extract_settings() const;
        void apply_settings(const GameplaySettings &s);
        void setup_scene(GameStateContext &ctx);
        void setup_environment(GameStateContext &ctx);
        void init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world);

        // Simulation and time warp
        double current_sim_time_s() const;
        void step_physics(GameStateContext &ctx, float fixed_dt);
        ComponentContext build_component_context(GameStateContext &ctx, float alpha = 0.0f);
        void reset_time_warp_state();
        void handle_time_warp_input(GameStateContext &ctx);
        void set_time_warp_level(GameStateContext &ctx, int level);
        void enter_rails_warp(GameStateContext &ctx);
        void exit_rails_warp(GameStateContext &ctx);
        void rails_warp_step(GameStateContext &ctx, double dt_s);
        void update_runtime_orbiter_rails();
        void sync_runtime_orbiter_rails(double dt_s);
        bool promote_orbiter_to_rails(OrbiterInfo &orbiter);
        bool demote_orbiter_from_rails(OrbiterInfo &orbiter);
        void sync_celestial_render_entities(GameStateContext &ctx);
        Physics::BodyId create_orbiter_physics_body(bool render_is_gltf,
                                                    Entity &entity,
                                                    const Physics::BodySettings &settings,
                                                    const WorldVec3 &position_world,
                                                    const glm::quat &rotation,
                                                    glm::vec3 *out_origin_offset_local = nullptr);
        bool destroy_orbiter_physics_body(bool render_is_gltf, Entity &entity);

        // Orbiter adapters
        void update_rebase_anchor();
        bool set_active_player_orbiter(GameStateContext &ctx, EntityId entity);
        bool cycle_player_orbiter(GameStateContext &ctx, int direction);
        void sync_player_camera_target(GameStateContext &ctx) const;
        void sync_player_collision_callbacks();
        void update_formation_hold(double dt_s);

        // Prediction entry points
        PredictionHostContext build_prediction_host_context(const GameStateContext *ctx = nullptr) const;
        void update_prediction(GameStateContext &ctx, float fixed_dt);
        void draw_prediction(GameStateContext &ctx);
        void mark_prediction_dirty();
        void clear_prediction_runtime();
        // Maneuver adapters
        void emit_maneuver_node_debug_overlay(GameStateContext &ctx);
        bool maneuver_live_preview_active(bool with_maneuvers) const;
        int active_maneuver_preview_anchor_node_id() const;
        void begin_maneuver_node_dv_edit_preview(int node_id);
        void update_maneuver_node_dv_edit_preview(int node_id);
        void finish_maneuver_node_dv_edit_preview(bool changed);
        void begin_maneuver_node_time_edit_preview(int node_id, double previous_time_s);
        void update_maneuver_node_time_edit_preview(int node_id, double previous_time_s);
        void finish_maneuver_node_time_edit_preview(bool changed);
        void cancel_maneuver_node_edit_preview();
        void cancel_maneuver_node_dv_edit_preview();
        bool build_maneuver_gizmo_view_context(const GameStateContext &ctx, ManeuverGizmoViewContext &out_view) const;
        bool begin_maneuver_axis_drag(GameStateContext &ctx, int node_id, ManeuverHandleAxis axis);
        void apply_maneuver_axis_drag(GameStateContext &ctx, ManeuverNode &node, const glm::vec2 &mouse_pos_window);
        void build_maneuver_gizmo_markers(const ManeuverGizmoViewContext &view,
                                          float overlay_size_px,
                                          std::vector<ManeuverHubMarker> &out_hubs,
                                          std::vector<ManeuverAxisMarker> &out_handles) const;
        void draw_maneuver_gizmo_markers(ImDrawList *draw_list,
                                         const std::vector<ManeuverHubMarker> &hubs,
                                         const std::vector<ManeuverAxisMarker> &handles,
                                         int hovered_hub_idx,
                                         int hovered_handle_idx,
                                         float hub_hit_px,
                                         float axis_hit_px) const;
        void draw_maneuver_gizmo_hover_tooltip(const std::vector<ManeuverAxisMarker> &handles, int hovered_handle_idx) const;

        void update_maneuver_ui_config(GameStateContext &ctx);
        void draw_maneuver_nodes_panel(GameStateContext &ctx);
        void draw_maneuver_imgui_gizmo(GameStateContext &ctx);
        void draw_orbit_drag_debug_window(GameStateContext &ctx);
        void refresh_maneuver_node_runtime_cache(GameStateContext &ctx);
        void clear_maneuver_gizmo_instances(GameStateContext &ctx);
        void update_maneuver_nodes_time_warp(GameStateContext &ctx, float fixed_dt);
        void update_maneuver_nodes_execution(GameStateContext &ctx);
        orbitsim::BodyId resolve_maneuver_node_primary_body_id(const ManeuverNode &node, double query_time_s) const;
        ManeuverCommandResult apply_maneuver_command(const ManeuverCommand &command);
        void remove_maneuver_node(int node_id, int hint_index = -1);
        void remove_maneuver_node_suffix(int node_id, int hint_index = -1);
        WorldVec3 compute_maneuver_align_delta(GameStateContext &ctx,
                                                const OrbitPredictionCache &cache,
                                                const std::vector<orbitsim::TrajectorySample> &traj_base);

        // Owned state
        GameWorld _world;
        VulkanEngine *_renderer{nullptr};

        std::unique_ptr<Physics::PhysicsWorld> _physics;
        std::unique_ptr<Physics::PhysicsContext> _physics_context;
        enum class VelocityOriginMode
        {
            PerStepAnchorSync,
            FreeFallAnchorFrame
        };

        VelocityOriginMode _velocity_origin_mode{VelocityOriginMode::FreeFallAnchorFrame};

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
        bool _rails_warp_active{false};
        double _last_sim_step_dt_s{0.0};
        bool _rails_thrust_applied_this_tick{false};
        glm::vec3 _rails_last_thrust_dir_local{0.0f};
        glm::vec3 _rails_last_torque_dir_local{0.0f};
    };

    #undef VULKAN_ENGINE_GAMEPLAY_STATE_PRIVATE

} // namespace Game
