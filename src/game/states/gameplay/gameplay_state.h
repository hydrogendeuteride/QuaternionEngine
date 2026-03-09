#pragma once

#include "game/game_world.h"
#include "game/state/game_state.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "orbit_helpers.h"
#include "time_warp_state.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct ImDrawList;

namespace Game
{
    // ============================================================================
    // GameplayState: Main gameplay — orbital mechanics, combat, ship control
    //
    // This is where the actual game simulation lives.
    // Owns GameWorld, physics, and orbital simulation.
    // ============================================================================

    class GameplayState : public IGameState
    {
    public:
        static constexpr double kDefaultRuntimeOrbiterRailsDistanceM = 10'000.0;
        static constexpr double kRuntimeOrbiterRailsReturnDistanceRatio = 0.85;

        // Compatibility aliases for gameplay internals referenced by tests/tools.
        using OrbitPredictionCache = Game::OrbitPredictionCache;
        using OrbitPlotPerfStats = Game::OrbitPlotPerfStats;
        using PredictionSubjectKey = Game::PredictionSubjectKey;
        using PredictionSubjectKind = Game::PredictionSubjectKind;
        using PredictionTrackState = Game::PredictionTrackState;
        using PredictionGroup = Game::PredictionGroup;
        using PredictionSelectionState = Game::PredictionSelectionState;
        using ManeuverNode = Game::ManeuverNode;
        using ManeuverPlanState = Game::ManeuverPlanState;
        using ManeuverHandleAxis = Game::ManeuverHandleAxis;
        using ManeuverGizmoStyle = Game::ManeuverGizmoStyle;
        using ManeuverGizmoInteraction = Game::ManeuverGizmoInteraction;
        using ManeuverGizmoViewContext = Game::ManeuverGizmoViewContext;
        using ManeuverHubMarker = Game::ManeuverHubMarker;
        using ManeuverAxisMarker = Game::ManeuverAxisMarker;

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
        // Scene bootstrap
        void setup_scene(GameStateContext &ctx);
        void setup_environment(GameStateContext &ctx);
        void init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world);

        // Simulation
        void step_physics(GameStateContext &ctx, float fixed_dt);
        ComponentContext build_component_context(GameStateContext &ctx, float alpha = 0.0f);
        bool get_player_world_state(WorldVec3 &out_pos_world,
                                    glm::dvec3 &out_vel_world,
                                    glm::vec3 &out_vel_local) const;
        bool get_orbiter_world_state(const OrbiterInfo &orbiter,
                                     WorldVec3 &out_pos_world,
                                     glm::dvec3 &out_vel_world,
                                     glm::vec3 &out_vel_local) const;
        bool get_prediction_subject_world_state(PredictionSubjectKey key,
                                                WorldVec3 &out_pos_world,
                                                glm::dvec3 &out_vel_world,
                                                glm::vec3 &out_vel_local) const;

        // Orbit prediction
        void update_prediction(GameStateContext &ctx, float fixed_dt);
        void clear_prediction_runtime();
        void clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects);
        void apply_completed_prediction_result(OrbitPredictionService::Result result);
        bool should_rebuild_prediction_track(const PredictionTrackState &track,
                                             double now_s,
                                             float fixed_dt,
                                             bool thrusting,
                                             bool with_maneuvers) const;
        bool request_orbiter_prediction_async(PredictionTrackState &track,
                                              const WorldVec3 &subject_pos_world,
                                              const glm::dvec3 &subject_vel_world,
                                              double now_s,
                                              bool thrusting,
                                              bool with_maneuvers);
        bool request_celestial_prediction_async(PredictionTrackState &track,
                                                double now_s);
        void update_orbiter_prediction_track(PredictionTrackState &track,
                                             double now_s,
                                             bool thrusting,
                                             bool with_maneuvers);
        void update_celestial_prediction_track(PredictionTrackState &track,
                                               double now_s);
        void refresh_prediction_world_points(PredictionTrackState &track);
        WorldVec3 prediction_reference_body_world() const;
        bool prediction_subject_thrust_applied_this_tick(PredictionSubjectKey key) const;
        void rebuild_prediction_subjects();
        void sync_prediction_dirty_flag();
        std::vector<PredictionSubjectKey> collect_visible_prediction_subjects() const;
        double prediction_future_window_s(PredictionSubjectKey key) const;
        PredictionTrackState *find_prediction_track(PredictionSubjectKey key);
        const PredictionTrackState *find_prediction_track(PredictionSubjectKey key) const;
        PredictionTrackState *active_prediction_track();
        const PredictionTrackState *active_prediction_track() const;
        PredictionTrackState *player_prediction_track();
        const PredictionTrackState *player_prediction_track() const;
        OrbitPredictionCache *player_prediction_cache();
        const OrbitPredictionCache *player_prediction_cache() const;
        bool prediction_subject_is_player(PredictionSubjectKey key) const;
        bool prediction_subject_supports_maneuvers(PredictionSubjectKey key) const;
        std::string prediction_subject_label(PredictionSubjectKey key) const;

        void emit_orbit_prediction_debug(GameStateContext &ctx);
        void emit_maneuver_node_debug_overlay(GameStateContext &ctx);
        void mark_prediction_dirty();

        // Time warp
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

        // Orbiter helpers
        const OrbiterInfo *find_player_orbiter() const;
        const OrbiterInfo *find_orbiter(EntityId entity) const;
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
            PerStepAnchorSync,
            FreeFallAnchorFrame
        };

        VelocityOriginMode _velocity_origin_mode{VelocityOriginMode::FreeFallAnchorFrame};

        // Scenario configuration
        ScenarioConfig _scenario_config;
        std::string _scenario_slot_rel_path{"scenarios/user_gameplay.json"};
        std::string _scenario_io_status{};
        bool _scenario_io_status_ok{true};

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
        bool _runtime_orbiter_rails_enabled{true};
        double _runtime_orbiter_rails_distance_m{kDefaultRuntimeOrbiterRailsDistanceM};

        // Orbit prediction (UI + debug draw)
        bool _prediction_enabled{true};
        bool _prediction_dirty{true};
        bool _prediction_draw_full_orbit{true};
        bool _prediction_draw_future_segment{true};
        bool _prediction_draw_velocity_ray{false};
        float _prediction_line_alpha_scale{1.0f};   // multiplier for orbit line alpha
        float _prediction_line_overlay_boost{0.0f}; // extra always-on-top alpha fraction
        double _prediction_periodic_refresh_s{0.0}; // 0 = never (cache is extended when horizon runs out)
        double _prediction_thrust_refresh_s{0.1};    // rebuild at most this often while thrusting
        double _prediction_future_window_orbiter_s{600.0};
        double _prediction_future_window_celestial_s{21600.0};
        double _orbit_plot_render_error_px{0.75};
        int _orbit_plot_render_max_segments_cpu{4'000};
        int _orbit_plot_pick_max_segments{8'000};
        double _orbit_plot_pick_frustum_margin_ratio{0.05};

        OrbitPlotPerfStats _orbit_plot_perf{};
        OrbitPredictionDrawConfig _prediction_draw_config{};
        OrbitPredictionService _prediction_service{};
        std::vector<PredictionTrackState> _prediction_tracks{};
        std::vector<PredictionGroup> _prediction_groups{};
        PredictionSelectionState _prediction_selection{};

        bool build_maneuver_gizmo_view_context(const GameStateContext &ctx, ManeuverGizmoViewContext &out_view) const;
        bool maneuver_gizmo_is_occluded(const ManeuverGizmoViewContext &view, const WorldVec3 &point_world) const;
        bool project_maneuver_gizmo_point(const ManeuverGizmoViewContext &view,
                                          const WorldVec3 &point_world,
                                          glm::vec2 &out_screen,
                                          double &out_depth_m) const;
        bool resolve_maneuver_axis(const ManeuverNode &node,
                                   ManeuverHandleAxis axis,
                                   glm::dvec3 &out_axis_dir_world,
                                   int &out_component,
                                   double &out_sign) const;
        const char *maneuver_axis_label(ManeuverHandleAxis axis) const;
        uint32_t maneuver_axis_color(ManeuverHandleAxis axis) const;
        bool begin_maneuver_axis_drag(GameStateContext &ctx, int node_id, ManeuverHandleAxis axis);
        void apply_maneuver_axis_drag(GameStateContext &ctx, ManeuverNode &node, const glm::vec2 &mouse_pos_window);
        void build_maneuver_gizmo_markers(const ManeuverGizmoViewContext &view,
                                          float overlay_size_px,
                                          std::vector<ManeuverHubMarker> &out_hubs,
                                          std::vector<ManeuverAxisMarker> &out_handles) const;
        void find_maneuver_gizmo_hover(const std::vector<ManeuverHubMarker> &hubs,
                                       const std::vector<ManeuverAxisMarker> &handles,
                                       const glm::vec2 &mouse_pos,
                                       float hub_hit_px2,
                                       float axis_hit_px2,
                                       int &out_hovered_hub_idx,
                                       int &out_hovered_handle_idx) const;
        void draw_maneuver_gizmo_markers(ImDrawList *draw_list,
                                         const std::vector<ManeuverHubMarker> &hubs,
                                         const std::vector<ManeuverAxisMarker> &handles,
                                         int hovered_hub_idx,
                                         int hovered_handle_idx,
                                         float hub_hit_px,
                                         float axis_hit_px) const;
        void draw_maneuver_gizmo_hover_tooltip(const std::vector<ManeuverAxisMarker> &handles, int hovered_handle_idx) const;

        void draw_maneuver_nodes_panel(GameStateContext &ctx);
        void draw_maneuver_imgui_gizmo(GameStateContext &ctx);
        void refresh_maneuver_node_runtime_cache(GameStateContext &ctx);
        void clear_maneuver_gizmo_instances(GameStateContext &ctx);
        void update_maneuver_nodes_time_warp(GameStateContext &ctx, float fixed_dt);
        void update_maneuver_nodes_execution(GameStateContext &ctx);

        bool _maneuver_nodes_enabled{true};
        bool _maneuver_nodes_debug_draw{true};
        double _maneuver_timeline_window_s{3600.0};
        ManeuverPlanState _maneuver_state{};
        ManeuverGizmoStyle _maneuver_gizmo_style{};
        ManeuverGizmoInteraction _maneuver_gizmo_interaction{};

        // Maneuver-node warp/execute helpers
        bool _warp_to_time_active{false};
        double _warp_to_time_target_s{0.0};
        int _warp_to_time_restore_level{0};

        bool _execute_node_armed{false};
        int _execute_node_id{-1};

        // Timing
        float _elapsed{0.0f};
        double _fixed_time_s{0.0};

        TimeWarpState _time_warp{};
        bool _rails_warp_active{false};
        double _last_sim_step_dt_s{0.0};
        bool _rails_thrust_applied_this_tick{false};
        glm::vec3 _rails_last_thrust_dir_local{0.0f};
        glm::vec3 _rails_last_torque_dir_local{0.0f};
    };
} // namespace Game
