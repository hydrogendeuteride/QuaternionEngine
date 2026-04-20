#pragma once

#include "game/game_world.h"
#include "game/state/game_state.h"
#include "core/game_api.h"
#include "game/states/gameplay/maneuver/gameplay_state_maneuver_types.h"
#include "game/states/gameplay/prediction/gameplay_prediction_derived_service.h"
#include "game/states/gameplay/prediction/gameplay_state_prediction_types.h"
#include "game/states/gameplay/gameplay_settings.h"
#include "game/input/keybinds.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "orbit_helpers.h"
#include "frame_monitor.h"
#include "time_warp_state.h"
#include "physics/physics_context.h"
#include "physics/physics_world.h"
#include "orbitsim/coordinate_frames.hpp"
#include "orbitsim/spacecraft_lookup.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

struct ImDrawList;

namespace Game
{
    namespace PredictionDrawDetail
    {
        struct PredictionGlobalDrawContext;
        struct PredictionTrackDrawContext;
    }

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
        // Critically-damped spring controller (exact integration, stable at any dt).
        static constexpr double kFormationHoldOmega = 0.5;          // natural frequency [rad/s]
        static constexpr double kFormationHoldMaxDvPerStepMps = 20.0; // safety clamp on dv per tick

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
        using ManeuverGizmoBasisMode = Game::ManeuverGizmoBasisMode;
        using ManeuverGizmoStyle = Game::ManeuverGizmoStyle;
        using ManeuverGizmoInteraction = Game::ManeuverGizmoInteraction;
        using ManeuverGizmoViewContext = Game::ManeuverGizmoViewContext;
        using ManeuverHubMarker = Game::ManeuverHubMarker;
        using ManeuverAxisMarker = Game::ManeuverAxisMarker;

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

    private:
        // Settings
        GameplaySettings extract_settings() const;
        void apply_settings(const GameplaySettings &s);

        // Scene bootstrap
        void setup_scene(GameStateContext &ctx);
        void setup_environment(GameStateContext &ctx);
        void init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world);

        // Simulation
        double current_sim_time_s() const;
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
        void poll_completed_prediction_results();
        void update_prediction(GameStateContext &ctx, float fixed_dt);
        void clear_prediction_runtime();
        void clear_visible_prediction_runtime(const std::vector<PredictionSubjectKey> &visible_subjects);
        void apply_completed_prediction_result(OrbitPredictionService::Result result);
        void apply_completed_prediction_derived_result(OrbitPredictionDerivedService::Result result);
        bool should_rebuild_prediction_track(const PredictionTrackState &track,
                                             double now_s,
                                             float fixed_dt,
                                             bool thrusting,
                                             bool with_maneuvers) const;
        void rebuild_prediction_frame_options();
        bool set_prediction_frame_spec(const orbitsim::TrajectoryFrameSpec &spec);
        orbitsim::TrajectoryFrameSpec default_prediction_frame_spec() const;
        void rebuild_prediction_analysis_options();
        bool set_prediction_analysis_spec(const PredictionAnalysisSpec &spec);
        bool request_orbiter_prediction_async(PredictionTrackState &track,
                                              const WorldVec3 &subject_pos_world,
                                              const glm::dvec3 &subject_vel_world,
                                              double now_s,
                                              bool thrusting,
                                              bool with_maneuvers,
                                              bool *out_throttled = nullptr);
        bool build_orbiter_prediction_request(PredictionTrackState &track,
                                              const WorldVec3 &subject_pos_world,
                                              const glm::dvec3 &subject_vel_world,
                                              double now_s,
                                              bool thrusting,
                                              bool with_maneuvers,
                                              OrbitPredictionService::Request &out_request,
                                              bool *out_interactive_request = nullptr,
                                              bool *out_preview_request_active = nullptr);
        bool request_celestial_prediction_async(PredictionTrackState &track,
                                                double now_s);
        void update_orbiter_prediction_track(PredictionTrackState &track,
                                             double now_s,
                                             bool thrusting,
                                             bool with_maneuvers);
        void update_celestial_prediction_track(PredictionTrackState &track,
                                               double now_s);
        WorldVec3 prediction_reference_body_world() const;
        bool prediction_subject_thrust_applied_this_tick(PredictionSubjectKey key) const;
        void rebuild_prediction_subjects();
        void sync_prediction_dirty_flag();
        std::vector<PredictionSubjectKey> collect_visible_prediction_subjects() const;
        double prediction_future_window_s(PredictionSubjectKey key) const;
        double maneuver_plan_horizon_s() const;
        PredictionTimeContext build_prediction_time_context(
                PredictionSubjectKey key,
                double sim_now_s,
                double trajectory_t0_s = std::numeric_limits<double>::quiet_NaN(),
                double trajectory_t1_s = std::numeric_limits<double>::quiet_NaN()) const;
        PredictionWindowPolicyResult resolve_prediction_window_policy(
                const PredictionTrackState *track,
                const PredictionTimeContext &time_ctx,
                bool with_maneuvers) const;
        double prediction_display_window_s(PredictionSubjectKey key,
                                           double now_s,
                                           bool with_maneuvers) const;
        void refresh_prediction_preview_anchor(PredictionTrackState &track, double now_s, bool with_maneuvers);
        double prediction_preview_exact_window_s(const PredictionTrackState &track,
                                                 double now_s,
                                                 bool with_maneuvers) const;
        double prediction_planned_exact_window_s(const PredictionTrackState &track,
                                                 double now_s,
                                                 bool with_maneuvers) const;
        double prediction_required_window_s(const PredictionTrackState &track,
                                            double now_s,
                                            bool with_maneuvers) const;
        double prediction_required_window_s(PredictionSubjectKey key,
                                            double now_s,
                                            bool with_maneuvers) const;
        PredictionTrackState *find_prediction_track(PredictionSubjectKey key);
        const PredictionTrackState *find_prediction_track(PredictionSubjectKey key) const;
        PredictionTrackState *active_prediction_track();
        const PredictionTrackState *active_prediction_track() const;
        PredictionTrackState *player_prediction_track();
        const PredictionTrackState *player_prediction_track() const;
        OrbitPredictionCache *effective_prediction_cache(PredictionTrackState *track);
        const OrbitPredictionCache *effective_prediction_cache(const PredictionTrackState *track) const;
        OrbitPredictionCache *player_prediction_cache();
        const OrbitPredictionCache *player_prediction_cache() const;
        bool prediction_subject_is_player(PredictionSubjectKey key) const;
        bool prediction_subject_supports_maneuvers(PredictionSubjectKey key) const;
        std::string prediction_subject_label(PredictionSubjectKey key) const;
        glm::vec3 prediction_subject_orbit_rgb(PredictionSubjectKey key) const;
        const CelestialBodyInfo *find_celestial_body_info(orbitsim::BodyId body_id) const;
        const orbitsim::MassiveBody *find_massive_body(const std::vector<orbitsim::MassiveBody> &bodies,
                                                       orbitsim::BodyId body_id) const;
        WorldVec3 prediction_body_world_position(orbitsim::BodyId body_id,
                                                 const OrbitPredictionCache *cache = nullptr,
                                                 double query_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool sample_prediction_inertial_state(const std::vector<orbitsim::TrajectorySample> &trajectory,
                                              double query_time_s,
                                              orbitsim::State &out_state) const;
        bool resolve_prediction_preview_anchor_state(const PredictionTrackState &track,
                                                     orbitsim::State &out_state) const;
        orbitsim::SpacecraftStateLookup build_prediction_player_lookup() const;
        orbitsim::TrajectoryFrameSpec resolve_prediction_display_frame_spec(
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool build_prediction_display_frame(const OrbitPredictionCache &cache,
                                            orbitsim::RotatingFrame &out_frame,
                                            double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        double resolve_prediction_display_reference_time_s(
                const OrbitPredictionCache &cache,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool build_prediction_display_transform(const OrbitPredictionCache &cache,
                                               WorldVec3 &out_origin_world,
                                               glm::dmat3 &out_frame_to_world,
                                               double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        WorldVec3 prediction_sample_position_world(const OrbitPredictionCache &cache,
                                                   const orbitsim::TrajectorySample &sample,
                                                   double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        WorldVec3 prediction_sample_hermite_world(const OrbitPredictionCache &cache,
                                                  const orbitsim::TrajectorySample &a,
                                                  const orbitsim::TrajectorySample &b,
                                                  double t_s,
                                                  double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        bool prediction_frame_is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec) const;
        orbitsim::BodyId select_prediction_primary_body_id(const std::vector<orbitsim::MassiveBody> &bodies,
                                                           const OrbitPredictionCache *cache,
                                                           const orbitsim::Vec3 &query_pos_m,
                                                           double query_time_s,
                                                           orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId) const;
        orbitsim::BodyId resolve_prediction_analysis_body_id(const OrbitPredictionCache &cache,
                                                             PredictionSubjectKey key,
                                                             double query_time_s,
                                                             orbitsim::BodyId preferred_body_id = orbitsim::kInvalidBodyId) const;
        WorldVec3 prediction_world_reference_body_world() const;
        WorldVec3 prediction_frame_origin_world(const OrbitPredictionCache &cache,
                                                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        void mark_prediction_derived_request_submitted(
                PredictionTrackState &track,
                const OrbitPredictionDerivedService::Request &request);
        bool request_prediction_derived_refresh(
                PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN());
        bool prediction_track_has_current_derived_cache(
                const PredictionTrackState &track,
                double display_time_s = std::numeric_limits<double>::quiet_NaN()) const;
        void refresh_prediction_derived_cache(PredictionTrackState &track,
                                              double display_time_s = std::numeric_limits<double>::quiet_NaN());
        void refresh_all_prediction_derived_caches();
        bool build_orbit_prediction_global_draw_context(
                GameStateContext &ctx,
                PredictionDrawDetail::PredictionGlobalDrawContext &out);
        bool build_orbit_prediction_track_draw_context(
                PredictionTrackState &track,
                const PredictionDrawDetail::PredictionGlobalDrawContext &global_ctx,
                PredictionDrawDetail::PredictionTrackDrawContext &out);
        void draw_orbit_prediction_track_windows(PredictionDrawDetail::PredictionTrackDrawContext &track_ctx);
        void emit_orbit_prediction_track_picks(
                const PredictionDrawDetail::PredictionGlobalDrawContext &global_ctx,
                PredictionDrawDetail::PredictionTrackDrawContext &track_ctx);

        void emit_orbit_prediction_debug(GameStateContext &ctx);
        void emit_maneuver_node_debug_overlay(GameStateContext &ctx);
        void mark_prediction_dirty();
        void mark_maneuver_plan_dirty();

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
        Physics::BodyId create_orbiter_physics_body(bool render_is_gltf,
                                                    Entity &entity,
                                                    const Physics::BodySettings &settings,
                                                    const WorldVec3 &position_world,
                                                    const glm::quat &rotation,
                                                    glm::vec3 *out_origin_offset_local = nullptr);
        bool destroy_orbiter_physics_body(bool render_is_gltf, Entity &entity);

        // Orbiter helpers
        OrbiterInfo *find_player_orbiter();
        const OrbiterInfo *find_player_orbiter() const;
        OrbiterInfo *find_orbiter(EntityId entity);
        const OrbiterInfo *find_orbiter(EntityId entity) const;
        OrbiterInfo *find_orbiter(std::string_view name);
        const OrbiterInfo *find_orbiter(std::string_view name) const;
        EntityId player_entity() const;
        EntityId select_rebase_anchor_entity() const;
        void update_rebase_anchor();
        bool set_active_player_orbiter(GameStateContext &ctx, EntityId entity);
        bool cycle_player_orbiter(GameStateContext &ctx, int direction);
        void sync_player_camera_target(GameStateContext &ctx) const;
        void sync_player_collision_callbacks();
        bool get_orbiter_inertial_state(const OrbiterInfo &orbiter, orbitsim::State &out_state) const;
        const orbitsim::MassiveBody *select_primary_body_for_state(const orbitsim::State &state) const;
        bool build_orbiter_lvlh_frame(const OrbiterInfo &leader,
                                      orbitsim::RotatingFrame &out_frame,
                                      orbitsim::State *out_leader_state = nullptr,
                                      orbitsim::State *out_primary_state = nullptr) const;
        void update_formation_hold(double dt_s);

        // Game world (entities + resource lifetime)
        GameWorld _world;
        VulkanEngine *_renderer{nullptr};

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
        bool _show_orbit_hud{true};
        bool _show_orbit_drag_debug{true};
        bool _show_frame_view{true};
        bool _show_maneuver_nodes_panel{false};
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
        PredictionSamplingPolicy _prediction_sampling_policy{};
        ManeuverPlanHorizonSettings _maneuver_plan_horizon{};
        ManeuverPlanWindowSettings _maneuver_plan_windows{};
        OrbitPlotBudgetSettings _orbit_plot_budget{};
        bool _maneuver_plan_live_preview_active{true};

        OrbitPlotPerfStats _orbit_plot_perf{};
        OrbitPredictionDrawConfig _prediction_draw_config{};
        OrbitPredictionService _prediction_service{};
        OrbitPredictionDerivedService _prediction_derived_service{};
        std::vector<PredictionTrackState> _prediction_tracks{};
        std::vector<PredictionGroup> _prediction_groups{};
        PredictionSelectionState _prediction_selection{};
        PredictionFrameSelectionState _prediction_frame_selection{};
        uint64_t _prediction_display_frame_revision{1};
        PredictionAnalysisSelectionState _prediction_analysis_selection{};

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

        void update_maneuver_ui_config(GameStateContext &ctx);
        void draw_maneuver_nodes_panel(GameStateContext &ctx);
        void draw_maneuver_imgui_gizmo(GameStateContext &ctx);
        void draw_orbit_drag_debug_window(GameStateContext &ctx);
        void refresh_maneuver_node_runtime_cache(GameStateContext &ctx);
        void clear_maneuver_gizmo_instances(GameStateContext &ctx);
        void update_maneuver_nodes_time_warp(GameStateContext &ctx, float fixed_dt);
        void update_maneuver_nodes_execution(GameStateContext &ctx);
        orbitsim::BodyId resolve_maneuver_node_primary_body_id(const ManeuverNode &node, double query_time_s) const;
        void remove_maneuver_node(int node_id, int hint_index = -1);
        void remove_maneuver_node_suffix(int node_id, int hint_index = -1);
        void finalize_maneuver_node_removal(bool removed_selected, bool removed_gizmo, bool removed_execute, int hint_index);
        WorldVec3 compute_maneuver_align_delta(GameStateContext &ctx,
                                               const OrbitPredictionCache &cache,
                                               const std::vector<orbitsim::TrajectorySample> &traj_base);

        bool _maneuver_nodes_enabled{true};
        bool _maneuver_nodes_debug_draw{true};
        double _maneuver_timeline_window_s{3600.0};
        ManeuverPlanState _maneuver_state{};
        ManeuverGizmoBasisMode _maneuver_gizmo_basis_mode{ManeuverGizmoBasisMode::ProgradeOutwardNormal};
        ManeuverGizmoStyle _maneuver_gizmo_style{};
        ManeuverUIConfig _maneuver_ui_config{};
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
        FrameMonitor _frame_monitor{};
        bool _rails_warp_active{false};
        double _last_sim_step_dt_s{0.0};
        bool _rails_thrust_applied_this_tick{false};
        glm::vec3 _rails_last_thrust_dir_local{0.0f};
        glm::vec3 _rails_last_torque_dir_local{0.0f};
    };
} // namespace Game
