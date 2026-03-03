#pragma once

#include "game/state/game_state.h"
#include "game/game_world.h"
#include "game/component/component.h"
#include "game/component/ship_controller.h"
#include "core/game_api.h"
#include "physics/physics_world.h"
#include "physics/physics_context.h"
#include "orbit_helpers.h"
#include "time_warp_state.h"
#include "orbitsim/trajectory_types.hpp"
#include "orbitsim/maneuvers_types.hpp"

#include <memory>
#include <deque>
#include <string>
#include <array>
#include <vector>
#include <limits>
#include <algorithm>

struct ImDrawList;

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

        void update_orbit_prediction_cache(const WorldVec3 &ship_pos_world,
                                          const glm::dvec3 &ship_vel_world,
                                          bool thrusting);
        void refresh_prediction_world_points();
        WorldVec3 prediction_reference_body_world() const;
        bool player_thrust_applied_this_tick() const;

        void emit_orbit_prediction_debug(GameStateContext &ctx);
        void emit_maneuver_node_debug_overlay(GameStateContext &ctx);

        void mark_prediction_dirty();

        void reset_time_warp_state();
        void handle_time_warp_input(GameStateContext &ctx);
        void set_time_warp_level(GameStateContext &ctx, int level);
        void enter_rails_warp(GameStateContext &ctx);
        void exit_rails_warp(GameStateContext &ctx);
        void rails_warp_step(GameStateContext &ctx, double dt_s);
        void sync_celestial_render_entities(GameStateContext &ctx);

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

        // Orbit prediction (UI + debug draw)
        struct OrbitPredictionCache
        {
            bool valid{false};
            double build_time_s{0.0};
            WorldVec3 build_pos_world{0.0, 0.0, 0.0};
            glm::dvec3 build_vel_world{0.0, 0.0, 0.0};

            // BCI = body-centered inertial (relative to the current reference body)
            std::vector<orbitsim::TrajectorySample> trajectory_bci;
            std::vector<orbitsim::TrajectorySample> trajectory_bci_planned;

            // Cached world-space polyline refreshed from trajectory_bci each frame.
            std::vector<WorldVec3> points_world;
            std::vector<WorldVec3> points_world_planned;
            std::vector<float> altitude_km;
            std::vector<float> speed_kmps;

            // Classical orbital-element-like values for HUD.
            double semi_major_axis_m{0.0};
            double eccentricity{0.0};
            double orbital_period_s{0.0};
            double periapsis_alt_km{0.0};
            double apoapsis_alt_km{std::numeric_limits<double>::infinity()};

            void clear()
            {
                valid = false;
                build_time_s = 0.0;
                build_pos_world = WorldVec3(0.0, 0.0, 0.0);
                build_vel_world = glm::dvec3(0.0, 0.0, 0.0);
                trajectory_bci.clear();
                trajectory_bci_planned.clear();
                points_world.clear();
                points_world_planned.clear();
                altitude_km.clear();
                speed_kmps.clear();
                semi_major_axis_m = 0.0;
                eccentricity = 0.0;
                orbital_period_s = 0.0;
                periapsis_alt_km = 0.0;
                apoapsis_alt_km = std::numeric_limits<double>::infinity();
            }
        };

        bool _prediction_enabled{true};
        bool _prediction_dirty{true};
        bool _prediction_draw_full_orbit{true};
        bool _prediction_draw_future_segment{true};
        bool _prediction_draw_velocity_ray{false};
        float _prediction_line_alpha_scale{1.0f};   // multiplier for orbit line alpha
        float _prediction_line_overlay_boost{0.0f}; // extra always-on-top alpha fraction
        double _prediction_periodic_refresh_s{0.0}; // 0 = never (cache is extended when horizon runs out)
        double _prediction_thrust_refresh_s{0.1};    // rebuild at most this often while thrusting
        double _prediction_future_window_s{600.0};
        OrbitPredictionCache _prediction_cache{};

        // Maneuver nodes (planning UI + prediction)
        struct ManeuverNode
        {
            int id{-1};                         // unique ID (stable)
            double time_s{0.0};                 // sim time (absolute)
            glm::dvec3 dv_rtn_mps{0.0, 0.0, 0.0}; // (Radial, Tangential/Prograde, Normal) [m/s]
            orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId}; // RTN primary body

            // Cached/derived values (updated by gameplay state)
            double total_dv_mps{0.0};
            WorldVec3 position_world{0.0, 0.0, 0.0};
            glm::dvec3 burn_direction_world{0.0, 0.0, 0.0};
            glm::dvec3 basis_r_world{1.0, 0.0, 0.0};
            glm::dvec3 basis_t_world{0.0, 1.0, 0.0};
            glm::dvec3 basis_n_world{0.0, 0.0, 1.0};
            float gizmo_scale_m{1.0f};
            bool gizmo_valid{false};
        };

        struct ManeuverPlanState
        {
            std::vector<ManeuverNode> nodes;
            int selected_node_id{-1};
            int next_node_id{0};

            ManeuverNode *find_node(const int id)
            {
                for (auto &n : nodes)
                {
                    if (n.id == id)
                    {
                        return &n;
                    }
                }
                return nullptr;
            }

            const ManeuverNode *find_node(const int id) const
            {
                for (const auto &n : nodes)
                {
                    if (n.id == id)
                    {
                        return &n;
                    }
                }
                return nullptr;
            }

            void sort_by_time()
            {
                std::sort(nodes.begin(), nodes.end(), [](const ManeuverNode &a, const ManeuverNode &b) {
                    return a.time_s < b.time_s;
                });
            }

            orbitsim::ManeuverPlan to_orbitsim_plan(const orbitsim::SpacecraftId sc_id) const
            {
                orbitsim::ManeuverPlan plan{};
                plan.impulses.reserve(nodes.size());

                for (const auto &node : nodes)
                {
                    orbitsim::ImpulseSegment imp{};
                    imp.t_s = node.time_s;
                    imp.primary_body_id = node.primary_body_id;
                    imp.dv_rtn_mps = orbitsim::Vec3{node.dv_rtn_mps.x, node.dv_rtn_mps.y, node.dv_rtn_mps.z};
                    imp.spacecraft_id = sc_id;
                    plan.impulses.push_back(imp);
                }

                orbitsim::sort_impulses_by_time(plan);
                return plan;
            }
        };

        enum class ManeuverHandleAxis
        {
            None = 0,
            Hub,
            ProgradePos,
            ProgradeNeg,
            RadialPos,
            RadialNeg,
            NormalPos,
            NormalNeg,
        };

        struct ManeuverGizmoStyle
        {
            glm::vec4 icon_color{0.30f, 0.80f, 1.00f, 0.90f};
            float icon_size_px{30.0f};
            float overlay_scale{1.0f};
            bool show_axis_labels{true};
            float drag_sensitivity_mps_per_m{0.0025f};
        };

        struct ManeuverGizmoInteraction
        {
            enum class State
            {
                Idle = 0,
                HoverAxis,
                DragAxis
            };

            State state{State::Idle};
            int node_id{-1};
            ManeuverHandleAxis axis{ManeuverHandleAxis::None};
            glm::dvec3 start_dv_rtn_mps{0.0, 0.0, 0.0};
            double start_axis_t_m{0.0};
            bool applied_delta{false};
        };

        struct ManeuverGizmoViewContext
        {
            glm::dvec3 camera_world{0.0, 0.0, 0.0};
            glm::dmat3 world_to_cam{1.0};
            double logical_w{0.0};
            double logical_h{0.0};
            double aspect{1.0};
            double tan_half_fov{0.0};
            VkRect2D letterbox_rect{};
            double draw_from_swap_x{1.0};
            double draw_from_swap_y{1.0};
            double window_from_draw_x{1.0};
            double window_from_draw_y{1.0};
            bool depth_occluder_valid{false};
            WorldVec3 depth_occluder_center{0.0, 0.0, 0.0};
            double depth_occluder_radius{0.0};
        };

        struct ManeuverHubMarker
        {
            int node_id{-1};
            glm::vec2 screen{0.0f, 0.0f};
            double depth_m{0.0};
        };

        struct ManeuverAxisMarker
        {
            int node_id{-1};
            ManeuverHandleAxis axis{ManeuverHandleAxis::None};
            glm::vec2 hub_screen{0.0f, 0.0f};
            glm::vec2 handle_screen{0.0f, 0.0f};
            uint32_t base_color{0};
            const char *label{""};
            double depth_m{0.0};
        };

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

        // Warp/execute helpers for maneuver nodes
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
