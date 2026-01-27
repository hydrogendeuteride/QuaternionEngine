#pragma once

#include "runtime/i_game_callbacks.h"
#include "game_world.h"
#include "physics/physics_world.h"
#include "physics/physics_context.h"
#include "core/world.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Game
{
    // ============================================================================
    // RebasingTestGame: Stress-test for floating origin + velocity rebasing.
    //
    // Spawns an Earth-sized terrain planet at a very large world coordinate and
    // simulates one (or more) bodies in a fast circular orbit. A selected body is
    // used as the physics-origin + velocity-origin anchor via GameWorld rebasing.
    // ============================================================================

    class RebasingTestGame final : public GameRuntime::IGameCallbacks
    {
    public:
        RebasingTestGame() = default;
        ~RebasingTestGame() override = default;

        void on_init(GameRuntime::Runtime &runtime) override;
        void on_update(float dt) override;
        void on_fixed_update(float fixed_dt) override;
        void on_shutdown() override;

    private:
        void setup_scene();
        void teardown_scene();
        void apply_rebase_settings();

        void draw_ui();
        void draw_debug();

        GameRuntime::Runtime *_runtime{nullptr};

        GameWorld _world;

        std::unique_ptr<Physics::PhysicsWorld> _physics;
        std::unique_ptr<Physics::PhysicsContext> _physics_context;

        EntityId _ship_entity;
        EntityId _probe_entity;

        // Planet configuration (world-space, meters)
        std::string _planet_name{"earth"};
        WorldVec3 _planet_center_world{1.0e12, 0.0, 0.0};
        double _planet_radius_m{6'371'000.0};

        // Orbit configuration (meters / seconds)
        double _orbit_altitude_m{400'000.0};
        double _orbit_speed_scale{10.0}; // scales circular speed via mu *= scale^2

        // Earth gravitational parameter (m^3/s^2). We scale this to get faster orbits.
        double _mu_base_m3ps2{3.986004418e14};

        // Initial offset for the "probe" relative to the ship (meters, ship-local orbit frame).
        glm::dvec3 _probe_offset_world{0.0, 25'000.0, 0.0};

        // Rebasing config (GameWorld::RebaseSettings)
        bool _enable_origin_rebasing{true};
        bool _enable_velocity_rebasing{true};
        double _origin_threshold_m{20'000.0};
        double _origin_snap_m{10'000.0};
        double _velocity_threshold_mps{250.0};

        // Debug visualization
        bool _ui_open{true};
        bool _debug_draw_enabled{true};
        bool _draw_orbit_circle{true};
        bool _draw_trail{true};
        double _velocity_vector_seconds{1.0}; // draw v * seconds
        double _debug_draw_ttl_s{0.25}; // must be > engine dt clamp (0.1) to survive begin_frame pruning

        // Velocity-origin integration: x_world = physics_origin_world + x_local, where
        // physics_origin_world is advanced by physics_velocity_origin_world each fixed step.
        bool _integrate_origin_from_velocity_origin{true};

        enum class VelocityOriginMode
        {
            // Every physics step: shift velocity origin so the anchor's local velocity stays ~0.
            // This is robust but effectively applies a Galilean rebase every step.
            PerStepAnchorSync,

            // Integrate v_origin using the anchor's gravity acceleration and apply gravity
            // in the anchor's free-fall frame (a_local = a_world - a_anchor_world). Then use
            // velocity rebasing only as a threshold-based correction for drift.
            FreeFallAnchorFrame
        };

        VelocityOriginMode _velocity_origin_mode{VelocityOriginMode::FreeFallAnchorFrame};

        std::vector<WorldVec3> _ship_trail_world;
        size_t _trail_max_points{256};
        double _trail_sample_interval_s{0.25};
        double _trail_sample_accum_s{0.0};

        // Rebase stats (counts + last deltas)
        uint64_t _origin_rebase_count{0};
        uint64_t _velocity_rebase_count{0};
        glm::dvec3 _last_origin_delta_world{0.0};
        glm::dvec3 _last_velocity_delta_world{0.0};

        // Deferred actions
        bool _reset_requested{false};
    };
} // namespace Game
