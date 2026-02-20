#pragma once

#include "core/world.h"
#include <glm/glm.hpp>
#include <cstdint>

namespace Physics
{
    class PhysicsWorld;

    // ============================================================================
    // PhysicsContext: Manages physics coordinate system origins and rebasing.
    //
    // This separates physics-related floating origin state from the render-focused
    // EngineContext, allowing physics simulation to operate in its own local bubble
    // independent of camera/render position.
    //
    // Coordinate system:
    //   world_position = physics_origin_world + local_position
    //   world_velocity = velocity_origin_world + local_velocity
    // ============================================================================

    class PhysicsContext
    {
    public:
        explicit PhysicsContext(PhysicsWorld &world)
            : _physics(&world)
        {
        }
        ~PhysicsContext() = default;

        // Non-copyable, movable
        PhysicsContext(const PhysicsContext &) = delete;
        PhysicsContext &operator=(const PhysicsContext &) = delete;
        PhysicsContext(PhysicsContext &&) = default;
        PhysicsContext &operator=(PhysicsContext &&) = default;

        // ========================================================================
        // Physics world reference (non-owning, bound at construction)
        // ========================================================================

        PhysicsWorld &physics_world() const { return *_physics; }

        // ========================================================================
        // Position origin
        // ========================================================================

        const WorldVec3 &origin_world() const { return _origin_world; }
        uint64_t origin_revision() const { return _origin_revision; }

        // Returns true if origin changed
        bool set_origin_world(const WorldVec3 &new_origin);

        // ========================================================================
        // Velocity origin (for Galilean transforms in high-speed scenarios)
        // ========================================================================

        const glm::dvec3 &velocity_origin_world() const { return _velocity_origin_world; }
        uint64_t velocity_origin_revision() const { return _velocity_origin_revision; }

        // Returns true if velocity origin changed
        bool set_velocity_origin_world(const glm::dvec3 &new_velocity);

        // ========================================================================
        // Anchor (for automatic rebasing decisions)
        // ========================================================================

        void set_anchor_world(const WorldVec3 &anchor);
        void clear_anchor();
        bool has_anchor() const { return _anchor_enabled; }
        const WorldVec3 &anchor_world() const { return _anchor_world; }

        // ========================================================================
        // Rebasing operations
        // ========================================================================

        // Rebase position origin to a body when its local position exceeds threshold.
        // Optionally snaps to grid for stability.
        // Returns true if rebasing occurred.
        bool maybe_rebase_origin_to_body(uint32_t body_value, double threshold_m, double snap_m = 0.0);

        // Rebase velocity origin to a body when its local velocity exceeds threshold.
        // Returns true if rebasing occurred.
        bool maybe_rebase_velocity_to_body(uint32_t body_value, double threshold_mps);

    private:
        PhysicsWorld *_physics{nullptr}; // non-owning; bound on construction, world must outlive context

        // Position origin
        WorldVec3 _origin_world{0.0, 0.0, 0.0};
        uint64_t _origin_revision{0};

        // Velocity origin (for inertial frame rebasing)
        glm::dvec3 _velocity_origin_world{0.0, 0.0, 0.0};
        uint64_t _velocity_origin_revision{0};

        // Anchor for automatic rebasing
        WorldVec3 _anchor_world{0.0, 0.0, 0.0};
        bool _anchor_enabled{false};
    };

} // namespace Physics
