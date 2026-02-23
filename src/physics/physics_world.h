#pragma once

#include "body_settings.h"
#include "physics_body.h"
#include <memory>
#include <vector>
#include <functional>
#include <utility>

namespace Physics
{
    // ============================================================================
    // PhysicsWorld: Abstract interface for physics simulation
    // ============================================================================

    class PhysicsWorld
    {
    public:
        struct BodyCallbacks;

        virtual ~PhysicsWorld() = default;

        // ========================================================================
        // Simulation
        // ========================================================================

        virtual void step(float dt) = 0;

        // Shift the local coordinate origin by translating all bodies by delta_local.
        // Used to keep world-space positions stable when the engine's floating origin changes.
        // IMPORTANT: Must be called BEFORE step() in the same frame to avoid stale transforms.
        // Default implementation is a no-op.
        virtual void shift_origin(const glm::dvec3 &delta_local) { (void)delta_local; }

        // Shift the local velocity origin by subtracting delta_local_velocity from all bodies' linear velocities.
        // Used to keep world-space velocities stable when switching inertial frames (Galilean transform).
        // IMPORTANT: Must be called BEFORE step() in the same frame to avoid stale velocities.
        // Default implementation is a no-op.
        virtual void shift_velocity_origin(const glm::dvec3 &delta_local_velocity) { (void)delta_local_velocity; }

        // ========================================================================
        // Debug / instrumentation (optional)
        // ========================================================================

        struct DebugStats
        {
            float last_step_ms{0.0f};
            float avg_step_ms{0.0f};
            float last_dt{0.0f}; // seconds

            uint32_t body_count{0};
            uint32_t active_body_count{0};
            uint32_t joint_count{0};
            uint32_t contact_event_count{0}; // last step (queued or dispatched, backend-dependent)
        };

        // Lightweight counters/timings for UI and telemetry. Backends may return zeros if unsupported.
        virtual DebugStats debug_stats() const { return {}; }

        struct DebugBodyView
        {
            BodyId id;

            glm::dvec3 position{0.0, 0.0, 0.0};
            glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};

            MotionType motion_type{MotionType::Static};
            uint32_t layer{0};
            bool is_sensor{false};
            bool is_active{false};
            uint64_t user_data{0};

            CollisionShape shape{};
        };

        using DebugBodyFn = std::function<void(const DebugBodyView &)>;

        // Iterate over bodies with enough data to drive debug UI and debug rendering.
        // Default implementation does nothing.
        virtual void for_each_debug_body(const DebugBodyFn &fn) const { (void) fn; }

        // ========================================================================
        // Body creation / destruction
        // ========================================================================

        // Create a body with given settings, returns ID
        virtual BodyId create_body(const BodySettings &settings) = 0;

        // Destroy a body by ID
        virtual void destroy_body(BodyId id) = 0;

        // Create a body and return RAII handle (auto-destroys on scope exit)
        BodyHandle create_body_handle(const BodySettings &settings)
        {
            return BodyHandle(this, create_body(settings));
        }

        // ========================================================================
        // Body queries
        // ========================================================================

        virtual bool is_body_valid(BodyId id) const = 0;

        virtual BodyTransform get_transform(BodyId id) const = 0;

        virtual glm::dvec3 get_position(BodyId id) const = 0;

        virtual glm::quat get_rotation(BodyId id) const = 0;

        virtual glm::mat4 get_transform_matrix(BodyId id) const = 0;

        virtual glm::vec3 get_linear_velocity(BodyId id) const = 0;

        virtual glm::vec3 get_angular_velocity(BodyId id) const = 0;

        virtual uint64_t get_user_data(BodyId id) const = 0;

        // ========================================================================
        // Body manipulation
        // ========================================================================

        virtual void set_position(BodyId id, const glm::dvec3 &position) = 0;

        virtual void set_rotation(BodyId id, const glm::quat &rotation) = 0;

        virtual void set_transform(BodyId id, const glm::dvec3 &position, const glm::quat &rotation) = 0;

        virtual void set_linear_velocity(BodyId id, const glm::vec3 &velocity) = 0;

        virtual void set_angular_velocity(BodyId id, const glm::vec3 &velocity) = 0;

        virtual void set_user_data(BodyId id, uint64_t user_data) = 0;

        // Runtime motion-type control.
        // Default implementations are no-op/unknown for backends that do not support this.
        virtual bool set_motion_type(BodyId id, MotionType motion_type)
        {
            (void) id;
            (void) motion_type;
            return false;
        }

        virtual MotionType get_motion_type(BodyId id) const
        {
            (void) id;
            return MotionType::Static;
        }

        // Force application (for dynamic bodies)
        virtual void add_force(BodyId id, const glm::vec3 &force) = 0;

        virtual void add_impulse(BodyId id, const glm::vec3 &impulse) = 0;

        virtual void add_torque(BodyId id, const glm::vec3 &torque) = 0;

        // Activation
        virtual void activate(BodyId id) = 0;

        virtual void deactivate(BodyId id) = 0;

        virtual bool is_active(BodyId id) const = 0;

        // ========================================================================
        // Raycasting
        // ========================================================================

        // Simple raycast (legacy interface)
        virtual RayHit raycast(const glm::dvec3 &origin, const glm::vec3 &direction, double max_distance) const = 0;

        // Extended raycast with filtering options
        virtual RayHit raycast(const glm::dvec3 &origin, const glm::vec3 &direction,
                               const RaycastOptions &options) const = 0;

        // ========================================================================
        // Shape queries
        // ========================================================================

        virtual RayHit sweep(const CollisionShape &shape,
                             const glm::dvec3 &origin,
                             const glm::quat &rotation,
                             const glm::vec3 &direction,
                             const SweepOptions &options) const = 0;

        virtual void overlap(const CollisionShape &shape,
                             const glm::dvec3 &position,
                             const glm::quat &rotation,
                             const OverlapOptions &options,
                             std::vector<OverlapHit> &out_hits) const = 0;

        // ========================================================================
        // Collision filtering
        // ========================================================================

        virtual void set_layer_collision(uint32_t layer_a, uint32_t layer_b, bool should_collide) = 0;

        virtual bool get_layer_collision(uint32_t layer_a, uint32_t layer_b) const = 0;

        // ========================================================================
        // Contact callbacks
        // ========================================================================

        struct BodyCallbacks
        {
            std::function<void(const CollisionEvent &)> on_collision;
            std::function<void(const TriggerEvent &)> on_trigger;
        };

        virtual void set_body_callbacks(BodyId id, const BodyCallbacks &callbacks) = 0;

        virtual void clear_body_callbacks(BodyId id) = 0;

        // ========================================================================
        // Joints
        // ========================================================================

        virtual JointId create_fixed_joint(BodyId body_a, BodyId body_b) = 0;

        virtual JointId create_hinge_joint(BodyId body_a, BodyId body_b, const HingeJointSettings &settings) = 0;

        virtual JointId create_slider_joint(BodyId body_a, BodyId body_b, const SliderJointSettings &settings) = 0;

        virtual void destroy_joint(JointId id) = 0;

        virtual bool is_joint_valid(JointId id) const = 0;

        // ========================================================================
        // World settings
        // ========================================================================

        virtual void set_gravity(const glm::vec3 &gravity) = 0;

        virtual glm::vec3 get_gravity() const = 0;
    };

    // ============================================================================
    // BodyBuilder: Fluent API for body creation
    // ============================================================================

    class BodyBuilder
    {
    public:
        explicit BodyBuilder(PhysicsWorld *world) : _world(world)
        {
        }

        // Shape
        BodyBuilder &shape(const CollisionShape &s)
        {
            _settings.shape = s;
            return *this;
        }

        BodyBuilder &user_data(uint64_t v)
        {
            _settings.user_data = v;
            return *this;
        }

        BodyBuilder &box(float hx, float hy, float hz)
        {
            _settings.shape = CollisionShape::Box(hx, hy, hz);
            return *this;
        }

        BodyBuilder &box(const glm::vec3 &half_extents)
        {
            _settings.shape = CollisionShape::Box(half_extents);
            return *this;
        }

        BodyBuilder &sphere(float radius)
        {
            _settings.shape = CollisionShape::Sphere(radius);
            return *this;
        }

        BodyBuilder &capsule(float radius, float half_height)
        {
            _settings.shape = CollisionShape::Capsule(radius, half_height);
            return *this;
        }

        BodyBuilder &cylinder(float radius, float half_height)
        {
            _settings.shape = CollisionShape::Cylinder(radius, half_height);
            return *this;
        }

        BodyBuilder &plane(const glm::vec3 &normal = {0, 1, 0})
        {
            _settings.shape = CollisionShape::Plane(normal);
            return *this;
        }

        BodyBuilder &compound(CompoundShape compound)
        {
            _settings.shape = CollisionShape::Compound(std::move(compound));
            return *this;
        }

        // Position / rotation
        BodyBuilder &position(const glm::dvec3 &p)
        {
            _settings.position = p;
            return *this;
        }

        BodyBuilder &position(const glm::vec3 &p)
        {
            _settings.position = glm::dvec3(p);
            return *this;
        }

        BodyBuilder &position(double x, double y, double z)
        {
            _settings.position = {x, y, z};
            return *this;
        }

        BodyBuilder &rotation(const glm::quat &r)
        {
            _settings.rotation = r;
            return *this;
        }

        // Motion type
        BodyBuilder &static_body()
        {
            _settings.motion_type = MotionType::Static;
            return *this;
        }

        BodyBuilder &kinematic_body()
        {
            _settings.motion_type = MotionType::Kinematic;
            return *this;
        }

        BodyBuilder &dynamic_body()
        {
            _settings.motion_type = MotionType::Dynamic;
            return *this;
        }

        // Physical properties
        BodyBuilder &mass(float m)
        {
            _settings.mass = m;
            return *this;
        }

        BodyBuilder &friction(float f)
        {
            _settings.friction = f;
            return *this;
        }

        BodyBuilder &restitution(float r)
        {
            _settings.restitution = r;
            return *this;
        }

        BodyBuilder &linear_damping(float d)
        {
            _settings.linear_damping = d;
            return *this;
        }

        BodyBuilder &angular_damping(float d)
        {
            _settings.angular_damping = d;
            return *this;
        }

        // Collision
        BodyBuilder &layer(uint32_t l)
        {
            _settings.layer = l;
            return *this;
        }

        BodyBuilder &sensor(bool s = true)
        {
            _settings.is_sensor = s;
            return *this;
        }

        // Gravity
        BodyBuilder &gravity_scale(float s)
        {
            _settings.gravity_scale = s;
            return *this;
        }

        BodyBuilder &no_gravity()
        {
            _settings.gravity_scale = 0.0f;
            return *this;
        }

        // Build
        BodyId build() { return _world->create_body(_settings); }
        BodyHandle build_handle() { return _world->create_body_handle(_settings); }

        // Access settings for inspection
        const BodySettings &settings() const { return _settings; }

    private:
        PhysicsWorld *_world;
        BodySettings _settings;
    };
} // namespace Physics
