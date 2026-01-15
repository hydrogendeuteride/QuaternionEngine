#pragma once

#include "body_settings.h"
#include "physics_body.h"
#include <memory>

namespace Physics
{

// ============================================================================
// PhysicsWorld: Abstract interface for physics simulation
// ============================================================================

class PhysicsWorld
{
public:
    virtual ~PhysicsWorld() = default;

    // ========================================================================
    // Simulation
    // ========================================================================

    virtual void step(float dt) = 0;

    // ========================================================================
    // Body creation / destruction
    // ========================================================================

    // Create a body with given settings, returns ID
    virtual BodyId create_body(const BodySettings& settings) = 0;

    // Destroy a body by ID
    virtual void destroy_body(BodyId id) = 0;

    // Create a body and return RAII handle (auto-destroys on scope exit)
    BodyHandle create_body_handle(const BodySettings& settings)
    {
        return BodyHandle(this, create_body(settings));
    }

    // ========================================================================
    // Body queries
    // ========================================================================

    virtual bool is_body_valid(BodyId id) const = 0;

    virtual BodyTransform get_transform(BodyId id) const = 0;
    virtual glm::vec3 get_position(BodyId id) const = 0;
    virtual glm::quat get_rotation(BodyId id) const = 0;
    virtual glm::mat4 get_transform_matrix(BodyId id) const = 0;

    virtual glm::vec3 get_linear_velocity(BodyId id) const = 0;
    virtual glm::vec3 get_angular_velocity(BodyId id) const = 0;

    // ========================================================================
    // Body manipulation
    // ========================================================================

    virtual void set_position(BodyId id, const glm::vec3& position) = 0;
    virtual void set_rotation(BodyId id, const glm::quat& rotation) = 0;
    virtual void set_transform(BodyId id, const glm::vec3& position, const glm::quat& rotation) = 0;

    virtual void set_linear_velocity(BodyId id, const glm::vec3& velocity) = 0;
    virtual void set_angular_velocity(BodyId id, const glm::vec3& velocity) = 0;

    // Force application (for dynamic bodies)
    virtual void add_force(BodyId id, const glm::vec3& force) = 0;
    virtual void add_impulse(BodyId id, const glm::vec3& impulse) = 0;
    virtual void add_torque(BodyId id, const glm::vec3& torque) = 0;

    // Activation
    virtual void activate(BodyId id) = 0;
    virtual void deactivate(BodyId id) = 0;
    virtual bool is_active(BodyId id) const = 0;

    // ========================================================================
    // Raycasting
    // ========================================================================

    // Simple raycast (legacy interface)
    virtual RayHit raycast(const glm::vec3& origin, const glm::vec3& direction, float max_distance) const = 0;

    // Extended raycast with filtering options
    virtual RayHit raycast(const glm::vec3& origin, const glm::vec3& direction, const RaycastOptions& options) const = 0;

    // ========================================================================
    // World settings
    // ========================================================================

    virtual void set_gravity(const glm::vec3& gravity) = 0;
    virtual glm::vec3 get_gravity() const = 0;
};

// ============================================================================
// BodyBuilder: Fluent API for body creation
// ============================================================================

class BodyBuilder
{
public:
    explicit BodyBuilder(PhysicsWorld* world) : _world(world) {}

    // Shape
    BodyBuilder& shape(const CollisionShape& s) { _settings.shape = s; return *this; }
    BodyBuilder& box(float hx, float hy, float hz) { _settings.shape = CollisionShape::Box(hx, hy, hz); return *this; }
    BodyBuilder& box(const glm::vec3& half_extents) { _settings.shape = CollisionShape::Box(half_extents); return *this; }
    BodyBuilder& sphere(float radius) { _settings.shape = CollisionShape::Sphere(radius); return *this; }
    BodyBuilder& capsule(float radius, float half_height) { _settings.shape = CollisionShape::Capsule(radius, half_height); return *this; }
    BodyBuilder& cylinder(float radius, float half_height) { _settings.shape = CollisionShape::Cylinder(radius, half_height); return *this; }
    BodyBuilder& plane(const glm::vec3& normal = {0,1,0}) { _settings.shape = CollisionShape::Plane(normal); return *this; }

    // Position / rotation
    BodyBuilder& position(const glm::vec3& p) { _settings.position = p; return *this; }
    BodyBuilder& position(float x, float y, float z) { _settings.position = {x, y, z}; return *this; }
    BodyBuilder& rotation(const glm::quat& r) { _settings.rotation = r; return *this; }

    // Motion type
    BodyBuilder& static_body() { _settings.motion_type = MotionType::Static; return *this; }
    BodyBuilder& kinematic_body() { _settings.motion_type = MotionType::Kinematic; return *this; }
    BodyBuilder& dynamic_body() { _settings.motion_type = MotionType::Dynamic; return *this; }

    // Physical properties
    BodyBuilder& mass(float m) { _settings.mass = m; return *this; }
    BodyBuilder& friction(float f) { _settings.friction = f; return *this; }
    BodyBuilder& restitution(float r) { _settings.restitution = r; return *this; }
    BodyBuilder& linear_damping(float d) { _settings.linear_damping = d; return *this; }
    BodyBuilder& angular_damping(float d) { _settings.angular_damping = d; return *this; }

    // Collision
    BodyBuilder& layer(uint32_t l) { _settings.layer = l; return *this; }
    BodyBuilder& sensor(bool s = true) { _settings.is_sensor = s; return *this; }

    // Gravity
    BodyBuilder& gravity_scale(float s) { _settings.gravity_scale = s; return *this; }
    BodyBuilder& no_gravity() { _settings.gravity_scale = 0.0f; return *this; }

    // Build
    BodyId build() { return _world->create_body(_settings); }
    BodyHandle build_handle() { return _world->create_body_handle(_settings); }

    // Access settings for inspection
    const BodySettings& settings() const { return _settings; }

private:
    PhysicsWorld* _world;
    BodySettings _settings;
};

} // namespace Physics
