#pragma once

#include "collision_shape.h"
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Physics
{

// ============================================================================
// Motion type
// ============================================================================

enum class MotionType
{
    Static,    // Never moves (walls, floors, terrain)
    Kinematic, // Moved by code, not by physics (platforms, doors)
    Dynamic    // Fully simulated by physics (boxes, balls, characters)
};

// ============================================================================
// Collision layers
// ============================================================================

namespace Layer
{
    constexpr uint32_t Default     = 0;
    constexpr uint32_t Static      = 1;
    constexpr uint32_t Dynamic     = 2;
    constexpr uint32_t Kinematic   = 3;
    constexpr uint32_t Player      = 4;
    constexpr uint32_t Enemy       = 5;
    constexpr uint32_t Projectile  = 6;
    constexpr uint32_t Trigger     = 7;
    constexpr uint32_t Debris      = 8;
    constexpr uint32_t Count       = 16;
}

// ============================================================================
// Body creation settings
// ============================================================================

struct BodySettings
{
    // Shape
    CollisionShape shape;

    // Transform
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};

    // Motion
    MotionType motion_type = MotionType::Dynamic;

    // Physical properties
    float mass = 1.0f;           // Only used for dynamic bodies
    float friction = 0.5f;
    float restitution = 0.0f;    // Bounciness (0 = no bounce, 1 = perfect bounce)
    float linear_damping = 0.0f;
    float angular_damping = 0.05f;

    // Collision filtering
    uint32_t layer = Layer::Default;

    // Flags
    bool is_sensor = false;       // Trigger volume, no physical response
    bool start_active = true;     // Start awake (dynamic bodies only)
    bool allow_sleeping = true;   // Can go to sleep when at rest

    // Gravity
    float gravity_scale = 1.0f;   // 0 = no gravity, 1 = normal, 2 = double gravity

    // ========================================================================
    // Builder-style setters (return *this for chaining)
    // ========================================================================

    BodySettings& set_shape(const CollisionShape& s) { shape = s; return *this; }

    BodySettings& set_position(const glm::vec3& p) { position = p; return *this; }
    BodySettings& set_position(float x, float y, float z) { position = {x, y, z}; return *this; }

    BodySettings& set_rotation(const glm::quat& r) { rotation = r; return *this; }

    BodySettings& set_static() { motion_type = MotionType::Static; return *this; }
    BodySettings& set_kinematic() { motion_type = MotionType::Kinematic; return *this; }
    BodySettings& set_dynamic() { motion_type = MotionType::Dynamic; return *this; }

    BodySettings& set_mass(float m) { mass = m; return *this; }
    BodySettings& set_friction(float f) { friction = f; return *this; }
    BodySettings& set_restitution(float r) { restitution = r; return *this; }
    BodySettings& set_linear_damping(float d) { linear_damping = d; return *this; }
    BodySettings& set_angular_damping(float d) { angular_damping = d; return *this; }

    BodySettings& set_layer(uint32_t l) { layer = l; return *this; }

    BodySettings& set_sensor(bool s = true) { is_sensor = s; return *this; }
    BodySettings& set_gravity_scale(float s) { gravity_scale = s; return *this; }
};

} // namespace Physics
