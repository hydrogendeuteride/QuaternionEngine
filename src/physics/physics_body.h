#pragma once

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>

namespace Physics
{

class PhysicsWorld;

// ============================================================================
// BodyId: strongly-typed body identifier
// ============================================================================

struct BodyId
{
    uint32_t value{0};

    BodyId() = default;
    explicit BodyId(uint32_t v) : value(v) {}

    bool is_valid() const { return value != 0; }
    explicit operator bool() const { return is_valid(); }
    bool operator==(const BodyId& other) const { return value == other.value; }
    bool operator!=(const BodyId& other) const { return value != other.value; }
};

// ============================================================================
// BodyHandle: RAII wrapper for physics body (optional, for automatic cleanup)
// ============================================================================

class BodyHandle
{
public:
    BodyHandle() = default;
    BodyHandle(PhysicsWorld* world, BodyId id);
    ~BodyHandle();

    // Move only
    BodyHandle(BodyHandle&& other) noexcept;
    BodyHandle& operator=(BodyHandle&& other) noexcept;
    BodyHandle(const BodyHandle&) = delete;
    BodyHandle& operator=(const BodyHandle&) = delete;

    BodyId id() const { return _id; }
    bool is_valid() const { return _world != nullptr && _id.is_valid(); }
    explicit operator bool() const { return is_valid(); }

    // Release ownership without destroying the body
    BodyId release();

    // Access the world (for calling methods)
    PhysicsWorld* world() const { return _world; }

private:
    PhysicsWorld* _world{nullptr};
    BodyId _id;
};

// ============================================================================
// Transform data returned from physics
// ============================================================================

struct BodyTransform
{
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};

    glm::mat4 to_matrix() const;
};

// ============================================================================
// Raycast options and results
// ============================================================================

struct RaycastOptions
{
    float max_distance = 1000.0f;
    uint32_t layer_mask = 0xFFFFFFFF;  // All layers by default
    BodyId ignore_body;                 // Optional body to ignore (e.g., the shooter)
    bool include_sensors = false;       // Whether to detect sensor bodies
    bool backface_culling = true;       // Ignore hits from inside shapes
};

struct RayHit
{
    bool hit{false};
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    float distance{0.0f};
    BodyId body_id;
    uint32_t sub_shape_id{0};           // For compound shapes
    uint32_t layer{0};                   // Layer of the hit body
};

} // namespace Physics
