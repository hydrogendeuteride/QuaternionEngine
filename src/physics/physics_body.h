#pragma once

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <cfloat>

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

        explicit BodyId(uint32_t v) : value(v)
        {
        }

        bool is_valid() const { return value != 0; }
        explicit operator bool() const { return is_valid(); }
        bool operator==(const BodyId &other) const { return value == other.value; }
        bool operator!=(const BodyId &other) const { return value != other.value; }
    };

    // ============================================================================
    // BodyHandle: RAII wrapper for physics body (optional, for automatic cleanup)
    // ============================================================================

    class BodyHandle
    {
    public:
        BodyHandle() = default;

        BodyHandle(PhysicsWorld *world, BodyId id);

        ~BodyHandle();

        // Move only
        BodyHandle(BodyHandle &&other) noexcept;

        BodyHandle &operator=(BodyHandle &&other) noexcept;

        BodyHandle(const BodyHandle &) = delete;

        BodyHandle &operator=(const BodyHandle &) = delete;

        BodyId id() const { return _id; }
        bool is_valid() const { return _world != nullptr && _id.is_valid(); }
        explicit operator bool() const { return is_valid(); }

        // Release ownership without destroying the body
        BodyId release();

        // Access the world (for calling methods)
        PhysicsWorld *world() const { return _world; }

    private:
        PhysicsWorld *_world{nullptr};
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

    struct QueryFilter
    {
        uint32_t layer_mask = 0xFFFFFFFF; // All layers by default
        BodyId ignore_body; // Optional body to ignore (e.g., the shooter)
        bool include_sensors = false; // Whether to detect sensor bodies
    };

    struct RaycastOptions : QueryFilter
    {
        float max_distance = 1000.0f;
        bool backface_culling = true; // Ignore hits from inside shapes
    };

    struct RayHit
    {
        bool hit{false};
        glm::vec3 position{0.0f};
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        float distance{0.0f};
        BodyId body_id;
        uint32_t sub_shape_id{0}; // For compound shapes
        uint32_t layer{0}; // Layer of the hit body
        uint64_t user_data{0};
    };

    // ============================================================================
    // Shape queries (sweep/overlap)
    // ============================================================================

    struct SweepOptions : QueryFilter
    {
        float max_distance = 1000.0f;
        bool backface_culling = true;
    };

    struct OverlapOptions : QueryFilter
    {
    };

    struct OverlapHit
    {
        BodyId body_id;
        uint32_t sub_shape_id{0};
        uint32_t layer{0};
        uint64_t user_data{0};
    };

    // ============================================================================
    // Contact events
    // ============================================================================

    enum class ContactEventType
    {
        Begin,
        Stay,
        End
    };

    struct CollisionEvent
    {
        ContactEventType type{ContactEventType::Begin};

        BodyId self;
        BodyId other;

        uint32_t self_sub_shape_id{0};
        uint32_t other_sub_shape_id{0};

        glm::vec3 point{0.0f};
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
        float penetration_depth{0.0f};

        uint32_t self_layer{0};
        uint32_t other_layer{0};

        uint64_t self_user_data{0};
        uint64_t other_user_data{0};
    };

    struct TriggerEvent
    {
        ContactEventType type{ContactEventType::Begin};

        BodyId self;
        BodyId other;

        uint32_t self_sub_shape_id{0};
        uint32_t other_sub_shape_id{0};

        glm::vec3 point{0.0f};

        bool self_is_sensor{false};
        bool other_is_sensor{false};

        uint32_t self_layer{0};
        uint32_t other_layer{0};

        uint64_t self_user_data{0};
        uint64_t other_user_data{0};
    };

    // ============================================================================
    // Joints
    // ============================================================================

    struct JointId
    {
        uint32_t value{0};

        JointId() = default;

        explicit JointId(uint32_t v) : value(v)
        {
        }

        bool is_valid() const { return value != 0; }
        explicit operator bool() const { return is_valid(); }
        bool operator==(const JointId &other) const { return value == other.value; }
        bool operator!=(const JointId &other) const { return value != other.value; }
    };

    struct HingeJointSettings
    {
        glm::vec3 anchor{0.0f};
        glm::vec3 axis{0.0f, 1.0f, 0.0f};
        float limit_min{-3.14159265f};
        float limit_max{3.14159265f};
        bool enable_limits{false};
    };

    struct SliderJointSettings
    {
        glm::vec3 anchor{0.0f};
        glm::vec3 axis{1.0f, 0.0f, 0.0f};
        float limit_min{-FLT_MAX};
        float limit_max{FLT_MAX};
        bool enable_limits{false};
    };
} // namespace Physics
