#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <string>
#include <vector>
#include <optional>
#include <cstdint>

// Forward declarations
namespace Physics
{
    struct BodyId;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{

// ============================================================================
// EntityId: Strongly-typed entity identifier
// ============================================================================

struct EntityId
{
    uint32_t value{0};

    EntityId() = default;
    explicit EntityId(uint32_t v) : value(v) {}

    bool is_valid() const { return value != 0; }
    explicit operator bool() const { return is_valid(); }
    bool operator==(const EntityId& other) const { return value == other.value; }
    bool operator!=(const EntityId& other) const { return value != other.value; }
};

// ============================================================================
// Transform: Position, rotation, scale
// ============================================================================

struct Transform
{
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};

    glm::mat4 to_matrix() const;
    static Transform from_matrix(const glm::mat4& m);

    // Combine transforms (parent * child)
    Transform operator*(const Transform& child) const;
};

// ============================================================================
// InterpolatedTransform: For smooth physics rendering
//// ============================================================================

struct InterpolatedTransform
{
    glm::vec3 prev_position{0.0f};
    glm::quat prev_rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 curr_position{0.0f};
    glm::quat curr_rotation{1.0f, 0.0f, 0.0f, 0.0f};

    glm::vec3 interpolated_position(float alpha) const;
    glm::quat interpolated_rotation(float alpha) const;

    void store_current_as_previous();
    void set_immediate(const glm::vec3& pos, const glm::quat& rot);
};

// ============================================================================
// Attachment: A part attached to an entity (child object)
// ============================================================================

struct Attachment
{
    std::string name;                              // Unique name within parent entity
    std::string render_name;                       // SceneManager instance name

    // Local transform relative to parent
    glm::vec3 local_position{0.0f};
    glm::quat local_rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 local_scale{1.0f};

    // Optional physics body (for kinematic parts or joints)
    std::optional<uint32_t> physics_body_value;    // Physics::BodyId::value
    bool sync_physics{false};                      // If true, sync attachment transform to physics body

    // Flags
    bool visible{true};

    // Get local transform as matrix
    glm::mat4 get_local_matrix() const;

    // Get local transform as Transform struct
    Transform get_local_transform() const;
};

// ============================================================================
// Entity: A game object with optional physics and rendering
// ============================================================================

class Entity
{
public:
    Entity() = default;
    explicit Entity(EntityId id, const std::string& name = "");

    // ------------------------------------------------------------------------
    // Identity
    // ------------------------------------------------------------------------

    EntityId id() const { return _id; }
    const std::string& name() const { return _name; }
    void set_name(const std::string& name) { _name = name; }

    // ------------------------------------------------------------------------
    // Transform (authoritative)
    // ------------------------------------------------------------------------

    const Transform& transform() const { return _transform; }
    void set_transform(const Transform& transform) { _transform = transform; }

    void set_position(const glm::vec3& pos) { _transform.position = pos; }
    void set_rotation(const glm::quat& rot) { _transform.rotation = rot; }
    void set_scale(const glm::vec3& scale) { _transform.scale = scale; }

    const glm::vec3& position() const { return _transform.position; }
    const glm::quat& rotation() const { return _transform.rotation; }
    const glm::vec3& scale() const { return _transform.scale; }

    glm::mat4 get_world_matrix() const { return _transform.to_matrix(); }

    // ------------------------------------------------------------------------
    // Interpolation (for physics smoothing)
    // ------------------------------------------------------------------------

    InterpolatedTransform& interpolation() { return _interp; }
    const InterpolatedTransform& interpolation() const { return _interp; }

    bool uses_interpolation() const { return _use_interpolation; }
    void set_use_interpolation(bool use) { _use_interpolation = use; }

    // Get interpolated transform for rendering
    glm::vec3 get_render_position(float alpha) const;
    glm::quat get_render_rotation(float alpha) const;
    glm::mat4 get_render_matrix(float alpha) const;

    // ------------------------------------------------------------------------
    // Physics binding
    // ------------------------------------------------------------------------

    bool has_physics() const { return _physics_body_value.has_value(); }

    uint32_t physics_body_value() const { return _physics_body_value.value_or(0); }
    void set_physics_body(uint32_t body_value) { _physics_body_value = body_value; }
    void clear_physics_body() { _physics_body_value.reset(); }

    // ------------------------------------------------------------------------
    // Render binding
    // ------------------------------------------------------------------------

    bool has_render() const { return !_render_name.empty(); }

    const std::string& render_name() const { return _render_name; }
    void set_render_name(const std::string& name) { _render_name = name; }

    // ------------------------------------------------------------------------
    // Attachments
    // ------------------------------------------------------------------------

    const std::vector<Attachment>& attachments() const { return _attachments; }
    std::vector<Attachment>& attachments() { return _attachments; }

    void add_attachment(const Attachment& attachment);
    bool remove_attachment(const std::string& name);
    Attachment* find_attachment(const std::string& name);
    const Attachment* find_attachment(const std::string& name) const;

    // Get world transform for an attachment
    glm::mat4 get_attachment_world_matrix(const Attachment& att) const;
    glm::mat4 get_attachment_world_matrix(const Attachment& att, float alpha) const;

    // ------------------------------------------------------------------------
    // Flags
    // ------------------------------------------------------------------------

    bool is_active() const { return _active; }
    void set_active(bool active) { _active = active; }

    bool is_visible() const { return _visible; }
    void set_visible(bool visible) { _visible = visible; }

private:
    EntityId _id;
    std::string _name;

    // Transform
    Transform _transform;
    InterpolatedTransform _interp;
    bool _use_interpolation{false};

    // Physics binding (stores BodyId::value)
    std::optional<uint32_t> _physics_body_value;

    // Render binding (SceneManager instance name)
    std::string _render_name;

    // Child attachments
    std::vector<Attachment> _attachments;

    // Flags
    bool _active{true};
    bool _visible{true};
};

} // namespace Game
