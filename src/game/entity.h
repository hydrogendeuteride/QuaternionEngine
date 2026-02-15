#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <core/world.h>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <unordered_map>
#include <typeindex>
#include <cstdint>
#include <algorithm>

// Forward declarations
namespace Physics
{
    struct BodyId;
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class IComponent;
    struct ComponentContext;

    // ============================================================================
    // EntityId: Strongly-typed entity identifier
    // ============================================================================

    struct EntityId
    {
        uint32_t value{0};

        EntityId() = default;

        explicit EntityId(uint32_t v) : value(v)
        {
        }

        bool is_valid() const { return value != 0; }
        explicit operator bool() const { return is_valid(); }
        bool operator==(const EntityId &other) const { return value == other.value; }
        bool operator!=(const EntityId &other) const { return value != other.value; }
    };

    // ============================================================================
    // Transform: World-space position (double precision), rotation, scale
    // ============================================================================

    struct Transform
    {
        WorldVec3 position_world{0.0, 0.0, 0.0};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f};

        // Returns local-space matrix (caller provides origin for position conversion)
        glm::mat4 to_local_matrix(const WorldVec3 &origin_world) const;

        // Combine transforms (parent * child) - child position is treated as local offset
        Transform operator*(const Transform &child) const;
    };

    // ============================================================================
    // InterpolatedTransform: For smooth physics rendering (world-space, double precision)
    // ============================================================================

    struct InterpolatedTransform
    {
        WorldVec3 prev_position{0.0, 0.0, 0.0};
        glm::quat prev_rotation{1.0f, 0.0f, 0.0f, 0.0f};
        WorldVec3 curr_position{0.0, 0.0, 0.0};
        glm::quat curr_rotation{1.0f, 0.0f, 0.0f, 0.0f};

        WorldVec3 interpolated_position(float alpha) const;

        glm::quat interpolated_rotation(float alpha) const;

        void store_current_as_previous();

        void set_immediate(const WorldVec3 &pos, const glm::quat &rot);
    };

    // ============================================================================
    // Attachment: A part attached to an entity (child object)
    // ============================================================================

    struct Attachment
    {
        std::string name; // Unique name within parent entity
        std::string render_name; // SceneManager instance name

        // Local transform relative to parent
        glm::vec3 local_position{0.0f};
        glm::quat local_rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 local_scale{1.0f};

        // Optional physics body (for kinematic parts or joints)
        std::optional<uint32_t> physics_body_value; // Physics::BodyId::value
        bool sync_physics{false}; // If true, sync attachment transform to physics body

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

        explicit Entity(EntityId id, const std::string &name = "");

        // ------------------------------------------------------------------------
        // Identity
        // ------------------------------------------------------------------------

        EntityId id() const { return _id; }
        const std::string &name() const { return _name; }
        void set_name(const std::string &name) { _name = name; }

        // ------------------------------------------------------------------------
        // Transform (authoritative, world-space position)
        // ------------------------------------------------------------------------

        const Transform &transform() const { return _transform; }
        void set_transform(const Transform &transform) { _transform = transform; }

        void set_position_world(const WorldVec3 &pos) { _transform.position_world = pos; }
        void set_rotation(const glm::quat &rot) { _transform.rotation = rot; }
        void set_scale(const glm::vec3 &scale) { _transform.scale = scale; }

        const WorldVec3 &position_world() const { return _transform.position_world; }
        const glm::quat &rotation() const { return _transform.rotation; }
        const glm::vec3 &scale() const { return _transform.scale; }

        // Get local-space matrix for rendering (requires origin)
        glm::mat4 get_local_matrix(const WorldVec3 &origin_world) const
        {
            return _transform.to_local_matrix(origin_world);
        }

        // ------------------------------------------------------------------------
        // Interpolation (for physics smoothing)
        // ------------------------------------------------------------------------

        InterpolatedTransform &interpolation() { return _interp; }
        const InterpolatedTransform &interpolation() const { return _interp; }

        bool uses_interpolation() const { return _use_interpolation; }
        void set_use_interpolation(bool use) { _use_interpolation = use; }

        // Get interpolated transform for rendering (requires origin for local-space conversion)
        WorldVec3 get_render_position_world(float alpha) const;

        glm::quat get_render_rotation(float alpha) const;

        glm::mat4 get_render_local_matrix(float alpha, const WorldVec3 &origin_world) const;

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

        const std::string &render_name() const { return _render_name; }
        void set_render_name(const std::string &name) { _render_name = name; }

        // ------------------------------------------------------------------------
        // Attachments
        // ------------------------------------------------------------------------

        const std::vector<Attachment> &attachments() const { return _attachments; }
        std::vector<Attachment> &attachments() { return _attachments; }

        void add_attachment(const Attachment &attachment);

        bool remove_attachment(const std::string &name);

        Attachment *find_attachment(const std::string &name);

        const Attachment *find_attachment(const std::string &name) const;

        // Get local-space transform for an attachment (requires origin)
        glm::mat4 get_attachment_local_matrix(const Attachment &att, const WorldVec3 &origin_world) const;

        glm::mat4 get_attachment_local_matrix(const Attachment &att, float alpha, const WorldVec3 &origin_world) const;

        // ------------------------------------------------------------------------
        // Components
        // ------------------------------------------------------------------------

        template<typename T, typename... Args>
        T *add_component(Args &&...args);

        template<typename T>
        T *get_component();

        template<typename T>
        const T *get_component() const;

        template<typename T>
        bool has_component() const;

        template<typename T>
        bool remove_component();

        const std::vector<std::unique_ptr<IComponent>> &components() const { return _components; }

        // ------------------------------------------------------------------------
        // Flags
        // ------------------------------------------------------------------------

        bool is_active() const { return _active; }
        void set_active(bool active) { _active = active; }

        bool is_visible() const { return _visible; }
        void set_visible(bool visible) { _visible = visible; }

    private:
        // Component lifecycle (called by EntityManager/GameWorld)
        void init_components(ComponentContext &ctx);
        void update_components(ComponentContext &ctx, float dt);
        void fixed_update_components(ComponentContext &ctx, float fixed_dt);
        void destroy_components(ComponentContext &ctx);

        friend class EntityManager;
        friend class GameWorld;
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

        // Components
        std::vector<std::unique_ptr<IComponent>> _components;
        std::unordered_map<std::type_index, IComponent *> _component_map;

        // Flags
        bool _active{true};
        bool _visible{true};
    };

    // ============================================================================
    // Template implementations (require IComponent definition)
    // ============================================================================

} // namespace Game

// Include full IComponent definition for template bodies
#include "component/component.h"

namespace Game
{
    template<typename T, typename... Args>
    T *Entity::add_component(Args &&...args)
    {
        static_assert(std::is_base_of_v<IComponent, T>,
                      "T must derive from IComponent");

        std::type_index idx = std::type_index(typeid(T));
        if (_component_map.find(idx) != _component_map.end())
        {
            return nullptr; // duplicate type
        }

        auto comp = std::make_unique<T>(std::forward<Args>(args)...);
        T *ptr = comp.get();
        ptr->_entity = this;

        _component_map[idx] = ptr;
        _components.push_back(std::move(comp));
        return ptr;
    }

    template<typename T>
    T *Entity::get_component()
    {
        auto it = _component_map.find(std::type_index(typeid(T)));
        return it != _component_map.end() ? static_cast<T *>(it->second) : nullptr;
    }

    template<typename T>
    const T *Entity::get_component() const
    {
        auto it = _component_map.find(std::type_index(typeid(T)));
        return it != _component_map.end() ? static_cast<const T *>(it->second) : nullptr;
    }

    template<typename T>
    bool Entity::has_component() const
    {
        return _component_map.find(std::type_index(typeid(T))) != _component_map.end();
    }

    template<typename T>
    bool Entity::remove_component()
    {
        auto map_it = _component_map.find(std::type_index(typeid(T)));
        if (map_it == _component_map.end())
        {
            return false;
        }

        IComponent *raw = map_it->second;
        _component_map.erase(map_it);

        auto vec_it = std::find_if(_components.begin(), _components.end(),
            [raw](const std::unique_ptr<IComponent> &c) { return c.get() == raw; });

        if (vec_it != _components.end())
        {
            _components.erase(vec_it);
        }
        return true;
    }
} // namespace Game
