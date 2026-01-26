#pragma once

#include "entity.h"
#include "core/world.h"

#include <unordered_map>
#include <functional>
#include <string>

// Forward declarations
namespace Physics
{
    struct BodyId;
    struct BodyTransform;
    class PhysicsWorld;
} // namespace Physics

class SceneManager;

namespace GameAPI
{
    class Engine;
} // namespace GameAPI

namespace Game
{

// ============================================================================
// EntityManager: Central manager for game entities
// ============================================================================

class EntityManager
{
public:
    EntityManager() = default;
    ~EntityManager() = default;

    // Non-copyable
    EntityManager(const EntityManager&) = delete;
    EntityManager& operator=(const EntityManager&) = delete;

    // ------------------------------------------------------------------------
    // Entity Creation / Destruction
    // ------------------------------------------------------------------------

    // Create a new entity with auto-generated ID
    Entity& create_entity(const std::string& name = "");

    // Create a new entity with specific ID (for deserialization)
    Entity& create_entity_with_id(EntityId id, const std::string& name = "");

    // Destroy an entity by ID
    bool destroy_entity(EntityId id);

    // Destroy an entity by name
    bool destroy_entity(const std::string& name);

    // Destroy all entities
    void clear();

    // ------------------------------------------------------------------------
    // Entity Access
    // ------------------------------------------------------------------------

    // Find entity by ID (returns nullptr if not found)
    Entity* find(EntityId id);
    const Entity* find(EntityId id) const;

    // Find entity by name (returns nullptr if not found)
    Entity* find(const std::string& name);
    const Entity* find(const std::string& name) const;

    // Check if entity exists
    bool exists(EntityId id) const;
    bool exists(const std::string& name) const;

    // Get entity count
    size_t count() const { return _entities.size(); }

    // Iterate all entities
    template <typename Func>
    void for_each(Func&& func);

    template <typename Func>
    void for_each(Func&& func) const;

    // ------------------------------------------------------------------------
    // Physics Synchronization
    // ------------------------------------------------------------------------

    // Called before physics step - store current transforms as previous
    void pre_physics_step();

    // Called after physics step - update entity transforms from physics
    void post_physics_step(Physics::PhysicsWorld& physics, const WorldVec3& physics_origin_world);

    // ------------------------------------------------------------------------
    // Render Synchronization
    // ------------------------------------------------------------------------

    // Sync all entity transforms to render instances
    // alpha: interpolation factor (0 = previous, 1 = current)
    void sync_to_render(GameAPI::Engine& api, float alpha);

    // Sync specific entity to render
    void sync_entity_to_render(Entity& entity, GameAPI::Engine& api, float alpha);

    // ------------------------------------------------------------------------
    // Convenience Methods
    // ------------------------------------------------------------------------

    // Create entity with physics body already bound
    Entity& create_entity_with_physics(const std::string& name, uint32_t physics_body_value);

    // Create entity with render instance already bound
    Entity& create_entity_with_render(const std::string& name, const std::string& render_name);

    // Create entity with both physics and render bound
    Entity& create_entity_with_physics_and_render(
        const std::string& name,
        uint32_t physics_body_value,
        const std::string& render_name);

    // Find entity by physics body
    Entity* find_by_physics_body(uint32_t physics_body_value);

    // Find entity by render name
    Entity* find_by_render_name(const std::string& render_name);

    // ------------------------------------------------------------------------
    // Bulk Operations
    // ------------------------------------------------------------------------

    // Set transform for entity and immediately sync to physics (teleport)
    void teleport(EntityId id, const WorldVec3& position_world, const glm::quat& rotation,
                  Physics::PhysicsWorld& physics, const WorldVec3& physics_origin_world);

    // Reset all interpolation states (for scene reloads, etc.)
    void reset_interpolation();

private:
    uint32_t _next_id{1};
    std::unordered_map<uint32_t, Entity> _entities;          // ID -> Entity
    std::unordered_map<std::string, uint32_t> _name_index;   // Name -> ID (for fast lookup)

    // Helper to update name index when entity name changes
    void update_name_index(EntityId id, const std::string& old_name, const std::string& new_name);
};

// ============================================================================
// Template implementations
// ============================================================================

template <typename Func>
void EntityManager::for_each(Func&& func)
{
    for (auto& [id, entity] : _entities)
    {
        func(entity);
    }
}

template <typename Func>
void EntityManager::for_each(Func&& func) const
{
    for (const auto& [id, entity] : _entities)
    {
        func(entity);
    }
}

} // namespace Game
