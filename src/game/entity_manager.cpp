#include "entity_manager.h"
#include "physics/physics_world.h"
#include "physics/physics_body.h"
#include "core/game_api.h"

namespace Game
{

// ============================================================================
// Entity Creation / Destruction
// ============================================================================

Entity& EntityManager::create_entity(const std::string& name)
{
    EntityId id{_next_id++};
    return create_entity_with_id(id, name);
}

Entity& EntityManager::create_entity_with_id(EntityId id, const std::string& name)
{
    // Ensure ID is at least as high as next_id
    if (id.value >= _next_id)
    {
        _next_id = id.value + 1;
    }

    auto [it, inserted] = _entities.emplace(id.value, Entity(id, name));
    Entity& entity = it->second;

    // Add to name index if named
    if (!name.empty())
    {
        _name_index[name] = id.value;
    }

    return entity;
}

bool EntityManager::destroy_entity(EntityId id)
{
    auto it = _entities.find(id.value);
    if (it == _entities.end())
    {
        return false;
    }

    // Remove from name index
    const std::string& name = it->second.name();
    if (!name.empty())
    {
        _name_index.erase(name);
    }

    _entities.erase(it);
    return true;
}

bool EntityManager::destroy_entity(const std::string& name)
{
    auto it = _name_index.find(name);
    if (it == _name_index.end())
    {
        return false;
    }

    uint32_t id = it->second;
    _name_index.erase(it);
    _entities.erase(id);
    return true;
}

void EntityManager::clear()
{
    _entities.clear();
    _name_index.clear();
    // Don't reset _next_id to avoid ID reuse issues
}

// ============================================================================
// Entity Access
// ============================================================================

Entity* EntityManager::find(EntityId id)
{
    auto it = _entities.find(id.value);
    return (it != _entities.end()) ? &it->second : nullptr;
}

const Entity* EntityManager::find(EntityId id) const
{
    auto it = _entities.find(id.value);
    return (it != _entities.end()) ? &it->second : nullptr;
}

Entity* EntityManager::find(const std::string& name)
{
    auto it = _name_index.find(name);
    if (it == _name_index.end())
    {
        return nullptr;
    }
    return find(EntityId{it->second});
}

const Entity* EntityManager::find(const std::string& name) const
{
    auto it = _name_index.find(name);
    if (it == _name_index.end())
    {
        return nullptr;
    }
    return find(EntityId{it->second});
}

bool EntityManager::exists(EntityId id) const
{
    return _entities.find(id.value) != _entities.end();
}

bool EntityManager::exists(const std::string& name) const
{
    return _name_index.find(name) != _name_index.end();
}

// ============================================================================
// Physics Synchronization
// ============================================================================

void EntityManager::pre_physics_step()
{
    for (auto& [id, entity] : _entities)
    {
        if (!entity.is_active() || !entity.uses_interpolation())
        {
            continue;
        }

        entity.interpolation().store_current_as_previous();
    }
}

void EntityManager::post_physics_step(Physics::PhysicsWorld& physics, const WorldVec3& physics_origin_world)
{
    for (auto& [id, entity] : _entities)
    {
        if (!entity.is_active() || !entity.has_physics())
        {
            continue;
        }

        Physics::BodyId body_id{entity.physics_body_value()};
        if (!physics.is_body_valid(body_id))
        {
            continue;
        }

        Physics::BodyTransform transform = physics.get_transform(body_id);

        const WorldVec3 position_world = physics_origin_world + transform.position;

        // Update entity transform
        entity.set_position(glm::vec3(position_world));
        entity.set_rotation(transform.rotation);

        // Update interpolation state
        if (entity.uses_interpolation())
        {
            entity.interpolation().curr_position = glm::vec3(position_world);
            entity.interpolation().curr_rotation = transform.rotation;
        }
    }
}

// ============================================================================
// Render Synchronization
// ============================================================================

void EntityManager::sync_to_render(GameAPI::Engine& api, float alpha)
{
    for (auto& [id, entity] : _entities)
    {
        if (!entity.is_active())
        {
            continue;
        }

        sync_entity_to_render(entity, api, alpha);
    }
}

void EntityManager::sync_entity_to_render(Entity& entity, GameAPI::Engine& api, float alpha)
{
    if (!entity.has_render() || !entity.is_visible())
    {
        return;
    }

    // Get render transform (interpolated if using interpolation)
    GameAPI::Transform tr;
    tr.position = entity.get_render_position(alpha);
    tr.rotation = entity.get_render_rotation(alpha);
    tr.scale = entity.scale();

    api.set_mesh_instance_transform(entity.render_name(), tr);

    // Sync attachments
    glm::mat4 parent_matrix = entity.get_render_matrix(alpha);

    for (const Attachment& att : entity.attachments())
    {
        if (!att.visible || att.render_name.empty())
        {
            continue;
        }

        glm::mat4 world_matrix = parent_matrix * att.get_local_matrix();
        GameAPI::Transform att_tr = GameAPI::Transform::from_matrix(world_matrix);
        api.set_mesh_instance_transform(att.render_name, att_tr);
    }
}

// ============================================================================
// Convenience Methods
// ============================================================================

Entity& EntityManager::create_entity_with_physics(const std::string& name, uint32_t physics_body_value)
{
    Entity& entity = create_entity(name);
    entity.set_physics_body(physics_body_value);
    entity.set_use_interpolation(true);
    return entity;
}

Entity& EntityManager::create_entity_with_render(const std::string& name, const std::string& render_name)
{
    Entity& entity = create_entity(name);
    entity.set_render_name(render_name);
    return entity;
}

Entity& EntityManager::create_entity_with_physics_and_render(
    const std::string& name,
    uint32_t physics_body_value,
    const std::string& render_name)
{
    Entity& entity = create_entity(name);
    entity.set_physics_body(physics_body_value);
    entity.set_render_name(render_name);
    entity.set_use_interpolation(true);
    return entity;
}

Entity* EntityManager::find_by_physics_body(uint32_t physics_body_value)
{
    for (auto& [id, entity] : _entities)
    {
        if (entity.has_physics() && entity.physics_body_value() == physics_body_value)
        {
            return &entity;
        }
    }
    return nullptr;
}

Entity* EntityManager::find_by_render_name(const std::string& render_name)
{
    for (auto& [id, entity] : _entities)
    {
        if (entity.render_name() == render_name)
        {
            return &entity;
        }
    }
    return nullptr;
}

// ============================================================================
// Bulk Operations
// ============================================================================

void EntityManager::teleport(EntityId id, const glm::vec3& position, const glm::quat& rotation,
                             Physics::PhysicsWorld& physics, const WorldVec3& physics_origin_world)
{
    Entity* entity = find(id);
    if (!entity)
    {
        return;
    }

    // Update entity transform
    entity->set_position(position);
    entity->set_rotation(rotation);

    // Reset interpolation to avoid visual blending
    if (entity->uses_interpolation())
    {
        entity->interpolation().set_immediate(position, rotation);
    }

    // Update physics body
    if (entity->has_physics())
    {
        Physics::BodyId body_id{entity->physics_body_value()};
        if (physics.is_body_valid(body_id))
        {
            const glm::dvec3 position_local = WorldVec3(position) - physics_origin_world;
            physics.set_transform(body_id, position_local, rotation);
            physics.set_linear_velocity(body_id, glm::vec3(0.0f));
            physics.set_angular_velocity(body_id, glm::vec3(0.0f));
            physics.activate(body_id);
        }
    }
}

void EntityManager::reset_interpolation()
{
    for (auto& [id, entity] : _entities)
    {
        if (entity.uses_interpolation())
        {
            entity.interpolation().set_immediate(entity.position(), entity.rotation());
        }
    }
}

void EntityManager::update_name_index(EntityId id, const std::string& old_name, const std::string& new_name)
{
    if (!old_name.empty())
    {
        _name_index.erase(old_name);
    }

    if (!new_name.empty())
    {
        _name_index[new_name] = id.value;
    }
}

} // namespace Game
