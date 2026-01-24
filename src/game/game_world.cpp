#include "game_world.h"

#include "core/game_api.h"
#include "physics/physics_world.h"

#include <unordered_set>

namespace Game
{
    namespace
    {
        GameAPI::Transform to_api_transform(const Transform &t)
        {
            GameAPI::Transform out{};
            out.position = t.position;
            out.rotation = t.rotation;
            out.scale = t.scale;
            return out;
        }

        void remove_any_instance(GameAPI::Engine &api, const std::string &name)
        {
            if (name.empty())
            {
                return;
            }
            (void) api.remove_mesh_instance(name);
            (void) api.remove_gltf_instance(name);
        }
    } // namespace

    GameWorld::GameWorld(GameAPI::Engine *api, Physics::PhysicsWorld *physics)
        : _api(api), _physics(physics)
    {
    }

    bool GameWorld::destroy_entity(EntityId id)
    {
        Entity *entity = _entities.find(id);
        if (!entity)
        {
            return false;
        }

        destroy_entity_resources(*entity);
        return _entities.destroy_entity(id);
    }

    void GameWorld::clear()
    {
        std::vector<EntityId> ids;
        ids.reserve(_entities.count());

        _entities.for_each([&](const Entity &entity) { ids.push_back(entity.id()); });

        for (EntityId id: ids)
        {
            (void) destroy_entity(id);
        }
    }

    Entity *GameWorld::spawn_primitive(const std::string &name, GameAPI::PrimitiveType type, const Transform &transform)
    {
        if (!_api)
        {
            return nullptr;
        }
        if (!name.empty() && _entities.exists(name))
        {
            return nullptr;
        }

        Entity &entity = _entities.create_entity(name);
        entity.set_transform(transform);
        entity.set_render_name(name);

        if (!_api->add_primitive_instance(name, type, to_api_transform(transform)))
        {
            _entities.destroy_entity(entity.id());
            return nullptr;
        }

        return &entity;
    }

    Entity *GameWorld::spawn_primitive_rigid_body(const std::string &name, GameAPI::PrimitiveType type,
                                                  const Transform &transform,
                                                  const Physics::BodySettings &body_settings_template,
                                                  bool override_user_data)
    {
        if (!_api || !_physics)
        {
            return nullptr;
        }
        if (!name.empty() && _entities.exists(name))
        {
            return nullptr;
        }

        Entity &entity = _entities.create_entity(name);
        entity.set_transform(transform);
        entity.set_render_name(name);

        if (!_api->add_primitive_instance(name, type, to_api_transform(transform)))
        {
            _entities.destroy_entity(entity.id());
            return nullptr;
        }

        Physics::BodySettings settings = body_settings_template;
        const WorldVec3 physics_origin_world = WorldVec3(_api->get_physics_origin());
        settings.position = WorldVec3(transform.position) - physics_origin_world;
        settings.rotation = transform.rotation;
        if (override_user_data || settings.user_data == 0)
        {
            settings.user_data = static_cast<uint64_t>(entity.id().value);
        }

        Physics::BodyId body_id = _physics->create_body(settings);
        if (!body_id.is_valid())
        {
            remove_any_instance(*_api, name);
            _entities.destroy_entity(entity.id());
            return nullptr;
        }

        if (override_user_data)
        {
            _physics->set_user_data(body_id, static_cast<uint64_t>(entity.id().value));
        }

        entity.set_physics_body(body_id.value);
        entity.set_use_interpolation(true);
        entity.interpolation().set_immediate(transform.position, transform.rotation);

        return &entity;
    }

    bool GameWorld::bind_render(EntityId id, const std::string &render_name)
    {
        Entity *entity = _entities.find(id);
        if (!entity)
        {
            return false;
        }
        entity->set_render_name(render_name);
        return true;
    }

    bool GameWorld::bind_physics(EntityId id, uint32_t body_value, bool use_interpolation, bool override_user_data)
    {
        Entity *entity = _entities.find(id);
        if (!entity)
        {
            return false;
        }

        entity->set_physics_body(body_value);
        entity->set_use_interpolation(use_interpolation);

        if (use_interpolation)
        {
            entity->interpolation().set_immediate(entity->position(), entity->rotation());
        }

        if (_physics && override_user_data)
        {
            Physics::BodyId body_id{body_value};
            if (_physics->is_body_valid(body_id))
            {
                _physics->set_user_data(body_id, static_cast<uint64_t>(entity->id().value));
            }
        }

        return true;
    }

    void GameWorld::destroy_entity_resources(Entity &entity)
    {
        if (_api)
        {
            if (entity.has_render())
            {
                remove_any_instance(*_api, entity.render_name());
            }

            for (const Attachment &att: entity.attachments())
            {
                remove_any_instance(*_api, att.render_name);
            }
        }

        if (_physics)
        {
            std::unordered_set<uint32_t> bodies;
            bodies.reserve(entity.attachments().size() + 1);

            if (entity.has_physics())
            {
                bodies.insert(entity.physics_body_value());
            }

            for (const Attachment &att: entity.attachments())
            {
                if (att.physics_body_value.has_value())
                {
                    bodies.insert(att.physics_body_value.value());
                }
            }

            for (uint32_t body_value: bodies)
            {
                Physics::BodyId body_id{body_value};
                if (_physics->is_body_valid(body_id))
                {
                    _physics->destroy_body(body_id);
                }
            }
        }
    }
} // namespace Game
