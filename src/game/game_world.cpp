#include "game_world.h"

#include "core/game_api.h"
#include "physics/physics_world.h"

#include <unordered_set>
#include <utility>

namespace Game
{
    namespace
    {
        GameAPI::TransformD to_api_transform_d(const Transform &t)
        {
            GameAPI::TransformD out{};
            out.position = glm::dvec3(t.position_world);
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

        if (_rebase_anchor == id)
        {
            clear_rebase_anchor();
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

    void GameWorld::pre_physics_step()
    {
        _entities.pre_physics_step();

        if (!_physics || !_api || !_rebase_anchor.is_valid())
        {
            return;
        }

        Entity *anchor = _entities.find(_rebase_anchor);
        if (!anchor || !anchor->has_physics())
        {
            return;
        }

        const uint32_t body_value = anchor->physics_body_value();
        if (_rebase_settings.origin_threshold_m > 0.0)
        {
            (void) _api->maybe_rebase_physics_origin_to_body(body_value,
                                                            _rebase_settings.origin_threshold_m,
                                                            _rebase_settings.origin_snap_m);
        }

        if (_rebase_settings.velocity_threshold_mps > 0.0)
        {
            (void) _api->maybe_rebase_physics_velocity_to_body(body_value,
                                                              _rebase_settings.velocity_threshold_mps);
        }
    }

    void GameWorld::post_physics_step()
    {
        if (!_physics)
        {
            return;
        }

        const WorldVec3 physics_origin_world =
            _api ? WorldVec3(_api->get_physics_origin()) : WorldVec3{0.0, 0.0, 0.0};
        _entities.post_physics_step(*_physics, physics_origin_world);
    }

    GameWorld::EntityBuilder GameWorld::builder(const std::string &name)
    {
        return EntityBuilder(*this, name);
    }

    GameWorld::EntityBuilder::EntityBuilder(GameWorld &world, std::string name)
        : _world(&world), _name(std::move(name))
    {
    }

    GameWorld::EntityBuilder &GameWorld::EntityBuilder::transform(const Transform &transform)
    {
        _transform = transform;
        return *this;
    }

    GameWorld::EntityBuilder &GameWorld::EntityBuilder::render_primitive(GameAPI::PrimitiveType type)
    {
        _render_kind = RenderKind::Primitive;
        _primitive_type = type;
        return *this;
    }

    GameWorld::EntityBuilder &GameWorld::EntityBuilder::render_gltf(const std::string &path, bool preload_textures)
    {
        _render_kind = RenderKind::GLTF;
        _gltf_path = path;
        _gltf_preload = preload_textures;
        return *this;
    }

    GameWorld::EntityBuilder &GameWorld::EntityBuilder::physics(const Physics::BodySettings &settings,
                                                                bool use_interpolation,
                                                                bool override_user_data)
    {
        _wants_physics = true;
        _physics_settings = settings;
        _use_interpolation = use_interpolation;
        _override_user_data = override_user_data;
        return *this;
    }

    Entity *GameWorld::EntityBuilder::build()
    {
        if (!_world)
        {
            return nullptr;
        }

        GameWorld &world = *_world;

        if (!world._api && _render_kind != RenderKind::None)
        {
            return nullptr;
        }

        if (!world._physics && _wants_physics)
        {
            return nullptr;
        }

        if (!_name.empty() && world._entities.exists(_name))
        {
            return nullptr;
        }

        if (_render_kind != RenderKind::None && _name.empty())
        {
            return nullptr;
        }

        Entity &entity = world._entities.create_entity(_name);
        entity.set_transform(_transform);

        if (_render_kind == RenderKind::Primitive)
        {
            entity.set_render_name(_name);
            if (!world._api->add_primitive_instance(_name, _primitive_type, to_api_transform_d(_transform)))
            {
                world._entities.destroy_entity(entity.id());
                return nullptr;
            }
        }
        else if (_render_kind == RenderKind::GLTF)
        {
            if (_gltf_path.empty())
            {
                world._entities.destroy_entity(entity.id());
                return nullptr;
            }

            entity.set_render_name(_name);
            if (!world._api->add_gltf_instance(_name, _gltf_path, to_api_transform_d(_transform), _gltf_preload))
            {
                world._entities.destroy_entity(entity.id());
                return nullptr;
            }
        }

        if (_wants_physics)
        {
            Physics::BodySettings settings = _physics_settings;
            const WorldVec3 physics_origin_world =
                    world._api ? WorldVec3(world._api->get_physics_origin()) : WorldVec3{0.0, 0.0, 0.0};
            settings.position = world_to_local_d(_transform.position_world, physics_origin_world);
            settings.rotation = _transform.rotation;
            if (_override_user_data || settings.user_data == 0)
            {
                settings.user_data = static_cast<uint64_t>(entity.id().value);
            }

            Physics::BodyId body_id = world._physics->create_body(settings);
            if (!body_id.is_valid())
            {
                if (world._api)
                {
                    remove_any_instance(*world._api, _name);
                }
                world._entities.destroy_entity(entity.id());
                return nullptr;
            }

            if (_override_user_data)
            {
                world._physics->set_user_data(body_id, static_cast<uint64_t>(entity.id().value));
            }

            entity.set_physics_body(body_id.value);
            entity.set_use_interpolation(_use_interpolation);
            if (_use_interpolation)
            {
                entity.interpolation().set_immediate(_transform.position_world, _transform.rotation);
            }
        }

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
            entity->interpolation().set_immediate(entity->position_world(), entity->rotation());
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
