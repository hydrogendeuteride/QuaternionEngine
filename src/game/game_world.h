#pragma once

#include "entity_manager.h"

#include <string>
#include <vector>

namespace Physics
{
    class PhysicsWorld;
    struct BodySettings;
} // namespace Physics

namespace GameAPI
{
    class Engine;
    enum class PrimitiveType;
} // namespace GameAPI

namespace Game
{
    // ============================================================================
    // GameWorld: Owns entities and manages their bound render/physics resources.
    // ============================================================================

    class GameWorld
    {
    public:
        GameWorld() = default;

        explicit GameWorld(GameAPI::Engine *api, Physics::PhysicsWorld *physics = nullptr);

        void set_api(GameAPI::Engine *api) { _api = api; }
        void set_physics(Physics::PhysicsWorld *physics) { _physics = physics; }

        GameAPI::Engine *api() const { return _api; }
        Physics::PhysicsWorld *physics() const { return _physics; }

        EntityManager &entities() { return _entities; }
        const EntityManager &entities() const { return _entities; }

        // ------------------------------------------------------------------------
        // Lifecycle
        // ------------------------------------------------------------------------

        bool destroy_entity(EntityId id);

        void clear();

        // ------------------------------------------------------------------------
        // Spawning helpers (simple, single-source-of-truth)
        // ------------------------------------------------------------------------

        Entity *spawn_primitive(const std::string &name, GameAPI::PrimitiveType type, const Transform &transform);

        Entity *spawn_primitive_rigid_body(const std::string &name, GameAPI::PrimitiveType type,
                                           const Transform &transform,
                                           const Physics::BodySettings &body_settings_template,
                                           bool override_user_data = true);

        // ------------------------------------------------------------------------
        // Binding existing resources (for incremental adoption)
        // ------------------------------------------------------------------------

        bool bind_render(EntityId id, const std::string &render_name);

        bool bind_physics(EntityId id, uint32_t body_value, bool use_interpolation = true,
                          bool override_user_data = true);

    private:
        EntityManager _entities;
        GameAPI::Engine *_api{nullptr};
        Physics::PhysicsWorld *_physics{nullptr};

        void destroy_entity_resources(Entity &entity);
    };
} // namespace Game

