#pragma once

#include "entity_manager.h"
#include "physics/body_settings.h"

#include <string>
#include <vector>

namespace Physics
{
    class PhysicsWorld;
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
        class EntityBuilder;

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
        // Physics sync + rebasing
        // ------------------------------------------------------------------------

        struct RebaseSettings
        {
            double origin_threshold_m{0.0};
            double origin_snap_m{0.0};
            double velocity_threshold_mps{0.0};
        };

        void set_rebase_settings(const RebaseSettings &settings) { _rebase_settings = settings; }
        const RebaseSettings &rebase_settings() const { return _rebase_settings; }

        void set_rebase_anchor(EntityId id) { _rebase_anchor = id; }
        void clear_rebase_anchor() { _rebase_anchor = EntityId{}; }
        EntityId rebase_anchor() const { return _rebase_anchor; }

        // Call before physics step.
        void pre_physics_step();

        // Call after physics step.
        void post_physics_step();

        // ------------------------------------------------------------------------
        // Entity builder
        // ------------------------------------------------------------------------

        EntityBuilder builder(const std::string &name);

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
        EntityId _rebase_anchor;
        RebaseSettings _rebase_settings{};

        void destroy_entity_resources(Entity &entity);
    };

    class GameWorld::EntityBuilder
    {
    public:
        EntityBuilder(GameWorld &world, std::string name);

        EntityBuilder &transform(const Transform &transform);

        EntityBuilder &render_primitive(GameAPI::PrimitiveType type);

        EntityBuilder &render_gltf(const std::string &path, bool preload_textures = true);

        EntityBuilder &physics(const Physics::BodySettings &settings,
                               bool use_interpolation = true,
                               bool override_user_data = true);

        Entity *build();

    private:
        GameWorld *_world{nullptr};
        std::string _name;
        Transform _transform{};

        enum class RenderKind
        {
            None,
            Primitive,
            GLTF
        };

        RenderKind _render_kind{RenderKind::None};
        GameAPI::PrimitiveType _primitive_type{};
        std::string _gltf_path{};
        bool _gltf_preload{true};

        bool _wants_physics{false};
        Physics::BodySettings _physics_settings{};
        bool _use_interpolation{true};
        bool _override_user_data{true};
    };
} // namespace Game
