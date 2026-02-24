#pragma once

#include <cstdint>
#include <typeinfo>
#include <typeindex>

// Forward declarations
class InputState;

namespace GameAPI
{
    class Engine;
} // namespace GameAPI

namespace Physics
{
    class PhysicsWorld;
} // namespace Physics

namespace Game
{
    class Entity;
    class GameWorld;

    // ============================================================================
    // ComponentContext: Shared data passed to all component callbacks
    // ============================================================================

    struct ComponentContext
    {
        GameWorld *world{nullptr};
        GameAPI::Engine *api{nullptr};
        const InputState *input{nullptr};
        Physics::PhysicsWorld *physics{nullptr};
        bool ui_capture_keyboard{false};
        float interpolation_alpha{0.0f};
    };

    // ============================================================================
    // IComponent: Base interface for entity behaviors
    // ============================================================================

    class IComponent
    {
    public:
        virtual ~IComponent() = default;

        // Lifecycle hooks (override as needed)
        virtual void on_init(ComponentContext &ctx) { (void)ctx; }
        virtual void on_update(ComponentContext &ctx, float dt) { (void)ctx; (void)dt; }
        virtual void on_fixed_update(ComponentContext &ctx, float fixed_dt) { (void)ctx; (void)fixed_dt; }
        virtual void on_destroy(ComponentContext &ctx) { (void)ctx; }

        // Enable/disable (disabled components skip updates)
        bool is_enabled() const { return _enabled; }
        void set_enabled(bool enabled) { _enabled = enabled; }

        // Type identity for runtime lookup
        virtual std::type_index component_type() const = 0;

        // Owner access
        Entity *entity() const { return _entity; }

    protected:
        Entity *_entity{nullptr};
        bool _enabled{true};

        friend class Entity;
    };

    // ============================================================================
    // Component<T>: CRTP base — provides automatic type identity
    //
    // Usage:
    //   class ShipController : public Component<ShipController> { ... };
    // ============================================================================

    template<typename T>
    class Component : public IComponent
    {
    public:
        std::type_index component_type() const override
        {
            return std::type_index(typeid(T));
        }
    };
} // namespace Game
