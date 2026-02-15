#pragma once

#include <memory>
#include <string>
#include <functional>

// Forward declarations
namespace GameRuntime
{
    class Runtime;
    class IAudioSystem;
} // namespace GameRuntime

namespace GameAPI
{
    class Engine;
} // namespace GameAPI

class VulkanEngine;
class InputState;

namespace Game
{
    class IGameState;

    // ============================================================================
    // StateTransition: Describes the desired state change after a frame
    // ============================================================================

    struct StateTransition
    {
        enum class Type
        {
            None, // Stay in current state
            Push, // Push a new state on top (current pauses)
            Pop, // Remove current state (return to previous)
            Switch, // Replace entire stack with new state
        };

        Type type{Type::None};

        // Factory that creates the new state (for Push / Switch)
        std::function<std::unique_ptr<IGameState>()> factory;

        // Convenience constructors
        static StateTransition none() { return {Type::None, nullptr}; }
        static StateTransition pop() { return {Type::Pop, nullptr}; }

        template<typename T, typename... Args>
        static StateTransition push(Args &&... args)
        {
            return {
                Type::Push, [... args = std::forward<Args>(args)]() mutable {
                    return std::make_unique<T>(std::move(args)...);
                }
            };
        }

        template<typename T, typename... Args>
        static StateTransition switch_to(Args &&... args)
        {
            return {
                Type::Switch, [... args = std::forward<Args>(args)]() mutable {
                    return std::make_unique<T>(std::move(args)...);
                }
            };
        }
    };

    // ============================================================================
    // GameStateContext: Shared data accessible by all game states
    // ============================================================================

    struct GameStateContext
    {
        GameRuntime::Runtime *runtime{nullptr};
        GameAPI::Engine *api{nullptr};
        GameRuntime::IAudioSystem *audio{nullptr};
        VulkanEngine *renderer{nullptr};
        const InputState *input{nullptr};

        // Request application exit
        void quit();

        // Convenience: delta time from runtime
        float delta_time() const;

        float fixed_delta_time() const;

        float interpolation_alpha() const;
    };

    // ============================================================================
    // IGameState: Interface for each game screen / mode
    // ============================================================================

    class IGameState
    {
    public:
        virtual ~IGameState() = default;

        // Lifecycle
        virtual void on_enter(GameStateContext &ctx) = 0;

        virtual void on_exit(GameStateContext &ctx) = 0;

        // Per-frame update (variable dt)
        virtual void on_update(GameStateContext &ctx, float dt) = 0;

        // Fixed-timestep update (physics / simulation)
        virtual void on_fixed_update(GameStateContext &ctx, float fixed_dt) = 0;

        // ImGui drawing
        virtual void on_draw_ui(GameStateContext &ctx) = 0;

        // Does this state need fixed_update calls? (default: false)
        // TitleScreen = false, GameplayState = true
        virtual bool wants_fixed_update() const { return false; }

        // Should the state below this one continue rendering?
        // Useful for transparent overlays (pause menu over gameplay)
        virtual bool is_overlay() const { return false; }

        // Human-readable name for debug display
        virtual const char *name() const = 0;

        // Called each frame; return a transition to change state
        // This is checked AFTER on_update, so set your transition in on_update
        StateTransition &pending_transition() { return _pending; }

    protected:
        // States set this to request a transition
        StateTransition _pending{StateTransition::none()};
    };
} // namespace Game
