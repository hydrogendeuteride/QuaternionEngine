#include "game_state_manager.h"

#include <fmt/core.h>
#include "core/util/logger.h"
#include <cassert>

namespace Game
{

GameStateManager::~GameStateManager()
{
    shutdown();
}

void GameStateManager::init(GameStateContext ctx)
{
    _ctx = ctx;
}

void GameStateManager::push(std::unique_ptr<IGameState> state)
{
    assert(state && "Cannot push null state");
    Logger::info("[StateManager] Push: {}", state->name());

    state->on_enter(_ctx);
    _stack.push_back(std::move(state));
}

void GameStateManager::pop()
{
    if (_stack.empty())
    {
        return;
    }

    auto &top_state = _stack.back();
    Logger::info("[StateManager] Pop: {}", top_state->name());

    top_state->on_exit(_ctx);
    _stack.pop_back();
}

void GameStateManager::switch_to(std::unique_ptr<IGameState> state)
{
    assert(state && "Cannot switch to null state");
    Logger::info("[StateManager] Switch to: {}", state->name());

    // Pop all existing states (in reverse order)
    while (!_stack.empty())
    {
        auto &top_state = _stack.back();
        top_state->on_exit(_ctx);
        _stack.pop_back();
    }

    // Push the new state
    state->on_enter(_ctx);
    _stack.push_back(std::move(state));
}

void GameStateManager::update(float dt)
{
    if (_stack.empty())
    {
        return;
    }

    // Update only the top state
    auto &top_state = _stack.back();
    top_state->on_update(_ctx, dt);

    // Process any transition the state requested
    auto &transition = top_state->pending_transition();
    if (transition.type != StateTransition::Type::None)
    {
        process_transition(transition);
    }
}

void GameStateManager::fixed_update(float fixed_dt)
{
    if (_stack.empty())
    {
        return;
    }

    auto &top_state = _stack.back();
    if (top_state->wants_fixed_update())
    {
        top_state->on_fixed_update(_ctx, fixed_dt);
    }
}

void GameStateManager::draw_ui()
{
    if (_stack.empty())
    {
        return;
    }

    // Find the lowest visible state (walk down from top through overlays)
    size_t first_visible = _stack.size() - 1;
    while (first_visible > 0 && _stack[first_visible]->is_overlay())
    {
        --first_visible;
    }

    // Draw from bottom-visible to top
    for (size_t i = first_visible; i < _stack.size(); ++i)
    {
        _stack[i]->on_draw_ui(_ctx);
    }
}

void GameStateManager::shutdown()
{
    while (!_stack.empty())
    {
        pop();
    }
}

IGameState *GameStateManager::top() const
{
    return _stack.empty() ? nullptr : _stack.back().get();
}

void GameStateManager::process_transition(StateTransition &transition)
{
    StateTransition t = std::move(transition);
    transition = StateTransition::none();

    switch (t.type)
    {
        case StateTransition::Type::None:
            break;

        case StateTransition::Type::Push:
            if (t.factory)
            {
                push(t.factory());
            }
            break;

        case StateTransition::Type::Pop:
            pop();
            break;

        case StateTransition::Type::Switch:
            if (t.factory)
            {
                switch_to(t.factory());
            }
            break;
    }
}

} // namespace Game
