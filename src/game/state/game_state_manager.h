#pragma once

#include "game_state.h"
#include <vector>
#include <memory>

namespace Game
{

// ============================================================================
// GameStateManager: Stack-based game state management
//
// - Top of stack receives update/draw calls
// - Push: new state on top (previous pauses)
// - Pop: remove top state (return to previous)
// - Switch: clear entire stack, push new state
// - Overlay states: is_overlay() = true, state below continues to render
// ============================================================================

class GameStateManager
{
public:
    GameStateManager() = default;
    ~GameStateManager();

    // Non-copyable
    GameStateManager(const GameStateManager &) = delete;
    GameStateManager &operator=(const GameStateManager &) = delete;

    // Initialize with context (must be called before any updates)
    void init(GameStateContext ctx);

    // State manipulation (immediate)
    void push(std::unique_ptr<IGameState> state);
    void pop();
    void switch_to(std::unique_ptr<IGameState> state);

    // Per-frame update: updates top state, then processes pending transition
    void update(float dt);

    // Fixed-timestep update: only if top state wants it
    void fixed_update(float fixed_dt);

    // ImGui draw: draws overlay-visible states + top state
    void draw_ui();

    // Shutdown: pops all states
    void shutdown();

    // Query
    bool empty() const { return _stack.empty(); }
    size_t depth() const { return _stack.size(); }
    IGameState *top() const;
    const GameStateContext &context() const { return _ctx; }

private:
    void process_transition(StateTransition &transition);

    std::vector<std::unique_ptr<IGameState>> _stack;
    GameStateContext _ctx;
};

} // namespace Game
