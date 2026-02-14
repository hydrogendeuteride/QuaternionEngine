#pragma once

#include "game_state.h"

namespace Game
{

// ============================================================================
// PauseState: Overlay on top of GameplayState
//
// is_overlay() = true so the gameplay scene still renders underneath.
// wants_fixed_update() = false so physics/simulation freezes.
// ============================================================================

class PauseState : public IGameState
{
public:
    void on_enter(GameStateContext &ctx) override;
    void on_exit(GameStateContext &ctx) override;
    void on_update(GameStateContext &ctx, float dt) override;
    void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;
    void on_draw_ui(GameStateContext &ctx) override;

    bool wants_fixed_update() const override { return false; }
    bool is_overlay() const override { return true; }
    const char *name() const override { return "Pause"; }
};

} // namespace Game
