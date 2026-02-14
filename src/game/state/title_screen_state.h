#pragma once

#include "game_state.h"

namespace Game
{

// ============================================================================
// TitleScreenState: Main menu — New Game, Load, Settings, Quit
// ============================================================================

class TitleScreenState : public IGameState
{
public:
    void on_enter(GameStateContext &ctx) override;
    void on_exit(GameStateContext &ctx) override;
    void on_update(GameStateContext &ctx, float dt) override;
    void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;
    void on_draw_ui(GameStateContext &ctx) override;

    bool wants_fixed_update() const override { return false; }
    const char *name() const override { return "TitleScreen"; }
};

} // namespace Game
