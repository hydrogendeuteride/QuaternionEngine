#pragma once

#include "game_state.h"

namespace Game
{

// ============================================================================
// SettingsState: Graphics, audio, controls settings
//
// is_overlay() = true — renders on top of whatever called it
// (title screen or pause menu)
// ============================================================================

class SettingsState : public IGameState
{
public:
    void on_enter(GameStateContext &ctx) override;
    void on_exit(GameStateContext &ctx) override;
    void on_update(GameStateContext &ctx, float dt) override;
    void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;
    void on_draw_ui(GameStateContext &ctx) override;

    bool wants_fixed_update() const override { return false; }
    bool is_overlay() const override { return true; }
    const char *name() const override { return "Settings"; }

private:
    // Settings state (local copy, applied on "Apply")
    float _master_volume{1.0f};
    float _sfx_volume{1.0f};
    float _bgm_volume{1.0f};
};

} // namespace Game
