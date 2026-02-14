#pragma once

#include "runtime/i_game_callbacks.h"
#include "state/game_state_manager.h"

namespace Game
{
    // ============================================================================
    // SpaceCombatGame: Top-level game class
    //
    // Implements IGameCallbacks to connect with the engine runtime.
    // Delegates all logic to GameStateManager, which dispatches to the
    // active IGameState (TitleScreen, Gameplay, Pause, Settings, etc.)
    // ============================================================================

    class MainGame : public GameRuntime::IGameCallbacks
    {
    public:
        MainGame() = default;
        ~MainGame() override = default;

        // IGameCallbacks
        void on_init(GameRuntime::Runtime &runtime) override;
        void on_update(float dt) override;
        void on_fixed_update(float fixed_dt) override;
        void on_shutdown() override;

    private:
        GameRuntime::Runtime *_runtime{nullptr};
        GameStateManager _state_manager;
    };
} // namespace Game
