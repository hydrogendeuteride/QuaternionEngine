#pragma once

#include "game_state.h"
#include "game/game_world.h"
#include "game/component/component.h"
#include "physics/physics_world.h"
#include "physics/physics_context.h"

#include <memory>

namespace Game
{

// ============================================================================
// GameplayState: Main gameplay — orbital mechanics, combat, ship control
//
// This is where the actual game simulation lives.
// Owns GameWorld, physics, and orbital simulation.
// ============================================================================

class GameplayState : public IGameState
{
public:
    void on_enter(GameStateContext &ctx) override;
    void on_exit(GameStateContext &ctx) override;
    void on_update(GameStateContext &ctx, float dt) override;
    void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;
    void on_draw_ui(GameStateContext &ctx) override;

    bool wants_fixed_update() const override { return true; }
    const char *name() const override { return "Gameplay"; }

private:
    void setup_scene(GameStateContext &ctx);
    ComponentContext build_component_context(GameStateContext &ctx, float alpha = 0.0f);

    // Game world (entities + resource lifetime)
    GameWorld _world;

    // Physics
    std::unique_ptr<Physics::PhysicsWorld> _physics;
    std::unique_ptr<Physics::PhysicsContext> _physics_context;

    // Timing
    float _elapsed{0.0f};
};

} // namespace Game
