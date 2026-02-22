#include "main_game.h"
#include "game/states/title_screen_state.h"
#include "runtime/game_runtime.h"
#include "core/engine.h"
#include "core/input/input_system.h"

namespace Game
{
    void MainGame::on_init(GameRuntime::Runtime &runtime)
    {
        _runtime = &runtime;

        // Build shared context for all game states
        GameStateContext ctx{};
        ctx.runtime = &runtime;
        ctx.api = &runtime.api();
        ctx.audio = runtime.audio();
        ctx.renderer = runtime.renderer();

        if (VulkanEngine *r = runtime.renderer())
        {
            if (r->input())
            {
                ctx.input = &r->input()->state();
            }
        }

        _state_manager.init(ctx);

        // Register ImGui draw callback
        if (VulkanEngine *renderer = runtime.renderer())
        {
            if (renderer->ui())
            {
                renderer->ui()->addDrawCallback([this]() {
                    _state_manager.draw_ui();
                });
            }
        }

        // Start at title screen
        _state_manager.push(std::make_unique<TitleScreenState>());
    }

    void MainGame::on_update(float dt)
    {
        _state_manager.update(dt);
    }

    void MainGame::on_fixed_update(float fixed_dt)
    {
        _state_manager.fixed_update(fixed_dt);
    }

    void MainGame::on_shutdown()
    {
        _state_manager.shutdown();
        _runtime = nullptr;
    }
} // namespace Game
