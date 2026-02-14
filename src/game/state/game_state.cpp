#include "game_state.h"
#include "runtime/game_runtime.h"

namespace Game
{
    void GameStateContext::quit()
    {
        if (runtime)
        {
            runtime->request_quit();
        }
    }

    float GameStateContext::delta_time() const
    {
        return runtime ? runtime->delta_time() : 0.0f;
    }

    float GameStateContext::fixed_delta_time() const
    {
        return runtime ? runtime->fixed_delta_time() : 0.0f;
    }

    float GameStateContext::interpolation_alpha() const
    {
        return runtime ? runtime->interpolation_alpha() : 1.0f;
    }
} // namespace Game
