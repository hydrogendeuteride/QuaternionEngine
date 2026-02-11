// Two modes available:
// 1. Legacy mode: Uses VulkanEngine::run() directly (simple, no game separation)
// 2. GameRuntime mode: Uses GameRuntime for clean game/engine separation
//
// Set USE_GAME_RUNTIME to 1
// Set USE_ENTITY_SYSTEM to 1 to use the new Entity-based example game

#define USE_GAME_RUNTIME 1
#define USE_ENTITY_SYSTEM 1

#include "core/engine.h"

#if USE_GAME_RUNTIME
#include "runtime/game_runtime.h"
#include "audio/miniaudio_system.h"

#if USE_ENTITY_SYSTEM
#include "game/example_game.h"
#include "game/rebasing_test_game.h"
#else
#endif // USE_ENTITY_SYSTEM
#endif // USE_GAME_RUNTIME

#include <memory>
#include <string>

int main(int argc, char *argv[])
{
    VulkanEngine engine;
    engine.init();

#if USE_GAME_RUNTIME
    {
        Audio::MiniAudioSystem audio;
        audio.init();

        GameRuntime::Runtime runtime(&engine);
        runtime.set_audio_system(&audio);
#if USE_ENTITY_SYSTEM
        std::string game_name = "example";
        for (int i = 1; i < argc; ++i)
        {
            const char *arg = argv[i];
            if (!arg)
            {
                continue;
            }

            const std::string a(arg);
            constexpr const char *prefix = "--game=";
            if (a.rfind(prefix, 0) == 0)
            {
                game_name = a.substr(std::char_traits<char>::length(prefix));
            }
        }

        std::unique_ptr<GameRuntime::IGameCallbacks> game;
        if (game_name == "rebase" || game_name == "rebasing" || game_name == "rebasing_test")
        {
            game = std::make_unique<Game::RebasingTestGame>();
        }
        else
        {
            game = std::make_unique<Game::ExampleGame>();
        }
#endif
        runtime.run(game.get());
    }
#else
    // Legacy
    engine.run();
#endif

    engine.cleanup();
    return 0;
}
