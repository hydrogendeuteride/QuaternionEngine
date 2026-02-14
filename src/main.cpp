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
#include "game/space_combat_game.h"
#else
#endif // USE_ENTITY_SYSTEM
#endif // USE_GAME_RUNTIME

#include <memory>
#include <string>
#include <string_view>

namespace
{
    struct StartupOptions
    {
#if USE_ENTITY_SYSTEM
        std::string game_name{"example"};
#endif
    };

    StartupOptions parse_startup_options(int argc, char *argv[])
    {
        StartupOptions options{};
        constexpr std::string_view game_prefix = "--game=";

        for (int i = 1; i < argc; ++i)
        {
            const char *arg = argv[i];
            if (!arg)
            {
                continue;
            }

            const std::string value(arg);
#if USE_ENTITY_SYSTEM
            if (value.rfind(game_prefix.data(), 0) == 0)
            {
                options.game_name = value.substr(game_prefix.size());
            }
#endif
        }

        return options;
    }
} // namespace

int main(int argc, char *argv[])
{
    const StartupOptions options = parse_startup_options(argc, argv);
    VulkanEngine engine;
    engine.init();

#if USE_GAME_RUNTIME
    {
        Audio::MiniAudioSystem audio;
        audio.init();

        GameRuntime::Runtime runtime(&engine);
        runtime.set_audio_system(&audio);
#if USE_ENTITY_SYSTEM
        std::unique_ptr<GameRuntime::IGameCallbacks> game;
        if (options.game_name == "space_combat" || options.game_name == "sc")
        {
            game = std::make_unique<Game::SpaceCombatGame>();
        }
        else if (options.game_name == "rebase" || options.game_name == "rebasing" ||
                 options.game_name == "rebasing_test")
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
