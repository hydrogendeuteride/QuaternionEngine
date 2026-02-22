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
#include "game/legacy/example_game.h"
#include "game/legacy/rebasing_test_game.h"
#include "game/main_game.h"
#else
#endif // USE_ENTITY_SYSTEM
#endif // USE_GAME_RUNTIME

#include "core/util/logger.h"

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
        std::string log_output{"console"};   // "console", "file", "both"
        std::string log_level{"info"};       // "debug", "info", "warn", "error"
    };

    StartupOptions parse_startup_options(int argc, char *argv[])
    {
        StartupOptions options{};
        constexpr std::string_view game_prefix = "--game=";
        constexpr std::string_view log_prefix  = "--log=";
        constexpr std::string_view loglevel_prefix = "--log-level=";

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
            if (value.rfind(log_prefix.data(), 0) == 0)
            {
                options.log_output = value.substr(log_prefix.size());
            }
            if (value.rfind(loglevel_prefix.data(), 0) == 0)
            {
                options.log_level = value.substr(loglevel_prefix.size());
            }
        }

        return options;
    }
} // namespace

int main(int argc, char *argv[])
{
    const StartupOptions options = parse_startup_options(argc, argv);

    // Initialize logger from command-line options
    {
        LogOutput output = LogOutput::Console;
        if (options.log_output == "file")        output = LogOutput::File;
        else if (options.log_output == "both")   output = LogOutput::Both;

        LogLevel level = LogLevel::Info;
        if (options.log_level == "debug")        level = LogLevel::Debug;
        else if (options.log_level == "warn")    level = LogLevel::Warn;
        else if (options.log_level == "error")   level = LogLevel::Error;

        Logger::init(output, level);
    }

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
            game = std::make_unique<Game::MainGame>();
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
    Logger::shutdown();
    return 0;
}
