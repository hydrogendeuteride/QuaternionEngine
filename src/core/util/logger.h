#pragma once

#include <string>
#include <string_view>
#include <fstream>
#include <fmt/core.h>

enum class LogLevel : uint8_t { Debug, Info, Warn, Error };
enum class LogOutput : uint8_t { Console, File, Both };

class Logger {
public:
    static void init(LogOutput output = LogOutput::Console,
                     LogLevel minLevel = LogLevel::Info,
                     const std::string& logDir = "logs");
    static void shutdown();

    static void set_level(LogLevel level) { s_minLevel = level; }
    static void set_output(LogOutput output) { s_output = output; }
    static LogLevel get_level() { return s_minLevel; }
    static LogOutput get_output() { return s_output; }

    template<typename... Args>
    static void debug(fmt::format_string<Args...> f, Args&&... args)
    {
        if (s_minLevel <= LogLevel::Debug)
            log(LogLevel::Debug, fmt::format(f, std::forward<Args>(args)...));
    }

    template<typename... Args>
    static void info(fmt::format_string<Args...> f, Args&&... args)
    {
        if (s_minLevel <= LogLevel::Info)
            log(LogLevel::Info, fmt::format(f, std::forward<Args>(args)...));
    }

    template<typename... Args>
    static void warn(fmt::format_string<Args...> f, Args&&... args)
    {
        if (s_minLevel <= LogLevel::Warn)
            log(LogLevel::Warn, fmt::format(f, std::forward<Args>(args)...));
    }

    template<typename... Args>
    static void error(fmt::format_string<Args...> f, Args&&... args)
    {
        log(LogLevel::Error, fmt::format(f, std::forward<Args>(args)...));
    }

private:
    static void log(LogLevel level, std::string_view message);

    static LogLevel s_minLevel;
    static LogOutput s_output;
    static std::ofstream s_file;
    static bool s_initialized;
};
