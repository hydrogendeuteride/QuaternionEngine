#include "logger.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

LogLevel Logger::s_minLevel = LogLevel::Info;
LogOutput Logger::s_output = LogOutput::Console;
std::ofstream Logger::s_file;
bool Logger::s_initialized = false;

static const char *level_tag(LogLevel level)
{
    switch (level)
    {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warn: return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

static std::string timestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%H:%M:%S")
            << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

void Logger::init(LogOutput output, LogLevel minLevel, const std::string &logDir)
{
    s_output = output;
    s_minLevel = minLevel;

    if (output == LogOutput::File || output == LogOutput::Both)
    {
        std::filesystem::create_directories(logDir);

        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::ostringstream filename;
        filename << logDir << "/engine_"
                << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S")
                << ".log";

        s_file.open(filename.str(), std::ios::out | std::ios::trunc);
        if (!s_file.is_open())
        {
            fmt::println(stderr, "[Logger] Failed to open log file: {}", filename.str());
        }
    }

    s_initialized = true;
}

void Logger::shutdown()
{
    if (s_file.is_open())
        s_file.close();
    s_initialized = false;
}

void Logger::log(LogLevel level, std::string_view message)
{
    if (s_output == LogOutput::Console || s_output == LogOutput::Both)
    {
        if (level == LogLevel::Error)
            fmt::println(stderr, "{}", message);
        else
            fmt::println("{}", message);
    }

    if ((s_output == LogOutput::File || s_output == LogOutput::Both) && s_file.is_open())
    {
        s_file << '[' << level_tag(level) << "] [" << timestamp() << "] "
                << message << '\n';
        s_file.flush();
    }
}
