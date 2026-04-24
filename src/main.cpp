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

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <typeinfo>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dbghelp.h>
#endif

namespace
{
#if defined(_WIN32)
    std::atomic_bool g_crash_exception_logged{false};
#endif

    struct StartupOptions
    {
#if USE_ENTITY_SYSTEM
        std::string game_name{"example"};
#endif
        std::string log_output{"both"};      // "console", "file", "both"
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

    std::string crash_timestamp()
    {
        const auto now = std::chrono::system_clock::now();
        const auto time = std::chrono::system_clock::to_time_t(now);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
            << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return oss.str();
    }

    void append_crash_log(const std::string &message)
    {
        std::error_code ec;
        std::filesystem::create_directories("logs", ec);

        std::ofstream crash_log("logs/crash_latest.log", std::ios::out | std::ios::app);
        if (crash_log.is_open())
        {
            crash_log << '[' << crash_timestamp() << "] " << message << '\n';
            crash_log.flush();
        }

        Logger::error("{}", message);
        Logger::flush();
    }

    const char *signal_name(const int signal)
    {
        switch (signal)
        {
            case SIGABRT: return "SIGABRT";
            case SIGFPE: return "SIGFPE";
            case SIGILL: return "SIGILL";
            case SIGINT: return "SIGINT";
            case SIGSEGV: return "SIGSEGV";
            case SIGTERM: return "SIGTERM";
            default: return "UNKNOWN";
        }
    }

    void handle_fatal_signal(const int signal)
    {
        append_crash_log(fmt::format("Fatal signal caught: {} ({})", signal_name(signal), signal));
        std::signal(signal, SIG_DFL);
        std::raise(signal);
    }

    void handle_terminate()
    {
        std::string reason = "std::terminate called";
        if (const std::exception_ptr ex = std::current_exception())
        {
            try
            {
                std::rethrow_exception(ex);
            }
            catch (const std::exception &e)
            {
                reason += fmt::format(": {}: {}", typeid(e).name(), e.what());
            }
            catch (...)
            {
                reason += ": non-std exception";
            }
        }

        append_crash_log(reason);
        std::abort();
    }

#if defined(_WIN32)
    std::string crash_dump_timestamp()
    {
        const auto now = std::chrono::system_clock::now();
        const auto time = std::chrono::system_clock::to_time_t(now);

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
        return oss.str();
    }

    std::string module_location_for_address(const void *address)
    {
        if (!address)
        {
            return "module=(null)";
        }

        MEMORY_BASIC_INFORMATION mbi{};
        if (VirtualQuery(address, &mbi, sizeof(mbi)) == 0 || !mbi.AllocationBase)
        {
            return "module=(unknown)";
        }

        char module_path[MAX_PATH]{};
        const auto module = static_cast<HMODULE>(mbi.AllocationBase);
        if (GetModuleFileNameA(module, module_path, static_cast<DWORD>(sizeof(module_path))) == 0)
        {
            return fmt::format("module_base={} module=(unknown)",
                               static_cast<const void *>(mbi.AllocationBase));
        }

        const auto absolute = reinterpret_cast<std::uintptr_t>(address);
        const auto base = reinterpret_cast<std::uintptr_t>(mbi.AllocationBase);
        return fmt::format("module='{}' module_base={} module_offset=0x{:x}",
                           module_path,
                           static_cast<const void *>(mbi.AllocationBase),
                           static_cast<unsigned long long>(absolute - base));
    }

    std::string symbol_for_address(const DWORD64 address)
    {
        HANDLE process = GetCurrentProcess();
        char buffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME]{};
        auto *symbol = reinterpret_cast<SYMBOL_INFO *>(buffer);
        symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
        symbol->MaxNameLen = MAX_SYM_NAME;

        DWORD64 displacement = 0;
        std::string out;
        if (SymFromAddr(process, address, &displacement, symbol))
        {
            out = fmt::format("{}+0x{:x}",
                              symbol->Name,
                              static_cast<unsigned long long>(displacement));
        }
        else
        {
            out = "(no symbol)";
        }

        IMAGEHLP_LINE64 line{};
        line.SizeOfStruct = sizeof(line);
        DWORD line_displacement = 0;
        if (SymGetLineFromAddr64(process, address, &line_displacement, &line))
        {
            out += fmt::format(" at {}:{}+0x{:x}",
                               line.FileName ? line.FileName : "(unknown)",
                               line.LineNumber,
                               line_displacement);
        }

        return out;
    }

    void log_exception_stack_trace(EXCEPTION_POINTERS *exception_info)
    {
        if (!exception_info || !exception_info->ContextRecord)
        {
            return;
        }

        HANDLE process = GetCurrentProcess();
        SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
        SymInitialize(process, nullptr, TRUE);

        CONTEXT context = *exception_info->ContextRecord;
        STACKFRAME64 frame{};
#if defined(_M_X64) || defined(__x86_64__)
        DWORD machine_type = IMAGE_FILE_MACHINE_AMD64;
        frame.AddrPC.Offset = context.Rip;
        frame.AddrFrame.Offset = context.Rbp;
        frame.AddrStack.Offset = context.Rsp;
#elif defined(_M_IX86) || defined(__i386__)
        DWORD machine_type = IMAGE_FILE_MACHINE_I386;
        frame.AddrPC.Offset = context.Eip;
        frame.AddrFrame.Offset = context.Ebp;
        frame.AddrStack.Offset = context.Esp;
#else
        DWORD machine_type = IMAGE_FILE_MACHINE_UNKNOWN;
#endif
        frame.AddrPC.Mode = AddrModeFlat;
        frame.AddrFrame.Mode = AddrModeFlat;
        frame.AddrStack.Mode = AddrModeFlat;

        append_crash_log("Crash stack trace:");
        for (int frame_index = 0; frame_index < 32; ++frame_index)
        {
            if (frame.AddrPC.Offset == 0)
            {
                break;
            }

            const void *address = reinterpret_cast<const void *>(static_cast<std::uintptr_t>(frame.AddrPC.Offset));
            append_crash_log(fmt::format("  #{:02d} address={} {} {}",
                                         frame_index,
                                         address,
                                         module_location_for_address(address),
                                         symbol_for_address(frame.AddrPC.Offset)));

            if (!StackWalk64(machine_type,
                             process,
                             GetCurrentThread(),
                             &frame,
                             &context,
                             nullptr,
                             SymFunctionTableAccess64,
                             SymGetModuleBase64,
                             nullptr))
            {
                break;
            }
        }
    }

    void write_crash_minidump(EXCEPTION_POINTERS *exception_info)
    {
        std::error_code ec;
        std::filesystem::create_directories("logs", ec);

        const std::string dump_path = fmt::format("logs/crash_{}_{}.dmp",
                                                  crash_dump_timestamp(),
                                                  GetCurrentProcessId());
        const HANDLE file = CreateFileA(dump_path.c_str(),
                                        GENERIC_WRITE,
                                        0,
                                        nullptr,
                                        CREATE_ALWAYS,
                                        FILE_ATTRIBUTE_NORMAL,
                                        nullptr);
        if (file == INVALID_HANDLE_VALUE)
        {
            append_crash_log(fmt::format("Failed to create minidump '{}': GetLastError={}",
                                         dump_path,
                                         GetLastError()));
            return;
        }

        MINIDUMP_EXCEPTION_INFORMATION exception_info_for_dump{};
        exception_info_for_dump.ThreadId = GetCurrentThreadId();
        exception_info_for_dump.ExceptionPointers = exception_info;
        exception_info_for_dump.ClientPointers = FALSE;

        const BOOL ok = MiniDumpWriteDump(GetCurrentProcess(),
                                          GetCurrentProcessId(),
                                          file,
                                          MiniDumpWithIndirectlyReferencedMemory,
                                          exception_info ? &exception_info_for_dump : nullptr,
                                          nullptr,
                                          nullptr);
        const DWORD error = GetLastError();
        CloseHandle(file);

        if (ok)
        {
            append_crash_log(fmt::format("Wrote crash minidump: {}", dump_path));
        }
        else
        {
            append_crash_log(fmt::format("MiniDumpWriteDump failed for '{}': GetLastError={}",
                                         dump_path,
                                         error));
        }
    }

    const char *seh_exception_name(const DWORD code)
    {
        switch (code)
        {
            case EXCEPTION_ACCESS_VIOLATION: return "EXCEPTION_ACCESS_VIOLATION";
            case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "EXCEPTION_ARRAY_BOUNDS_EXCEEDED";
            case EXCEPTION_BREAKPOINT: return "EXCEPTION_BREAKPOINT";
            case EXCEPTION_DATATYPE_MISALIGNMENT: return "EXCEPTION_DATATYPE_MISALIGNMENT";
            case EXCEPTION_FLT_DENORMAL_OPERAND: return "EXCEPTION_FLT_DENORMAL_OPERAND";
            case EXCEPTION_FLT_DIVIDE_BY_ZERO: return "EXCEPTION_FLT_DIVIDE_BY_ZERO";
            case EXCEPTION_FLT_INEXACT_RESULT: return "EXCEPTION_FLT_INEXACT_RESULT";
            case EXCEPTION_FLT_INVALID_OPERATION: return "EXCEPTION_FLT_INVALID_OPERATION";
            case EXCEPTION_FLT_OVERFLOW: return "EXCEPTION_FLT_OVERFLOW";
            case EXCEPTION_FLT_STACK_CHECK: return "EXCEPTION_FLT_STACK_CHECK";
            case EXCEPTION_FLT_UNDERFLOW: return "EXCEPTION_FLT_UNDERFLOW";
            case EXCEPTION_ILLEGAL_INSTRUCTION: return "EXCEPTION_ILLEGAL_INSTRUCTION";
            case EXCEPTION_IN_PAGE_ERROR: return "EXCEPTION_IN_PAGE_ERROR";
            case EXCEPTION_INT_DIVIDE_BY_ZERO: return "EXCEPTION_INT_DIVIDE_BY_ZERO";
            case EXCEPTION_INT_OVERFLOW: return "EXCEPTION_INT_OVERFLOW";
            case EXCEPTION_INVALID_DISPOSITION: return "EXCEPTION_INVALID_DISPOSITION";
            case EXCEPTION_NONCONTINUABLE_EXCEPTION: return "EXCEPTION_NONCONTINUABLE_EXCEPTION";
            case EXCEPTION_PRIV_INSTRUCTION: return "EXCEPTION_PRIV_INSTRUCTION";
            case EXCEPTION_SINGLE_STEP: return "EXCEPTION_SINGLE_STEP";
            case EXCEPTION_STACK_OVERFLOW: return "EXCEPTION_STACK_OVERFLOW";
            default: return "UNKNOWN_SEH_EXCEPTION";
        }
    }

    void log_windows_exception(EXCEPTION_POINTERS *exception_info, const char *handler_name)
    {
        DWORD code = 0;
        void *address = nullptr;
        ULONG_PTR access_kind = 0;
        ULONG_PTR access_address = 0;
        if (exception_info && exception_info->ExceptionRecord)
        {
            const EXCEPTION_RECORD *record = exception_info->ExceptionRecord;
            code = record->ExceptionCode;
            address = record->ExceptionAddress;
            if (code == EXCEPTION_ACCESS_VIOLATION && record->NumberParameters >= 2)
            {
                access_kind = record->ExceptionInformation[0];
                access_address = record->ExceptionInformation[1];
            }
        }

        append_crash_log(fmt::format("{}: {} code=0x{:08x} address={} access_kind={} access_address=0x{:x} {}",
                                     handler_name,
                                     seh_exception_name(code),
                                     static_cast<unsigned int>(code),
                                     address,
                                     static_cast<unsigned long long>(access_kind),
                                     static_cast<unsigned long long>(access_address),
                                     module_location_for_address(address)));
        log_exception_stack_trace(exception_info);
        write_crash_minidump(exception_info);
    }

    LONG WINAPI handle_vectored_exception(EXCEPTION_POINTERS *exception_info)
    {
        if (exception_info && exception_info->ExceptionRecord &&
            exception_info->ExceptionRecord->ExceptionCode == EXCEPTION_ACCESS_VIOLATION &&
            !g_crash_exception_logged.exchange(true))
        {
            log_windows_exception(exception_info, "Vectored SEH exception");
        }

        return EXCEPTION_CONTINUE_SEARCH;
    }

    LONG WINAPI handle_unhandled_exception(EXCEPTION_POINTERS *exception_info)
    {
        if (!g_crash_exception_logged.exchange(true))
        {
            log_windows_exception(exception_info, "Unhandled SEH exception");
        }
        return EXCEPTION_EXECUTE_HANDLER;
    }
#endif

    void install_crash_handlers()
    {
        std::set_terminate(handle_terminate);
        std::signal(SIGABRT, handle_fatal_signal);
        std::signal(SIGFPE, handle_fatal_signal);
        std::signal(SIGILL, handle_fatal_signal);
        std::signal(SIGSEGV, handle_fatal_signal);
        std::signal(SIGTERM, handle_fatal_signal);
#if defined(_WIN32)
        AddVectoredExceptionHandler(1, handle_vectored_exception);
        SetUnhandledExceptionFilter(handle_unhandled_exception);
#endif
    }

    int run_application(const StartupOptions &options)
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
        return 0;
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
    install_crash_handlers();

    int exit_code = 0;
    try
    {
        exit_code = run_application(options);
    }
    catch (const std::exception &e)
    {
        append_crash_log(fmt::format("Unhandled C++ exception in main: {}: {}", typeid(e).name(), e.what()));
        exit_code = 1;
    }
    catch (...)
    {
        append_crash_log("Unhandled non-std exception in main");
        exit_code = 1;
    }

    Logger::shutdown();
    return exit_code;
}
