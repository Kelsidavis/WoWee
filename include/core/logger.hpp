#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <fstream>
#include <chrono>
#include <atomic>

namespace wowee {
namespace core {

// Suppress the wingdi.h "#define ERROR 0" macro for the entire header so that
// LogLevel::ERROR inside template bodies compiles correctly on Windows.
#ifdef _WIN32
#pragma push_macro("ERROR")
#undef ERROR
#endif

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& getInstance();

    void log(LogLevel level, const std::string& message);
    void setLogLevel(LogLevel level);
    bool shouldLog(LogLevel level) const;

    template<typename... Args>
    void debug(Args&&... args) {
        if (!shouldLog(LogLevel::DEBUG)) return;
        log(LogLevel::DEBUG, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void info(Args&&... args) {
        if (!shouldLog(LogLevel::INFO)) return;
        log(LogLevel::INFO, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void warning(Args&&... args) {
        if (!shouldLog(LogLevel::WARNING)) return;
        log(LogLevel::WARNING, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void error(Args&&... args) {
        if (!shouldLog(LogLevel::ERROR)) return;
        log(LogLevel::ERROR, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void fatal(Args&&... args) {
        if (!shouldLog(LogLevel::FATAL)) return;
        log(LogLevel::FATAL, format(std::forward<Args>(args)...));
    }

private:
    static constexpr int kDefaultMinLevelValue =
#if defined(NDEBUG) || defined(WOWEE_RELEASE_LOGGING)
        static_cast<int>(LogLevel::WARNING);
#else
        static_cast<int>(LogLevel::INFO);
#endif

    Logger() = default;
    ~Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    template<typename... Args>
    std::string format(Args&&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        return oss.str();
    }

    std::atomic<int> minLevel_{kDefaultMinLevelValue};
    std::mutex mutex;
    std::ofstream fileStream;
    bool fileReady = false;
    bool echoToStdout_ = true;
    std::chrono::steady_clock::time_point lastFlushTime_{};
    uint32_t flushIntervalMs_ = 250;
    bool dedupeEnabled_ = true;
    uint32_t dedupeWindowMs_ = 250;
    LogLevel lastLevel_ = LogLevel::DEBUG;
    std::string lastMessage_;
    std::chrono::steady_clock::time_point lastMessageTime_{};
    uint64_t suppressedCount_ = 0;
    void emitLineLocked(LogLevel level, const std::string& message);
    void flushSuppressedLocked();
    void ensureFile();
};

// Convenience macros.
//
// Each Logger method (debug/info/warning/error/fatal) already guards on
// shouldLog() internally, so the macros are simple direct calls.
//
// IMPORTANT: do NOT reference LogLevel enum values here.  Windows'
// wingdi.h defines `#define ERROR 0` which would corrupt `LogLevel::ERROR`
// at any call site compiled after <windows.h>.  Keeping the enum names
// out of the macro bodies avoids that problem entirely.
#define LOG_DEBUG(...)   wowee::core::Logger::getInstance().debug(__VA_ARGS__)
#define LOG_INFO(...)    wowee::core::Logger::getInstance().info(__VA_ARGS__)
#define LOG_WARNING(...) wowee::core::Logger::getInstance().warning(__VA_ARGS__)
#define LOG_ERROR(...)   wowee::core::Logger::getInstance().error(__VA_ARGS__)
#define LOG_FATAL(...)   wowee::core::Logger::getInstance().fatal(__VA_ARGS__)

} // namespace core
} // namespace wowee

// Restore the ERROR macro now that all LogLevel::ERROR references are done.
#ifdef _WIN32
#pragma pop_macro("ERROR")
#endif
