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
// Guard calls at the macro site so variadic arguments are not evaluated
// when the corresponding level is disabled.
#define LOG_DEBUG(...) do { \
    auto& _wowee_logger = wowee::core::Logger::getInstance(); \
    if (_wowee_logger.shouldLog(wowee::core::LogLevel::DEBUG)) { \
        _wowee_logger.debug(__VA_ARGS__); \
    } \
} while (0)

#define LOG_INFO(...) do { \
    auto& _wowee_logger = wowee::core::Logger::getInstance(); \
    if (_wowee_logger.shouldLog(wowee::core::LogLevel::INFO)) { \
        _wowee_logger.info(__VA_ARGS__); \
    } \
} while (0)

#define LOG_WARNING(...) do { \
    auto& _wowee_logger = wowee::core::Logger::getInstance(); \
    if (_wowee_logger.shouldLog(wowee::core::LogLevel::WARNING)) { \
        _wowee_logger.warning(__VA_ARGS__); \
    } \
} while (0)

#define LOG_ERROR(...) do { \
    auto& _wowee_logger = wowee::core::Logger::getInstance(); \
    if (_wowee_logger.shouldLog(wowee::core::LogLevel::ERROR)) { \
        _wowee_logger.error(__VA_ARGS__); \
    } \
} while (0)

#define LOG_FATAL(...) do { \
    auto& _wowee_logger = wowee::core::Logger::getInstance(); \
    if (_wowee_logger.shouldLog(wowee::core::LogLevel::FATAL)) { \
        _wowee_logger.fatal(__VA_ARGS__); \
    } \
} while (0)

} // namespace core
} // namespace wowee

// Restore the ERROR macro now that all LogLevel::ERROR references are done.
#ifdef _WIN32
#pragma pop_macro("ERROR")
#endif
