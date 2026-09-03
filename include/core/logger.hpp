#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <fstream>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <cstdint>

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

// Avoid direct token use of `ERROR` at call sites because Windows headers
// define `ERROR` as a macro.
inline constexpr LogLevel kLogLevelError = LogLevel::ERROR;

class Logger {
public:
    static Logger& getInstance();

    void log(LogLevel level, const std::string& message);
    void setLogLevel(LogLevel level);
    [[nodiscard]] bool shouldLog(LogLevel level) const;

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
    /// Something is written and not yet on disk. See flushIfStale.
    bool unflushed_ = false;
    bool dedupeEnabled_ = true;
    uint32_t dedupeWindowMs_ = 250;
    LogLevel lastLevel_ = LogLevel::DEBUG;
    std::string lastMessage_;
    std::chrono::steady_clock::time_point lastMessageTime_{};
    uint64_t suppressedCount_ = 0;
    void emitLineLocked(LogLevel level, const std::string& message);
public:
    /// Put whatever is buffered on disk if it has been there long enough.
    ///
    /// Called once a frame. A warning used to flush the stream by itself, which
    /// is a write for every line - and this client puts its diagnostics at
    /// warning on purpose, so a burst is hundreds of them in a row: a hundred
    /// and twelve while FrameXML loads, two hundred for one takeover check. The
    /// interval coalesces a burst into one write and this keeps the tail of it
    /// from sitting in the buffer afterwards, so a warning still reaches the
    /// file within a frame or two of being written.
    void flushIfStale();
private:
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
    if (_wowee_logger.shouldLog(wowee::core::kLogLevelError)) { \
        _wowee_logger.error(__VA_ARGS__); \
    } \
} while (0)

#define LOG_FATAL(...) do { \
    auto& _wowee_logger = wowee::core::Logger::getInstance(); \
    if (_wowee_logger.shouldLog(wowee::core::LogLevel::FATAL)) { \
        _wowee_logger.fatal(__VA_ARGS__); \
    } \
} while (0)

/// A budget for a diagnostic that would otherwise repeat as long as its
/// condition holds.
///
/// A rate limit bounds how often a line is written, not how many times: at one
/// a second, a condition that holds for a minute writes sixty of them, all
/// saying what the first one said. Declare one of these static beside the line
/// and the diagnostic keeps its first few - which is where the information is
/// - then says so once and stops.
///
///     static core::LogBudget budget(12, "Floor jump");
///     if (budget.take()) LOG_WARNING("Floor jump: player z=", z, ...);
class LogBudget {
public:
    LogBudget(int total, const char* what) : left_(total), total_(total), what_(what) {}

    /// True while there is budget left. The call after the last one writes a
    /// closing line naming the diagnostic, so a reader can tell a report that
    /// stopped from one that never fired again.
    bool take() {
        if (left_ > 0) {
            --left_;
            return true;
        }
        if (!closed_) {
            closed_ = true;
            LOG_WARNING(what_, ": said ", total_, " times and will not be said "
                        "again this run");
        }
        return false;
    }

private:
    int left_ = 0;
    int total_ = 0;
    const char* what_ = "";
    bool closed_ = false;
};

inline std::string toHexString(const uint8_t* data, size_t len, bool spaces = false) {
    std::string s;
    s.reserve(len * (spaces ? 3 : 2));
    for (size_t i = 0; i < len; ++i) {
        char buf[4];
        std::snprintf(buf, sizeof(buf), spaces ? "%02x " : "%02x", data[i]);
        s += buf;
    }
    return s;
}

} // namespace core
} // namespace wowee

// Restore the ERROR macro now that all LogLevel::ERROR references are done.
#ifdef _WIN32
#pragma pop_macro("ERROR")
#endif
