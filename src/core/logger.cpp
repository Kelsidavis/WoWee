#include "core/logger.hpp"
#include <chrono>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <cstdlib>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <iterator>
#include <ranges>
#include "core/local_time.hpp"
#include <cstdio>
#ifdef __ANDROID__
#include <android/log.h>
#endif

namespace wowee {
namespace core {

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

namespace {

/// Where to write when the working directory will not take a log.
///
/// Deliberately not getConfigRoot(): the logger is linked into every tool and
/// most of the tests, and none of them wants config_paths and what it pulls in
/// behind it. WOWEE_CONFIG_ROOT is still honoured, so a harness that redirects
/// its config redirects its log with it.
std::filesystem::path perUserLogDir() {
    if (const char* root = std::getenv("WOWEE_CONFIG_ROOT"); root && *root) {
        return std::filesystem::path(root) / "logs";
    }
#if defined(_WIN32)
    if (const char* local = std::getenv("LOCALAPPDATA"); local && *local) {
        return std::filesystem::path(local) / "Wowee" / "logs";
    }
#elif defined(__APPLE__)
    if (const char* home = std::getenv("HOME"); home && *home) {
        return std::filesystem::path(home) / "Library" / "Logs" / "Wowee";
    }
#else
    if (const char* state = std::getenv("XDG_STATE_HOME"); state && *state) {
        return std::filesystem::path(state) / "wowee" / "logs";
    }
    if (const char* home = std::getenv("HOME"); home && *home) {
        return std::filesystem::path(home) / ".local" / "state" / "wowee" / "logs";
    }
#endif
    return std::filesystem::temp_directory_path() / "wowee-logs";
}

}  // namespace

void Logger::ensureFile() {
    if (fileReady) return;
    fileReady = true;
    if (const char* logStdout = std::getenv("WOWEE_LOG_STDOUT")) {
        if (logStdout[0] == '0') {
            echoToStdout_ = false;
        }
    }
    if (const char* flushMs = std::getenv("WOWEE_LOG_FLUSH_MS")) {
        char* end = nullptr;
        unsigned long parsed = std::strtoul(flushMs, &end, 10);
        if (end != flushMs && parsed <= 10000ul) {
            flushIntervalMs_ = static_cast<uint32_t>(parsed);
        }
    }
    if (const char* dedupe = std::getenv("WOWEE_LOG_DEDUPE")) {
        dedupeEnabled_ = !(dedupe[0] == '0' || dedupe[0] == 'f' || dedupe[0] == 'F' ||
                           dedupe[0] == 'n' || dedupe[0] == 'N');
    }
    if (const char* dedupeMs = std::getenv("WOWEE_LOG_DEDUPE_MS")) {
        char* end = nullptr;
        unsigned long parsed = std::strtoul(dedupeMs, &end, 10);
        if (end != dedupeMs && parsed <= 60000ul) {
            dedupeWindowMs_ = static_cast<uint32_t>(parsed);
        }
    }
    if (const char* level = std::getenv("WOWEE_LOG_LEVEL")) {
        auto toLower = [] (unsigned char c) { return std::tolower(c); };
        using namespace std::literals;

        auto v = std::string_view{level} | std::views::transform(toLower);
        if (std::ranges::equal(v, "debug"sv)) setLogLevel(LogLevel::DEBUG);
        else if (std::ranges::equal(v, "info"sv)) setLogLevel(LogLevel::INFO);
        else if (std::ranges::equal(v, "warn"sv) || std::ranges::equal(v, "warning"sv))
			setLogLevel(LogLevel::WARNING);
        else if (std::ranges::equal(v, "error"sv)) setLogLevel(kLogLevelError);
        else if (std::ranges::equal(v, "fatal"sv)) setLogLevel(LogLevel::FATAL);
    }
    std::error_code ec;
    std::filesystem::create_directories("logs", ec);
    // WOWEE_LOG_FILE names the file, so a tool run beside the client does not
    // destroy the log the client wrote.
    //
    // This opens with trunc, and every process using this logger opened the
    // same path - so running framexml_run from the repository root wiped the
    // session log of the client that had just been played, which is the one
    // file anyone diagnosing a report needs. It was found the only way it
    // could be: by being asked to read a log and finding my own run in it.
    const char* logName = std::getenv("WOWEE_LOG_FILE");
    const std::string logFile = (logName && *logName) ? logName : "wowee.log";
    const std::string logPath = std::string("logs/") + logFile;
    fileStream.open(logPath, std::ios::out | std::ios::trunc);

    // Beside the working directory when that is writable, which is how this is
    // run from a checkout and where every tool expects to find it.
    //
    // A bundled application has no such directory. macOS launches an .app with
    // the working directory set to "/", so create_directories("logs") fails on
    // a read-only root, the open fails with it, and the client runs with no log
    // at all. That is not a quiet degradation: the log is the only thing a bug
    // report has to go on, and the absence looks exactly like a client that
    // wrote nothing worth saying.
    if (!fileStream.is_open()) {
        const std::filesystem::path fallback = perUserLogDir();
        std::filesystem::create_directories(fallback, ec);
        const std::filesystem::path at = fallback / logFile;
        fileStream.open(at, std::ios::out | std::ios::trunc);
        if (fileStream.is_open()) {
            // Said on the console, because the file it names is the one thing
            // someone reading this needs and it is not where they will look.
            std::fprintf(stderr, "wowee: writing the log to %s\n", at.string().c_str());
        }
    }
    lastFlushTime_ = std::chrono::steady_clock::now();
}

void Logger::emitLineLocked(LogLevel level, const std::string& message) {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm = core::localTime(time);
    // Format: [YYYY-MM-DD HH:MM:SS.mmm] [LEVEL] message
    std::ostringstream line;
    line << "["
         << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
         << "." << std::setfill('0') << std::setw(3) << ms.count()
         << "] [";

    switch (level) {
        case LogLevel::DEBUG:   line << "DEBUG"; break;
        case LogLevel::INFO:    line << "INFO "; break;
        case LogLevel::WARNING: line << "WARN "; break;
        case kLogLevelError:    line << "ERROR"; break;
        case LogLevel::FATAL:   line << "FATAL"; break;
    }

    line << "] " << message;

    if (echoToStdout_) {
        std::cout << line.str() << '\n';
    }
#ifdef __ANDROID__
    // stdout goes nowhere on Android and the file has to be pulled off the
    // device to be read, so every line also goes to logcat, where `adb logcat
    // -s wowee` shows it live. The timestamp and level are logcat's own job,
    // so this passes the message rather than the formatted line.
    int priority = ANDROID_LOG_INFO;
    switch (level) {
        case LogLevel::DEBUG:   priority = ANDROID_LOG_DEBUG; break;
        case LogLevel::INFO:    priority = ANDROID_LOG_INFO; break;
        case LogLevel::WARNING: priority = ANDROID_LOG_WARN; break;
        case kLogLevelError:    priority = ANDROID_LOG_ERROR; break;
        case LogLevel::FATAL:   priority = ANDROID_LOG_FATAL; break;
    }
    __android_log_write(priority, "wowee", message.c_str());
#endif
    if (fileStream.is_open()) {
        fileStream << line.str() << '\n';
        bool shouldFlush = (level >= LogLevel::WARNING);
        if (!shouldFlush) {
            auto nowSteady = std::chrono::steady_clock::now();
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(nowSteady - lastFlushTime_).count();
            shouldFlush = (elapsedMs >= static_cast<long long>(flushIntervalMs_));
            if (shouldFlush) {
                lastFlushTime_ = nowSteady;
            }
        }
        if (shouldFlush) {
            fileStream.flush();
        }
    }
}

void Logger::flushSuppressedLocked() {
    if (suppressedCount_ == 0) return;
    emitLineLocked(lastLevel_, "Previous message repeated " + std::to_string(suppressedCount_) + " times");
    suppressedCount_ = 0;
}

void Logger::log(LogLevel level, const std::string& message) {
    if (!shouldLog(level)) {
        return;
    }

    // Capture timestamp before acquiring lock to minimize critical section
    auto nowSteady = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex);
    ensureFile();
    if (dedupeEnabled_ && !lastMessage_.empty() &&
        level == lastLevel_ && message == lastMessage_) {
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(nowSteady - lastMessageTime_).count();
        if (elapsedMs >= 0 && elapsedMs <= static_cast<long long>(dedupeWindowMs_)) {
            ++suppressedCount_;
            lastMessageTime_ = nowSteady;
            return;
        }
    }

    flushSuppressedLocked();
    emitLineLocked(level, message);
    lastLevel_ = level;
    lastMessage_ = message;
    lastMessageTime_ = nowSteady;
}

void Logger::setLogLevel(LogLevel level) {
    minLevel_.store(static_cast<int>(level), std::memory_order_relaxed);
}

bool Logger::shouldLog(LogLevel level) const {
    return static_cast<int>(level) >= minLevel_.load(std::memory_order_relaxed);
}

} // namespace core
} // namespace wowee
