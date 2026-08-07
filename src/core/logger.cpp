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

namespace wowee {
namespace core {

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

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
    // same path — so running framexml_run from the repository root wiped the
    // session log of the client that had just been played, which is the one
    // file anyone diagnosing a report needs. It was found the only way it
    // could be: by being asked to read a log and finding my own run in it.
    const char* logName = std::getenv("WOWEE_LOG_FILE");
    const std::string logPath =
        std::string("logs/") + ((logName && *logName) ? logName : "wowee.log");
    fileStream.open(logPath, std::ios::out | std::ios::trunc);
    lastFlushTime_ = std::chrono::steady_clock::now();
}

void Logger::emitLineLocked(LogLevel level, const std::string& message) {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &time);
#else
    localtime_r(&time, &tm);
#endif

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
