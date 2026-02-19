#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <fstream>

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

    template<typename... Args>
    void debug(Args&&... args) {
        log(LogLevel::DEBUG, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void info(Args&&... args) {
        log(LogLevel::INFO, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void warning(Args&&... args) {
        log(LogLevel::WARNING, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void error(Args&&... args) {
        log(LogLevel::ERROR, format(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void fatal(Args&&... args) {
        log(LogLevel::FATAL, format(std::forward<Args>(args)...));
    }

private:
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

    LogLevel minLevel = LogLevel::INFO;  // Changed from DEBUG to reduce log spam
    std::mutex mutex;
    std::ofstream fileStream;
    bool fileReady = false;
    void ensureFile();
};

// Convenience macros
#define LOG_DEBUG(...) wowee::core::Logger::getInstance().debug(__VA_ARGS__)
#define LOG_INFO(...) wowee::core::Logger::getInstance().info(__VA_ARGS__)
#define LOG_WARNING(...) wowee::core::Logger::getInstance().warning(__VA_ARGS__)
#define LOG_ERROR(...) wowee::core::Logger::getInstance().error(__VA_ARGS__)
#define LOG_FATAL(...) wowee::core::Logger::getInstance().fatal(__VA_ARGS__)

} // namespace core
} // namespace wowee

// Restore the ERROR macro now that all LogLevel::ERROR references are done.
#ifdef _WIN32
#pragma pop_macro("ERROR")
#endif
