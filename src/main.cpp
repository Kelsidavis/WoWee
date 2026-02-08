#include "core/application.hpp"
#include "core/logger.hpp"
#include <exception>
#include <csignal>
#include <SDL2/SDL.h>
#include <X11/Xlib.h>

// Keep a persistent X11 connection for emergency mouse release in signal handlers.
// XOpenDisplay inside a signal handler is unreliable, so we open it once at startup.
static Display* g_emergencyDisplay = nullptr;

static void releaseMouseGrab() {
    if (g_emergencyDisplay) {
        XUngrabPointer(g_emergencyDisplay, CurrentTime);
        XUngrabKeyboard(g_emergencyDisplay, CurrentTime);
        XFlush(g_emergencyDisplay);
    }
}

static void crashHandler(int sig) {
    releaseMouseGrab();
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
    g_emergencyDisplay = XOpenDisplay(nullptr);
    std::signal(SIGSEGV, crashHandler);
    std::signal(SIGABRT, crashHandler);
    std::signal(SIGFPE,  crashHandler);
    std::signal(SIGTERM, crashHandler);
    std::signal(SIGINT,  crashHandler);
    try {
        wowee::core::Logger::getInstance().setLogLevel(wowee::core::LogLevel::DEBUG);
        LOG_INFO("=== Wowee Native Client ===");
        LOG_INFO("Starting application...");

        wowee::core::Application app;

        if (!app.initialize()) {
            LOG_FATAL("Failed to initialize application");
            return 1;
        }

        app.run();
        app.shutdown();

        LOG_INFO("Application exited successfully");
        if (g_emergencyDisplay) { XCloseDisplay(g_emergencyDisplay); g_emergencyDisplay = nullptr; }
        return 0;
    }
    catch (const std::exception& e) {
        releaseMouseGrab();
        LOG_FATAL("Unhandled exception: ", e.what());
        return 1;
    }
    catch (...) {
        releaseMouseGrab();
        LOG_FATAL("Unknown exception occurred");
        return 1;
    }
}
