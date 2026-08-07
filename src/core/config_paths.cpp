#include "core/config_paths.hpp"

#include "core/logger.hpp"

#include <cstdlib>
#include <filesystem>
#include <system_error>

#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#else
#include <unistd.h>
#endif

namespace wowee::core {

namespace fs = std::filesystem;

namespace {

// Per-user config location: %APPDATA%\wowee on Windows, ~/.wowee elsewhere.
// This is the default (non-portable) home and the migration source.
std::string perUserConfigDir() {
#if defined(_WIN32)
    const char* appdata = std::getenv("APPDATA");
    return appdata ? std::string(appdata) + "\\wowee" : ".";
#else
    const char* home = std::getenv("HOME");
    return home ? std::string(home) + "/.wowee" : ".";
#endif
}

}  // namespace

std::string getExecutableDir() {
#if defined(_WIN32)
    std::wstring buf(MAX_PATH, L'\0');
    DWORD len = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
    // Grow if the path was truncated (GetModuleFileNameW does not report the
    // required size; a full buffer signals truncation).
    while (len == buf.size()) {
        buf.resize(buf.size() * 2);
        len = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
    }
    if (len == 0) return {};
    return fs::path(buf.substr(0, len)).parent_path().string();
#elif defined(__APPLE__)
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::string buf(size, '\0');
    if (_NSGetExecutablePath(buf.data(), &size) != 0) return {};
    // buf includes a trailing NUL from the API; strip to the reported path.
    if (auto nul = buf.find('\0'); nul != std::string::npos) buf.resize(nul);
    return fs::path(buf).parent_path().string();
#else
    char buf[4096];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf));
    if (len <= 0) return {};
    return fs::path(std::string(buf, static_cast<size_t>(len))).parent_path().string();
#endif
}

std::string getConfigRoot() {
    std::error_code ec;
    // An explicit root wins over everything.
    //
    // The tools write here too — the missing-API list and the Lua error list
    // are both rewritten on exit — so a headless run beside a real session
    // overwrote the two files a bug report is read from. A harness needs its
    // own corner rather than the player's.
    if (const char* root = std::getenv("WOWEE_CONFIG_ROOT"); root && *root) {
        fs::create_directories(root, ec);
        return root;
    }
    const std::string exeDir = getExecutableDir();
    if (!exeDir.empty()) {
        const fs::path portableMarker = fs::path(exeDir) / "portable.txt";
        const fs::path portableDir = fs::path(exeDir) / "config";
        // Either the opt-in marker file, or a config folder created by a prior
        // portable run, keeps config folder-local.
        if (fs::exists(portableMarker, ec) || fs::is_directory(portableDir, ec)) {
            return portableDir.string();
        }
    }
    return perUserConfigDir();
}

void migratePortableConfigIfNeeded() {
    std::error_code ec;
    const std::string exeDir = getExecutableDir();
    if (exeDir.empty()) return;

    const fs::path marker = fs::path(exeDir) / "portable.txt";
    const fs::path portableDir = fs::path(exeDir) / "config";

    // Migration only applies the first time the user opts in via the marker.
    // Once the portable config folder exists it is the established store, so
    // there is nothing to seed and later launches copy nothing.
    if (!fs::exists(marker, ec)) return;
    if (fs::exists(portableDir, ec)) return;

    const fs::path userDir = perUserConfigDir();
    if (userDir.empty() || !fs::is_directory(userDir, ec)) {
        // Nothing to migrate; create the folder so portable mode is established
        // and this check does not repeat every launch.
        fs::create_directories(portableDir, ec);
        return;
    }

    fs::create_directories(portableDir, ec);
    fs::copy(userDir, portableDir,
             fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
    if (ec) {
        LOG_WARNING("Portable config migration from ", userDir.string(), " to ",
                    portableDir.string(), " failed: ", ec.message());
    } else {
        LOG_INFO("Migrated existing config from ", userDir.string(),
                 " into portable folder ", portableDir.string());
    }
}

}  // namespace wowee::core
