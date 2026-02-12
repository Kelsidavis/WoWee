#include "pipeline/mpq_manager.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cctype>

#ifdef HAVE_STORMLIB
#include <StormLib.h>
#endif

// Define HANDLE and INVALID_HANDLE_VALUE for both cases
#ifndef HAVE_STORMLIB
typedef void* HANDLE;
#endif

#ifndef INVALID_HANDLE_VALUE
#define INVALID_HANDLE_VALUE ((HANDLE)(long long)-1)
#endif

namespace wowee {
namespace pipeline {

namespace {
std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}
}

MPQManager::MPQManager() = default;

MPQManager::~MPQManager() {
    shutdown();
}

bool MPQManager::initialize(const std::string& dataPath_) {
    if (initialized) {
        LOG_WARNING("MPQManager already initialized");
        return true;
    }

    dataPath = dataPath_;
    LOG_INFO("Initializing MPQ manager with data path: ", dataPath);

    // Check if data directory exists
    if (!std::filesystem::exists(dataPath)) {
        LOG_ERROR("Data directory does not exist: ", dataPath);
        return false;
    }

#ifdef HAVE_STORMLIB
    // Load base archives (in order of priority)
    std::vector<std::string> baseArchives = {
        "common.MPQ",
        "common-2.MPQ",
        "expansion.MPQ",
        "lichking.MPQ",
    };

    for (const auto& archive : baseArchives) {
        std::string fullPath = dataPath + "/" + archive;
        if (std::filesystem::exists(fullPath)) {
            loadArchive(fullPath, 100);  // Base archives have priority 100
        } else {
            LOG_DEBUG("Base archive not found (optional): ", archive);
        }
    }

    // Load patch archives (highest priority)
    loadPatchArchives();

    // Load locale archives
    loadLocaleArchives("enUS");  // TODO: Make configurable

    if (archives.empty()) {
        LOG_WARNING("No MPQ archives loaded - will use loose file fallback");
    } else {
        LOG_INFO("MPQ manager initialized with ", archives.size(), " archives");
    }
#else
    LOG_WARNING("StormLib not available - using loose file fallback only");
#endif

    initialized = true;
    return true;
}

void MPQManager::shutdown() {
    if (!initialized) {
        return;
    }

#ifdef HAVE_STORMLIB
    LOG_INFO("Shutting down MPQ manager");
    for (auto& entry : archives) {
        if (entry.handle != INVALID_HANDLE_VALUE) {
            SFileCloseArchive(entry.handle);
        }
    }
#endif

    archives.clear();
    archiveNames.clear();
    {
        std::lock_guard<std::mutex> lock(missingFileMutex_);
        missingFileWarnings_.clear();
    }
    initialized = false;
}

bool MPQManager::loadArchive(const std::string& path, int priority) {
#ifndef HAVE_STORMLIB
    LOG_ERROR("Cannot load archive - StormLib not available");
    return false;
#endif

#ifdef HAVE_STORMLIB
    // Check if file exists
    if (!std::filesystem::exists(path)) {
        LOG_ERROR("Archive file not found: ", path);
        return false;
    }

    HANDLE handle = INVALID_HANDLE_VALUE;
    if (!SFileOpenArchive(path.c_str(), 0, 0, &handle)) {
        LOG_ERROR("Failed to open MPQ archive: ", path);
        return false;
    }

    ArchiveEntry entry;
    entry.handle = handle;
    entry.path = path;
    entry.priority = priority;

    archives.push_back(entry);
    archiveNames.push_back(path);

    // Sort archives by priority (highest first)
    std::sort(archives.begin(), archives.end(),
              [](const ArchiveEntry& a, const ArchiveEntry& b) {
                  return a.priority > b.priority;
              });

    LOG_INFO("Loaded MPQ archive: ", path, " (priority ", priority, ")");
    return true;
#endif

    return false;
}

bool MPQManager::fileExists(const std::string& filename) const {
#ifdef HAVE_STORMLIB
    // Check MPQ archives first if available
    if (!archives.empty()) {
        HANDLE archive = findFileArchive(filename);
        if (archive != INVALID_HANDLE_VALUE) {
            return true;
        }
    }
#endif

    // Fall back to checking for loose file
    std::string loosePath = filename;
    std::replace(loosePath.begin(), loosePath.end(), '\\', '/');
    std::string fullPath = dataPath + "/" + loosePath;
    return std::filesystem::exists(fullPath);
}

std::vector<uint8_t> MPQManager::readFile(const std::string& filename) const {
#ifdef HAVE_STORMLIB
    // Try MPQ archives first if available
    if (!archives.empty()) {
        HANDLE archive = findFileArchive(filename);
        if (archive != INVALID_HANDLE_VALUE) {
            // Open the file
            HANDLE file = INVALID_HANDLE_VALUE;
            if (SFileOpenFileEx(archive, filename.c_str(), 0, &file)) {
                // Get file size
                DWORD fileSize = SFileGetFileSize(file, nullptr);
                if (fileSize > 0 && fileSize != SFILE_INVALID_SIZE) {
                    // Read file data
                    std::vector<uint8_t> data(fileSize);
                    DWORD bytesRead = 0;
                    if (SFileReadFile(file, data.data(), fileSize, &bytesRead, nullptr)) {
                        SFileCloseFile(file);
                        LOG_DEBUG("Read file from MPQ: ", filename, " (", bytesRead, " bytes)");
                        return data;
                    }
                }
                SFileCloseFile(file);
            }
        }
    }
#endif

    // Fall back to loose file loading
    // Convert WoW path (backslashes) to filesystem path (forward slashes)
    std::string loosePath = filename;
    std::replace(loosePath.begin(), loosePath.end(), '\\', '/');

    // Try with original case
    std::string fullPath = dataPath + "/" + loosePath;
    if (std::filesystem::exists(fullPath)) {
        std::ifstream file(fullPath, std::ios::binary | std::ios::ate);
        if (file.is_open()) {
            size_t size = file.tellg();
            file.seekg(0, std::ios::beg);
            std::vector<uint8_t> data(size);
            file.read(reinterpret_cast<char*>(data.data()), size);
            LOG_DEBUG("Read loose file: ", loosePath, " (", size, " bytes)");
            return data;
        }
    }

    // Try case-insensitive search (common for Linux)
    std::filesystem::path searchPath = dataPath;
    std::vector<std::string> pathComponents;
    std::istringstream iss(loosePath);
    std::string component;
    while (std::getline(iss, component, '/')) {
        if (!component.empty()) {
            pathComponents.push_back(component);
        }
    }

    // Try to find file with case-insensitive matching
    for (const auto& comp : pathComponents) {
        bool found = false;
        if (std::filesystem::exists(searchPath) && std::filesystem::is_directory(searchPath)) {
            for (const auto& entry : std::filesystem::directory_iterator(searchPath)) {
                std::string entryName = entry.path().filename().string();
                // Case-insensitive comparison
                if (std::equal(comp.begin(), comp.end(), entryName.begin(), entryName.end(),
                              [](char a, char b) { return std::tolower(a) == std::tolower(b); })) {
                    searchPath = entry.path();
                    found = true;
                    break;
                }
            }
        }
        if (!found) {
            logMissingFileOnce(filename);
            return std::vector<uint8_t>();
        }
    }

    // Try to read the found file
    if (std::filesystem::exists(searchPath) && std::filesystem::is_regular_file(searchPath)) {
        std::ifstream file(searchPath, std::ios::binary | std::ios::ate);
        if (file.is_open()) {
            size_t size = file.tellg();
            file.seekg(0, std::ios::beg);
            std::vector<uint8_t> data(size);
            file.read(reinterpret_cast<char*>(data.data()), size);
            LOG_DEBUG("Read loose file (case-insensitive): ", searchPath.string(), " (", size, " bytes)");
            return data;
        }
    }

    logMissingFileOnce(filename);
    return std::vector<uint8_t>();
}

void MPQManager::logMissingFileOnce(const std::string& filename) const {
    std::string normalized = toLowerCopy(filename);
    std::lock_guard<std::mutex> lock(missingFileMutex_);
    if (missingFileWarnings_.insert(normalized).second) {
        LOG_WARNING("File not found: ", filename);
    }
}

uint32_t MPQManager::getFileSize(const std::string& filename) const {
#ifndef HAVE_STORMLIB
    return 0;
#endif

#ifdef HAVE_STORMLIB
    HANDLE archive = findFileArchive(filename);
    if (archive == INVALID_HANDLE_VALUE) {
        return 0;
    }

    HANDLE file = INVALID_HANDLE_VALUE;
    if (!SFileOpenFileEx(archive, filename.c_str(), 0, &file)) {
        return 0;
    }

    DWORD fileSize = SFileGetFileSize(file, nullptr);
    SFileCloseFile(file);

    return (fileSize == SFILE_INVALID_SIZE) ? 0 : fileSize;
#endif

    return 0;
}

HANDLE MPQManager::findFileArchive(const std::string& filename) const {
#ifndef HAVE_STORMLIB
    return INVALID_HANDLE_VALUE;
#endif

#ifdef HAVE_STORMLIB
    // Search archives in priority order (already sorted)
    for (const auto& entry : archives) {
        if (SFileHasFile(entry.handle, filename.c_str())) {
            return entry.handle;
        }
    }
#endif

    return INVALID_HANDLE_VALUE;
}

bool MPQManager::loadPatchArchives() {
#ifndef HAVE_STORMLIB
    return false;
#endif

    // WoW 3.3.5a patch archives (in order of priority, highest first)
    std::vector<std::pair<std::string, int>> patchArchives = {
        {"patch-5.MPQ", 500},
        {"patch-4.MPQ", 400},
        {"patch-3.MPQ", 300},
        {"patch-2.MPQ", 200},
        {"patch.MPQ", 150},
    };

    int loadedPatches = 0;
    for (const auto& [archive, priority] : patchArchives) {
        std::string fullPath = dataPath + "/" + archive;
        if (std::filesystem::exists(fullPath)) {
            if (loadArchive(fullPath, priority)) {
                loadedPatches++;
            }
        }
    }

    LOG_INFO("Loaded ", loadedPatches, " patch archives");
    return loadedPatches > 0;
}

bool MPQManager::loadLocaleArchives(const std::string& locale) {
#ifndef HAVE_STORMLIB
    return false;
#endif

    std::string localePath = dataPath + "/" + locale;
    if (!std::filesystem::exists(localePath)) {
        LOG_WARNING("Locale directory not found: ", localePath);
        return false;
    }

    // Locale-specific archives (including speech MPQs for NPC voices)
    std::vector<std::pair<std::string, int>> localeArchives = {
        {"locale-" + locale + ".MPQ", 250},
        {"speech-" + locale + ".MPQ", 240},  // Base speech/NPC voices
        {"expansion-speech-" + locale + ".MPQ", 245},  // TBC speech
        {"lichking-speech-" + locale + ".MPQ", 248},  // WotLK speech
        {"patch-" + locale + ".MPQ", 450},
        {"patch-" + locale + "-2.MPQ", 460},
        {"patch-" + locale + "-3.MPQ", 470},
    };

    int loadedLocale = 0;
    for (const auto& [archive, priority] : localeArchives) {
        std::string fullPath = localePath + "/" + archive;
        if (std::filesystem::exists(fullPath)) {
            if (loadArchive(fullPath, priority)) {
                loadedLocale++;
            }
        }
    }

    LOG_INFO("Loaded ", loadedLocale, " locale archives for ", locale);
    return loadedLocale > 0;
}

} // namespace pipeline
} // namespace wowee
