#include "pipeline/asset_manager.hpp"
#include "pipeline/base_fallback.hpp"
#include "core/logger.hpp"
#include "core/memory_monitor.hpp"
#include "core/profiler.hpp"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <unordered_set>

#include "stb_image.h"

namespace wowee {
namespace pipeline {

namespace {
size_t parseEnvSizeMB(const char* name) {
    const char* v = std::getenv(name);
    if (!v || !*v) {
        return 0;
    }
    char* end = nullptr;
    unsigned long long mb = std::strtoull(v, &end, 10);
    if (end == v || mb == 0) {
        return 0;
    }
    if (mb > (std::numeric_limits<size_t>::max() / (1024ull * 1024ull))) {
        return 0;
    }
    return static_cast<size_t>(mb);
}

size_t parseEnvCount(const char* name, size_t defValue) {
    const char* v = std::getenv(name);
    if (!v || !*v) {
        return defValue;
    }
    char* end = nullptr;
    unsigned long long n = std::strtoull(v, &end, 10);
    if (end == v || n == 0) {
        return defValue;
    }
    return static_cast<size_t>(n);
}
} // namespace

AssetManager::AssetManager() = default;
AssetManager::~AssetManager() {
    shutdown();
}

bool AssetManager::initialize(const std::string& dataPath_) {
    if (initialized) {
        LOG_WARNING("AssetManager already initialized");
        return true;
    }

    dataPath = dataPath_;
    overridePath_ = dataPath + "/override";
    LOG_INFO("Initializing asset manager with data path: ", dataPath);

    setupFileCacheBudget();

    std::string manifestPath = dataPath + "/manifest.json";
    if (!std::filesystem::exists(manifestPath)) {
        LOG_ERROR("manifest.json not found in: ", dataPath);
        LOG_ERROR("Run asset_extract to extract MPQ archives first");
        return false;
    }

    if (!manifest_.load(manifestPath)) {
        LOG_ERROR("Failed to load manifest");
        return false;
    }

    if (std::filesystem::is_directory(overridePath_)) {
        LOG_INFO("Override directory found: ", overridePath_);
    }

    initialized = true;
    LOG_INFO("Asset manager initialized: ", manifest_.getEntryCount(),
             " files indexed (file cache: ", fileCacheBudget / (1024 * 1024), " MB)");
    return true;
}

bool AssetManager::switchDataPath(const std::string& newDataPath) {
    if (newDataPath.empty()) return false;
    if (initialized && newDataPath == dataPath) return true;

    const std::string manifestPath = newDataPath + "/manifest.json";
    AssetManifest nextManifest;
    if (!std::filesystem::exists(manifestPath) || !nextManifest.load(manifestPath)) {
        LOG_ERROR("Cannot switch asset path; valid manifest not found in: ", newDataPath);
        return false;
    }

    clearCache();
    manifest_ = std::move(nextManifest);
    baseFallbackManifest_ = AssetManifest{};
    baseFallbackDataPath_.clear();
    dataPath = newDataPath;
    overridePath_ = dataPath + "/override";
    initialized = true;
    LOG_INFO("Switched asset manager to: ", dataPath, " (",
             manifest_.getEntryCount(), " files indexed)");
    return true;
}

void AssetManager::setupFileCacheBudget() {
    auto& memMonitor = core::MemoryMonitor::getInstance();
    size_t recommendedBudget = memMonitor.getRecommendedCacheBudget();
    size_t dynamicBudget = (recommendedBudget * 3) / 4;

    const size_t envFixedMB = parseEnvSizeMB("WOWEE_FILE_CACHE_MB");
    const size_t envMaxMB = parseEnvSizeMB("WOWEE_FILE_CACHE_MAX_MB");

    const size_t minBudgetBytes = 256ull * 1024ull * 1024ull;
#ifdef __ANDROID__
    // Half of available RAM is a desktop rule. Android does not let one app
    // have that: it enforces a per-app limit far below the machine's memory and
    // kills the process rather than swapping when it is passed. A phone with
    // 8 GB was handing this cache 840 MB, which is both more than the app may
    // hold and a good way to be killed the moment it goes to the background.
    const size_t defaultMaxBudgetBytes = 384ull * 1024ull * 1024ull;
#else
    const size_t defaultMaxBudgetBytes = 12288ull * 1024ull * 1024ull;  // 12 GB max for file cache
#endif
    const size_t maxBudgetBytes = (envMaxMB > 0)
        ? (envMaxMB * 1024ull * 1024ull)
        : defaultMaxBudgetBytes;

    if (envFixedMB > 0) {
        fileCacheBudget = envFixedMB * 1024ull * 1024ull;
        if (fileCacheBudget < minBudgetBytes) {
            fileCacheBudget = minBudgetBytes;
        }
        LOG_WARNING("Asset file cache fixed via WOWEE_FILE_CACHE_MB=", envFixedMB,
                    " (effective ", fileCacheBudget / (1024 * 1024), " MB)");
    } else {
        fileCacheBudget = std::clamp(dynamicBudget, minBudgetBytes, maxBudgetBytes);
    }
}

void AssetManager::shutdown() {
    if (!initialized) {
        return;
    }

    LOG_INFO("Shutting down asset manager");

    if (fileCacheHits + fileCacheMisses > 0) {
        float hitRate = static_cast<float>(fileCacheHits) / (fileCacheHits + fileCacheMisses) * 100.0f;
        LOG_INFO("File cache stats: ", fileCacheHits, " hits, ", fileCacheMisses, " misses (",
                 static_cast<int>(hitRate), "% hit rate), ", fileCacheTotalBytes / 1024 / 1024, " MB cached");
    }

    clearCache();
    initialized = false;
}

std::string AssetManager::resolveFile(const std::string& normalizedPath) const {
    // Check override directory first (for HD upgrades, custom textures)
    if (!overridePath_.empty()) {
        const auto* entry = manifest_.lookup(normalizedPath);
        if (entry && !entry->filesystemPath.empty()) {
            std::string overrideFsPath = overridePath_ + "/" + entry->filesystemPath;
            if (LooseFileReader::fileExists(overrideFsPath)) {
                return overrideFsPath;
            }
        }
    }
    // Primary manifest
    std::string primaryPath = manifest_.resolveFilesystemPath(normalizedPath);
    if (!primaryPath.empty()) return primaryPath;

    // If a base-path fallback is configured (expansion-specific primary that only
    // holds DBC overrides), retry against the base extraction.
    if (!baseFallbackDataPath_.empty()) {
        std::string baseFallbackPath = baseFallbackManifest_.resolveFilesystemPath(normalizedPath);
        if (!baseFallbackPath.empty()) {
            baseFallbackHits_.fetch_add(1, std::memory_order_relaxed);
            return baseFallbackPath;
        }
    }

    // Last resort: some files (e.g. DBFilesClient\TransportAnimation.dbc) can end up
    // present on disk under dataPath without ever having been captured by whatever
    // extraction produced manifest.json. Try the loose file directly at its expected
    // location before giving up, so a missing manifest entry doesn't silently mean
    // "file doesn't exist" when it plainly does.
    std::string looseCandidate = normalizedPath;
    std::replace(looseCandidate.begin(), looseCandidate.end(), '\\', '/');
    looseCandidate = dataPath + "/" + looseCandidate;
    if (LooseFileReader::fileExists(looseCandidate)) {
        return looseCandidate;
    }
    return {};
}

bool AssetManager::setBaseFallbackPath(const std::string& basePath,
                                       const std::string& expansionId) {
    if (baseFallbackHits_.load(std::memory_order_relaxed) > 0) {
        // Said on the way out rather than per lookup. Next to a warning about
        // an unlabelled or forced base, this is how much of what was on screen
        // came from it.
        LOG_INFO("AssetManager: previous base fallback '", baseFallbackDataPath_,
                 "' answered ", baseFallbackHits_.load(std::memory_order_relaxed),
                 " lookups");
    }
    baseFallbackDataPath_.clear();
    baseFallbackHits_.store(0, std::memory_order_relaxed);
    if (basePath.empty() || basePath == dataPath) return false;  // nothing to do
    std::string manifestPath = basePath + "/manifest.json";
    if (!std::filesystem::exists(manifestPath)) {
        LOG_DEBUG("AssetManager: base fallback manifest not found at ", manifestPath,
                  " - fallback disabled");
        return false;
    }
    if (!baseFallbackManifest_.load(manifestPath)) return false;

    const std::string& baseExpansion = baseFallbackManifest_.getExpansion();
    const char* forcedEnv = std::getenv("WOWEE_ASSET_BASE_FALLBACK");
    const bool forced = forcedEnv && forcedEnv[0] == '1';

    switch (decideBaseFallback(baseExpansion, expansionId, forced)) {
        case BaseFallbackDecision::Use:
            break;
        case BaseFallbackDecision::UseUnlabelled:
            // Nothing can be concluded, so it is used and said: an unlabelled
            // tree is how the wrong client's assets reach the screen without
            // anybody having chosen that.
            LOG_WARNING("AssetManager: base fallback '", basePath,
                        "' does not record which client it was extracted from, so it "
                        "cannot be checked against '", expansionId,
                        "'. Re-extract to label it. Files it holds and '", expansionId,
                        "' does not will be used as they are");
            break;
        case BaseFallbackDecision::Refuse:
            LOG_ERROR("AssetManager: base fallback '", basePath, "' was extracted from '",
                      baseExpansion, "' and the client is running '", expansionId,
                      "'. Refusing it: every file '", expansionId,
                      "' does not cover would otherwise be drawn from '", baseExpansion,
                      "' with nothing to say so. Extract '", expansionId,
                      "' into its own data root, or set WOWEE_ASSET_BASE_FALLBACK=1 "
                      "to use it anyway");
            return false;
        case BaseFallbackDecision::UseForced:
            LOG_WARNING("AssetManager: using base fallback '", basePath, "' from '",
                        baseExpansion, "' while running '", expansionId,
                        "' because WOWEE_ASSET_BASE_FALLBACK=1");
            break;
    }

    baseFallbackDataPath_ = basePath;
    LOG_INFO("AssetManager: base fallback path set to '", basePath, "' (",
             baseFallbackManifest_.getEntryCount(), " files, expansion '",
             baseExpansion.empty() ? std::string("unrecorded") : baseExpansion, "')");
    return true;
}

BLPImage AssetManager::loadTexture(const std::string& path, bool keepCompressed) {
    ZoneScopedN("AssetManager::loadTexture");
    // Callers ask for the blocks to save the decompression and the memory. On a
    // GPU that cannot sample them the saving is a blank surface, so the answer
    // is no and the loader unpacks to RGBA8 instead.
    keepCompressed = keepCompressed && blockCompressionSupported();
    if (!initialized) {
        LOG_ERROR("AssetManager not initialized");
        return BLPImage();
    }

    std::string normalizedPath = normalizePath(path);

    LOG_DEBUG("Loading texture: ", normalizedPath);

    // Check for PNG override
    BLPImage pngImage = tryLoadPngOverride(normalizedPath);
    if (pngImage.isValid()) {
        return pngImage;
    }

    std::vector<uint8_t> blpData = readFile(normalizedPath);
    if (blpData.empty()) {
        static std::mutex logMtx;
        static std::unordered_set<std::string> loggedMissingTextures;
        static bool missingTextureLogSuppressed = false;
        static const size_t kMaxMissingTextureLogKeys =
            parseEnvCount("WOWEE_TEXTURE_MISS_LOG_KEYS", 400);
        std::lock_guard<std::mutex> lock(logMtx);
        if (loggedMissingTextures.size() < kMaxMissingTextureLogKeys &&
            loggedMissingTextures.insert(normalizedPath).second) {
            LOG_WARNING("Texture not found: ", normalizedPath);
        } else if (!missingTextureLogSuppressed && loggedMissingTextures.size() >= kMaxMissingTextureLogKeys) {
            LOG_WARNING("Texture-not-found warning key cache reached ", kMaxMissingTextureLogKeys,
                        " entries; suppressing new unique texture-miss logs");
            missingTextureLogSuppressed = true;
        }
        return BLPImage();
    }

    BLPImage image = BLPLoader::load(blpData, keepCompressed);
    if (!image.isValid()) {
        static std::mutex logMtx;
        static std::unordered_set<std::string> loggedDecodeFails;
        static bool decodeFailLogSuppressed = false;
        static const size_t kMaxDecodeFailLogKeys =
            parseEnvCount("WOWEE_TEXTURE_DECODE_LOG_KEYS", 200);
        std::lock_guard<std::mutex> lock(logMtx);
        if (loggedDecodeFails.size() < kMaxDecodeFailLogKeys &&
            loggedDecodeFails.insert(normalizedPath).second) {
            LOG_ERROR("Failed to load texture: ", normalizedPath);
        } else if (!decodeFailLogSuppressed && loggedDecodeFails.size() >= kMaxDecodeFailLogKeys) {
            LOG_WARNING("Texture-decode warning key cache reached ", kMaxDecodeFailLogKeys,
                        " entries; suppressing new unique decode-failure logs");
            decodeFailLogSuppressed = true;
        }
        return BLPImage();
    }

    LOG_DEBUG("Loaded texture: ", normalizedPath, " (", image.width, "x", image.height, ")");
    return image;
}

BLPImage AssetManager::tryLoadPngOverride(const std::string& normalizedPath) const {
    if (normalizedPath.size() < 4) return BLPImage();

    std::string ext = normalizedPath.substr(normalizedPath.size() - 4);
    if (ext != ".blp") return BLPImage();

    // Try the standard sidecar path first: extracted .blp's directory + .png.
    std::string fsPath = resolveFile(normalizedPath);
    std::string pngPath;
    if (!fsPath.empty() && fsPath.size() >= 4) {
        pngPath = fsPath.substr(0, fsPath.size() - 4) + ".png";
        if (!LooseFileReader::fileExists(pngPath)) pngPath.clear();
    }

    // Fallback: probe well-known custom-zone texture roots so that PNG-only
    // assets ship without needing a phantom BLP manifest entry. Path is
    // forward-slash + lowercase to match the editor's PNG export convention.
    if (pngPath.empty()) {
        std::string norm = normalizedPath;
        std::replace(norm.begin(), norm.end(), '\\', '/');
        std::transform(norm.begin(), norm.end(), norm.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        std::string candidate = norm.substr(0, norm.size() - 4) + ".png";
        for (const char* root : {"custom_zones/textures/", "output/textures/"}) {
            std::string p = std::string(root) + candidate;
            if (LooseFileReader::fileExists(p)) { pngPath = p; break; }
        }
    }
    if (pngPath.empty()) return BLPImage();

    int w, h, channels;
    unsigned char* pixels = stbi_load(pngPath.c_str(), &w, &h, &channels, 4);
    if (!pixels) {
        LOG_WARNING("PNG override exists but failed to load: ", pngPath);
        return BLPImage();
    }
    // Cap texture dimensions. WoW textures top out at 4K; stbi can return
    // 32K x 32K which would allocate 4GB on a malicious PNG.
    if (w <= 0 || h <= 0 || w > 8192 || h > 8192) {
        LOG_WARNING("PNG override dimensions out of range (", w, "x", h, "): ", pngPath);
        stbi_image_free(pixels);
        return BLPImage();
    }

    BLPImage image;
    image.width = w;
    image.height = h;
    image.channels = 4;
    image.format = BLPFormat::BLP2;
    image.compression = BLPCompression::ARGB8888;
    image.data.assign(pixels, pixels + (static_cast<size_t>(w) * h * 4));
    stbi_image_free(pixels);

    LOG_INFO("PNG override loaded: ", pngPath, " (", w, "x", h, ")");
    return image;
}

void AssetManager::setExpansionDataPath(const std::string& path) {
    expansionDataPath_ = path;
    LOG_INFO("Expansion data path for CSV DBCs: ", expansionDataPath_);
}

std::shared_ptr<DBCFile> AssetManager::loadDBC(const std::string& name) {
    ZoneScopedN("AssetManager::loadDBC");
    if (!initialized) {
        LOG_ERROR("AssetManager not initialized");
        return nullptr;
    }

    auto it = dbcCache.find(name);
    if (it != dbcCache.end()) {
        LOG_DEBUG("DBC already loaded (cached): ", name);
        return it->second;
    }

    LOG_DEBUG("Loading DBC: ", name);

    std::vector<uint8_t> dbcData;

    // Try binary DBC from extracted MPQs first (preferred source).
    std::string dbcPath = "DBFilesClient\\" + name;
    {
        dbcData = readFile(dbcPath);
    }

    // If asset_extract was run with --emit-json-dbc, the DBC's directory
    // also contains a JSON sidecar. Use it when the binary is missing
    // (lets users run with PNG/JSON-only extractions for testing the
    // open-format end-to-end path). Server-mode never reads via this
    // code path, so private-server compat is unaffected.
    if (dbcData.empty()) {
        std::string normalizedDbc = normalizePath(dbcPath);
        std::string fsPath = resolveFile(normalizedDbc);
        if (!fsPath.empty() && fsPath.size() >= 4) {
            std::string sidecar = fsPath.substr(0, fsPath.size() - 4) + ".json";
            if (std::filesystem::exists(sidecar)) {
                std::ifstream jf(sidecar, std::ios::binary | std::ios::ate);
                if (jf) {
                    auto sz = jf.tellg();
                    if (sz > 0) {
                        dbcData.resize(static_cast<size_t>(sz));
                        jf.seekg(0);
                        jf.read(reinterpret_cast<char*>(dbcData.data()), sz);
                        LOG_INFO("Loading JSON DBC sidecar: ", sidecar);
                    }
                }
            }
        }
    }

    // Try Data/db/ directory (pre-extracted binary DBCs shared across expansions)
    if (dbcData.empty()) {
        // Expansion overlay first (e.g. Data/expansions/tbc/overlay/db/), then
        // expansion db/, then shared Data/db/.
        std::vector<std::string> dbDirs;
        if (!expansionDataPath_.empty())
            dbDirs.push_back(expansionDataPath_ + "/overlay/db");
        dbDirs.push_back(dataPath + "/db");
        dbDirs.push_back(dataPath + "/../../db");
        dbDirs.emplace_back("Data/db");
        // Try exact-case first, then case-insensitive scan (Linux is case-sensitive
        // but DBC filenames in Data/db/ are often all-lowercase).
        std::string nameLower = name;
        std::transform(nameLower.begin(), nameLower.end(), nameLower.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        for (const auto& dir : dbDirs) {
            if (!std::filesystem::is_directory(dir)) continue;
            std::string exact = dir + "/" + name;
            std::string resolved;
            if (std::filesystem::exists(exact)) {
                resolved = exact;
            } else {
                for (const auto& entry : std::filesystem::directory_iterator(dir)) {
                    if (!entry.is_regular_file()) continue;
                    std::string fn = entry.path().filename().string();
                    std::string fnLower = fn;
                    std::transform(fnLower.begin(), fnLower.end(), fnLower.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                    if (fnLower == nameLower) {
                        resolved = entry.path().string();
                        break;
                    }
                }
            }
            if (resolved.empty()) continue;
            std::ifstream f(resolved, std::ios::binary | std::ios::ate);
            if (f) {
                auto size = f.tellg();
                if (size > 0) {
                    f.seekg(0);
                    dbcData.resize(static_cast<size_t>(size));
                    f.read(reinterpret_cast<char*>(dbcData.data()), size);
                    LOG_INFO("Loaded binary DBC from: ", resolved, " (", size, " bytes)");
                    break;
                }
            }
        }
    }

    // Check for JSON DBC from custom zones (wowee open format)
    if (dbcData.empty()) {
        std::string baseName = name;
        auto dot = baseName.rfind('.');
        if (dot != std::string::npos) baseName = baseName.substr(0, dot);
        for (const char* dir : {"custom_zones", "output"}) {
            if (!std::filesystem::exists(dir)) continue;
            for (auto& entry : std::filesystem::directory_iterator(dir)) {
                if (!entry.is_directory()) continue;
                std::string jsonPath = entry.path().string() + "/data/" + baseName + ".json";
                if (std::filesystem::exists(jsonPath)) {
                    std::ifstream jf(jsonPath, std::ios::binary | std::ios::ate);
                    if (jf) {
                        auto sz = jf.tellg();
                        if (sz > 0) {
                            dbcData.resize(static_cast<size_t>(sz));
                            jf.seekg(0);
                            jf.read(reinterpret_cast<char*>(dbcData.data()), sz);
                            LOG_INFO("Loading JSON DBC override: ", jsonPath);
                        }
                    }
                    break;
                }
            }
            if (!dbcData.empty()) break;
        }
    }

    // Fall back to expansion-specific CSV (e.g. Data/expansions/wotlk/db/Spell.csv)
    if (dbcData.empty() && !expansionDataPath_.empty()) {
        std::string baseName = name;
        auto dot = baseName.rfind('.');
        if (dot != std::string::npos) {
            baseName = baseName.substr(0, dot);
        }
        std::string csvPath = expansionDataPath_ + "/db/" + baseName + ".csv";
        if (std::filesystem::exists(csvPath)) {
            std::ifstream f(csvPath, std::ios::binary | std::ios::ate);
            if (f) {
                auto size = f.tellg();
                if (size > 0) {
                    f.seekg(0);
                    dbcData.resize(static_cast<size_t>(size));
                    f.read(reinterpret_cast<char*>(dbcData.data()), size);
                    LOG_INFO("Binary DBC not found, using CSV fallback: ", csvPath);
                }
            }
        }
    }

    if (dbcData.empty()) {
        LOG_WARNING("DBC not found: ", name);
        return nullptr;
    }

    auto dbc = std::make_shared<DBCFile>();
    if (!dbc->load(dbcData)) {
        LOG_ERROR("Failed to load DBC: ", name);
        return nullptr;
    }

    dbcCache[name] = dbc;

    LOG_INFO("Loaded DBC: ", name, " (", dbc->getRecordCount(), " records)");
    return dbc;
}

std::shared_ptr<DBCFile> AssetManager::loadDBCOptional(const std::string& name) {
    // Check cache first
    auto it = dbcCache.find(name);
    if (it != dbcCache.end()) return it->second;

    // Try binary DBC
    std::vector<uint8_t> dbcData;
    {
        std::string dbcPath = "DBFilesClient\\" + name;
        dbcData = readFile(dbcPath);
    }

    // Fall back to expansion-specific CSV
    if (dbcData.empty() && !expansionDataPath_.empty()) {
        std::string baseName = name;
        auto dot = baseName.rfind('.');
        if (dot != std::string::npos) baseName = baseName.substr(0, dot);
        std::string csvPath = expansionDataPath_ + "/db/" + baseName + ".csv";
        if (std::filesystem::exists(csvPath)) {
            std::ifstream f(csvPath, std::ios::binary | std::ios::ate);
            if (f) {
                auto size = f.tellg();
                if (size > 0) {
                    f.seekg(0);
                    dbcData.resize(static_cast<size_t>(size));
                    f.read(reinterpret_cast<char*>(dbcData.data()), size);
                    LOG_INFO("Binary DBC not found, using CSV fallback: ", csvPath);
                }
            }
        }
    }

    if (dbcData.empty()) {
        // Expected on some expansions - log at debug level only.
        LOG_DEBUG("Optional DBC not found (expected on some expansions): ", name);
        return nullptr;
    }

    auto dbc = std::make_shared<DBCFile>();
    if (!dbc->load(dbcData)) {
        LOG_ERROR("Failed to load DBC: ", name);
        return nullptr;
    }

    dbcCache[name] = dbc;
    LOG_INFO("Loaded optional DBC: ", name, " (", dbc->getRecordCount(), " records)");
    return dbc;
}
bool AssetManager::fileExists(const std::string& path) const {
    if (!initialized) {
        return false;
    }
    std::string normalized = normalizePath(path);
    // Resolved the same way a read is, not looked up in the primary manifest.
    //
    // A read walks override, primary manifest, base fallback manifest, then the
    // loose file; this asked the primary manifest alone. With one asset source
    // the two agree, which is why it stood. The moment an expansion overlay
    // becomes the primary - an overlay holding a few thousand models over a
    // two-hundred-thousand file base - this answered NO for every file in the
    // base game, while the read that follows would have found it.
    //
    // Fifty-eight callers ask this before deciding what to read, so every one
    // of them took the wrong branch at once: a weapon texture that is present
    // under Weapon\ was declared missing and looked for under Shield\, where
    // it has never been, and the weapon drew white.
    return !resolveFile(normalized).empty();
}

std::vector<uint8_t> AssetManager::readFile(const std::string& path) const {
    if (!initialized) {
        return {};
    }

    std::string normalized = normalizePath(path);

    // Check cache first (shared lock allows concurrent reads)
    {
        std::shared_lock<std::shared_mutex> cacheLock(cacheMutex);
        auto it = fileCache.find(normalized);
        if (it != fileCache.end()) {
            auto data = it->second.data;
            cacheLock.unlock();
            fileCacheHits++;
            return data;
        }
    }

    // Read from filesystem (override dir first, then base manifest)
    std::string fsPath = resolveFile(normalized);
    if (fsPath.empty()) {
        return {};
    }

    auto data = LooseFileReader::readFile(fsPath);
    if (data.empty()) {
        LOG_WARNING("Manifest entry exists but file unreadable: ", fsPath);
        return data;
    }

    // Add to cache if within budget
    size_t fileSize = data.size();
    if (fileSize > 0 && fileSize < fileCacheBudget / 2) {
        std::lock_guard<std::shared_mutex> cacheLock(cacheMutex);
        // Evict old entries if needed (LRU)
        while (fileCacheTotalBytes + fileSize > fileCacheBudget && !fileCache.empty()) {
            auto lru = fileCache.begin();
            for (auto it = fileCache.begin(); it != fileCache.end(); ++it) {
                if (it->second.lastAccessTime < lru->second.lastAccessTime) {
                    lru = it;
                }
            }
            fileCacheTotalBytes -= lru->second.data.size();
            fileCache.erase(lru);
        }

        CachedFile cached;
        cached.data = data;
        cached.lastAccessTime = ++fileCacheAccessCounter;
        fileCache[normalized] = std::move(cached);
        fileCacheTotalBytes += fileSize;
    }

    return data;
}

std::vector<uint8_t> AssetManager::readFileOptional(const std::string& path) const {
    if (!initialized) {
        return {};
    }
    if (!fileExists(path)) {
        return {};
    }
    return readFile(path);
}

void AssetManager::clearDBCCache() {
    std::lock_guard<std::shared_mutex> lock(cacheMutex);
    dbcCache.clear();
    LOG_INFO("Cleared DBC cache");
}

void AssetManager::clearCache() {
    std::lock_guard<std::shared_mutex> lock(cacheMutex);
    dbcCache.clear();
    fileCache.clear();
    fileCacheTotalBytes = 0;
    fileCacheAccessCounter = 0;
    LOG_INFO("Cleared asset cache (DBC + file cache)");
}
std::string AssetManager::normalizePath(const std::string& path) const {
    std::string normalized = path;
    std::replace(normalized.begin(), normalized.end(), '/', '\\');
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    // Reject path traversal sequences
    if (normalized.find("..\\") != std::string::npos ||
        normalized.find("../") != std::string::npos ||
        normalized == "..") {
        LOG_WARNING("Path traversal rejected: ", path);
        return {};
    }

    return normalized;
}

} // namespace pipeline
} // namespace wowee
