#pragma once

#include "pipeline/blp_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/asset_manifest.hpp"
#include "pipeline/loose_file_reader.hpp"
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <mutex>
#include <shared_mutex>

namespace wowee {
namespace pipeline {

/**
 * AssetManager - Unified interface for loading WoW assets
 *
 * Reads pre-extracted loose files indexed by manifest.json.
 * Supports an override directory (Data/override/) checked before the manifest
 * for HD textures, custom content, or mod overrides.
 * Use the asset_extract tool to extract MPQ archives first.
 * All reads are fully parallel (no serialization mutex needed).
 */
class AssetManager {
public:
    AssetManager();
    ~AssetManager();

    /**
     * Initialize asset manager
     * @param dataPath Path to directory containing manifest.json and extracted assets
     * @return true if initialization succeeded
     */
    bool initialize(const std::string& dataPath);

    /**
     * Replace the active manifest without replacing the AssetManager object.
     * Intended for expansion changes from the login screen, before world work
     * is active. Existing file/DBC caches are cleared on success.
     */
    bool switchDataPath(const std::string& newDataPath);

    /**
     * Shutdown and cleanup
     */
    void shutdown();

    /**
     * Check if asset manager is initialized
     */
    [[nodiscard]] bool isInitialized() const { return initialized; }
    [[nodiscard]] const std::string& getDataPath() const { return dataPath; }
    [[nodiscard]] const AssetManifest& getManifest() const { return manifest_; }

    /**
     * Load a BLP texture
     * @param path Virtual path to BLP file (e.g., "Textures\\Minimap\\Background.blp")
     * @return BLP image (check isValid())
     */
    /// keepCompressed asks the loader to leave a DXT texture in its blocks.
    /// A PNG override is decoded regardless - it has no blocks to keep.
    BLPImage loadTexture(const std::string& path, bool keepCompressed = false);

    /**
     * Set expansion-specific data path for CSV DBC lookup.
     * When set, loadDBC() checks expansionDataPath/db/Name.csv before
     * falling back to the manifest (binary DBC from extracted MPQs).
     */
    void setExpansionDataPath(const std::string& path);

    /**
     * Set a base data path to fall back to when the primary manifest does not
     * contain a requested file. Call this when the primary dataPath is an
     * expansion-specific subset (e.g. Data/expansions/wotlk/) that only holds
     * overrides rather than the full world asset set.
     *
     * The fallback is how a thin expansion tree works at all: the wotlk tree
     * here holds 18,943 files against the base tree's 199,468, so nine assets
     * in ten are answered by the base. It is also how one expansion's assets
     * come to stand in for another's, because it is resolved per file with no
     * check on where the file came from, and a borrowed file is not an error
     * anywhere. Cataclysm is the case that makes this visible rather than
     * merely wrong: its Azeroth is the sundered one, so a tile it does not
     * cover is drawn as the old world and nothing says so.
     *
     * So the two trees are compared. A base extracted from a different client
     * than the one being run is refused, and named, rather than used. A base
     * whose manifest predates the `expansion` field cannot be compared, so it
     * is used and said once. WOWEE_ASSET_BASE_FALLBACK=1 forces a refused one
     * back on for whoever is deliberately mixing trees.
     *
     * @param basePath   Path to the base extraction (Data/) with a manifest.json
     * @param expansionId The expansion actually being run, to compare against
     * @return whether the fallback is now armed
     */
    bool setBaseFallbackPath(const std::string& basePath,
                             const std::string& expansionId);

    /// How many lookups have been answered by the base fallback rather than by
    /// the expansion's own tree. Nine in ten is normal for a thin expansion
    /// tree and says nothing on its own; it is the count next to a mismatch
    /// that says how much of what is on screen came from the wrong client.
    [[nodiscard]] uint64_t getBaseFallbackHits() const {
        return baseFallbackHits_.load(std::memory_order_relaxed);
    }

    /**
     * Load a DBC file
     * @param name DBC file name (e.g., "Map.dbc")
     * @return Loaded DBC file (check isLoaded())
     */
    std::shared_ptr<DBCFile> loadDBC(const std::string& name);

    /**
     * Load a DBC file that is optional (not all expansions ship it).
     * Returns nullptr quietly (debug-level log only) when the file is absent.
     * @param name DBC file name (e.g., "Item.dbc")
     * @return Loaded DBC file, or nullptr if not available
     */
    std::shared_ptr<DBCFile> loadDBCOptional(const std::string& name);


    /**
     * Check if a file exists
     * @param path Virtual file path
     * @return true if file exists
     */
    [[nodiscard]] bool fileExists(const std::string& path) const;

    /**
     * Read raw file data
     * @param path Virtual file path
     * @return File contents (empty if not found)
     */
    [[nodiscard]] std::vector<uint8_t> readFile(const std::string& path) const;

    /**
     * Read optional file data without warning spam.
     * Intended for probe-style lookups (e.g. external .anim variants).
     * @param path Virtual file path
     * @return File contents (empty if not found)
     */
    [[nodiscard]] std::vector<uint8_t> readFileOptional(const std::string& path) const;

    /**
     * Get loaded DBC count
     */
    [[nodiscard]] size_t getLoadedDBCCount() const { return dbcCache.size(); }

    /**
     * Get file cache stats
     */
    [[nodiscard]] size_t getFileCacheSize() const { return fileCacheTotalBytes; }
    [[nodiscard]] size_t getFileCacheHits() const { return fileCacheHits; }
    [[nodiscard]] size_t getFileCacheMisses() const { return fileCacheMisses; }

    /**
     * Clear all cached resources
     */
    void clearCache();

    /**
     * Clear only DBC cache (forces reload on next loadDBC call)
     */
    void clearDBCCache();


    /**
     * Resolve a normalized WoW path to its on-disk location. Checks the
     * override directory first, then the manifest, then the base-fallback
     * manifest. Public so callers (e.g. terrain_manager probing for
     * sidecar files like .whm/.wot/.woc next to a .adt) can locate the
     * extracted file's directory without reading it.
     * @return absolute or relative fs path, or "" if not found
     */
    [[nodiscard]] std::string resolveFile(const std::string& normalizedPath) const;

private:
    bool initialized = false;
    std::string dataPath;
    std::string expansionDataPath_;  // e.g. "Data/expansions/wotlk"
    std::string overridePath_;       // e.g. "Data/override"

    // Base manifest (loaded from dataPath/manifest.json)
    AssetManifest manifest_;

    // Optional base-path fallback: used when manifest_ doesn't contain a file.
    // Populated by setBaseFallbackPath(); ignored if baseFallbackDataPath_ is empty.
    std::string    baseFallbackDataPath_;
    AssetManifest  baseFallbackManifest_;
    // Counted from resolveAssetPath, which is const and takes no lock while
    // several threads stream assets through it, so relaxed atomic rather than
    // a plain mutable: nothing orders against this and a torn count would be
    // a data race for a diagnostic.
    mutable std::atomic<uint64_t> baseFallbackHits_{0};

    // (resolveFile moved to public - declaration above.)

    // Guards fileCache, dbcCache, fileCacheTotalBytes, fileCacheAccessCounter, and
    // fileCacheBudget.  Shared lock for read-only cache lookups (readFile cache hit,
    // loadDBC cache hit); exclusive lock for inserts and eviction.
    mutable std::shared_mutex cacheMutex;
    // THREAD-SAFE: protected by cacheMutex (exclusive lock for writes).
    std::unordered_map<std::string, std::shared_ptr<DBCFile>> dbcCache;

    // File cache (LRU, dynamic budget based on system RAM)
    struct CachedFile {
        std::vector<uint8_t> data;
        uint64_t lastAccessTime;
    };
    // THREAD-SAFE: protected by cacheMutex (shared_mutex - shared_lock for reads,
    // exclusive lock_guard for writes/eviction).
    mutable std::unordered_map<std::string, CachedFile> fileCache;
    mutable size_t fileCacheTotalBytes = 0;
    mutable uint64_t fileCacheAccessCounter = 0;
    // THREAD-SAFE: atomic - incremented from any thread after releasing cacheMutex.
    mutable std::atomic<size_t> fileCacheHits{0};
    mutable std::atomic<size_t> fileCacheMisses{0};
    mutable size_t fileCacheBudget = 1024 * 1024 * 1024;  // Dynamic, starts at 1GB

    void setupFileCacheBudget();

    /**
     * Try to load a PNG override for a BLP path.
     * Returns valid BLPImage if PNG found, invalid otherwise.
     */
    [[nodiscard]] BLPImage tryLoadPngOverride(const std::string& normalizedPath) const;

    /**
     * Normalize path for case-insensitive lookup
     */
    [[nodiscard]] std::string normalizePath(const std::string& path) const;
};

} // namespace pipeline
} // namespace wowee
