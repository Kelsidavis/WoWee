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
    BLPImage loadTexture(const std::string& path);

    /**
     * Set expansion-specific data path for CSV DBC lookup.
     * When set, loadDBC() checks expansionDataPath/db/Name.csv before
     * falling back to the manifest (binary DBC from extracted MPQs).
     */
    void setExpansionDataPath(const std::string& path);

    /**
     * Set a base data path to fall back to when the primary manifest
     * does not contain a requested file.  Call this when the primary
     * dataPath is an expansion-specific subset (e.g. Data/expansions/vanilla/)
     * that only holds DBC overrides, not the full world asset set.
     * @param basePath  Path to the base extraction (Data/) that has a manifest.json
     */
    void setBaseFallbackPath(const std::string& basePath);

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
