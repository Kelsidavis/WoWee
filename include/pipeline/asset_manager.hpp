#pragma once

#include "pipeline/blp_loader.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/asset_manifest.hpp"
#include "pipeline/loose_file_reader.hpp"
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>

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
     * Shutdown and cleanup
     */
    void shutdown();

    /**
     * Check if asset manager is initialized
     */
    bool isInitialized() const { return initialized; }
    const std::string& getDataPath() const { return dataPath; }

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
     * Load a DBC file
     * @param name DBC file name (e.g., "Map.dbc")
     * @return Loaded DBC file (check isLoaded())
     */
    std::shared_ptr<DBCFile> loadDBC(const std::string& name);

    /**
     * Get a cached DBC file
     * @param name DBC file name
     * @return Cached DBC or nullptr if not loaded
     */
    std::shared_ptr<DBCFile> getDBC(const std::string& name) const;

    /**
     * Check if a file exists
     * @param path Virtual file path
     * @return true if file exists
     */
    bool fileExists(const std::string& path) const;

    /**
     * Read raw file data
     * @param path Virtual file path
     * @return File contents (empty if not found)
     */
    std::vector<uint8_t> readFile(const std::string& path) const;

    /**
     * Read optional file data without warning spam.
     * Intended for probe-style lookups (e.g. external .anim variants).
     * @param path Virtual file path
     * @return File contents (empty if not found)
     */
    std::vector<uint8_t> readFileOptional(const std::string& path) const;

    /**
     * Get loaded DBC count
     */
    size_t getLoadedDBCCount() const { return dbcCache.size(); }

    /**
     * Get file cache stats
     */
    size_t getFileCacheSize() const { return fileCacheTotalBytes; }
    size_t getFileCacheHits() const { return fileCacheHits; }
    size_t getFileCacheMisses() const { return fileCacheMisses; }

    /**
     * Clear all cached resources
     */
    void clearCache();

    /**
     * Clear only DBC cache (forces reload on next loadDBC call)
     */
    void clearDBCCache();

private:
    bool initialized = false;
    std::string dataPath;
    std::string expansionDataPath_;  // e.g. "Data/expansions/wotlk"
    std::string overridePath_;       // e.g. "Data/override"

    // Base manifest (loaded from dataPath/manifest.json)
    AssetManifest manifest_;
    LooseFileReader looseReader_;

    /**
     * Resolve filesystem path: check override dir first, then base manifest.
     * Returns empty string if not found.
     */
    std::string resolveFile(const std::string& normalizedPath) const;

    mutable std::mutex cacheMutex;
    std::map<std::string, std::shared_ptr<DBCFile>> dbcCache;

    // File cache (LRU, dynamic budget based on system RAM)
    struct CachedFile {
        std::vector<uint8_t> data;
        uint64_t lastAccessTime;
    };
    mutable std::map<std::string, CachedFile> fileCache;
    mutable size_t fileCacheTotalBytes = 0;
    mutable uint64_t fileCacheAccessCounter = 0;
    mutable size_t fileCacheHits = 0;
    mutable size_t fileCacheMisses = 0;
    mutable size_t fileCacheBudget = 1024 * 1024 * 1024;  // Dynamic, starts at 1GB

    void setupFileCacheBudget();

    /**
     * Try to load a PNG override for a BLP path.
     * Returns valid BLPImage if PNG found, invalid otherwise.
     */
    BLPImage tryLoadPngOverride(const std::string& normalizedPath) const;

    /**
     * Normalize path for case-insensitive lookup
     */
    std::string normalizePath(const std::string& path) const;
};

} // namespace pipeline
} // namespace wowee
