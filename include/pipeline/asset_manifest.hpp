#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>

namespace wowee {
namespace pipeline {

/**
 * AssetManifest - Maps WoW virtual paths to filesystem paths
 *
 * Loaded once at startup from manifest.json. Read-only after init,
 * so concurrent reads are safe without a mutex.
 */
class AssetManifest {
public:
    struct Entry {
        std::string filesystemPath;  // Relative path from basePath (forward slashes)
        uint64_t size;               // File size in bytes
        uint32_t crc32;              // CRC32 for integrity verification
    };

    AssetManifest() = default;

    /**
     * Load manifest from JSON file
     * @param manifestPath Full path to manifest.json
     * @return true if loaded successfully
     */
    [[nodiscard]] bool load(const std::string& manifestPath);

    /**
     * Lookup an entry by normalized WoW path (lowercase, backslash)
     * @return Pointer to entry or nullptr if not found
     */
    [[nodiscard]] const Entry* lookup(const std::string& normalizedWowPath) const;

    /**
     * Resolve full filesystem path for a WoW virtual path
     * @return Full filesystem path or empty string if not found
     */
    [[nodiscard]] std::string resolveFilesystemPath(const std::string& normalizedWowPath) const;

    /**
     * Check if an entry exists
     */
    [[nodiscard]] bool hasEntry(const std::string& normalizedWowPath) const;

    /**
     * Get base path (directory containing extracted assets)
     */
    [[nodiscard]] const std::string& getBasePath() const { return basePath_; }

    /// Which client this tree was extracted from, e.g. "wotlk". Empty when the
    /// manifest predates the field, which is not the same as "no expansion":
    /// it means the answer is unknown and nothing may be concluded from it.
    [[nodiscard]] const std::string& getExpansion() const { return expansion_; }

    /**
     * Get total number of entries
     */
    [[nodiscard]] size_t getEntryCount() const { return entries_.size(); }

    /**
     * Access entries map (read-only) for iteration
     */
    [[nodiscard]] const std::unordered_map<std::string, Entry>& getEntries() const { return entries_; }

    /**
     * Check if manifest is loaded
     */
    [[nodiscard]] bool isLoaded() const { return loaded_; }

private:
    bool loaded_ = false;
    std::string basePath_;           // Root directory for extracted assets
    std::string expansion_;          // Expansion this tree came from, empty if unrecorded
    std::string manifestDir_;        // Directory containing manifest.json
    std::unordered_map<std::string, Entry> entries_;
};

} // namespace pipeline
} // namespace wowee
