#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace wowee {
namespace tools {

/**
 * Generates manifest.json from extracted file metadata.
 */
class ManifestWriter {
public:
    struct FileEntry {
        std::string wowPath;        // Normalized WoW virtual path (lowercase, backslash)
        std::string filesystemPath; // Relative path from basePath (forward slashes, original case)
        uint64_t size;              // File size in bytes
        uint32_t crc32;             // CRC32 checksum
    };

    /**
     * Write manifest.json
     * @param outputPath Full path to manifest.json
     * @param basePath Value for basePath field (e.g., "assets")
     * @param expansion Which client this was extracted from, e.g. "wotlk".
     *        The client compares it against the expansion it is running before
     *        letting one asset tree stand in for another's missing files, so
     *        an empty value means it cannot tell and has to assume the best.
     * @param entries All extracted file entries
     * @return true on success
     */
    static bool write(const std::string& outputPath,
                      const std::string& basePath,
                      const std::string& expansion,
                      const std::vector<FileEntry>& entries);

    /**
     * Compute CRC32 of file data
     */
    static uint32_t computeCRC32(const uint8_t* data, size_t size);
};

} // namespace tools
} // namespace wowee
