#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace wowee {
namespace game {

/**
 * Provides WoW.exe PE memory image for Warden MEM_CHECK responses.
 * Parses PE headers to build a flat virtual memory image, then serves
 * readMemory() calls with real bytes. Also mocks KUSER_SHARED_DATA.
 */
class WardenMemory {
public:
    WardenMemory();
    ~WardenMemory();

    /** Search standard candidate dirs for WoW.exe and load it.
     *  @param build Client build number (e.g. 5875 for Classic 1.12.1) to select the right exe.
     *  @param isTurtle If true, prefer the Turtle WoW custom exe (different code bytes). */
    bool load(uint16_t build = 0, bool isTurtle = false);

    /** Load PE image from a specific file path. */
    bool loadFromFile(const std::string& exePath);

    /**
     * Read bytes from virtual address space.
     * Handles PE sections + KUSER_SHARED_DATA mock.
     */
    bool readMemory(uint32_t va, uint8_t length, uint8_t* outBuf) const;

    [[nodiscard]] bool isLoaded() const { return loaded_; }

    /**
     * Search PE image for a byte pattern matching HMAC-SHA1(seed, pattern).
     * Used for FIND_MEM_IMAGE_CODE_BY_HASH and FIND_CODE_BY_HASH scans.
     * @param seed 4-byte HMAC key
     * @param expectedHash 20-byte expected HMAC-SHA1 digest
     * @param patternLen Length of the pattern to search for
     * @param imageOnly If true, search only executable sections (.text)
     * @param hintOffset RVA hint from PAGE_A request - check this position first
     * @return true if a matching pattern was found in the PE image
     */
    bool searchCodePattern(const uint8_t seed[4], const uint8_t expectedHash[20],
                           uint8_t patternLen, bool imageOnly,
                           uint32_t hintOffset = 0, bool hintOnly = false) const;

    /** Write a little-endian uint32 at the given virtual address in the PE image. */
    void writeLE32(uint32_t va, uint32_t value);

private:
    bool loaded_ = false;
    uint32_t imageBase_ = 0;
    uint32_t imageSize_ = 0;
    std::vector<uint8_t> image_;

    // KUSER_SHARED_DATA mock (0x7FFE0000 - 0x7FFE0FFF)
    static constexpr uint32_t KUSER_BASE = 0x7FFE0000;
    static constexpr uint32_t KUSER_SIZE = 0x1000;
    uint8_t kuserData_[KUSER_SIZE] = {};

    bool parsePE(const std::vector<uint8_t>& fileData);
    void initKuserSharedData();
    void patchRuntimeGlobals();
    void patchTurtleWowBinary();
    void verifyWardenScanEntries();
    bool isTurtle_ = false;
    [[nodiscard]] std::string findWowExe(uint16_t build) const;
    static uint32_t expectedImageSizeForBuild(uint16_t build, bool isTurtle);

    // Cache for searchCodePattern results to avoid repeated 5-second brute-force searches.
    // Key: hex string of seed(4)+hash(20)+patLen(1)+imageOnly(1) = 26 bytes.
    mutable std::unordered_map<std::string, bool> codePatternCache_;
};

} // namespace game
} // namespace wowee
