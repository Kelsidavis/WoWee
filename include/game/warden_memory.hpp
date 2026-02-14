#pragma once

#include <vector>
#include <cstdint>
#include <string>

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

    /** Search standard candidate dirs for WoW.exe and load it. */
    bool load();

    /** Load PE image from a specific file path. */
    bool loadFromFile(const std::string& exePath);

    /**
     * Read bytes from virtual address space.
     * Handles PE sections + KUSER_SHARED_DATA mock.
     */
    bool readMemory(uint32_t va, uint8_t length, uint8_t* outBuf) const;

    bool isLoaded() const { return loaded_; }

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
    std::string findWowExe() const;
};

} // namespace game
} // namespace wowee
