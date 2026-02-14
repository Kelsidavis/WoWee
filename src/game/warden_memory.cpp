#include "game/warden_memory.hpp"
#include "core/logger.hpp"
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace wowee {
namespace game {

static inline uint32_t readLE32(const std::vector<uint8_t>& data, size_t offset) {
    return data[offset] | (uint32_t(data[offset+1]) << 8)
         | (uint32_t(data[offset+2]) << 16) | (uint32_t(data[offset+3]) << 24);
}

static inline uint16_t readLE16(const std::vector<uint8_t>& data, size_t offset) {
    return data[offset] | (uint16_t(data[offset+1]) << 8);
}

WardenMemory::WardenMemory() = default;
WardenMemory::~WardenMemory() = default;

bool WardenMemory::parsePE(const std::vector<uint8_t>& fileData) {
    // DOS header: MZ magic
    if (fileData.size() < 64) return false;
    if (fileData[0] != 'M' || fileData[1] != 'Z') {
        LOG_ERROR("WardenMemory: Not a valid PE file (no MZ header)");
        return false;
    }

    // e_lfanew at offset 0x3C -> PE signature offset
    uint32_t peOffset = readLE32(fileData, 0x3C);
    if (peOffset + 4 > fileData.size()) return false;

    // PE signature "PE\0\0"
    if (fileData[peOffset] != 'P' || fileData[peOffset+1] != 'E'
        || fileData[peOffset+2] != 0 || fileData[peOffset+3] != 0) {
        LOG_ERROR("WardenMemory: Invalid PE signature");
        return false;
    }

    // COFF header at peOffset + 4
    size_t coffOfs = peOffset + 4;
    if (coffOfs + 20 > fileData.size()) return false;

    uint16_t numSections = readLE16(fileData, coffOfs + 2);
    uint16_t optHeaderSize = readLE16(fileData, coffOfs + 16);

    // Optional header
    size_t optOfs = coffOfs + 20;
    if (optOfs + optHeaderSize > fileData.size()) return false;

    uint16_t magic = readLE16(fileData, optOfs);
    if (magic != 0x10B) {
        LOG_ERROR("WardenMemory: Not PE32 (magic=0x", std::hex, magic, std::dec, ")");
        return false;
    }

    // PE32 fields
    imageBase_ = readLE32(fileData, optOfs + 28);
    imageSize_ = readLE32(fileData, optOfs + 56);
    uint32_t sizeOfHeaders = readLE32(fileData, optOfs + 60);

    LOG_INFO("WardenMemory: PE ImageBase=0x", std::hex, imageBase_,
             " ImageSize=0x", imageSize_,
             " Sections=", std::dec, numSections);

    // Allocate flat image (zero-filled)
    image_.resize(imageSize_, 0);

    // Copy headers
    uint32_t headerCopy = std::min({sizeOfHeaders, imageSize_, static_cast<uint32_t>(fileData.size())});
    std::memcpy(image_.data(), fileData.data(), headerCopy);

    // Section table follows optional header
    size_t secTableOfs = optOfs + optHeaderSize;

    for (uint16_t i = 0; i < numSections; i++) {
        size_t secOfs = secTableOfs + i * 40;
        if (secOfs + 40 > fileData.size()) break;

        char secName[9] = {};
        std::memcpy(secName, fileData.data() + secOfs, 8);

        uint32_t virtualSize   = readLE32(fileData, secOfs + 8);
        uint32_t virtualAddr   = readLE32(fileData, secOfs + 12);
        uint32_t rawDataSize   = readLE32(fileData, secOfs + 16);
        uint32_t rawDataOffset = readLE32(fileData, secOfs + 20);

        if (rawDataSize == 0 || rawDataOffset == 0) continue;

        // Clamp copy size to file and image bounds
        uint32_t copySize = std::min(rawDataSize, virtualSize);
        if (rawDataOffset + copySize > fileData.size())
            copySize = static_cast<uint32_t>(fileData.size()) - rawDataOffset;
        if (virtualAddr + copySize > imageSize_)
            copySize = imageSize_ - virtualAddr;

        std::memcpy(image_.data() + virtualAddr, fileData.data() + rawDataOffset, copySize);

        LOG_INFO("WardenMemory:   Section '", secName,
                 "' VA=0x", std::hex, imageBase_ + virtualAddr,
                 " size=0x", copySize, std::dec);
    }

    return true;
}

void WardenMemory::initKuserSharedData() {
    std::memset(kuserData_, 0, KUSER_SIZE);

    // NtMajorVersion at offset 0x026C = 6 (Vista/7/8/10)
    uint32_t ntMajor = 6;
    std::memcpy(kuserData_ + 0x026C, &ntMajor, 4);

    // NtMinorVersion at offset 0x0270 = 1 (Windows 7)
    uint32_t ntMinor = 1;
    std::memcpy(kuserData_ + 0x0270, &ntMinor, 4);
}

bool WardenMemory::readMemory(uint32_t va, uint8_t length, uint8_t* outBuf) const {
    if (length == 0) return true;

    // KUSER_SHARED_DATA range
    if (va >= KUSER_BASE && static_cast<uint64_t>(va) + length <= KUSER_BASE + KUSER_SIZE) {
        std::memcpy(outBuf, kuserData_ + (va - KUSER_BASE), length);
        return true;
    }

    // PE image range
    if (!loaded_ || va < imageBase_) return false;
    uint32_t offset = va - imageBase_;
    if (static_cast<uint64_t>(offset) + length > imageSize_) return false;

    std::memcpy(outBuf, image_.data() + offset, length);
    return true;
}

std::string WardenMemory::findWowExe() const {
    std::vector<std::string> candidateDirs;
    if (const char* env = std::getenv("WOWEE_INTEGRITY_DIR")) {
        if (env && *env) candidateDirs.push_back(env);
    }
    candidateDirs.push_back("Data/misc");
    if (const char* home = std::getenv("HOME")) {
        if (home && *home) {
            candidateDirs.push_back(std::string(home) + "/Downloads/twmoa_1180");
            candidateDirs.push_back(std::string(home) + "/twmoa_1180");
        }
    }

    const char* candidateExes[] = { "WoW.exe", "TurtleWoW.exe", "Wow.exe" };

    for (const auto& dir : candidateDirs) {
        for (const char* exe : candidateExes) {
            std::string path = dir;
            if (!path.empty() && path.back() != '/') path += '/';
            path += exe;
            if (std::filesystem::exists(path)) {
                return path;
            }
        }
    }
    return "";
}

bool WardenMemory::load() {
    std::string path = findWowExe();
    if (path.empty()) {
        LOG_WARNING("WardenMemory: WoW.exe not found in any candidate directory");
        return false;
    }
    LOG_INFO("WardenMemory: Found ", path);
    return loadFromFile(path);
}

bool WardenMemory::loadFromFile(const std::string& exePath) {
    std::ifstream f(exePath, std::ios::binary);
    if (!f.is_open()) {
        LOG_ERROR("WardenMemory: Cannot open ", exePath);
        return false;
    }

    f.seekg(0, std::ios::end);
    auto fileSize = f.tellg();
    f.seekg(0, std::ios::beg);

    std::vector<uint8_t> fileData(static_cast<size_t>(fileSize));
    f.read(reinterpret_cast<char*>(fileData.data()), fileSize);

    if (!parsePE(fileData)) {
        LOG_ERROR("WardenMemory: Failed to parse PE from ", exePath);
        return false;
    }

    initKuserSharedData();
    loaded_ = true;
    LOG_INFO("WardenMemory: Loaded PE image (", fileData.size(), " bytes on disk, ",
             imageSize_, " bytes virtual)");
    return true;
}

} // namespace game
} // namespace wowee
