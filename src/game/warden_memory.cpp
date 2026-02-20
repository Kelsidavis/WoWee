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

void WardenMemory::writeLE32(uint32_t va, uint32_t value) {
    if (va < imageBase_) return;
    uint32_t rva = va - imageBase_;
    if (rva + 4 > imageSize_) return;
    image_[rva]   = value & 0xFF;
    image_[rva+1] = (value >> 8) & 0xFF;
    image_[rva+2] = (value >> 16) & 0xFF;
    image_[rva+3] = (value >> 24) & 0xFF;
}

void WardenMemory::patchRuntimeGlobals() {
    // Only patch Classic 1.12.1 (build 5875) WoW.exe
    // Identified by: ImageBase=0x400000, ImageSize=0x906000 (unique to 1.12.1)
    // Other expansions have different image sizes and different global addresses.
    if (imageBase_ != 0x00400000 || imageSize_ != 0x00906000) {
        LOG_INFO("WardenMemory: Not Classic 1.12.1 WoW.exe (imageSize=0x",
                 std::hex, imageSize_, std::dec, "), skipping runtime global patches");
        return;
    }

    // Classic 1.12.1 (build 5875) runtime globals
    // These are in the .data BSS region - zero on disk, populated at runtime.
    // We patch them with fake but valid values so Warden checks pass.
    //
    // Offsets from CMaNGOS anticheat module (wardenwin.cpp):
    //   WardenModule      = 0xCE897C
    //   OfsWardenSysInfo  = 0x228
    //   OfsWardenWinSysInfo = 0x08
    //   g_theGxDevicePtr  = 0xC0ED38
    //   OfsDevice2        = 0x38A8
    //   OfsDevice3        = 0x0
    //   OfsDevice4        = 0xA8
    //   WorldEnables      = 0xC7B2A4
    //   LastHardwareAction= 0xCF0BC8

    // === Warden SYSTEM_INFO chain (3-level pointer chain) ===
    // Stage 0: [0xCE897C] → fake warden struct base
    constexpr uint32_t WARDEN_MODULE_PTR = 0xCE897C;
    constexpr uint32_t FAKE_WARDEN_BASE  = 0xCE8000;
    writeLE32(WARDEN_MODULE_PTR, FAKE_WARDEN_BASE);

    // Stage 1: [FAKE_WARDEN_BASE + 0x228] → pointer to sysinfo container
    constexpr uint32_t OFS_WARDEN_SYSINFO = 0x228;
    constexpr uint32_t FAKE_SYSINFO_CONTAINER = 0xCE8300;
    writeLE32(FAKE_WARDEN_BASE + OFS_WARDEN_SYSINFO, FAKE_SYSINFO_CONTAINER);

    // Stage 2: [FAKE_SYSINFO_CONTAINER + 0x08] → 36-byte SYSTEM_INFO struct
    constexpr uint32_t OFS_WARDEN_WIN_SYSINFO = 0x08;
    uint32_t sysInfoAddr = FAKE_SYSINFO_CONTAINER + OFS_WARDEN_WIN_SYSINFO; // 0xCE8308
    // WIN_SYSTEM_INFO is 36 bytes (0x24):
    //   uint16 wProcessorArchitecture (must be 0 = x86)
    //   uint16 wReserved
    //   uint32 dwPageSize
    //   uint32 lpMinimumApplicationAddress
    //   uint32 lpMaximumApplicationAddress  (MUST be non-zero!)
    //   uint32 dwActiveProcessorMask
    //   uint32 dwNumberOfProcessors
    //   uint32 dwProcessorType  (must be 386, 486, or 586)
    //   uint32 dwAllocationGranularity
    //   uint16 wProcessorLevel
    //   uint16 wProcessorRevision
    struct {
        uint16_t wProcessorArchitecture;
        uint16_t wReserved;
        uint32_t dwPageSize;
        uint32_t lpMinimumApplicationAddress;
        uint32_t lpMaximumApplicationAddress;
        uint32_t dwActiveProcessorMask;
        uint32_t dwNumberOfProcessors;
        uint32_t dwProcessorType;
        uint32_t dwAllocationGranularity;
        uint16_t wProcessorLevel;
        uint16_t wProcessorRevision;
    } __attribute__((packed)) sysInfo = {
        0,          // x86
        0,
        4096,       // 4K page size
        0x00010000, // min app address
        0x7FFEFFFF, // max app address (CRITICAL: must be non-zero)
        0x0F,       // 4 processors
        4,          // 4 CPUs
        586,        // Pentium
        65536,      // 64K granularity
        6,          // P6 family
        0x3A09      // revision
    };
    static_assert(sizeof(sysInfo) == 36, "SYSTEM_INFO must be 36 bytes");
    uint32_t rva = sysInfoAddr - imageBase_;
    if (rva + 36 <= imageSize_) {
        std::memcpy(image_.data() + rva, &sysInfo, 36);
    }

    LOG_INFO("WardenMemory: Patched SYSTEM_INFO chain: [0x", std::hex,
             WARDEN_MODULE_PTR, "]→0x", FAKE_WARDEN_BASE,
             " [0x", FAKE_WARDEN_BASE + OFS_WARDEN_SYSINFO, "]→0x", FAKE_SYSINFO_CONTAINER,
             " SYSTEM_INFO@0x", sysInfoAddr, std::dec);

    // === EndScene chain (4-level pointer chain) ===
    // Stage 1: [0xC0ED38] → fake D3D device
    constexpr uint32_t GX_DEVICE_PTR = 0xC0ED38;
    constexpr uint32_t FAKE_DEVICE   = 0xCE8400;
    writeLE32(GX_DEVICE_PTR, FAKE_DEVICE);

    // Stage 2: [FAKE_DEVICE + 0x38A8] → fake intermediate
    constexpr uint32_t OFS_DEVICE2 = 0x38A8;
    constexpr uint32_t FAKE_INTERMEDIATE = 0xCE8500;
    writeLE32(FAKE_DEVICE + OFS_DEVICE2, FAKE_INTERMEDIATE);

    // Stage 3: [FAKE_INTERMEDIATE + 0x0] → fake vtable
    constexpr uint32_t OFS_DEVICE3 = 0x0;
    constexpr uint32_t FAKE_VTABLE = 0xCE8600;
    writeLE32(FAKE_INTERMEDIATE + OFS_DEVICE3, FAKE_VTABLE);

    // Stage 4: [FAKE_VTABLE + 0xA8] → address of "EndScene" function
    // Point to a real .text address with normal code (not 0xE9/0xCC = not hooked)
    constexpr uint32_t OFS_DEVICE4 = 0xA8;
    constexpr uint32_t FAKE_ENDSCENE = 0x00401000; // Start of .text section
    writeLE32(FAKE_VTABLE + OFS_DEVICE4, FAKE_ENDSCENE);

    LOG_INFO("WardenMemory: Patched EndScene chain: [0x", std::hex,
             GX_DEVICE_PTR, "]→0x", FAKE_DEVICE,
             " ... →EndScene@0x", FAKE_ENDSCENE, std::dec);

    // === WorldEnables (single value) ===
    // Required flags: TerrainDoodads|Terrain|MapObjects|MapObjectLighting|MapObjectTextures|Water
    // Plus typical defaults (no Prohibited bits set)
    constexpr uint32_t WORLD_ENABLES = 0xC7B2A4;
    uint32_t enables = 0x1 | 0x2 | 0x10 | 0x20 | 0x40 | 0x100 | 0x200 | 0x400 | 0x800
                     | 0x8000 | 0x10000 | 0x100000 | 0x1000000 | 0x2000000
                     | 0x4000000 | 0x8000000 | 0x10000000;
    writeLE32(WORLD_ENABLES, enables);
    LOG_INFO("WardenMemory: Patched WorldEnables=0x", std::hex, enables, std::dec);

    // === LastHardwareAction (tick count) ===
    // Must be <= currentTime from timing check. Set to a plausible value.
    constexpr uint32_t LAST_HARDWARE_ACTION = 0xCF0BC8;
    writeLE32(LAST_HARDWARE_ACTION, 60000); // 1 minute
    LOG_INFO("WardenMemory: Patched LastHardwareAction=60000ms");
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

uint32_t WardenMemory::expectedImageSizeForBuild(uint16_t build) {
    switch (build) {
        case 5875: return 0x00906000; // Classic 1.12.1
        default:   return 0;          // Unknown — accept any
    }
}

std::string WardenMemory::findWowExe(uint16_t build) const {
    std::vector<std::string> candidateDirs;
    if (const char* env = std::getenv("WOWEE_INTEGRITY_DIR")) {
        if (env && *env) candidateDirs.push_back(env);
    }
    if (const char* home = std::getenv("HOME")) {
        if (home && *home) {
            candidateDirs.push_back(std::string(home) + "/Downloads");
            candidateDirs.push_back(std::string(home) + "/Downloads/twmoa_1180");
            candidateDirs.push_back(std::string(home) + "/twmoa_1180");
        }
    }
    candidateDirs.push_back("Data/misc");

    const char* candidateExes[] = { "WoW.exe", "TurtleWoW.exe", "Wow.exe" };

    // Collect all candidate paths
    std::vector<std::string> allPaths;
    for (const auto& dir : candidateDirs) {
        for (const char* exe : candidateExes) {
            std::string path = dir;
            if (!path.empty() && path.back() != '/') path += '/';
            path += exe;
            if (std::filesystem::exists(path)) {
                allPaths.push_back(path);
            }
        }
    }

    // If we know the expected imageSize for this build, try to find a matching PE
    uint32_t expectedSize = expectedImageSizeForBuild(build);
    if (expectedSize != 0 && allPaths.size() > 1) {
        for (const auto& path : allPaths) {
            std::ifstream f(path, std::ios::binary);
            if (!f.is_open()) continue;
            // Read PE headers to get imageSize
            f.seekg(0, std::ios::end);
            auto fileSize = f.tellg();
            if (fileSize < 256) continue;
            f.seekg(0x3C);
            uint32_t peOfs = 0;
            f.read(reinterpret_cast<char*>(&peOfs), 4);
            if (peOfs + 4 + 20 + 60 > static_cast<uint32_t>(fileSize)) continue;
            f.seekg(peOfs + 4 + 20 + 56); // OptionalHeader + 56 = SizeOfImage
            uint32_t imgSize = 0;
            f.read(reinterpret_cast<char*>(&imgSize), 4);
            if (imgSize == expectedSize) {
                LOG_INFO("WardenMemory: Matched build ", build, " to ", path,
                         " (imageSize=0x", std::hex, imgSize, std::dec, ")");
                return path;
            }
        }
    }

    // Fallback: return first available
    return allPaths.empty() ? "" : allPaths[0];
}

bool WardenMemory::load(uint16_t build) {
    std::string path = findWowExe(build);
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
    patchRuntimeGlobals();
    loaded_ = true;
    LOG_INFO("WardenMemory: Loaded PE image (", fileData.size(), " bytes on disk, ",
             imageSize_, " bytes virtual)");
    return true;
}

} // namespace game
} // namespace wowee
