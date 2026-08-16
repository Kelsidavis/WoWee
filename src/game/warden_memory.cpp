#include "game/warden_memory.hpp"
#include "core/logger.hpp"
#include <chrono>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <openssl/hmac.h>
#include <openssl/evp.h>

namespace wowee {
namespace game {

// Bounds-checked little-endian reads for PE parsing - malformed Warden modules
// must not cause out-of-bounds access.
static inline uint32_t readLE32(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 4 > data.size()) return 0;
    return data[offset] | (uint32_t(data[offset+1]) << 8)
         | (uint32_t(data[offset+2]) << 16) | (uint32_t(data[offset+3]) << 24);
}

static inline uint16_t readLE16(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 2 > data.size()) return 0;
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

        // Clamp copy size to file and image bounds.
        // Guard against underflow: if offset exceeds buffer size, skip the section
        // entirely rather than wrapping to a huge uint32_t in the subtraction.
        if (rawDataOffset >= fileData.size() || virtualAddr >= imageSize_) continue;
        uint32_t copySize = std::min(rawDataSize, virtualSize);
        uint32_t maxFromFile  = static_cast<uint32_t>(fileData.size()) - rawDataOffset;
        uint32_t maxFromImage = imageSize_ - virtualAddr;
        copySize = std::min({copySize, maxFromFile, maxFromImage});

        std::memcpy(image_.data() + virtualAddr, fileData.data() + rawDataOffset, copySize);

        LOG_INFO("WardenMemory:   Section '", secName,
                 "' VA=0x", std::hex, imageBase_ + virtualAddr,
                 " size=0x", copySize, std::dec);
    }

    LOG_WARNING("WardenMemory: PE loaded - imageBase=0x", std::hex, imageBase_,
                " imageSize=0x", imageSize_, std::dec,
                " (", numSections, " sections, ", fileData.size(), " bytes on disk)");

    return true;
}

void WardenMemory::initKuserSharedData() {
    std::memset(kuserData_, 0, KUSER_SIZE);

    // -------------------------------------------------------------------
    // KUSER_SHARED_DATA layout - Windows 7 SP1 x86 (from ntddk.h PDB)
    // Warden reads this in 238-byte chunks for OS fingerprinting.
    // All offsets verified against the canonical _KUSER_SHARED_DATA struct.
    // -------------------------------------------------------------------

    auto w32 = [&](uint32_t off, uint32_t v) { std::memcpy(kuserData_ + off, &v, 4); };
    auto w16 = [&](uint32_t off, uint16_t v) { std::memcpy(kuserData_ + off, &v, 2); };
    auto w8  = [&](uint32_t off, uint8_t  v) { kuserData_[off] = v; };

    // +0x000 TickCountLowDeprecated (ULONG)
    w32(0x0000, 0x003F4A00); // ~70 min uptime

    // +0x004 TickCountMultiplier (ULONG)
    w32(0x0004, 0x0FA00000);

    // +0x008 InterruptTime (KSYSTEM_TIME: Low4 + High1_4 + High2_4)
    w32(0x0008, 0x6B49D200);
    w32(0x000C, 0x00000029);
    w32(0x0010, 0x00000029);

    // +0x014 SystemTime (KSYSTEM_TIME) - ~2024 epoch FILETIME
    w32(0x0014, 0xA0B71B00);
    w32(0x0018, 0x01DA5E80);
    w32(0x001C, 0x01DA5E80);

    // +0x020 TimeZoneBias (KSYSTEM_TIME) - 0 = UTC
    // (leave zeros)

    // +0x02C ImageNumberLow / ImageNumberHigh (USHORT each)
    w16(0x002C, 0x014C); // IMAGE_FILE_MACHINE_I386
    w16(0x002E, 0x014C);

    // +0x030 NtSystemRoot (WCHAR[260] = 520 bytes, ends at +0x238)
    const wchar_t* sysRoot = L"C:\\WINDOWS";
    for (size_t i = 0; i < 10; i++) {
        w16(0x0030 + static_cast<uint32_t>(i) * 2, static_cast<uint16_t>(sysRoot[i]));
    }

    // +0x238 MaxStackTraceDepth (ULONG)
    w32(0x0238, 0);

    // +0x23C CryptoExponent (ULONG) - 65537
    w32(0x023C, 0x00010001);

    // +0x240 TimeZoneId (ULONG) - TIME_ZONE_ID_UNKNOWN
    w32(0x0240, 0);

    // +0x244 LargePageMinimum (ULONG) - 2 MB
    w32(0x0244, 0x00200000);

    // +0x248 Reserved2[7] (28 bytes) - zeros
    // (leave zeros)

    // +0x264 NtProductType (NT_PRODUCT_TYPE = ULONG) - VER_NT_WORKSTATION
    w32(0x0264, 1);

    // +0x268 ProductTypeIsValid (BOOLEAN = UCHAR)
    w8(0x0268, 1);

    // +0x269 Reserved9[3] - padding
    // (leave zeros)

    // +0x26C NtMajorVersion (ULONG) - 6 (Windows Vista/7/8/10)
    w32(0x026C, 6);

    // +0x270 NtMinorVersion (ULONG) - 1 (Windows 7)
    w32(0x0270, 1);

    // +0x274 ProcessorFeatures (BOOLEAN[64] = 64 bytes, ends at +0x2B4)
    //   Each entry is a single UCHAR (0 or 1).
    //   Index  Name                                 Value
    //   [0]    PF_FLOATING_POINT_PRECISION_ERRATA    0
    //   [1]    PF_FLOATING_POINT_EMULATED            0
    //   [2]    PF_COMPARE_EXCHANGE_DOUBLE            1
    //   [3]    PF_MMX_INSTRUCTIONS_AVAILABLE         1
    //   [4]    PF_PPC_MOVEMEM_64BIT_OK               0
    //   [5]    PF_ALPHA_BYTE_INSTRUCTIONS            0
    //   [6]    PF_XMMI_INSTRUCTIONS_AVAILABLE (SSE)  1
    //   [7]    PF_3DNOW_INSTRUCTIONS_AVAILABLE       0
    //   [8]    PF_RDTSC_INSTRUCTION_AVAILABLE        1
    //   [9]    PF_PAE_ENABLED                        1
    //   [10]   PF_XMMI64_INSTRUCTIONS_AVAILABLE(SSE2)1
    //   [11]   PF_SSE_DAZ_MODE_AVAILABLE             0
    //   [12]   PF_NX_ENABLED                         1
    //   [13]   PF_SSE3_INSTRUCTIONS_AVAILABLE        1
    //   [14]   PF_COMPARE_EXCHANGE128                0  (x86 typically 0)
    //   [15]   PF_COMPARE64_EXCHANGE128              0
    //   [16]   PF_CHANNELS_ENABLED                   0
    //   [17]   PF_XSAVE_ENABLED                      0
    w8(0x0274 +  2, 1); // PF_COMPARE_EXCHANGE_DOUBLE
    w8(0x0274 +  3, 1); // PF_MMX
    w8(0x0274 +  6, 1); // PF_SSE
    w8(0x0274 +  8, 1); // PF_RDTSC
    w8(0x0274 +  9, 1); // PF_PAE_ENABLED
    w8(0x0274 + 10, 1); // PF_SSE2
    w8(0x0274 + 12, 1); // PF_NX_ENABLED
    w8(0x0274 + 13, 1); // PF_SSE3

    // +0x2B4 Reserved1 (ULONG)
    // +0x2B8 Reserved3 (ULONG)
    // +0x2BC TimeSlip (ULONG)
    // +0x2C0 AlternativeArchitecture (ULONG) = 0 (StandardDesign)
    // +0x2C4 AltArchitecturePad[1] (ULONG)
    // +0x2C8 SystemExpirationDate (LARGE_INTEGER = 8 bytes)
    // (leave zeros)

    // +0x2D0 SuiteMask (ULONG) - VER_SUITE_SINGLEUSERTS | VER_SUITE_TERMINAL
    w32(0x02D0, 0x0110); // 0x0100=SINGLEUSERTS, 0x0010=TERMINAL

    // +0x2D4 KdDebuggerEnabled (BOOLEAN = UCHAR)
    w8(0x02D4, 0);

    // +0x2D5 NXSupportPolicy (UCHAR) - 2 = OptIn
    w8(0x02D5, 2);

    // +0x2D6 Reserved6[2]
    // (leave zeros)

    // +0x2D8 ActiveConsoleId (ULONG) - session 0 or 1
    w32(0x02D8, 1);

    // +0x2DC DismountCount (ULONG)
    w32(0x02DC, 0);

    // +0x2E0 ComPlusPackage (ULONG)
    w32(0x02E0, 0);

    // +0x2E4 LastSystemRITEventTickCount (ULONG) - recent input tick
    w32(0x02E4, 0x003F4900);

    // +0x2E8 NumberOfPhysicalPages (ULONG) - 4GB / 4KB ≈ 1M pages
    w32(0x02E8, 0x000FF000);

    // +0x2EC SafeBootMode (BOOLEAN) - 0 = normal boot
    w8(0x02EC, 0);

    // +0x2F0 SharedDataFlags / TraceLogging (ULONG)
    w32(0x02F0, 0);

    // +0x2F8 TestRetInstruction (ULONGLONG = 8 bytes) - RET opcode
    w8(0x02F8, 0xC3); // x86 RET instruction

    // +0x300 SystemCall (ULONG)
    w32(0x0300, 0);

    // +0x304 SystemCallReturn (ULONG)
    w32(0x0304, 0);

    // +0x308 SystemCallPad[3] (24 bytes)
    // (leave zeros)

    // +0x320 TickCount (KSYSTEM_TIME) - matches TickCountLowDeprecated
    w32(0x0320, 0x003F4A00);

    // +0x32C TickCountPad[1]
    // (leave zeros)

    // +0x330 Cookie (ULONG) - stack cookie, random-looking value
    w32(0x0330, 0x4A2F8C15);

    // +0x334 ConsoleSessionForegroundProcessId (ULONG) - some PID
    w32(0x0334, 0x00001234);

    // Everything after +0x338 is typically zero on Win7 x86
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
    if (imageBase_ != 0x00400000) {
        LOG_WARNING("WardenMemory: unexpected imageBase=0x", std::hex, imageBase_, std::dec,
                    " - skipping runtime global patches");
        return;
    }

    // Classic 1.12.1 (build 5875) runtime globals
    // VMaNGOS has TWO types of Warden scans that read these addresses:
    //
    // 1. DB-driven scans (warden_scans table): memcmp against expected bytes.
    //    These check CODE sections for integrity - never check runtime data addresses.
    //
    // 2. Scripted scans (WardenWin::LoadScriptedScans): READ and INTERPRET values.
    //    - "Warden locate" reads 0xCE897C as a pointer, follows chain to SYSTEM_INFO
    //    - "Anti-AFK hack" reads 0xCF0BC8 as a timestamp, compares vs TIMING ticks
    //    - "CWorld::enables" reads 0xC7B2A4, checks flag bits
    //    - "EndScene" reads 0xC0ED38, follows pointer chain to find EndScene address
    //
    // We MUST patch these for ALL clients (including Turtle WoW) because the scripted
    // scans interpret the values as runtime state, not static code bytes. Returning
    // raw PE data causes the Anti-AFK scan to see lastHardwareAction > currentTime
    // (PE bytes happen to be a large value), triggering a kick after ~3.5 minutes.

    // === Runtime global patches (applied unconditionally for all image variants) ===

    // Warden SYSTEM_INFO chain
    constexpr uint32_t WARDEN_MODULE_PTR = 0xCE897C;
    constexpr uint32_t FAKE_WARDEN_BASE  = 0xCE8000;
    writeLE32(WARDEN_MODULE_PTR, FAKE_WARDEN_BASE);
    constexpr uint32_t FAKE_SYSINFO_CONTAINER = 0xCE8300;
    writeLE32(FAKE_WARDEN_BASE + 0x228, FAKE_SYSINFO_CONTAINER);

    // Write SYSINFO pointer at many offsets from FAKE_WARDEN_BASE so the
    // chain works regardless of which module-specific offset the server uses.
    // MUST be done BEFORE writing the actual SYSTEM_INFO struct, because this
    // loop's range (0xCE8200-0xCE8400) overlaps with the struct at 0xCE8308.
    for (uint32_t off = 0x200; off <= 0x400; off += 4) {
        uint32_t addr = FAKE_WARDEN_BASE + off;
        if (addr >= imageBase_ && (addr - imageBase_) + 4 <= imageSize_) {
            writeLE32(addr, FAKE_SYSINFO_CONTAINER);
        }
    }

    // Now write the actual WIN_SYSTEM_INFO struct AFTER the pointer fill loop,
    // so it overwrites any values the loop placed in the 0xCE8308+ range.
    uint32_t sysInfoAddr = FAKE_SYSINFO_CONTAINER + 0x08;
#pragma pack(push, 1)
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
    } sysInfo = {.wProcessorArchitecture = 0, .wReserved = 0, .dwPageSize = 4096, .lpMinimumApplicationAddress = 0x00010000, .lpMaximumApplicationAddress = 0x7FFEFFFF, .dwActiveProcessorMask = 0x0F, .dwNumberOfProcessors = 4, .dwProcessorType = 586, .dwAllocationGranularity = 65536, .wProcessorLevel = 6, .wProcessorRevision = 0x3A09};
#pragma pack(pop)
    static_assert(sizeof(sysInfo) == 36, "SYSTEM_INFO must be 36 bytes");
    uint32_t rva = sysInfoAddr - imageBase_;
    if (rva + 36 <= imageSize_) {
        std::memcpy(image_.data() + rva, &sysInfo, 36);
    }

    // Fallback: if the pointer chain breaks and stage 3 reads from address
    // 0x00000000 + 0x08 = 8, write valid SYSINFO at RVA 8 (PE DOS header area).
    if (8 + 36 <= imageSize_) {
        std::memcpy(image_.data() + 8, &sysInfo, 36);
    }

    LOG_WARNING("WardenMemory: Patched SYSINFO chain @0x", std::hex, WARDEN_MODULE_PTR, std::dec);

    // EndScene chain
    // VMaNGOS reads g_theGxDevicePtr → device, then device+0x1FC for API kind
    // (0=OpenGL, 1=Direct3D). If Direct3D, follows device+0x38A8 → ptr → ptr+0xA8 → EndScene.
    // We set API=1 (Direct3D) and provide the full pointer chain.
    constexpr uint32_t GX_DEVICE_PTR = 0xC0ED38;
    constexpr uint32_t FAKE_DEVICE   = 0xCE8400;
    writeLE32(GX_DEVICE_PTR, FAKE_DEVICE);
    writeLE32(FAKE_DEVICE + 0x1FC, 1);                 // API kind = Direct3D
    // Set up the full EndScene pointer chain at the canonical offsets.
    constexpr uint32_t FAKE_VTABLE1 = 0xCE8500;
    constexpr uint32_t FAKE_VTABLE2 = 0xCE8600;
    constexpr uint32_t FAKE_ENDSCENE = 0x00401000; // start of .text
    writeLE32(FAKE_DEVICE + 0x38A8, FAKE_VTABLE1);
    writeLE32(FAKE_VTABLE1, FAKE_VTABLE2);
    writeLE32(FAKE_VTABLE2 + 0xA8, FAKE_ENDSCENE);

    // The EndScene device+sOfsDevice2 offset may differ from 0x38A8 in Turtle WoW.
    // Also set API=1 (Direct3D) at multiple offsets so the API kind check passes.
    // Fill the entire fake device area with the vtable pointer for robustness.
    for (uint32_t off = 0x3800; off <= 0x3A00; off += 4) {
        uint32_t addr = FAKE_DEVICE + off;
        if (addr >= imageBase_ && (addr - imageBase_) + 4 <= imageSize_) {
            writeLE32(addr, FAKE_VTABLE1);
        }
    }
    LOG_WARNING("WardenMemory: Patched EndScene chain @0x", std::hex, GX_DEVICE_PTR, std::dec);

    // WorldEnables
    constexpr uint32_t WORLD_ENABLES = 0xC7B2A4;
    uint32_t enables = 0x1 | 0x2 | 0x10 | 0x20 | 0x40 | 0x100 | 0x200 | 0x400 | 0x800
                     | 0x8000 | 0x10000 | 0x100000 | 0x1000000 | 0x2000000
                     | 0x4000000 | 0x8000000 | 0x10000000;
    writeLE32(WORLD_ENABLES, enables);
    LOG_WARNING("WardenMemory: Patched WorldEnables @0x", std::hex, WORLD_ENABLES, std::dec);

    // LastHardwareAction - must be a recent GetTickCount()-style timestamp
    // so the anti-AFK scan sees (currentTime - lastAction) < threshold.
    constexpr uint32_t LAST_HARDWARE_ACTION = 0xCF0BC8;
    uint32_t nowMs = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    writeLE32(LAST_HARDWARE_ACTION, nowMs - 2000);
    LOG_WARNING("WardenMemory: Patched LastHardwareAction @0x", std::hex, LAST_HARDWARE_ACTION, std::dec);

    // Embed the 37-byte Warden module memcpy pattern in BSS so that
    // FIND_CODE_BY_HASH (PAGE_B) brute-force search can find it.
    // This is the pattern VMaNGOS's "Warden Memory Read check" looks for.
    constexpr uint32_t MEMCPY_PATTERN_VA = 0xCE8700;
    static constexpr uint8_t kWardenMemcpyPattern[37] = {
        0x56, 0x57, 0xFC, 0x8B, 0x54, 0x24, 0x14, 0x8B,
        0x74, 0x24, 0x10, 0x8B, 0x44, 0x24, 0x0C, 0x8B,
        0xCA, 0x8B, 0xF8, 0xC1, 0xE9, 0x02, 0x74, 0x02,
        0xF3, 0xA5, 0xB1, 0x03, 0x23, 0xCA, 0x74, 0x02,
        0xF3, 0xA4, 0x5F, 0x5E, 0xC3
    };
    uint32_t patRva = MEMCPY_PATTERN_VA - imageBase_;
    if (patRva + sizeof(kWardenMemcpyPattern) <= imageSize_) {
        std::memcpy(image_.data() + patRva, kWardenMemcpyPattern, sizeof(kWardenMemcpyPattern));
        LOG_WARNING("WardenMemory: Embedded Warden memcpy pattern at 0x", std::hex, MEMCPY_PATTERN_VA, std::dec);
    }
}

void WardenMemory::patchTurtleWowBinary() {
    // Apply TurtlePatcher byte patches to make our PE image match a real Turtle WoW client.
    // These patches are applied at file offsets which equal RVAs for this PE.
    // Source: TurtlePatcher/Main.cpp PatchBinary() + PatchVersion()

    auto patchBytes = [&](uint32_t fileOffset, const std::vector<uint8_t>& bytes) {
        if (fileOffset + bytes.size() > imageSize_) {
            LOG_WARNING("WardenMemory: Turtle patch at 0x", std::hex, fileOffset,
                        " exceeds image size, skipping");
            return;
        }
        std::memcpy(image_.data() + fileOffset, bytes.data(), bytes.size());
    };

    auto patchString = [&](uint32_t fileOffset, const char* str) {
        size_t len = std::strlen(str) + 1; // include null terminator
        if (fileOffset + len > imageSize_) return;
        std::memcpy(image_.data() + fileOffset, str, len);
    };

    // --- PatchBinary() patches ---

    // Patches 1-4: Unknown purpose code patches in .text
    patchBytes(0x2F113A, {0xEB, 0x19});
    patchBytes(0x2F1158, {0x03});
    patchBytes(0x2F11A7, {0x03});
    patchBytes(0x2F11F0, {0xEB, 0xB2});

    // PvP rank check removal (6x NOP)
    patchBytes(0x2093B0, {0x90, 0x90, 0x90, 0x90, 0x90, 0x90});

    // Dwarf mage hackfix removal
    patchBytes(0x0706E5, {0xFE});
    patchBytes(0x0706EB, {0xFE});
    patchBytes(0x07075D, {0xFE});
    patchBytes(0x070763, {0xFE});

    // Emote sound race ID checks (High Elf support)
    patchBytes(0x059289, {0x40});
    patchBytes(0x057C81, {0x40});

    // Nameplate distance (41 yards)
    patchBytes(0x40C448, {0x00, 0x00, 0x24, 0x42});

    // Large address aware flag in PE header
    patchBytes(0x000126, {0x2F, 0x01});

    // Sound channel patches
    patchBytes(0x05728C, {0x38, 0x5D, 0x83, 0x00}); // software channels
    patchBytes(0x057250, {0x38, 0x5D, 0x83, 0x00}); // hardware channels
    patchBytes(0x0572C8, {0x6C, 0x5C, 0x83, 0x00}); // memory cache

    // Sound in background (non-FoV build)
    patchBytes(0x3A4869, {0x14});

    // Hardcore chat patches
    patchBytes(0x09B0B8, {0x5F});
    patchBytes(0x09B193, {0xE9, 0xA8, 0xAE, 0x86});
    patchBytes(0x09F7A5, {0x70, 0x53, 0x56, 0x33, 0xF6, 0xE9, 0x71, 0x68, 0x86, 0x00});
    patchBytes(0x09F864, {0x94});
    patchBytes(0x09F878, {0x0E});
    patchBytes(0x09F887, {0x90});
    patchBytes(0x11BAE1, {0x0C, 0x60, 0xD0});

    // Hardcore chat code cave at 0x48E000 (85 bytes)
    patchBytes(0x48E000, {
        0x48, 0x41, 0x52, 0x44, 0x43, 0x4F, 0x52, 0x45, 0x00, 0x00, 0x00, 0x00, 0x43, 0x48, 0x41, 0x54,
        0x5F, 0x4D, 0x53, 0x47, 0x5F, 0x48, 0x41, 0x52, 0x44, 0x43, 0x4F, 0x52, 0x45, 0x00, 0x00, 0x00,
        0x57, 0x8B, 0xDA, 0x8B, 0xF9, 0xC7, 0x45, 0x94, 0x00, 0x60, 0xD0, 0x00, 0xC7, 0x45, 0x90, 0x5E,
        0x00, 0x00, 0x00, 0xE9, 0x77, 0x97, 0x79, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x68, 0x08, 0x46, 0x84, 0x00, 0x83, 0x7D, 0xF0, 0x5E, 0x75, 0x05, 0xB9, 0x1F, 0x02, 0x00, 0x00,
        0xE9, 0x43, 0x51, 0x79, 0xFF
    });

    // Blue child moon patch
    patchBytes(0x3E5B83, {
        0xC7, 0x05, 0xA4, 0x98, 0xCE, 0x00, 0xD4, 0xE2, 0xE7, 0xFF, 0xC2, 0x04, 0x00
    });

    // Blue child moon timer
    patchBytes(0x2D2095, {0x00, 0x00, 0x80, 0x3F});

    // SetUnit codecave jump
    patchBytes(0x105E19, {0xE9, 0x02, 0x03, 0x80, 0x00});

    // SetUnit main code cave at 0x48E060 (291 bytes)
    patchBytes(0x48E060, {
        0x55, 0x89, 0xE5, 0x83, 0xEC, 0x10, 0x85, 0xD2, 0x53, 0x56, 0x57, 0x89, 0xCF, 0x0F, 0x84, 0xA2,
        0x00, 0x00, 0x00, 0x89, 0xD0, 0x85, 0xC0, 0x0F, 0x8C, 0x98, 0x00, 0x00, 0x00, 0x3B, 0x05, 0x94,
        0xDE, 0xC0, 0x00, 0x0F, 0x8F, 0x8C, 0x00, 0x00, 0x00, 0x8B, 0x0D, 0x90, 0xDE, 0xC0, 0x00, 0x8B,
        0x04, 0x81, 0x85, 0xC0, 0x89, 0x45, 0xF0, 0x74, 0x7C, 0x8B, 0x40, 0x04, 0x85, 0xC0, 0x7C, 0x75,
        0x3B, 0x05, 0x6C, 0xDE, 0xC0, 0x00, 0x7F, 0x6D, 0x8B, 0x15, 0x68, 0xDE, 0xC0, 0x00, 0x8B, 0x1C,
        0x82, 0x85, 0xDB, 0x74, 0x60, 0x8B, 0x43, 0x08, 0x6A, 0x00, 0x50, 0x89, 0xF9, 0xE8, 0xFE, 0x6E,
        0xA6, 0xFF, 0x89, 0xC1, 0xE8, 0x87, 0x12, 0xA0, 0xFF, 0x89, 0xC6, 0x85, 0xF6, 0x74, 0x46, 0x8B,
        0x55, 0xF0, 0x53, 0x89, 0xF1, 0xE8, 0xD6, 0x36, 0x77, 0xFF, 0x8B, 0x17, 0x56, 0x89, 0xF9, 0xFF,
        0x92, 0x90, 0x00, 0x00, 0x00, 0x89, 0xF8, 0x99, 0x52, 0x50, 0x68, 0xA0, 0x62, 0x50, 0x00, 0x89,
        0xF1, 0xE8, 0xBA, 0xBA, 0xA0, 0xFF, 0x6A, 0x01, 0x6A, 0x01, 0x68, 0x00, 0x00, 0x80, 0x3F, 0x6A,
        0x00, 0x6A, 0xFF, 0x6A, 0x00, 0x6A, 0xFF, 0x89, 0xF1, 0xE8, 0x92, 0xC0, 0xA0, 0xFF, 0x89, 0xF1,
        0xE8, 0x8B, 0xA2, 0xA0, 0xFF, 0x5F, 0x5E, 0x5B, 0x89, 0xEC, 0x5D, 0xC3, 0x90, 0x90, 0x90, 0x90,
        0xBA, 0x02, 0x00, 0x00, 0x00, 0x89, 0xF1, 0xE8, 0xD4, 0xD2, 0x9E, 0xFF, 0x83, 0xF8, 0x03, 0x75,
        0x43, 0xBA, 0x02, 0x00, 0x00, 0x00, 0x89, 0xF1, 0xE8, 0xE3, 0xD4, 0x9E, 0xFF, 0xE8, 0x6E, 0x41,
        0x70, 0xFF, 0x56, 0x8B, 0xB7, 0xD4, 0x00, 0x00, 0x00, 0x31, 0xD2, 0x39, 0xD6, 0x89, 0x97, 0xE0,
        0x03, 0x00, 0x00, 0x89, 0x97, 0xE4, 0x03, 0x00, 0x00, 0x89, 0x97, 0xF0, 0x03, 0x00, 0x00, 0x5E,
        0x0F, 0x84, 0xD3, 0xFC, 0x7F, 0xFF, 0x89, 0xC2, 0x89, 0xF9, 0xE8, 0xF1, 0xFE, 0xFF, 0xFF, 0xE9,
        0xC5, 0xFC, 0x7F, 0xFF, 0xBA, 0x02, 0x00, 0x00, 0x00, 0xE9, 0xA0, 0xFC, 0x7F, 0xFF, 0x90, 0x90,
        0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90
    });

    // --- PatchVersion() patches ---

    // Net version: build 7199 (0x1C1F LE)
    patchBytes(0x1B2122, {0x1F, 0x1C});

    // Visual version string
    patchString(0x437C04, "1.17.2");

    // Visual build string
    patchString(0x437BFC, "7199");

    // Build date string
    patchString(0x434798, "May 20 2024");

    // Website filters
    patchString(0x45CCD8, "*.turtle-wow.org");
    patchString(0x45CC9C, "*.discord.gg");

    LOG_WARNING("WardenMemory: Applied TurtlePatcher binary patches (build 7199)");
}

bool WardenMemory::readMemory(uint32_t va, uint8_t length, uint8_t* outBuf) const {
    if (length == 0) return true;

    // KUSER_SHARED_DATA range
    if (va >= KUSER_BASE && static_cast<uint64_t>(va) + length <= KUSER_BASE + KUSER_SIZE) {
        std::memcpy(outBuf, kuserData_ + (va - KUSER_BASE), length);
        return true;
    }

    if (!loaded_) return false;

    // Warden MEM_CHECK offsets are seen in multiple forms:
    // 1) Absolute VA (e.g. 0x00401337)
    // 2) RVA (e.g. 0x000139A9)
    // 3) Tiny module-relative offsets (e.g. 0x00000229, 0x00000008)
    // Accept all three to avoid fallback-to-zeros on Classic/Turtle.
    uint32_t offset = 0;
    if (va >= imageBase_) {
        // Absolute VA.
        offset = va - imageBase_;
    } else if (va < imageSize_) {
        // RVA into WoW.exe image.
        offset = va;
    } else {
        // Tiny relative offsets frequently target fake Warden runtime globals.
        constexpr uint32_t kFakeWardenBase = 0xCE8000;
        const uint32_t remappedVa = kFakeWardenBase + va;
        if (remappedVa < imageBase_) return false;
        offset = remappedVa - imageBase_;
    }

    if (static_cast<uint64_t>(offset) + length > imageSize_) return false;

    std::memcpy(outBuf, image_.data() + offset, length);
    return true;
}

uint32_t WardenMemory::expectedImageSizeForBuild(uint16_t build, bool isTurtle) {
    switch (build) {
        case 5875:
            // Turtle WoW uses a custom WoW.exe with different code bytes.
            // Their warden_scans DB expects bytes from this custom exe.
            return isTurtle ? 0x00906000 : 0x009FD000;
        default:   return 0;          // Unknown - accept any
    }
}

std::string WardenMemory::findWowExe(uint16_t build) const {
    std::vector<std::string> candidateDirs;
    if (const char* env = std::getenv("WOWEE_INTEGRITY_DIR")) {
        if (env && *env) candidateDirs.emplace_back(env);
    }
    if (const char* home = std::getenv("HOME")) {
        if (home && *home) {
            candidateDirs.push_back(std::string(home) + "/Downloads");
            candidateDirs.push_back(std::string(home) + "/Downloads/twmoa_1180");
            candidateDirs.push_back(std::string(home) + "/twmoa_1180");
        }
    }
    candidateDirs.emplace_back("Data/expansions/turtle/misc");
    candidateDirs.emplace_back("Data/expansions/classic/misc");
    candidateDirs.emplace_back("Data/misc");
    candidateDirs.emplace_back("Data/expansions/turtle/overlay/misc");

    const char* candidateExes[] = { "WoW.exe", "TurtleWoW.exe", "Wow.exe", "wow.exe" };

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
    uint32_t expectedSize = expectedImageSizeForBuild(build, isTurtle_);
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

    // Fallback: prefer the largest PE file (modified clients like Turtle WoW are
    // larger than vanilla, and Warden checks target the actual running client).
    std::string bestPath;
    uintmax_t bestSize = 0;
    for (const auto& path : allPaths) {
        std::error_code ec;
        auto sz = std::filesystem::file_size(path, ec);
        if (!ec && sz > bestSize) {
            bestSize = sz;
            bestPath = path;
        }
    }
    return bestPath.empty() && !allPaths.empty() ? allPaths[0] : bestPath;
}

bool WardenMemory::load(uint16_t build, bool isTurtle) {
    isTurtle_ = isTurtle;
    std::string path = findWowExe(build);
    if (path.empty()) {
        LOG_WARNING("WardenMemory: WoW.exe not found in any candidate directory");
        return false;
    }
    LOG_WARNING("WardenMemory: Loading PE image: ", path, " (build=", build, ")");
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
    if (isTurtle_ && imageSize_ != 0x00906000) {
        // Only apply TurtlePatcher patches if we loaded the vanilla exe.
        // The real Turtle WoW.exe (imageSize=0x906000) already has these bytes.
        patchTurtleWowBinary();
        LOG_WARNING("WardenMemory: Applied Turtle patches to vanilla PE (imageSize=0x", std::hex, imageSize_, std::dec, ")");
    } else if (isTurtle_) {
        LOG_WARNING("WardenMemory: Loaded native Turtle PE - skipping patches");
    }
    loaded_ = true;
    LOG_INFO("WardenMemory: Loaded PE image (", fileData.size(), " bytes on disk, ",
             imageSize_, " bytes virtual)");

    // Verify all known warden_scans MEM_CHECK entries against our PE image.
    // This checks the exact bytes the server will memcmp against.
    verifyWardenScanEntries();

    return true;
}

void WardenMemory::verifyWardenScanEntries() {
    struct ScanEntry { int id; uint32_t address; uint8_t length; const char* expectedHex; const char* comment; };
    static const ScanEntry entries[] = {
        { .id = 1, .address = 8679268, .length = 6, .expectedHex = "686561646572", .comment = "Packet internal sign - header"},
        { .id = 3, .address = 8530960, .length = 6, .expectedHex = "53595354454D", .comment = "Packet internal sign - SYSTEM"},
        { .id = 8, .address = 8151666, .length = 4, .expectedHex = "D893FEC0", .comment = "Jump gravity"},
        { .id = 9, .address = 8151646, .length = 2, .expectedHex = "3075", .comment = "Jump gravity water"},
        {.id = 10, .address = 6382555, .length = 2, .expectedHex = "8A47", .comment = "Anti root"},
        {.id = 11, .address = 6380789, .length = 1, .expectedHex = "F8", .comment = "Anti move"},
        {.id = 12, .address = 8151647, .length = 1, .expectedHex = "75", .comment = "Anti jump"},
        {.id = 13, .address = 8152026, .length = 4, .expectedHex = "8B4F7889", .comment = "No fall damage"},
        {.id = 14, .address = 6504892, .length = 2, .expectedHex = "7425", .comment = "Super fly"},
        {.id = 15, .address = 6383433, .length = 2, .expectedHex = "780F", .comment = "Heartbeat interval speedhack"},
        {.id = 16, .address = 6284623, .length = 1, .expectedHex = "F4", .comment = "Anti slow hack"},
        {.id = 17, .address = 6504931, .length = 2, .expectedHex = "85D2", .comment = "No fall damage"},
        {.id = 18, .address = 8151565, .length = 2, .expectedHex = "2000", .comment = "Fly hack"},
        {.id = 19, .address = 7153475, .length = 6, .expectedHex = "890D509CCE00", .comment = "General hacks"},
        {.id = 20, .address = 7138894, .length = 6, .expectedHex = "A3D89BCE00EB", .comment = "Wall climb"},
        {.id = 21, .address = 7138907, .length = 6, .expectedHex = "890DD89BCE00", .comment = "Wall climb"},
        {.id = 22, .address = 6993044, .length = 1, .expectedHex = "74", .comment = "Zero gravity"},
        {.id = 23, .address = 6502300, .length = 1, .expectedHex = "FC", .comment = "Air walk"},
        {.id = 24, .address = 6340512, .length = 2, .expectedHex = "7F7D", .comment = "Wall climb"},
        {.id = 25, .address = 6380455, .length = 4, .expectedHex = "F4010000", .comment = "Wall climb"},
        {.id = 26, .address = 8151657, .length = 4, .expectedHex = "488C11C1", .comment = "Wall climb"},
        {.id = 27, .address = 6992319, .length = 3, .expectedHex = "894704", .comment = "Wall climb"},
        {.id = 28, .address = 6340529, .length = 2, .expectedHex = "746C", .comment = "No water hack"},
        {.id = 29, .address = 6356016, .length = 10, .expectedHex = "C70588D8C4000C000000", .comment = "No water hack"},
        {.id = 30, .address = 4730584, .length = 6, .expectedHex = "0F8CE1000000", .comment = "WMO collision"},
        {.id = 31, .address = 4803152, .length = 7, .expectedHex = "A1C0EACE0085C0", .comment = "noclip hack"},
        {.id = 32, .address = 5946704, .length = 6, .expectedHex = "8BD18B0D80E0", .comment = "M2 collision"},
        {.id = 33, .address = 6340543, .length = 2, .expectedHex = "7546", .comment = "M2 collision"},
        {.id = 34, .address = 5341282, .length = 1, .expectedHex = "7F", .comment = "Warden disable"},
        {.id = 35, .address = 4989376, .length = 1, .expectedHex = "72", .comment = "No fog hack"},
        {.id = 36, .address = 8145237, .length = 1, .expectedHex = "8B", .comment = "No fog hack"},
        {.id = 37, .address = 6392083, .length = 8, .expectedHex = "8B450850E824DA1A", .comment = "No fog hack"},
        {.id = 38, .address = 8146241, .length = 10, .expectedHex = "D9818C0000008BE55DC2", .comment = "tp2plane hack"},
        {.id = 39, .address = 6995731, .length = 1, .expectedHex = "74", .comment = "Air swim hack"},
        {.id = 40, .address = 6964859, .length = 1, .expectedHex = "75", .comment = "Infinite jump hack"},
        {.id = 41, .address = 6382558, .length = 10, .expectedHex = "84C074178B86A4000000", .comment = "Gravity water hack"},
        {.id = 42, .address = 8151997, .length = 3, .expectedHex = "895108", .comment = "Gravity hack"},
        {.id = 43, .address = 8152025, .length = 1, .expectedHex = "34", .comment = "Plane teleport"},
        {.id = 44, .address = 6516436, .length = 1, .expectedHex = "FC", .comment = "Zero fall time"},
        {.id = 45, .address = 6501616, .length = 1, .expectedHex = "FC", .comment = "No fall damage"},
        {.id = 46, .address = 6511674, .length = 1, .expectedHex = "FC", .comment = "Fall time hack"},
        {.id = 47, .address = 6513048, .length = 1, .expectedHex = "FC", .comment = "Death bug hack"},
        {.id = 48, .address = 6514072, .length = 1, .expectedHex = "FC", .comment = "Anti slow hack"},
        {.id = 49, .address = 8152029, .length = 3, .expectedHex = "894E38", .comment = "Anti slow hack"},
        {.id = 50, .address = 4847346, .length = 3, .expectedHex = "8B45D4", .comment = "Max camera distance hack"},
        {.id = 51, .address = 4847069, .length = 1, .expectedHex = "74", .comment = "Wall climb"},
        {.id = 52, .address = 8155231, .length = 3, .expectedHex = "000000", .comment = "Signature check"},
        {.id = 53, .address = 6356849, .length = 1, .expectedHex = "74", .comment = "Signature check"},
        {.id = 54, .address = 6354889, .length = 6, .expectedHex = "0F8A71FFFFFF", .comment = "Signature check"},
        {.id = 55, .address = 4657642, .length = 1, .expectedHex = "74", .comment = "Max interact distance hack"},
        {.id = 56, .address = 6211360, .length = 8, .expectedHex = "558BEC83EC0C8B45", .comment = "Hover speed hack"},
        {.id = 57, .address = 8153504, .length = 3, .expectedHex = "558BEC", .comment = "Flight speed hack"},
        {.id = 58, .address = 6214285, .length = 6, .expectedHex = "8B82500E0000", .comment = "Track all units hack"},
        {.id = 59, .address = 8151558, .length = 11, .expectedHex = "25FFFFDFFB0D0020000089", .comment = "No fall damage"},
        {.id = 60, .address = 8155228, .length = 6, .expectedHex = "89868C000000", .comment = "Run speed hack"},
        {.id = 61, .address = 6356837, .length = 2, .expectedHex = "7474", .comment = "Follow anything hack"},
        {.id = 62, .address = 6751806, .length = 1, .expectedHex = "74", .comment = "No water hack"},
        {.id = 63, .address = 4657632, .length = 2, .expectedHex = "740A", .comment = "Any name hack"},
        {.id = 64, .address = 8151976, .length = 4, .expectedHex = "84E5FFFF", .comment = "Plane teleport"},
        {.id = 65, .address = 6214371, .length = 6, .expectedHex = "8BB1540E0000", .comment = "Object tracking hack"},
        {.id = 66, .address = 6818689, .length = 5, .expectedHex = "A388F2C700", .comment = "No water hack"},
        {.id = 67, .address = 6186028, .length = 5, .expectedHex = "C705ACD2C4", .comment = "No fog hack"},
        {.id = 68, .address = 5473808, .length = 4, .expectedHex = "30855300", .comment = "Warden disable hack"},
        {.id = 69, .address = 4208171, .length = 3, .expectedHex = "6B2C00", .comment = "Warden disable hack"},
        {.id = 70, .address = 7119285, .length = 1, .expectedHex = "74", .comment = "Warden disable hack"},
        {.id = 71, .address = 4729827, .length = 1, .expectedHex = "5E", .comment = "Daylight hack"},
        {.id = 72, .address = 6354512, .length = 6, .expectedHex = "0F84EA000000", .comment = "Ranged attack stop hack"},
        {.id = 73, .address = 5053463, .length = 2, .expectedHex = "7415", .comment = "Officer note hack"},
        {.id = 79, .address = 8139737, .length = 5, .expectedHex = "D84E14DEC1", .comment = "UNKNOWN movement hack"},
        {.id = 80, .address = 8902804, .length = 4, .expectedHex = "8E977042", .comment = "Wall climb hack"},
        {.id = 81, .address = 8902808, .length = 4, .expectedHex = "0000E040", .comment = "Run speed hack"},
        {.id = 82, .address = 8154755, .length = 7, .expectedHex = "8166403FFFDFFF", .comment = "Moveflag hack"},
        {.id = 83, .address = 8445948, .length = 4, .expectedHex = "BB8D243F", .comment = "Wall climb hack"},
        {.id = 84, .address = 6493717, .length = 2, .expectedHex = "741D", .comment = "Speed hack"},
    };

    auto hexToByte = [](char hi, char lo) -> uint8_t {
        auto nibble = [](char c) -> uint8_t {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'A' && c <= 'F') return 10 + c - 'A';
            if (c >= 'a' && c <= 'f') return 10 + c - 'a';
            return 0;
        };
        return (nibble(hi) << 4) | nibble(lo);
    };

    int mismatches = 0;
    int patched = 0;
    for (const auto& e : entries) {
        std::string hexStr(e.expectedHex);
        std::vector<uint8_t> expected;
        for (size_t i = 0; i + 1 < hexStr.size(); i += 2)
            expected.push_back(hexToByte(hexStr[i], hexStr[i+1]));

        std::vector<uint8_t> actual(e.length, 0);
        bool ok = readMemory(e.address, e.length, actual.data());

        if (!ok || actual != expected) {
            mismatches++;

            // In Turtle mode, write the expected bytes into the PE image so
            // MEM_CHECK responses return what the server expects.
            if (isTurtle_ && e.address >= imageBase_) {
                uint32_t offset = e.address - imageBase_;
                if (offset + expected.size() <= imageSize_) {
                    std::memcpy(image_.data() + offset, expected.data(), expected.size());
                    patched++;
                }
            }
        }
    }

    if (mismatches == 0) {
        LOG_WARNING("WardenScan: All ", sizeof(entries)/sizeof(entries[0]),
                    " DB scan entries MATCH PE image");
    } else if (patched > 0) {
        LOG_WARNING("WardenScan: Patched ", patched, "/", mismatches,
                    " mismatched scan entries into PE image");
    } else {
        LOG_WARNING("WardenScan: ", mismatches, " / ", sizeof(entries)/sizeof(entries[0]),
                    " DB scan entries MISMATCH");
    }
}

bool WardenMemory::searchCodePattern(const uint8_t seed[4], const uint8_t expectedHash[20],
                                     uint8_t patternLen, bool imageOnly,
                                     uint32_t hintOffset, bool hintOnly) const {
    if (!loaded_ || patternLen == 0) return false;

    // Build cache key from all inputs: seed(4) + hash(20) + patLen(1) + imageOnly(1)
    std::string cacheKey(26, '\0');
    std::memcpy(&cacheKey[0], seed, 4);
    std::memcpy(&cacheKey[4], expectedHash, 20);
    cacheKey[24] = patternLen;
    cacheKey[25] = imageOnly ? 1 : 0;

    auto cacheIt = codePatternCache_.find(cacheKey);
    if (cacheIt != codePatternCache_.end()) {
        return cacheIt->second;
    }

    // --- Fast path: check the hint offset directly (single HMAC) ---
    // The PAGE_A offset field is the RVA where the server expects the pattern.
    if (hintOffset > 0 && hintOffset + patternLen <= imageSize_) {
        uint8_t hmacOut[20];
        unsigned int hmacLen = 0;
        HMAC(EVP_sha1(), seed, 4,
             image_.data() + hintOffset, patternLen,
             hmacOut, &hmacLen);
        if (hmacLen == 20 && std::memcmp(hmacOut, expectedHash, 20) == 0) {
            LOG_WARNING("WardenMemory: Code pattern found at hint RVA 0x", std::hex,
                        hintOffset, std::dec, " (direct hit)");
            codePatternCache_[cacheKey] = true;
            return true;
        }
    }

    // --- Wider hint window: search ±4096 bytes around hint offset ---
    if (hintOffset > 0) {
        size_t winStart = (hintOffset > 4096) ? hintOffset - 4096 : 0;
        size_t winEnd = std::min(static_cast<size_t>(hintOffset) + 4096 + patternLen,
                                 static_cast<size_t>(imageSize_));
        if (winEnd > winStart + patternLen) {
            for (size_t i = winStart; i + patternLen <= winEnd; i++) {
                if (i == hintOffset) continue; // already checked
                uint8_t hmacOut[20];
                unsigned int hmacLen = 0;
                HMAC(EVP_sha1(), seed, 4,
                     image_.data() + i, patternLen,
                     hmacOut, &hmacLen);
                if (hmacLen == 20 && std::memcmp(hmacOut, expectedHash, 20) == 0) {
                    LOG_WARNING("WardenMemory: Code pattern found at RVA 0x", std::hex, i,
                                std::dec, " (hint window, delta=", static_cast<int>(i) - static_cast<int>(hintOffset), ")");
                    codePatternCache_[cacheKey] = true;
                    return true;
                }
            }
        }
    }

    // If hint-only mode, skip the expensive brute-force search.
    if (hintOnly) return false;

    // --- Brute-force fallback: search all PE sections ---
    struct Range { size_t start; size_t end; };
    std::vector<Range> ranges;

    if (imageOnly && image_.size() >= 64) {
        uint32_t peOffset = image_[0x3C] | (uint32_t(image_[0x3D]) << 8)
                          | (uint32_t(image_[0x3E]) << 16) | (uint32_t(image_[0x3F]) << 24);
        if (peOffset + 4 + 20 <= image_.size()) {
            uint16_t numSections = image_[peOffset+4+2] | (uint16_t(image_[peOffset+4+3]) << 8);
            uint16_t optHeaderSize = image_[peOffset+4+16] | (uint16_t(image_[peOffset+4+17]) << 8);
            size_t secTable = peOffset + 4 + 20 + optHeaderSize;
            for (uint16_t i = 0; i < numSections; i++) {
                size_t secOfs = secTable + i * 40;
                if (secOfs + 40 > image_.size()) break;
                uint32_t va = image_[secOfs+12] | (uint32_t(image_[secOfs+13]) << 8)
                            | (uint32_t(image_[secOfs+14]) << 16) | (uint32_t(image_[secOfs+15]) << 24);
                uint32_t vsize = image_[secOfs+8] | (uint32_t(image_[secOfs+9]) << 8)
                               | (uint32_t(image_[secOfs+10]) << 16) | (uint32_t(image_[secOfs+11]) << 24);
                size_t rEnd = std::min(static_cast<size_t>(va + vsize), static_cast<size_t>(imageSize_));
                if (va + patternLen <= rEnd)
                    ranges.push_back({va, rEnd});
            }
        }
    }

    if (ranges.empty()) {
        if (patternLen <= imageSize_)
            ranges.push_back({0, imageSize_});
    }

    auto bruteStart = std::chrono::steady_clock::now();
    LOG_WARNING("WardenMemory: Brute-force searching ", ranges.size(), " section(s), hint=0x",
                std::hex, hintOffset, std::dec, " patLen=", static_cast<int>(patternLen));

    size_t totalPositions = 0;
    for (const auto& r : ranges) {
        size_t positions = r.end - r.start - patternLen + 1;
        for (size_t i = 0; i < positions; i++) {
            uint8_t hmacOut[20];
            unsigned int hmacLen = 0;
            HMAC(EVP_sha1(), seed, 4,
                 image_.data() + r.start + i, patternLen,
                 hmacOut, &hmacLen);
            if (hmacLen == 20 && std::memcmp(hmacOut, expectedHash, 20) == 0) {
                auto elapsed = std::chrono::duration<float>(
                    std::chrono::steady_clock::now() - bruteStart).count();
                LOG_WARNING("WardenMemory: Code pattern found at RVA 0x", std::hex,
                            r.start + i, std::dec, " (searched ", totalPositions + i + 1,
                            " positions in ", elapsed, "s)");
                codePatternCache_[cacheKey] = true;
                return true;
            }
        }
        totalPositions += positions;
    }

    auto elapsed = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - bruteStart).count();
    LOG_WARNING("WardenMemory: Code pattern NOT found after ", totalPositions, " positions in ",
                ranges.size(), " section(s), took ", elapsed, "s");
    codePatternCache_[cacheKey] = false;
    return false;
}

} // namespace game
} // namespace wowee
