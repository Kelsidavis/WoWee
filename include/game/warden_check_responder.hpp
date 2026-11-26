#pragma once

// Answering a Warden CHEAT_CHECKS_REQUEST.
//
// One function, so the handler's two paths - answered on the spot, or off the
// main thread when a code scan would stall it - give the same answers, and so
// a test can put a server's own check list to it and grade the reply the way
// that server does, with no account on the line.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace wowee {
namespace game {

class WardenMemory;

// What a real client answers to a PAGE_A/PAGE_B, MODULE or DRIVER scan.
// vmangos and CMaNGOS read anything but 0x4A as not found; TrinityCore and
// AzerothCore pass only 0xE9 and penalise everything else, 0x00 included -
// so 0xE9 is the one "not found" that every server takes as clean.
inline constexpr uint8_t kWardenScanFound = 0x4A;
inline constexpr uint8_t kWardenScanNotFound = 0xE9;

/// What answering checks needs from the client around it.
struct WardenCheckContext {
    /// The module's nine check opcodes: MEM, MODULE, PAGE_A, PAGE_B, MPQ,
    /// LUA, PROC, DRIVER, TIMING.
    const uint8_t* opcodes = nullptr;
    /// The WoW.exe image memory reads are served from; null answers every
    /// read as unmapped. Not const: the tick-count global is refreshed.
    WardenMemory* memory = nullptr;
    /// The loaded module's own image, searched when the executable does not
    /// hold a scanned pattern. A raw pointer into the module's mapping: the
    /// owning shared_ptr must outlive the whole buildWardenCheckResponse call,
    /// or a code scan hashing this image faults on a released mmap. Null (with
    /// moduleImageSize 0) answers the scan without searching the image.
    const uint8_t* moduleImage = nullptr;
    size_t moduleImageSize = 0;
    bool turtle = false;
    /// A game file's bytes by its in-game path, empty when there is none.
    std::function<std::vector<uint8_t>(const std::string&)> readGameFile;
};

/// The plaintext CHEAT_CHECKS_RESULT - [0x02][uint16 length][uint32 checksum]
/// [results] - for a decrypted CHEAT_CHECKS_REQUEST
/// ([0x02][strings][0x00][checks][xorByte]). `pathTag` prefixes the log lines.
std::vector<uint8_t> buildWardenCheckResponse(const std::vector<uint8_t>& request,
                                              const WardenCheckContext& ctx,
                                              const char* pathTag);

/// Whether a request holds a PAGE_A/PAGE_B scan, which can take seconds.
bool wardenRequestHasCodeScan(const std::vector<uint8_t>& request, const uint8_t opcodes[9]);

}  // namespace game
}  // namespace wowee
