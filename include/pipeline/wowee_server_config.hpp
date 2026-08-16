#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Server Config catalog (.wcfg) - novel
// replacement for the worldserver.conf / mangosd.conf
// flat-text configuration files vanilla server forks
// shipped. Each entry binds one configId to its
// polymorphic value via the valueKind enum
// (Float / Int / Bool / String) - only the matching
// value field is meaningful per entry.
//
// Polymorphic value is the novel data shape: each
// entry stores ALL four possible value fields
// (floatValue / intValue / strValue presence + boolean
// folded into intValue's low bit), and the valueKind
// enum picks which is authoritative. JSON export
// reflects this: the active field is rendered with
// its kind label so operators can edit the right one.
//
// Cross-references with previously-added formats:
//   No catalog cross-references - WCFG is a TOP-LEVEL
//   bootstrap catalog read at world startup, before
//   any in-world data is loaded.
//
// Binary layout (little-endian):
//   magic[4]            = "WCFG"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     configId (uint32)
//     nameLen + name
//     descLen + description
//     configKind (uint8)         - XPRate / DropRate /
//                                   HonorRate / RestedXP
//                                   / RealmType / World-
//                                   Flag / Performance /
//                                   Security / Misc
//     valueKind (uint8)          - Float / Int / Bool /
//                                   String
//     restartRequired (uint8)    - 0/1 bool
//     pad0 (uint8)
//     floatValue (float)         - meaningful if
//                                   valueKind == Float
//     intValue (int64)           - meaningful if
//                                   valueKind == Int
//                                   (or Bool: 0/1)
//     strLen + strValue          - meaningful if
//                                   valueKind == String
//     iconColorRGBA (uint32)
struct WoweeServerConfig {
    enum ConfigKind : uint8_t {
        XPRate       = 0,
        DropRate     = 1,
        HonorRate    = 2,
        RestedXP     = 3,
        RealmType    = 4,    // Normal / PvP / RP /
                              // RP-PvP - see WMSP
        WorldFlag    = 5,    // bool world-state flags
                              // (DoubleXPWeekend, etc.)
        Performance  = 6,    // server tuning (cell-grid
                              // sizes, etc.)
        Security     = 7,    // anti-cheat thresholds
        Misc         = 255,
    };

    enum ValueKind : uint8_t {
        Float  = 0,
        Int    = 1,
        Bool   = 2,    // serialized as int 0/1
        String = 3,
    };

    struct Entry {
        uint32_t configId = 0;
        std::string name;
        std::string description;
        uint8_t configKind = Misc;
        uint8_t valueKind = Float;
        uint8_t restartRequired = 0;
        uint8_t pad0 = 0;
        float floatValue = 0.0f;
        int64_t intValue = 0;
        std::string strValue;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t configId) const;

    // Returns all entries of one configKind. Used by
    // server startup code to iterate the matching
    // tunables: pull all XPRate entries to seed the
    // experience-rate matrix per character class, etc.
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t configKind) const;
};

class WoweeServerConfigLoader {
public:
    static bool save(const WoweeServerConfig& cat,
                     const std::string& basePath);
    static WoweeServerConfig load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cfg* variants.
    //
    //   makeRates       - 4 rate-multiplier configs
    //                      (XPRate 1.0x / DropRate 1.0x /
    //                      HonorRate 1.0x / RestedXP
    //                      200%) - vanilla baseline.
    //   makePerformance - 4 server tuning configs
    //                      (max creatures per cell 100 /
    //                      view distance 533 yards /
    //                      spawn-rate 1.0x / GC interval
    //                      300s) - Performance kind.
    //   makeSecurity    - 4 anti-cheat configs
    //                      (speedhack tolerance 1.05x /
    //                      trade gold cap 100000g /
    //                      GM command audit logging
    //                      enabled / cheat-detection
    //                      sensitivity "high" - String
    //                      value).
    static WoweeServerConfig makeRates(const std::string& catalogName);
    static WoweeServerConfig makePerformance(const std::string& catalogName);
    static WoweeServerConfig makeSecurity(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
