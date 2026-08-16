#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Random Suffix catalog (.wsuf) - novel
// replacement for Blizzard's ItemRandomProperties.dbc +
// ItemRandomSuffix.dbc plus the AzerothCore-style suffix-
// roll tables. Defines the random "of the X" suffixes and
// "Y of the Z" prefixes that randomly roll on green and
// blue items at world drop, e.g. "Sturdy Cloth Cap of the
// Bear" = base item + Strength + Stamina suffix.
//
// Each entry binds a suffix name to up to 5 stat bonuses
// (matches WoW canonical max). statValuePoints isn't an
// absolute number - it's a scaling base that the runtime
// multiplies by an item-level coefficient to compute the
// final per-item bonus. This way "of the Bear" gives
// proportionally more strength on a level-60 item than on
// a level-20 item.
//
// Cross-references with previously-added formats:
//   WSUF.entry.statKind values match the WIT.entry.statType
//                       enum (Strength=4, Intellect=5,
//                       Stamina=7 etc.) so item generators
//                       roll consistent stats with base items.
//   WSUF.entry.restrictedSlotMask uses the same equipment
//                       slot bit positions as WCEQ.
//
// Binary layout (little-endian):
//   magic[4]            = "WSUF"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     suffixId (uint32)
//     nameLen + name
//     descLen + description
//     itemQualityFloor (uint8) / itemQualityCeiling (uint8) /
//       suffixCategory (uint8) / pad[1]
//     restrictedSlotMask (uint32)
//     statKind1..5 (uint8) [5 bytes] / pad[3]
//     statValuePoints1..5 (uint16) [10 bytes]
struct WoweeItemSuffix {
    static constexpr size_t kMaxStats = 5;

    enum SuffixCategory : uint8_t {
        Generic    = 0,    // canonical "of the X" stat suffix
        Elemental  = 1,    // "of Fire" / "of Frost" - magic damage
        Defensive  = 2,    // "of Defense" / "of Toughness"
        PvPSuffix  = 3,    // PvP-themed (resilience, honor)
        Crafted    = 4,    // tradeskill-applied (jewelcrafting socket)
    };

    // Equipment slot bits for restrictedSlotMask. 0 = no
    // restriction (suffix can apply to any slot).
    static constexpr uint32_t kSlotHead     = 1u << 0;
    static constexpr uint32_t kSlotNeck     = 1u << 1;
    static constexpr uint32_t kSlotShoulder = 1u << 2;
    static constexpr uint32_t kSlotChest    = 1u << 3;
    static constexpr uint32_t kSlotWaist    = 1u << 4;
    static constexpr uint32_t kSlotLegs     = 1u << 5;
    static constexpr uint32_t kSlotFeet     = 1u << 6;
    static constexpr uint32_t kSlotWrist    = 1u << 7;
    static constexpr uint32_t kSlotHands    = 1u << 8;
    static constexpr uint32_t kSlotFinger   = 1u << 9;
    static constexpr uint32_t kSlotTrinket  = 1u << 10;
    static constexpr uint32_t kSlotWeapon   = 1u << 11;
    static constexpr uint32_t kSlotShield   = 1u << 12;
    static constexpr uint32_t kSlotRanged   = 1u << 13;
    static constexpr uint32_t kSlotCloak    = 1u << 14;
    static constexpr uint32_t kSlotAny      = 0;

    struct Entry {
        uint32_t suffixId = 0;
        std::string name;
        std::string description;
        uint8_t itemQualityFloor = 0;     // 0 = poor, 4 = epic
        uint8_t itemQualityCeiling = 4;
        uint8_t suffixCategory = Generic;
        uint32_t restrictedSlotMask = kSlotAny;
        uint8_t statKind[kMaxStats] = {0, 0, 0, 0, 0};
        uint16_t statValuePoints[kMaxStats] = {0, 0, 0, 0, 0};
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t suffixId) const;

    static const char* suffixCategoryName(uint8_t c);
};

class WoweeItemSuffixLoader {
public:
    static bool save(const WoweeItemSuffix& cat,
                     const std::string& basePath);
    static WoweeItemSuffix load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-suf* variants.
    //
    //   makeStarter  - 3 common Generic stat suffixes
    //                   ("of the Bear" STR+STA, "of the
    //                   Eagle" INT+SPI, "of the Tiger"
    //                   STR+AGI) covering the canonical
    //                   greens-tier stat triads.
    //   makeMagical  - 4 Elemental suffixes ("of Fire",
    //                   "of Frost", "of Shadow", "of
    //                   Arcane") - flat spell power into a
    //                   single school for caster gear.
    //   makePvP      - 3 PvPSuffix-category suffixes ("of
    //                   the Champion", "of the Gladiator",
    //                   "of Resilience") combining
    //                   resilience with offensive stats.
    static WoweeItemSuffix makeStarter(const std::string& catalogName);
    static WoweeItemSuffix makeMagical(const std::string& catalogName);
    static WoweeItemSuffix makePvP(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
