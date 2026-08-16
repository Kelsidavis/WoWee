#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Quality catalog (.wiqr) - novel
// replacement for the hardcoded item quality tiers in
// the WoW client (Poor / Common / Uncommon / Rare / Epic
// / Legendary / Artifact / Heirloom). Defines each tier's
// tooltip text color, inventory slot border color,
// vendor price multiplier, drop-level gating, and
// disenchant eligibility.
//
// The hardcoded client uses a static color table:
//   Poor      = gray   #9d9d9d
//   Common    = white  #ffffff
//   Uncommon  = green  #1eff00
//   Rare      = blue   #0070dd
//   Epic      = purple #a335ee
//   Legendary = orange #ff8000
//   Artifact  = red    #e6cc80
//   Heirloom  = gold   #00ccff
//
// This catalog lets server admins:
//   - retune the colors (rename "Epic" to "Tier 1" with a
//     custom orange-red, etc.)
//   - add server-custom tiers above Heirloom
//     (Donator / Weekly / Anniversary)
//   - change vendor markup per tier (legendary sells for
//     20x base price)
//   - gate quality drops by character level (Heirlooms
//     unlock at lvl 80)
//
// Cross-references with previously-added formats:
//   WIT: item entries reference qualityId here for
//        tooltip color and sort order.
//
// Binary layout (little-endian):
//   magic[4]            = "WIQR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     qualityId (uint32)
//     nameLen + name
//     descLen + description
//     nameColorRGBA (uint32)
//     borderColorRGBA (uint32)
//     vendorPriceMultiplier (float)
//     minLevelToDrop (uint8) / maxLevelToDrop (uint8)
//     canBeDisenchanted (uint8) / pad (uint8)
//     borderTexLen + inventoryBorderTexture
struct WoweeItemQuality {
    struct Entry {
        uint32_t qualityId = 0;
        std::string name;
        std::string description;
        uint32_t nameColorRGBA = 0xFFFFFFFFu;
        uint32_t borderColorRGBA = 0xFFFFFFFFu;
        float vendorPriceMultiplier = 1.0f;
        uint8_t minLevelToDrop = 1;
        uint8_t maxLevelToDrop = 0;     // 0 = no max
        uint8_t canBeDisenchanted = 0;  // 0/1 bool
        uint8_t pad0 = 0;
        std::string inventoryBorderTexture;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t qualityId) const;

    // Returns true if an item of this quality can drop
    // for a character of the given level (gated by
    // [minLevelToDrop, maxLevelToDrop] when maxLevelToDrop
    // is non-zero).
    [[nodiscard]] bool canDropAtLevel(uint32_t qualityId,
                         uint8_t characterLevel) const;
};

class WoweeItemQualityLoader {
public:
    static bool save(const WoweeItemQuality& cat,
                     const std::string& basePath);
    static WoweeItemQuality load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-iqr* variants.
    //
    //   makeStandard   - 8 canonical WoW item quality
    //                     tiers (Poor through Heirloom)
    //                     with their standard hex colors
    //                     and disenchant rules. Heirloom
    //                     gated to lvl 80.
    //   makeServerCustom - 4 server-custom tiers
    //                     (Junk / Weekly / QuestLocked /
    //                     Donator) with custom colors and
    //                     non-standard vendor multipliers.
    //   makeRaidTiers  - 4 raid progression tiers
    //                     (T1 lvl 60 / T2 lvl 60 / T3
    //                     lvl 60 / Legendary lvl 60+)
    //                     gated by minLevelToDrop and
    //                     priced for server economy.
    static WoweeItemQuality makeStandard(const std::string& catalogName);
    static WoweeItemQuality makeServerCustom(const std::string& catalogName);
    static WoweeItemQuality makeRaidTiers(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
