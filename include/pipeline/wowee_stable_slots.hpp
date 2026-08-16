#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Hunter Stable Slot catalog (.wstc) - novel
// replacement for the hardcoded hunter pet stable slot
// progression. Defines each stable slot's display order
// in the stable UI, the character level at which the
// slot becomes available, the gold cost to unlock, and
// whether it's a premium / donator-only slot.
//
// In WoW 3.3.5a hunters get 5 stable slots total: the
// active pet plus 4 stabled (slots 1-4 unlocking at
// hunter levels 10/20/30/40 with escalating gold costs).
// Cataclysm raised the cap to 5 stabled slots, and
// server-custom expansions go higher. This catalog lets
// admins parameterize the entire progression instead of
// editing engine source.
//
// Cross-references with previously-added formats:
//   None - this catalog is consumed directly by the
//   stable master service in WBKD entries with
//   serviceKind=StableMaster.
//
// Binary layout (little-endian):
//   magic[4]            = "WSTC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     slotId (uint32)
//     nameLen + name
//     descLen + description
//     displayOrder (uint8) / minLevelToUnlock (uint8)
//     isPremium (uint8) / pad (uint8)
//     copperCost (uint32)
//     iconColorRGBA (uint32)
struct WoweeStableSlot {
    struct Entry {
        uint32_t slotId = 0;
        std::string name;
        std::string description;
        uint8_t displayOrder = 0;
        uint8_t minLevelToUnlock = 1;
        uint8_t isPremium = 0;       // 0/1 bool
        uint8_t pad0 = 0;
        uint32_t copperCost = 0;     // 1g = 10000c
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t slotId) const;

    // Returns the count of slots a hunter at the given
    // character level has unlocked. Used by the stable
    // master frame to decide how many slot tabs to render.
    [[nodiscard]] int unlockedSlotCount(uint8_t characterLevel) const;
};

class WoweeStableSlotLoader {
public:
    static bool save(const WoweeStableSlot& cat,
                     const std::string& basePath);
    static WoweeStableSlot load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-stc* variants.
    //
    //   makeStandard - 5 canonical slots matching WoW
    //                   3.3.5a (Active + 4 stabled,
    //                   unlocking at lvl 10/20/30/40 with
    //                   10s/50s/2g/10g costs).
    //   makeCata     - 6 Cata-style slots (Active + 5
    //                   stabled with later unlock at
    //                   lvl 50, 25g cost for slot 5).
    //   makePremium  - 4 server-custom premium slots
    //                   (donator-only, marked premium=1,
    //                   no level gate, no gold cost).
    static WoweeStableSlot makeStandard(const std::string& catalogName);
    static WoweeStableSlot makeCata(const std::string& catalogName);
    static WoweeStableSlot makePremium(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
