#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Random Property Pool catalog
// (.wirc) - novel replacement for the random-
// suffix enchant pool that vanilla WoW carried in
// ItemRandomProperties.dbc + ItemRandomSuffix.dbc
// (TBC+) + the per-item RandomProperty rolls baked
// into the LootMgr. Each WIRC entry binds one
// random-property pool to a name suffix ("of the
// Bear", "of the Eagle"), a weighted enchant table
// (variable-length array of {enchantId, weight}),
// and the equipment slots + class restrictions
// where the suffix can roll.
//
// At loot time, each green+ item rolls one pool
// based on its slot, then picks one enchant from
// that pool weighted by enchant.weight /
// totalWeight.
//
// Cross-references with previously-added formats:
//   WIT:   pool affinity is determined by the
//          item's slot (which lives in WIT). At
//          runtime the loot generator picks the
//          pool whose allowedSlots bitmask matches
//          the item's slot.
//   WSPL:  enchantId references the WSPL spell
//          catalog (enchant spells have spellId
//          ranges 7000..30000 in vanilla).
//
// Binary layout (little-endian):
//   magic[4]            = "WIRC"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     poolId (uint32)
//     nameLen + name (suffix display: "of the X")
//     scaleLevel (uint8)         - itemLevel band
//                                   (1..60 vanilla);
//                                   0 = any
//     allowedSlotsMask (uint8)   - 0x01=Helm,
//                                   0x02=Shoulder,
//                                   0x04=Chest,
//                                   0x08=Leg,
//                                   0x10=Boot,
//                                   0x20=Glove,
//                                   0x40=Bracer,
//                                   0x80=Belt
//     allowedClassesMask (uint16) - bitmask 1<<class
//                                    (0 = all
//                                    classes)
//     totalWeight (uint32)       - denormalized sum
//                                   of enchant
//                                   weights
//     enchantCount (uint32)
//     enchants (each: enchantId(4) + weight(4) = 8B)
struct WoweeRandomProperty {
    enum SlotMask : uint8_t {
        Helm     = 0x01,
        Shoulder = 0x02,
        Chest    = 0x04,
        Leg      = 0x08,
        Boot     = 0x10,
        Glove    = 0x20,
        Bracer   = 0x40,
        Belt     = 0x80,
    };

    struct EnchantEntry {
        uint32_t enchantId = 0;
        uint32_t weight = 0;
    };

    struct Entry {
        uint32_t poolId = 0;
        std::string name;
        uint8_t scaleLevel = 0;
        uint8_t allowedSlotsMask = 0;
        uint16_t allowedClassesMask = 0;
        uint32_t totalWeight = 0;
        std::vector<EnchantEntry> enchants;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t poolId) const;

    // Returns pools applicable to a given slot mask
    // - used by the loot generator to pick eligible
    // suffix pools at roll time.
    [[nodiscard]] std::vector<const Entry*> findBySlot(uint8_t slotMask) const;
};

class WoweeRandomPropertyLoader {
public:
    static bool save(const WoweeRandomProperty& cat,
                     const std::string& basePath);
    static WoweeRandomProperty load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-irc* variants.
    //
    //   makeOfTheBear  - STA-focused suffix pool for
    //                     plate/mail slots (Helm,
    //                     Chest, Leg, Boot). 4
    //                     enchants weighted toward
    //                     +Sta variants.
    //   makeOfTheEagle - INT+STA caster pool for
    //                     cloth slots. 5 enchants
    //                     weighted toward +Int +Sta.
    //   makeOfTheTiger - STR+AGI hybrid pool for
    //                     leather slots. 5 enchants
    //                     covering AGI/STR/AP
    //                     variants.
    static WoweeRandomProperty makeOfTheBear(const std::string& catalogName);
    static WoweeRandomProperty makeOfTheEagle(const std::string& catalogName);
    static WoweeRandomProperty makeOfTheTiger(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
