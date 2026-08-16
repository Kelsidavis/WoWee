#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Item Flag Set catalog (.wifs) - novel
// replacement for the bit-flag meanings used in
// Item.dbc / item_template.Flags. Documents every
// individual bit of the 32-bit item flags field with a
// human-readable name, description, kind classification,
// and is-positive hint.
//
// WoW's Item.dbc Flags field packs ~25 bits of metadata
// like Heroic, Lootable, NoLoot, Conjured, Unique,
// AccountBound, BindOnPickup, BindOnEquip - each
// controlling a specific gameplay behavior. The hardcoded
// client knows what each bit means via a switch
// statement; this catalog exposes that table to
// data-driven editors so:
//   - server admins can document custom flag bits
//   - tooltip generators can explain "why is this item
//     soulbound?" by flag-name lookup
//   - validators can warn about contradictory flag
//     combinations (Heroic + non-Epic quality, etc.)
//
// Cross-references with previously-added formats:
//   WIT:  every WIT item entry's flags field is decoded
//         via this catalog. Item editors can show "Item
//         Foo has flags: Heroic, BindOnPickup, Unique"
//         instead of raw 0x40240000.
//   WIQR: validators can pair Heroic flag with WIQR Epic+
//         quality requirement.
//
// Binary layout (little-endian):
//   magic[4]            = "WIFS"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     flagId (uint32)
//     nameLen + name
//     descLen + description
//     bitMask (uint32)
//     flagKind (uint8) / isPositive (uint8) / pad[2]
//     iconColorRGBA (uint32)
struct WoweeItemFlags {
    enum FlagKind : uint8_t {
        Quality       = 0,    // affects quality presentation (Heroic)
        Drop          = 1,    // affects drop / loot behavior
        Trade         = 2,    // bind / soul / trade restrictions
        Magic         = 3,    // magical properties (Conjured, Unique)
        Account       = 4,    // account / heirloom rules
        Server        = 5,    // server-custom flag
        Misc          = 6,    // catch-all
    };

    struct Entry {
        uint32_t flagId = 0;
        std::string name;
        std::string description;
        uint32_t bitMask = 0;          // typically 1 << N
        uint8_t flagKind = Misc;
        uint8_t isPositive = 0;        // 0/1 bool
        uint8_t pad0 = 0;
        uint8_t pad1 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t flagId) const;
    [[nodiscard]] const Entry* findByBit(uint32_t bitMask) const;

    // Decode an item.flags integer into the list of
    // matching entries. Returns the entries whose bitMask
    // is set in the input. Used by tooltip generators
    // to expand 0x40240000 -> ["Heroic", "BindOnPickup",
    // "Unique"].
    [[nodiscard]] std::vector<const Entry*> decode(uint32_t flagsValue) const;

    static const char* flagKindName(uint8_t k);
};

class WoweeItemFlagsLoader {
public:
    static bool save(const WoweeItemFlags& cat,
                     const std::string& basePath);
    static WoweeItemFlags load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-ifs* variants.
    //
    //   makeStandard  - 8 canonical Item.dbc flag bits
    //                    (NoLoot / Conjured / Lootable /
    //                    Wrapped / Heroic / Deprecated /
    //                    NoUserDestroy / NoEquipCooldown).
    //   makeBinding   - 5 binding-related flags
    //                    (BindOnPickup / BindOnEquip /
    //                    BindOnUse / BindToAccount /
    //                    Soulbound) with isPositive=0
    //                    (these flags restrict trading).
    //   makeServer    - 5 server-custom flag bits in the
    //                    high range (Donator / EventReward
    //                    / Anniversary / Honored /
    //                    Heroic25man) demonstrating
    //                    custom-flag conventions.
    static WoweeItemFlags makeStandard(const std::string& catalogName);
    static WoweeItemFlags makeBinding(const std::string& catalogName);
    static WoweeItemFlags makeServer(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
