#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Gem / Enchantment catalog (.wgem) - novel
// replacement for Blizzard's ItemEnchantment.dbc +
// GemProperties.dbc + SpellItemEnchantment.dbc. The 35th
// open format added to the editor.
//
// Defines two related kinds of item enhancement:
//   • Gems         - socketable jewelry pieces (red / blue /
//                     yellow / meta colors) that fit into
//                     gear sockets
//   • Enchantments - persistent buffs applied to weapon /
//                     armor pieces, either by an enchanter
//                     spell or by an item proc
//
// Cross-references with previously-added formats:
//   WGEM.gem.itemIdToInsert       → WIT.entry.itemId
//   WGEM.gem.spellId              → WSPL.entry.spellId
//   WGEM.enchantment.spellId      → WSPL.entry.spellId
//
// Binary layout (little-endian):
//   magic[4]            = "WGEM"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   gemCount (uint32)
//   gems (each):
//     gemId (uint32)
//     itemIdToInsert (uint32)
//     nameLen + name
//     color (uint8) / statType (uint8) /
//       requiredItemQuality (uint8) / pad[1]
//     statValue (int16) / pad[2]
//     spellId (uint32)
//   enchantCount (uint32)
//   enchantments (each):
//     enchantId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     enchantSlot (uint8) / statType (uint8) / pad[2]
//     statValue (int16) / pad[2]
//     spellId (uint32)
//     durationSeconds (uint32)
//     chargeCount (uint16) / pad[2]
struct WoweeGem {
    enum Color : uint8_t {
        Meta       = 0,
        Red        = 1,
        Yellow     = 2,
        Blue       = 3,
        Purple     = 4,    // red + blue
        Green      = 5,    // blue + yellow
        Orange     = 6,    // red + yellow
        Prismatic  = 7,    // matches any color socket
    };

    // Stat types match WIT.StatType (Stamina / Strength /
    // Intellect / Spirit / Agility / Defense / etc).
    struct GemEntry {
        uint32_t gemId = 0;
        uint32_t itemIdToInsert = 0;
        std::string name;
        uint8_t color = Red;
        uint8_t statType = 0;
        uint8_t requiredItemQuality = 0;    // 0 = any quality
        int16_t statValue = 0;
        uint32_t spellId = 0;               // 0 = none (stat-only gem)
    };

    enum EnchantSlot : uint8_t {
        Permanent  = 0,    // weapon enchant by enchanter
        Temporary  = 1,    // poison / oil
        SocketColor = 2,   // socket recolor
        Ring       = 3,
        Cloak      = 4,
    };

    struct EnchantEntry {
        uint32_t enchantId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t enchantSlot = Permanent;
        uint8_t statType = 0;
        int16_t statValue = 0;
        uint32_t spellId = 0;            // 0 = stat-only
        uint32_t durationSeconds = 0;    // 0 = permanent
        uint16_t chargeCount = 0;        // 0 = unlimited
    };

    std::string name;
    std::vector<GemEntry> gems;
    std::vector<EnchantEntry> enchantments;

    [[nodiscard]] bool isValid() const { return !gems.empty() || !enchantments.empty(); }

    [[nodiscard]] const GemEntry* findGem(uint32_t gemId) const;
    [[nodiscard]] const EnchantEntry* findEnchant(uint32_t enchantId) const;

    static const char* colorName(uint8_t c);
    static const char* enchantSlotName(uint8_t s);
};

class WoweeGemLoader {
public:
    static bool save(const WoweeGem& cat,
                     const std::string& basePath);
    static WoweeGem load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-gems* variants.
    //
    //   makeStarter - 3 gems (one per primary color) + 2
    //                  enchantments (one weapon proc + one
    //                  armor stat).
    //   makeGemSet  - 6 gems covering all primary + secondary
    //                  colors (red/yellow/blue/purple/green/
    //                  orange).
    //   makeEnchants - 5 enchantment variants spanning slots
    //                   (permanent stat / temporary poison /
    //                   ring / cloak / weapon proc).
    static WoweeGem makeStarter(const std::string& catalogName);
    static WoweeGem makeGemSet(const std::string& catalogName);
    static WoweeGem makeEnchants(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
