#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Trade Skill / Recipe catalog (.wtsk) - novel
// replacement for Blizzard's SkillLineAbility.dbc plus the
// recipe portions of SkillLine.dbc plus the AzerothCore
// trade_skill SQL tables. The 50th open format added to
// the editor - a milestone format that closes the crafting
// gap left by WSKL (which only carries the skill lines
// themselves, not the recipes that bind to them).
//
// Defines per-profession recipes: Blacksmithing, Tailoring,
// Engineering, Alchemy, Enchanting, Leatherworking, Mining,
// Skinning, Herbalism, Cooking, First Aid, Fishing. Each
// recipe binds a craft spell (WSPL) to a produced item
// (WIT) and up to 4 reagent slots, gated by a skill rank
// threshold and bracket-coloured (orange / yellow / green
// / gray) for skill-up probability.
//
// Cross-references with previously-added formats:
//   WTSK.entry.craftSpellId    → WSPL.spellId
//   WTSK.entry.producedItemId  → WIT.itemId
//   WTSK.entry.toolItemId      → WIT.itemId (anvil/loom/...)
//   WTSK.entry.reagent[0..3]   → WIT.itemId
//   WTSK.entry.skillId         → WSKL.skillId
//
// Binary layout (little-endian):
//   magic[4]            = "WTSK"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     recipeId (uint32)
//     nameLen + name
//     descLen + description
//     iconLen + iconPath
//     profession (uint8) / pad[3]
//     skillId (uint32)
//     orangeRank (uint16) / yellowRank (uint16) /
//       greenRank (uint16)  / grayRank (uint16)
//     craftSpellId (uint32)
//     producedItemId (uint32)
//     producedMinCount (uint8) / producedMaxCount (uint8) / pad[2]
//     toolItemId (uint32)
//     reagentItemId[4] (uint32) / reagentCount[4] (uint8) / pad[4]
struct WoweeTradeSkill {
    enum Profession : uint8_t {
        Blacksmithing  = 0,
        Tailoring      = 1,
        Engineering    = 2,
        Alchemy        = 3,
        Enchanting     = 4,
        Leatherworking = 5,
        Jewelcrafting  = 6,
        Inscription    = 7,
        Mining         = 8,
        Skinning       = 9,
        Herbalism      = 10,
        Cooking        = 11,
        FirstAid       = 12,
        Fishing        = 13,
    };

    static constexpr size_t kMaxReagents = 4;

    struct Entry {
        uint32_t recipeId = 0;
        std::string name;
        std::string description;
        std::string iconPath;
        uint8_t profession = Blacksmithing;
        uint32_t skillId = 0;        // WSKL cross-ref
        uint16_t orangeRank = 1;     // 100% skill-up chance
        uint16_t yellowRank = 25;    // ~75% skill-up
        uint16_t greenRank = 50;     // ~25% skill-up
        uint16_t grayRank = 75;      // 0% skill-up
        uint32_t craftSpellId = 0;   // WSPL cross-ref
        uint32_t producedItemId = 0; // WIT cross-ref
        uint8_t producedMinCount = 1;
        uint8_t producedMaxCount = 1;
        uint32_t toolItemId = 0;     // WIT cross-ref (anvil/loom)
        uint32_t reagentItemId[kMaxReagents] = {0, 0, 0, 0};
        uint8_t reagentCount[kMaxReagents] = {0, 0, 0, 0};
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t recipeId) const;

    static const char* professionName(uint8_t p);
};

class WoweeTradeSkillLoader {
public:
    static bool save(const WoweeTradeSkill& cat,
                     const std::string& basePath);
    static WoweeTradeSkill load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tsk* variants.
    //
    //   makeStarter        - 3 recipes covering the entry-tier
    //                         spread (Coarse Sharpening Stone,
    //                         Linen Cloth Bandage, Minor
    //                         Healing Potion) - one each for
    //                         Blacksmithing / First Aid /
    //                         Alchemy.
    //   makeBlacksmithing  - 5 progression recipes
    //                         (sharpening stone, copper chain
    //                         belt, runed copper bracers,
    //                         ironforge breastplate,
    //                         truesilver champion).
    //   makeAlchemy        - 5 progression recipes (minor
    //                         healing, swiftness, lesser
    //                         mana, greater healing,
    //                         flask of titans).
    static WoweeTradeSkill makeStarter(const std::string& catalogName);
    static WoweeTradeSkill makeBlacksmithing(const std::string& catalogName);
    static WoweeTradeSkill makeAlchemy(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
