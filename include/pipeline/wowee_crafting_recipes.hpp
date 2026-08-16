#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Crafting Recipe catalog (.wcra) -
// novel replacement for the implicit recipe
// expansion that vanilla WoW carried in
// SpellReagents.dbc + Spell.dbc effect-24
// (CREATE_ITEM) + per-trade-skill SkillLineAbility
// rows. Each WCRA entry binds one trade-skill
// recipe spell to its reagent list (variable-
// length itemId+count pairs), produced-item id +
// count, the trade skill it belongs to, the
// minimum skill level to cast, and the source
// item that teaches the recipe.
//
// Cross-references with previously-added formats:
//   WSPL: spellId references the WSPL spell catalog
//         (the actual spell-to-cast - Brilliant
//         Mana Oil is spellId 25127, etc.).
//   WIT:  every reagent[i].itemId, producedItemId,
//         and learnedFromItemId references the WIT
//         item catalog.
//   WSKL: tradeSkillId references the WSKL skill
//         catalog (Alchemy=171, Blacksmithing=164,
//         Engineering=202, etc.).
//   WCAT: categoryId references the trade-skill
//         category catalog (within a skill, the
//         subcategory tab - "Potion / Elixir /
//         Transmute" for Alchemy).
//
// Binary layout (little-endian):
//   magic[4]            = "WCRA"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     recipeId (uint32)
//     spellId (uint32)             - WSPL cast
//                                     spell
//     nameLen + name               - display label
//     tradeSkillId (uint16)        - WSKL ref
//     requiredSkillLevel (uint16)  - 0..300 vanilla
//     producedItemId (uint32)
//     producedCount (uint16)
//     categoryId (uint16)          - within-skill
//                                     tab
//     learnedFromItemId (uint32)   - recipe scroll
//                                     (0 = trainer-
//                                     learned)
//     reagentCount (uint32)
//     reagents (each: uint32 itemId + uint32 count)
struct WoweeCraftingRecipes {
    struct Reagent {
        uint32_t itemId = 0;
        uint32_t count = 0;
    };

    struct Entry {
        uint32_t recipeId = 0;
        uint32_t spellId = 0;
        std::string name;
        uint16_t tradeSkillId = 0;
        uint16_t requiredSkillLevel = 0;
        uint32_t producedItemId = 0;
        uint16_t producedCount = 1;
        uint16_t categoryId = 0;
        uint32_t learnedFromItemId = 0;
        std::vector<Reagent> reagents;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t recipeId) const;

    // Returns the recipe for a given cast spellId -
    // the lookup the trade-skill cast handler uses
    // to resolve which item to produce + which
    // reagents to consume.
    [[nodiscard]] const Entry* findBySpellId(uint32_t spellId) const;

    // Returns all recipes belonging to a trade
    // skill. Used by the trade-skill window UI to
    // populate the per-skill recipe list.
    [[nodiscard]] std::vector<const Entry*> findByTradeSkill(uint16_t tradeSkillId) const;

    // Returns all recipes that produce a given
    // itemId - useful for "how do I make this?"
    // tooltip-link queries.
    [[nodiscard]] std::vector<const Entry*> findByProducedItem(uint32_t itemId) const;
};

class WoweeCraftingRecipesLoader {
public:
    static bool save(const WoweeCraftingRecipes& cat,
                     const std::string& basePath);
    static WoweeCraftingRecipes load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-cra* variants.
    //
    //   makeAlchemyPotions  - 4 vanilla Alchemy
    //                          potions (Healing /
    //                          Mana / Greater Healing
    //                          / Major Mana). Each
    //                          uses 2 herb reagents +
    //                          1 vial.
    //   makeEngineering    - 3 vanilla Engineering
    //                          recipes (Rough Blasting
    //                          Powder / Mechanical
    //                          Squirrel Box / Target
    //                          Dummy). Demonstrates
    //                          variable reagent count
    //                          (Target Dummy needs 5
    //                          reagents).
    //   makeBlacksmithing  - 3 Blacksmithing recipes
    //                          (Rough Sharpening Stone
    //                          / Coarse Grinding Stone
    //                          / Heavy Mithril Helm)
    //                          covering low/mid/high
    //                          skill tiers.
    static WoweeCraftingRecipes makeAlchemyPotions(const std::string& catalogName);
    static WoweeCraftingRecipes makeEngineering(const std::string& catalogName);
    static WoweeCraftingRecipes makeBlacksmithing(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
