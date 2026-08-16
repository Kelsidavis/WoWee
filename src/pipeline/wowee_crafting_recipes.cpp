#include "pipeline/wowee_crafting_recipes.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'C', 'R', 'A'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wcra";

} // namespace

const WoweeCraftingRecipes::Entry*
WoweeCraftingRecipes::findById(uint32_t recipeId) const {
    for (const auto& e : entries)
        if (e.recipeId == recipeId) return &e;
    return nullptr;
}

const WoweeCraftingRecipes::Entry*
WoweeCraftingRecipes::findBySpellId(uint32_t spellId) const {
    for (const auto& e : entries)
        if (e.spellId == spellId) return &e;
    return nullptr;
}

std::vector<const WoweeCraftingRecipes::Entry*>
WoweeCraftingRecipes::findByTradeSkill(uint16_t tradeSkillId) const {
    std::vector<const Entry*> out;
    for (const auto& e : entries)
        if (e.tradeSkillId == tradeSkillId) out.push_back(&e);
    return out;
}

std::vector<const WoweeCraftingRecipes::Entry*>
WoweeCraftingRecipes::findByProducedItem(uint32_t itemId) const {
    std::vector<const Entry*> out;
    for (const auto& e : entries)
        if (e.producedItemId == itemId) out.push_back(&e);
    return out;
}

bool WoweeCraftingRecipesLoader::save(
    const WoweeCraftingRecipes& cat,
    const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const auto& e) {
        writePOD(os, e.recipeId);
        writePOD(os, e.spellId);
        writeStr(os, e.name);
        writePOD(os, e.tradeSkillId);
        writePOD(os, e.requiredSkillLevel);
        writePOD(os, e.producedItemId);
        writePOD(os, e.producedCount);
        writePOD(os, e.categoryId);
        writePOD(os, e.learnedFromItemId);
        uint32_t reagentCount =
            static_cast<uint32_t>(e.reagents.size());
        writePOD(os, reagentCount);
        for (const auto& r : e.reagents) {
            writePOD(os, r.itemId);
            writePOD(os, r.count);
        }
    });
}

WoweeCraftingRecipes WoweeCraftingRecipesLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweeCraftingRecipes>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweeCraftingRecipes::Entry& e) {
        if (!readPOD(is, e.recipeId) ||
            !readPOD(is, e.spellId)) { return false; }
        if (!readStr(is, e.name)) { return false; }
        if (!readPOD(is, e.tradeSkillId) ||
            !readPOD(is, e.requiredSkillLevel) ||
            !readPOD(is, e.producedItemId) ||
            !readPOD(is, e.producedCount) ||
            !readPOD(is, e.categoryId) ||
            !readPOD(is, e.learnedFromItemId)) { return false; }
        uint32_t reagentCount = 0;
        if (!readPOD(is, reagentCount)) { return false; }
        // Sanity cap - no recipe should have more than
        // 32 reagents; vanilla cap is 8.
        if (reagentCount > 32) { return false; }
        e.reagents.resize(reagentCount);
        for (auto& r : e.reagents) {
            if (!readPOD(is, r.itemId) ||
                !readPOD(is, r.count)) { return false; }
        }
                                  return true;
                              });
}

bool WoweeCraftingRecipesLoader::exists(
    const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

namespace {

// Vanilla trade-skill IDs from SkillLine.dbc:
//   Alchemy=171, Blacksmithing=164,
//   Engineering=202, Enchanting=333,
//   LeatherWorking=165, Tailoring=197,
//   Cooking=185, FirstAid=129.
constexpr uint16_t kAlchemy = 171;
constexpr uint16_t kEngineering = 202;
constexpr uint16_t kBlacksmithing = 164;

WoweeCraftingRecipes::Entry makeRecipe(
    uint32_t recipeId, uint32_t spellId, const char* name,
    uint16_t tradeSkillId, uint16_t requiredSkill,
    uint32_t producedItemId, uint16_t producedCount,
    uint16_t categoryId, uint32_t learnedFromItemId,
    std::vector<WoweeCraftingRecipes::Reagent> reagents) {
    WoweeCraftingRecipes::Entry e;
    e.recipeId = recipeId; e.spellId = spellId;
    e.name = name;
    e.tradeSkillId = tradeSkillId;
    e.requiredSkillLevel = requiredSkill;
    e.producedItemId = producedItemId;
    e.producedCount = producedCount;
    e.categoryId = categoryId;
    e.learnedFromItemId = learnedFromItemId;
    e.reagents = std::move(reagents);
    return e;
}

} // namespace

WoweeCraftingRecipes WoweeCraftingRecipesLoader::makeAlchemyPotions(
    const std::string& catalogName) {
    WoweeCraftingRecipes c;
    c.name = catalogName;
    // Vanilla Alchemy potions. Reagent itemIds are
    // canonical: Peacebloom=2447, Silverleaf=765,
    // Briarthorn=2450, Mageroyal=785,
    // Bruiseweed=2453, Stranglekelp=3820,
    // Liferoot=3357, Goldthorn=3821, Khadgar's
    // Whisker=3358, Empty Vial=3371.
    c.entries.push_back(makeRecipe(
        1, 2330, "Minor Healing Potion", kAlchemy, 1,
        118, 1, 1, 0,
        {{.itemId = 2447, .count = 1}, {.itemId = 765, .count = 1}, {.itemId = 3371, .count = 1}}));
    // Lesser Mana Potion: Mageroyal + Stranglekelp.
    c.entries.push_back(makeRecipe(
        2, 2331, "Lesser Mana Potion", kAlchemy, 100,
        3385, 1, 1, 0,
        {{.itemId = 785, .count = 1}, {.itemId = 3820, .count = 1}, {.itemId = 3371, .count = 1}}));
    // Greater Healing Potion: Liferoot + Khadgar's
    // Whisker.
    c.entries.push_back(makeRecipe(
        3, 11457, "Greater Healing Potion", kAlchemy, 155,
        3928, 1, 1, 0,
        {{.itemId = 3357, .count = 1}, {.itemId = 3358, .count = 1}, {.itemId = 3371, .count = 1}}));
    // Major Mana Potion: Sungrass=8838 + Blindweed
    // =8839 + Crystal Vial=8766 (uses larger vial).
    c.entries.push_back(makeRecipe(
        4, 17580, "Major Mana Potion", kAlchemy, 295,
        13444, 1, 1, 0,
        {{.itemId = 8838, .count = 3}, {.itemId = 8839, .count = 3}, {.itemId = 8766, .count = 1}}));
    return c;
}

WoweeCraftingRecipes WoweeCraftingRecipesLoader::makeEngineering(
    const std::string& catalogName) {
    WoweeCraftingRecipes c;
    c.name = catalogName;
    // Rough Blasting Powder: 1 Rough Stone (2835)
    // for 1 powder. Lowest-skill engineering recipe.
    c.entries.push_back(makeRecipe(
        10, 3918, "Rough Blasting Powder", kEngineering, 1,
        4357, 1, 1, 0,
        {{.itemId = 2835, .count = 1}}));
    // Mechanical Squirrel Box: rough copper-bar
    // recipe - 4 reagents.
    c.entries.push_back(makeRecipe(
        11, 4413, "Mechanical Squirrel Box", kEngineering, 75,
        4401, 1, 1, 0,
        {{.itemId = 2840, .count = 2}, {.itemId = 4399, .count = 1}, {.itemId = 2589, .count = 1}, {.itemId = 4357, .count = 1}}));
    // Target Dummy: 5 reagents - demonstrates
    // variable reagent count within the recipe
    // catalog. Blueprint is itemId 4406.
    c.entries.push_back(makeRecipe(
        12, 4079, "Target Dummy", kEngineering, 75,
        2092, 1, 1, 4406,
        {{.itemId = 2840, .count = 4}, {.itemId = 4361, .count = 2}, {.itemId = 2997, .count = 2}, {.itemId = 2589, .count = 4}, {.itemId = 4357, .count = 1}}));
    return c;
}

WoweeCraftingRecipes WoweeCraftingRecipesLoader::makeBlacksmithing(
    const std::string& catalogName) {
    WoweeCraftingRecipes c;
    c.name = catalogName;
    // Rough Sharpening Stone: 1 Rough Stone -> 1
    // sharpening stone. Skill 1 (default).
    c.entries.push_back(makeRecipe(
        20, 2660, "Rough Sharpening Stone", kBlacksmithing, 1,
        2862, 1, 1, 0,
        {{.itemId = 2835, .count = 1}}));
    // Coarse Grinding Stone: 2 Coarse Stone (2836).
    // Skill 50.
    c.entries.push_back(makeRecipe(
        21, 3326, "Coarse Grinding Stone", kBlacksmithing, 50,
        3486, 1, 1, 0,
        {{.itemId = 2836, .count = 2}}));
    // Heavy Mithril Helm: high-skill plate piece
    // requiring multiple bar types. Skill 235.
    c.entries.push_back(makeRecipe(
        22, 9938, "Heavy Mithril Helm", kBlacksmithing, 235,
        7909, 1, 2, 11163,
        {{.itemId = 3860, .count = 8}, {.itemId = 3859, .count = 1}, {.itemId = 3864, .count = 4}, {.itemId = 3866, .count = 2}}));
    return c;
}

} // namespace pipeline
} // namespace wowee
