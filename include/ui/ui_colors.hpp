#pragma once

#include <imgui.h>
#include "game/inventory.hpp"
#include "game/item_text.hpp"

namespace wowee::ui {

// ---- Common UI colors ----
namespace colors {
    constexpr ImVec4 kRed         = {1.0f, 0.3f, 0.3f, 1.0f};
    constexpr ImVec4 kGreen       = {0.4f, 1.0f, 0.4f, 1.0f};
    constexpr ImVec4 kBrightGreen = {0.3f, 1.0f, 0.3f, 1.0f};
    constexpr ImVec4 kYellow      = {1.0f, 1.0f, 0.3f, 1.0f};
    constexpr ImVec4 kGray        = {0.6f, 0.6f, 0.6f, 1.0f};
    // The three steps of the level-difficulty scale that had no name, written
    // out identically at each of the places that drew one.
    constexpr ImVec4 kSkullRed       = {1.0f, 0.1f, 0.1f, 1.0f};
    constexpr ImVec4 kDifficultOrange= {1.0f, 0.5f, 0.1f, 1.0f};
    constexpr ImVec4 kEvenYellow     = {1.0f, 1.0f, 0.1f, 1.0f};
    constexpr ImVec4 kDarkGray    = {0.5f, 0.5f, 0.5f, 1.0f};
    constexpr ImVec4 kLightGray   = {0.7f, 0.7f, 0.7f, 1.0f};
    constexpr ImVec4 kWhite       = {1.0f, 1.0f, 1.0f, 1.0f};
    constexpr ImVec4 kTooltipGold = {1.0f, 0.82f, 0.0f, 1.0f};
    constexpr ImVec4 kBrightGold  = {1.0f, 0.85f, 0.0f, 1.0f};
    constexpr ImVec4 kPaleRed     = {1.0f, 0.5f, 0.5f, 1.0f};
    constexpr ImVec4 kBrightRed   = {1.0f, 0.2f, 0.2f, 1.0f};
    constexpr ImVec4 kLightBlue   = {0.4f, 0.6f, 1.0f, 1.0f};
    constexpr ImVec4 kManaBlue    = {0.2f, 0.2f, 0.9f, 1.0f};
    constexpr ImVec4 kCyan        = {0.0f, 0.8f, 1.0f, 1.0f};
    constexpr ImVec4 kDarkRed     = {0.9f, 0.2f, 0.2f, 1.0f};
    constexpr ImVec4 kSoftRed     = {1.0f, 0.4f, 0.4f, 1.0f};
    constexpr ImVec4 kHostileRed  = {1.0f, 0.35f, 0.35f, 1.0f};
    constexpr ImVec4 kMediumGray  = {0.65f, 0.65f, 0.65f, 1.0f};
    constexpr ImVec4 kWarmGold    = {1.0f, 0.84f, 0.0f, 1.0f};
    constexpr ImVec4 kOrange      = {0.9f, 0.6f, 0.1f, 1.0f};
    constexpr ImVec4 kFriendlyGreen = {0.2f, 0.7f, 0.2f, 1.0f};
    constexpr ImVec4 kHealthGreen   = {0.2f, 0.8f, 0.2f, 1.0f};
    constexpr ImVec4 kLightGreen    = {0.6f, 1.0f, 0.6f, 1.0f};
    constexpr ImVec4 kActiveGreen   = {0.5f, 1.0f, 0.5f, 1.0f};
    constexpr ImVec4 kSocketGreen   = {0.5f, 0.8f, 0.5f, 1.0f};

    // UI element colors
    constexpr ImVec4 kInactiveGray   = {0.55f, 0.55f, 0.55f, 1.0f};
    constexpr ImVec4 kVeryLightGray  = {0.85f, 0.85f, 0.85f, 1.0f};
    constexpr ImVec4 kSymbolGold     = {1.0f, 0.85f, 0.1f, 1.0f};
    constexpr ImVec4 kDarkYellow     = {0.8f, 0.8f, 0.0f, 1.0f};
    constexpr ImVec4 kLowHealthRed   = {0.8f, 0.2f, 0.2f, 1.0f};
    constexpr ImVec4 kDangerRed      = {0.7f, 0.2f, 0.2f, 1.0f};

    // Cast bar / status colors
    constexpr ImVec4 kCastGreen      = {0.2f, 0.75f, 0.2f, 1.0f};
    constexpr ImVec4 kQueueGreen     = {0.3f, 0.9f, 0.3f, 1.0f};

    // Button styling colors (accept/decline patterns)
    constexpr ImVec4 kBtnGreen       = {0.15f, 0.5f, 0.15f, 1.0f};
    constexpr ImVec4 kBtnRed         = {0.5f, 0.15f, 0.15f, 1.0f};
    constexpr ImVec4 kBtnDkGreen     = {0.2f, 0.5f, 0.2f, 1.0f};
    constexpr ImVec4 kBtnDkGreenHover= {0.3f, 0.7f, 0.3f, 1.0f};
    constexpr ImVec4 kBtnDkRed       = {0.5f, 0.2f, 0.2f, 1.0f};
    constexpr ImVec4 kBtnDkRedHover  = {0.7f, 0.3f, 0.3f, 1.0f};
    constexpr ImVec4 kMidHealthYellow= {0.8f, 0.8f, 0.2f, 1.0f};

    // Power-type colors (unit resource bars)
    constexpr ImVec4 kEnergyYellow    = {0.9f, 0.9f, 0.2f, 1.0f};
    constexpr ImVec4 kHappinessGreen  = {0.5f, 0.9f, 0.3f, 1.0f};
    constexpr ImVec4 kRunicRed        = {0.8f, 0.1f, 0.2f, 1.0f};
    constexpr ImVec4 kSoulShardPurple = {0.4f, 0.1f, 0.6f, 1.0f};

    // Coin colors
    constexpr ImVec4 kGold   = {1.00f, 0.82f, 0.00f, 1.0f};
    constexpr ImVec4 kSilver = {0.80f, 0.80f, 0.80f, 1.0f};
    constexpr ImVec4 kCopper = {0.72f, 0.45f, 0.20f, 1.0f};
// ---- Health bar colour ----
//
// Green above half, yellow above a fifth, red below. Every unit frame agreed on
// where the two thresholds are and none of them agreed on the three colours:
// four palettes across seven bars, so a mob at a third health was one yellow on
// the target frame and a slightly duller one on the frame below it. These are
// the shades the target frame and the party list were already using.
//
// The player's own bar is not one of these - it ramps continuously between the
// thresholds and pulses when critical - and neither is the boss frame, which is
// deliberately a red-to-yellow scale rather than a green one.
inline ImVec4 healthBarColor(float pct) {
    if (pct > 0.5f) return kHealthGreen;
    if (pct > 0.2f) return kMidHealthYellow;
    return kLowHealthRed;
}

/// The same colour packed the way a draw list wants it.
inline ImU32 healthBarColorU32(float pct) {
    return ImGui::ColorConvertFloat4ToU32(healthBarColor(pct));
}

/// An item's durability, coloured the same way in the bag and in the tooltip.
///
/// Two copies with the same thresholds and different greens, so the strip under
/// an icon and the line in its own tooltip disagreed about the same item. The
/// strip is drawn semi-transparent, which is why the alpha is the caller's.
inline ImVec4 durabilityColor(float pct) {
    if (pct > 0.5f) return {0.1f, 1.0f, 0.1f, 1.0f};
    if (pct > 0.25f) return {1.0f, 1.0f, 0.0f, 1.0f};
    return kBrightRed;
}

// ---- Power bar colour ----
//
// Five places mapped a power type to its bar colour and none of them covered
// the same set. The pet frame stopped at Energy, so a hunter pet - the one unit
// whose power is Happiness - drew a mana-blue bar for it. The focus frame had
// no Focus and no Happiness either.
//
// `fallback` is what an unrecognised type gets. Mana blue everywhere except the
// raid list, which greys a power it does not know rather than claiming it is
// mana.
inline ImVec4 powerTypeColor(uint8_t powerType, ImVec4 fallback = kManaBlue) {
    switch (powerType) {
        case 0: return kManaBlue;
        case 1: return kDarkRed;            // rage
        case 2: return kOrange;             // focus
        case 3: return kEnergyYellow;       // energy
        case 4: return kHappinessGreen;     // happiness - a hunter pet's
        case 6: return kRunicRed;           // runic power
        case 7: return kSoulShardPurple;    // soul shards
        default: return fallback;
    }
}

} // namespace colors

// ---- Item quality colors ----
inline ImVec4 getQualityColor(game::ItemQuality quality) {
    switch (quality) {
        case game::ItemQuality::POOR:      return {0.62f, 0.62f, 0.62f, 1.0f};
        case game::ItemQuality::COMMON:    return {1.0f, 1.0f, 1.0f, 1.0f};
        case game::ItemQuality::UNCOMMON:  return {0.12f, 1.0f, 0.0f, 1.0f};
        case game::ItemQuality::RARE:      return {0.0f, 0.44f, 0.87f, 1.0f};
        case game::ItemQuality::EPIC:      return {0.64f, 0.21f, 0.93f, 1.0f};
        case game::ItemQuality::LEGENDARY: return {1.0f, 0.50f, 0.0f, 1.0f};
        case game::ItemQuality::ARTIFACT:  return {0.90f, 0.80f, 0.50f, 1.0f};
        case game::ItemQuality::HEIRLOOM:  return {0.90f, 0.80f, 0.50f, 1.0f};
        default:                           return {1.0f, 1.0f, 1.0f, 1.0f};
    }
}

// ---- Battleground team colours ----

/// The two team colours a battleground draws its players in.
///
/// The heads-up display and the minimap both plot the same players, and a flag
/// carrier being blue on one and red on the other is worse than either colour
/// alone. Both files carried the pair, under a comment in one of them saying
/// it was the same pair as the other.
inline ImU32 bgGroupColor(uint32_t group) {
    static const ImU32 kByGroup[2] = {
        IM_COL32( 80, 180, 255, 240),   // group 0
        IM_COL32(220,  50,  50, 240),   // group 1
    };
    return kByGroup[group & 1];
}

// ---- Coin display (gold/silver/copper) ----
inline void renderCoinsText(uint32_t g, uint32_t s, uint32_t c) {
    bool any = false;
    if (g > 0) {
        ImGui::TextColored(colors::kGold, "%ug", g);
        any = true;
    }
    if (s > 0 || g > 0) {
        if (any) ImGui::SameLine(0, 3);
        ImGui::TextColored(colors::kSilver, "%us", s);
        any = true;
    }
    if (any) ImGui::SameLine(0, 3);
    ImGui::TextColored(colors::kCopper, "%uc", c);
}

// Convenience overload: decompose copper amount and render as gold/silver/copper
inline void renderCoinsFromCopper(uint64_t copper) {
    const auto coins = game::splitCopper(copper);
    renderCoinsText(coins.gold, coins.silver, coins.copper);
}

// ---- Inventory slot name from WoW inventory type ----
inline const char* getInventorySlotName(uint32_t inventoryType) {
    switch (inventoryType) {
        case 1:  return "Head";
        case 2:  return "Neck";
        case 3:  return "Shoulder";
        case 4:  return "Shirt";
        case 5:  return "Chest";
        case 6:  return "Waist";
        case 7:  return "Legs";
        case 8:  return "Feet";
        case 9:  return "Wrist";
        case 10: return "Hands";
        case 11: return "Finger";
        case 12: return "Trinket";
        case 13: return "One-Hand";
        case 14: return "Shield";
        case 15: return "Ranged";
        case 16: return "Back";
        case 17: return "Two-Hand";
        case 18: return "Bag";
        case 19: return "Tabard";
        case 20: return "Robe";
        case 21: return "Main Hand";
        case 22: return "Off Hand";
        case 23: return "Held In Off-hand";
        case 25: return "Thrown";
        case 26: return "Ranged";
        case 28: return "Relic";
        default: return "";
    }
}

// ---- Aura border colours ----
//
// One colour per dispel type, and green for anything that is a buff. Five
// places drew this border and four agreed; the raid panel's copy was a shade
// off on all four types - brighter magic, deeper curse, lighter disease and
// poison - so the same debuff was one colour on a unit frame and another in the
// raid list.
//
// The four that agreed are the ones kept, on the same grounds as any other
// majority: they are what the game has been showing.
inline ImVec4 dispelTypeColor(uint8_t dispelType) {
    switch (dispelType) {
        case 1:  return ImVec4(0.15f, 0.50f, 1.00f, 0.9f);  // magic: blue
        case 2:  return ImVec4(0.70f, 0.20f, 0.90f, 0.9f);  // curse: purple
        case 3:  return ImVec4(0.55f, 0.30f, 0.10f, 0.9f);  // disease: brown
        case 4:  return ImVec4(0.10f, 0.70f, 0.10f, 0.9f);  // poison: green
        default: return ImVec4(0.80f, 0.20f, 0.20f, 0.9f);  // undispellable: red
    }
}

// ---- Binding type display ----
inline void renderBindingType(uint32_t bindType) {
    if (const char* text = game::itemBindText(bindType)) {
        ImGui::TextColored(colors::kTooltipGold, "%s", text);
    }
}

// ---- DBC item-set spell field keys ----
inline constexpr const char* kItemSetItemKeys[10] = {
    "Item0","Item1","Item2","Item3","Item4",
    "Item5","Item6","Item7","Item8","Item9"
};
inline constexpr const char* kItemSetSpellKeys[10] = {
    "Spell0","Spell1","Spell2","Spell3","Spell4",
    "Spell5","Spell6","Spell7","Spell8","Spell9"
};
inline constexpr const char* kItemSetThresholdKeys[10] = {
    "Threshold0","Threshold1","Threshold2","Threshold3","Threshold4",
    "Threshold5","Threshold6","Threshold7","Threshold8","Threshold9"
};

// ---- Socket type display (gem sockets) ----
struct SocketTypeDef { uint32_t mask; const char* label; ImVec4 col; };
inline constexpr SocketTypeDef kSocketTypes[] = {
    { .mask = 1, .label = "Meta Socket",   .col = { 0.7f, 0.7f, 0.9f, 1.0f } },
    { .mask = 2, .label = "Red Socket",    .col = { 1.0f, 0.3f, 0.3f, 1.0f } },
    { .mask = 4, .label = "Yellow Socket", .col = { 1.0f, 0.9f, 0.3f, 1.0f } },
    { .mask = 8, .label = "Blue Socket",   .col = { 0.3f, 0.6f, 1.0f, 1.0f } },
};

// ---- Class/race bitmask lookup (for allowableClass/allowableRace display) ----
struct ClassMaskEntry { uint32_t mask; const char* name; };
inline constexpr ClassMaskEntry kClassMasks[] = {
    {.mask = 1,.name = "Warrior"}, {.mask = 2,.name = "Paladin"}, {.mask = 4,.name = "Hunter"}, {.mask = 8,.name = "Rogue"},
    {.mask = 16,.name = "Priest"}, {.mask = 32,.name = "Death Knight"}, {.mask = 64,.name = "Shaman"},
    {.mask = 128,.name = "Mage"}, {.mask = 256,.name = "Warlock"}, {.mask = 1024,.name = "Druid"},
};

struct RaceMaskEntry { uint32_t mask; const char* name; };
inline constexpr RaceMaskEntry kRaceMasks[] = {
    {.mask = 1,.name = "Human"}, {.mask = 2,.name = "Orc"}, {.mask = 4,.name = "Dwarf"}, {.mask = 8,.name = "Night Elf"},
    {.mask = 16,.name = "Undead"}, {.mask = 32,.name = "Tauren"}, {.mask = 64,.name = "Gnome"}, {.mask = 128,.name = "Troll"},
    {.mask = 512,.name = "Blood Elf"}, {.mask = 1024,.name = "Draenei"},
};

// ---- WoW class colors (Blizzard canonical) ----
inline ImVec4 getClassColor(uint8_t classId) {
    switch (classId) {
        case 1:  return {0.78f, 0.61f, 0.43f, 1.0f}; // Warrior  #C79C6E
        case 2:  return {0.96f, 0.55f, 0.73f, 1.0f}; // Paladin  #F58CBA
        case 3:  return {0.67f, 0.83f, 0.45f, 1.0f}; // Hunter   #ABD473
        case 4:  return {1.00f, 0.96f, 0.41f, 1.0f}; // Rogue    #FFF569
        case 5:  return {1.00f, 1.00f, 1.00f, 1.0f}; // Priest   #FFFFFF
        case 6:  return {0.77f, 0.12f, 0.23f, 1.0f}; // DK       #C41F3B
        case 7:  return {0.00f, 0.44f, 0.87f, 1.0f}; // Shaman   #0070DE
        case 8:  return {0.41f, 0.80f, 0.94f, 1.0f}; // Mage     #69CCF0
        case 9:  return {0.58f, 0.51f, 0.79f, 1.0f}; // Warlock  #9482C9
        case 11: return {1.00f, 0.49f, 0.04f, 1.0f}; // Druid    #FF7D0A
        default: return colors::kVeryLightGray;
    }
}

inline ImU32 getClassColorU32(uint8_t classId, int alpha = 255) {
    // ColorConvertFloat4ToU32 rounds to the nearest byte where a cast to int
    // truncates, and the panels that draw a name as an ImVec4 get the rounded
    // one. Truncating here made the same class one step darker on the minimap
    // and the nameplates than in the party list beside them - the druid orange
    // came out 7c where the rest of the client draws 7d.
    const ImVec4 c = getClassColor(classId);
    return ImGui::ColorConvertFloat4ToU32(
        ImVec4(c.x, c.y, c.z, static_cast<float>(alpha) / 255.0f));
}

} // namespace wowee::ui
