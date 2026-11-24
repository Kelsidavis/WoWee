#pragma once

#include <imgui.h>
#include <cstdint>
#include <cstdio>
#include "ui/ui_colors.hpp"
#include "game/entity.hpp"
#include "game/update_field_table.hpp"
#include "game/character.hpp"
#include "game/combat_handler.hpp"

namespace wowee::ui::helpers {

// ---- Level difficulty ----

/// The colour WoW draws a hostile mob's level in: grey once it stops giving
/// experience, then green, yellow, orange and red as it gets harder.
///
/// Written out three times - the target frame's name, the focus frame's name,
/// and the selection circle on the ground - and the circle's copy was missing
/// the rule for an unlevelled mob. A raid boss reports level 0, so the
/// subtraction gave it a difference of minus the player's level and it came out
/// green: the one mob in the game that should read as unkillable drew the circle
/// that means trivial, while its name two inches away was skull red.
inline ImVec4 levelDifficultyColor(uint32_t playerLevel, uint32_t mobLevel) {
    // Level 0 is "??" - a boss, or a unit whose level has not arrived yet.
    if (mobLevel == 0) return colors::kSkullRed;
    if (game::CombatHandler::killXp(playerLevel, mobLevel) == 0) return colors::kGray;

    const int32_t diff = static_cast<int32_t>(mobLevel) - static_cast<int32_t>(playerLevel);
    if (diff >= 10) return colors::kSkullRed;
    if (diff >= 5) return colors::kDifficultOrange;
    if (diff >= -2) return colors::kEvenYellow;
    return colors::kBrightGreen;
}

// ---- Class color / name helpers ----

inline ImVec4 classColorVec4(uint8_t classId) { return getClassColor(classId); }
inline ImU32 classColorU32(uint8_t classId, int alpha = 255) { return getClassColorU32(classId, alpha); }

inline const char* classNameStr(uint8_t classId) {
    return game::getClassName(static_cast<game::Class>(classId));
}

// Extract class id from a unit's UNIT_FIELD_BYTES_0 update field.
// Returns 0 if the entity pointer is null or field is unset.
inline uint8_t entityClassId(const game::Entity* entity) {
    if (!entity) return 0;
    using UF = game::UF;
    uint32_t bytes0 = entity->getField(game::fieldIndex(UF::UNIT_FIELD_BYTES_0));
    return static_cast<uint8_t>((bytes0 >> 8) & 0xFF);
}

// ---- Shared UI data tables ----

// Aura dispel-type names (indexed by dispelType 0-4)
inline constexpr const char* kDispelNames[] = { "", "Magic", "Curse", "Disease", "Poison" };

// Raid mark names with symbol prefixes (indexed 0-7: Star..Skull)
inline constexpr const char* kRaidMarkNames[] = {
    "{*} Star", "{O} Circle", "{<>} Diamond", "{^} Triangle",
    "{)} Moon", "{ } Square", "{x} Cross", "{8} Skull"
};

} // namespace wowee::ui::helpers
