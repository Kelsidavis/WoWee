#pragma once

#include <cstdint>

namespace wowee {
namespace rendering {

/// Ranged weapon type for animation selection.
///
/// A wand is separate from a gun: INVTYPE_RANGEDRIGHT carries guns, crossbows
/// and wands alike, so the inventory type alone gives a wand the rifle pose.
enum class RangedWeaponType : uint8_t { NONE = 0, BOW, GUN, CROSSBOW, THROWN, WAND };

// ============================================================================
// WeaponLoadout - extracted from AnimationController
//
// Consolidates the 6 weapon boolean fields + inventory type + ranged type
// into a single value type.
// ============================================================================
struct WeaponLoadout {
    uint32_t inventoryType = 0;
    bool is2HLoose  = false;  // Polearm or staff
    bool isFist     = false;  // Fist weapon
    bool isDagger   = false;  // Dagger (uses pierce variants)
    bool hasOffHand = false;  // Has off-hand weapon (dual wield)
    // The off-hand weapon's own kind. isFist and isDagger above are the main
    // hand's, and the off-hand swing was picked from those - so a sword and a
    // dagger swung the dagger's pierce animation with the sword, and the
    // sword's with the dagger, whichever way round they were equipped.
    bool offHandIsFist   = false;
    bool offHandIsDagger = false;
    bool hasShield  = false;  // Has shield equipped (for SHIELD_BASH)
    RangedWeaponType rangedType = RangedWeaponType::NONE;
};

} // namespace rendering
} // namespace wowee
