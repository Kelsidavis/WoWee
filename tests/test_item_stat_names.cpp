// Which stat types each of the two name tables answers for.
//
// There are two on purpose and the difference is the whole point. A query
// response carries Strength through Spirit in fields of their own, and the
// tooltips print them from there - so itemStatName, which names the *extra*
// stats a response lists beside those fields, has to stay silent about the
// five primaries or every one of them appears twice.
//
// A random suffix has no such fields. It rolls raw ITEM_MOD types, so it needs
// the table that does name them, and it was reading the other one: "of the
// Boar" is Strength and Spirit and "of the Tiger" is Agility and Strength, so
// the animal suffixes came out with a name and no stats under it while "of
// Spell Power" and the rating suffixes listed theirs.
#include <catch_amalgamated.hpp>

#include "game/item_text.hpp"

using wowee::game::itemModStatName;
using wowee::game::itemStatName;

namespace {
// ITEM_MOD_*, as the wire and SpellItemEnchantment number them.
constexpr uint32_t kMana = 0, kHealth = 1, kAgility = 3, kStrength = 4;
constexpr uint32_t kIntellect = 5, kSpirit = 6, kStamina = 7;
constexpr uint32_t kCritRating = 32, kSpellPower = 45;
}  // namespace

TEST_CASE("itemStatName leaves the five primaries unnamed") {
    // Not an oversight to be tidied up: the extra-stat lists that call this
    // walk a response whose primaries are already printed from their own
    // fields.
    CHECK(itemStatName(kAgility) == nullptr);
    CHECK(itemStatName(kStrength) == nullptr);
    CHECK(itemStatName(kIntellect) == nullptr);
    CHECK(itemStatName(kSpirit) == nullptr);
    CHECK(itemStatName(kStamina) == nullptr);
}

TEST_CASE("itemModStatName names the five primaries") {
    CHECK(std::string(itemModStatName(kAgility)) == "Agility");
    CHECK(std::string(itemModStatName(kStrength)) == "Strength");
    CHECK(std::string(itemModStatName(kIntellect)) == "Intellect");
    CHECK(std::string(itemModStatName(kSpirit)) == "Spirit");
    CHECK(std::string(itemModStatName(kStamina)) == "Stamina");
}

TEST_CASE("itemModStatName answers the extra stats the same way") {
    // The primaries are the only difference between the two tables; everything
    // else has to agree, or a suffix would name a rating differently from the
    // item that carries one.
    for (uint32_t t = 0; t < 64; ++t) {
        if (t == kAgility || t == kStrength || t == kIntellect ||
            t == kSpirit || t == kStamina) {
            continue;
        }
        CHECK(itemModStatName(t) == itemStatName(t));
    }
    CHECK(std::string(itemModStatName(kMana)) == "Mana");
    CHECK(std::string(itemModStatName(kHealth)) == "Health");
    CHECK(std::string(itemModStatName(kCritRating)) == "Crit Rating");
    CHECK(std::string(itemModStatName(kSpellPower)) == "Spell Power");
}
