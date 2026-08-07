#include <catch_amalgamated.hpp>

#include "game/spell_classification.hpp"
#include "game/spell_defines.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>

using namespace wowee::game::spellclass;

namespace {

// Real values from Spell.dbc / SpellRange.dbc (WotLK 3.3.5a).
constexpr float kSelfOnly     = 0.0f;   // "Self Only"    — Battle Shout, Hearthstone
constexpr float kCombatRange  = 5.0f;   // "Combat Range" — every melee ability
constexpr float kDeadlyThrow  = 30.0f;  // physical, but thrown
constexpr float kSteadyShot   = 35.0f;  // physical, but shot
constexpr float kFireball     = 35.0f;

// Stand-in for the spell name cache.
class FakeSpellDbc {
public:
    void add(uint32_t id, std::string name, std::string rank) {
        entries_[id] = SpellRankInfo{std::move(name), std::move(rank)};
    }
    std::function<const SpellRankInfo*(uint32_t)> lookup() const {
        return [this](uint32_t id) -> const SpellRankInfo* {
            auto it = entries_.find(id);
            return it != entries_.end() ? &it->second : nullptr;
        };
    }
private:
    std::unordered_map<uint32_t, SpellRankInfo> entries_;
};

} // namespace

TEST_CASE("DONT_REPORT cast failures remain player-facing", "[spell][failure]") {
    REQUIRE(std::string(wowee::game::getSpellCastResultString(27)) ==
            "You can't do that right now");
}

// ---------------------------------------------------------------------------
// Range classification
// ---------------------------------------------------------------------------

TEST_CASE("Self-only spells are cast on the caster", "[spell][range]") {
    REQUIRE(isSelfCastRange(kSelfOnly));

    // Anything that can reach another unit is not a self-cast.
    REQUIRE_FALSE(isSelfCastRange(kCombatRange));
    REQUIRE_FALSE(isSelfCastRange(kSteadyShot));

    // Unknown range: SpellRange.dbc was missing, so assume nothing.
    REQUIRE_FALSE(isSelfCastRange(kUnknownRange));
}

TEST_CASE("Melee is decided by range, not by school", "[spell][range]") {
    // Melee abilities all sit at Combat Range.
    REQUIRE(isMeleeRange(kCombatRange));
    REQUIRE(isMeleeRange(4.0f));

    // Regression: Steady Shot, Aimed Shot, Multi-Shot, Taunt and Deadly Throw are all
    // physical school. Classifying by school made the client range-check them at 8
    // yards and swallow the cast, so Hunters could not shoot past 8 yards.
    REQUIRE_FALSE(isMeleeRange(kSteadyShot));
    REQUIRE_FALSE(isMeleeRange(kDeadlyThrow));
    REQUIRE_FALSE(isMeleeRange(kFireball));

    // Regression: Battle Shout is a physical-school self-buff. Read as a melee ability,
    // it was blocked by the range check whenever a distant target was selected.
    REQUIRE_FALSE(isMeleeRange(kSelfOnly));

    // Unknown range must not block a cast: let the server arbitrate.
    REQUIRE_FALSE(isMeleeRange(kUnknownRange));
}

TEST_CASE("A spell is never both self-cast and melee", "[spell][range]") {
    for (float r : {kUnknownRange, kSelfOnly, 1.0f, kCombatRange, 8.0f, kSteadyShot}) {
        const bool both = isSelfCastRange(r) && isMeleeRange(r);
        REQUIRE_FALSE(both);
    }
}

TEST_CASE("Ranged weapon auto-attacks ignore placeholder DBC resource costs",
          "[spell][ranged-auto]") {
    REQUIRE(isRangedWeaponAutoAttack(75));    // Auto Shot
    REQUIRE(isRangedWeaponAutoAttack(5019));  // Shoot
    REQUIRE(isRangedWeaponAutoAttack(2764));  // Throw

    REQUIRE_FALSE(isRangedWeaponAutoAttack(0));
    REQUIRE_FALSE(isRangedWeaponAutoAttack(19434)); // Aimed Shot still uses real costs
}

TEST_CASE("Fishing ranks use targetless server-placed casts", "[spell][fishing]") {
    for (uint32_t spellId : {7620u, 7731u, 7732u, 18248u, 33095u, 51294u}) {
        REQUIRE(isFishingCast(spellId));
    }

    REQUIRE_FALSE(isFishingCast(0));
    REQUIRE_FALSE(isFishingCast(2366));  // Herbalism
    REQUIRE_FALSE(isFishingCast(2575));  // Mining
}

TEST_CASE("Only restoration food and drink spells use the seated consumption loop",
          "[spell][consumable]") {
    REQUIRE(classifyRestChannel("Food") == RestChannelKind::FOOD);
    REQUIRE(classifyRestChannel("Drink") == RestChannelKind::DRINK);

    // Potions use the same drinking motion, but unrelated quest-item actions do not.
    REQUIRE(classifyRestChannel("Drink Potion") == RestChannelKind::POTION);
    REQUIRE(classifyRestChannel("Drink Minor Potion") == RestChannelKind::POTION);
    REQUIRE(classifyRestChannel("Drink Eye Potion") == RestChannelKind::POTION);
    REQUIRE(classifyRestChannel("Healing Potion") == RestChannelKind::POTION);
    REQUIRE(classifyRestChannel("Minor Mana Potion") == RestChannelKind::POTION);
    REQUIRE(classifyRestChannel("Swiftness Potion") == RestChannelKind::POTION);
    REQUIRE(classifyRestChannel("Create Fervor Potion") == RestChannelKind::NONE);
    REQUIRE(classifyRestChannel("Potion Toss") == RestChannelKind::NONE);
    REQUIRE(classifyRestChannel("Drink Disease Bottle") == RestChannelKind::NONE);
    REQUIRE(classifyRestChannel("Food (TEST)") == RestChannelKind::NONE);
    REQUIRE(classifyRestChannel("") == RestChannelKind::NONE);

    // The undead racial is the same kind of thing and neither of the two
    // shapes above: it stays standing, so the seated food path is wrong for
    // it, and it channels, so the single swig a potion gets is wrong too. It
    // has its own kind so the call site can tell all three apart.
    REQUIRE(classifyRestChannel("Cannibalize") == RestChannelKind::CANNIBALIZE);
    // And nothing else drifts into it on a prefix.
    REQUIRE(classifyRestChannel("Cannibalize Corpse") == RestChannelKind::NONE);
    REQUIRE(classifyRestChannel("Cannibal") == RestChannelKind::NONE);

    const uint32_t alcoholEffects[3] = {0, 100, 0};
    const uint32_t ordinaryEffects[3] = {6, 10, 0};
    REQUIRE(hasInebriateEffect(alcoholEffects, 3));
    REQUIRE_FALSE(hasInebriateEffect(ordinaryEffects, 3));
}

// ---------------------------------------------------------------------------
// Rank strings
// ---------------------------------------------------------------------------

TEST_CASE("Rank display strings parse to their number", "[spell][rank]") {
    REQUIRE(rankValue("Rank 1") == 1);
    REQUIRE(rankValue("Rank 9") == 9);
    REQUIRE(rankValue("Rank 14") == 14);

    // Rankless spells sort below every ranked one.
    REQUIRE(rankValue("") == 0);
    REQUIRE(rankValue("Racial") == 0);
}

// ---------------------------------------------------------------------------
// Superseded rank resolution
// ---------------------------------------------------------------------------

TEST_CASE("A superseded rank resolves to the highest known rank", "[spell][rank]") {
    FakeSpellDbc dbc;
    dbc.add(6673,  "Battle Shout", "Rank 1");
    dbc.add(5242,  "Battle Shout", "Rank 2");
    dbc.add(6192,  "Battle Shout", "Rank 3");
    dbc.add(11549, "Battle Shout", "Rank 4");

    // The player learned ranks 2 and 3; rank 1 is superseded and no longer active, so
    // the server silently discards casts of it. An action bar restored from the server
    // can still be holding rank 1.
    const std::unordered_set<uint32_t> known{5242, 6192};

    REQUIRE(resolveHighestKnownRank(6673, known, dbc.lookup()) == 6192);

    // Rank 4 is not known, so it must not be picked.
    REQUIRE(resolveHighestKnownRank(11549, known, dbc.lookup()) != 11549);
    REQUIRE(resolveHighestKnownRank(11549, known, dbc.lookup()) == 6192);
}

TEST_CASE("Rank resolution tolerates a lookup adapter with reused storage", "[spell][rank]") {
    const std::unordered_map<uint32_t, SpellRankInfo> entries{
        {6673, {"Battle Shout", "Rank 1"}},
        {5242, {"Battle Shout", "Rank 2"}},
        {6192, {"Battle Shout", "Rank 3"}},
        {133,  {"Fireball", "Rank 14"}},
    };
    SpellRankInfo scratch;
    auto reusedLookup = [&entries, &scratch](uint32_t id) -> const SpellRankInfo* {
        auto it = entries.find(id);
        if (it == entries.end()) return nullptr;
        scratch = it->second;
        return &scratch;
    };

    // The production Spell.dbc adapter reuses scratch storage. Looking up Fireball
    // must not overwrite the requested Battle Shout name and win merely by rank.
    const std::unordered_set<uint32_t> known{5242, 6192, 133};
    REQUIRE(resolveHighestKnownRank(6673, known, reusedLookup) == 6192);
}

TEST_CASE("A known spell is left alone", "[spell][rank]") {
    FakeSpellDbc dbc;
    dbc.add(772, "Rend", "Rank 1");
    dbc.add(6546, "Rend", "Rank 2");

    const std::unordered_set<uint32_t> known{772};

    // Rank 1 is still the active rank — casting it must not be rewritten.
    REQUIRE(resolveHighestKnownRank(772, known, dbc.lookup()) == 772);
}

TEST_CASE("Spells with no known sibling pass through unchanged", "[spell][rank]") {
    FakeSpellDbc dbc;
    dbc.add(8690, "Hearthstone", "");
    dbc.add(133,  "Fireball",    "Rank 1");
    dbc.add(6673, "Battle Shout", "Rank 1");
    dbc.add(5242, "Battle Shout", "Rank 2");

    const std::unordered_set<uint32_t> known{5242};

    // Item spells are absent from the known-spell list but have no known same-name
    // sibling, so they must be left alone rather than rewritten to something else.
    REQUIRE(resolveHighestKnownRank(8690, known, dbc.lookup()) == 8690);

    // A spell belonging to another class must not be rewritten either.
    REQUIRE(resolveHighestKnownRank(133, known, dbc.lookup()) == 133);
}

TEST_CASE("Rank resolution copes with missing data", "[spell][rank]") {
    FakeSpellDbc dbc;
    dbc.add(5242, "Battle Shout", "Rank 2");
    const std::unordered_set<uint32_t> known{5242};

    // Spell id 0 is "no spell".
    REQUIRE(resolveHighestKnownRank(0, known, dbc.lookup()) == 0);

    // A spell absent from Spell.dbc entirely cannot be resolved, so pass it through.
    REQUIRE(resolveHighestKnownRank(999999, known, dbc.lookup()) == 999999);

    // Nothing known at all: pass through rather than resolving to 0.
    REQUIRE(resolveHighestKnownRank(6673, {}, dbc.lookup()) == 6673);
}

// A heal and a nuke can share an effect id and a school, so neither tells them
// apart. EffectImplicitTargetA does: it is what the spell expects to be aimed
// at. These values are read from the shipped Spell.dbc.
TEST_CASE("friendly-target spells are the ones that may fall back to the caster",
          "[spell][classification]") {
    using namespace wowee::game::spellclass;

    SECTION("heals and buffs need a friendly unit") {
        CHECK(requiresFriendlyTarget(kImplicitTargetAlly));   // Flash Heal, Renew,
        CHECK(requiresFriendlyTarget(21));                    // Mark of the Wild...
    }

    SECTION("damage does not") {
        CHECK_FALSE(requiresFriendlyTarget(kImplicitTargetEnemy));  // Smite, Fireball
        CHECK_FALSE(requiresFriendlyTarget(6));
    }

    SECTION("self-only spells are handled by range, not by this") {
        // Inner Fire and Blink read 1; they never carry a target either way, so
        // they must not be dragged through the friendly-target path.
        CHECK_FALSE(requiresFriendlyTarget(kImplicitTargetCaster));
    }

    SECTION("spells that take either target are left to the player") {
        // Dispel Magic reads 25. Retail does not self-cast it, because picking
        // between cleansing yourself and purging an enemy is guessing.
        CHECK_FALSE(requiresFriendlyTarget(kImplicitTargetAny));
    }

    SECTION("an unknown spell reads 0 and never self-casts") {
        CHECK_FALSE(requiresFriendlyTarget(0));
    }
}
