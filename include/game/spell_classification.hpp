#pragma once

// Pure spell-classification rules, kept free of GameHandler so they can be tested
// directly. Every decision here is driven by Spell.dbc / SpellRange.dbc data rather
// than by hardcoded spell ids: classifying by school instead once cost us Battle Shout
// (a physical-school self-buff read as melee) and every Hunter shot past 8 yards.

#include <cstdint>
#include <cstddef>
#include <functional>
#include <string>
#include <unordered_set>

namespace wowee {
namespace game {
namespace spellclass {

/// SpellRange calls melee "Combat Range", and every melee ability resolves to it.
inline constexpr float kCombatRangeYards = 5.0f;

/// Returned when SpellRange.dbc was unavailable - nothing may be inferred from it.
inline constexpr float kUnknownRange = -1.0f;

/// "Self Only" range: the spell cannot reach another unit, so it lands on the caster
/// no matter what is targeted (shouts, self-buffs, hearthstone).
inline bool isSelfCastRange(float maxRange) {
    return maxRange == 0.0f;
}

/// A melee ability, i.e. one cast at Combat Range. Ranged physical abilities such as
/// Steady Shot or Taunt carry 30-35 yards and are deliberately excluded. An unknown
/// range is not melee: the server should arbitrate rather than the client blocking a
/// legitimate cast.
inline bool isMeleeRange(float maxRange) {
    return maxRange > 0.0f && maxRange <= kCombatRangeYards;
}

/// Spell.dbc EffectImplicitTargetA values the client reasons about.
///
/// The column says what a spell expects to be aimed at, which is the only
/// honest way to tell a heal or a buff from a nuke: both are APPLY_AURA, and
/// both can share a school. Verified against the shipped data - Flash Heal,
/// Rejuvenation, Mark of the Wild, Arcane Intellect and Blessing of Might all
/// read 21, while Smite, Fireball, Shadow Bolt and Shadow Word: Pain read 6.
enum ImplicitTarget : uint32_t {
    kImplicitTargetCaster    = 1,   ///< Lands on the caster whatever is selected.
    kImplicitTargetEnemy     = 6,   ///< Needs a hostile unit.
    kImplicitTargetAlly      = 21,  ///< Needs a friendly unit - heals and buffs.
    kImplicitTargetAny       = 25,  ///< Either, e.g. Dispel Magic.
    kImplicitTargetParty     = 35,  ///< A party member.
    kImplicitTargetChainHeal = 45,  ///< A friendly unit the heal jumps from.
    kImplicitTargetRaid      = 57,  ///< A raid member.
};

/// Whether a spell has to be aimed at a friendly unit, which is what makes it a
/// candidate for falling back to the caster when nothing friendly is selected.
///
/// Four values, measured over the shipped Spell.dbc: Holy Light and Healing
/// Wave are 45, Hand of Protection and Beacon of Light 57.
///
/// Not 25, which takes either target (Dispel Magic, Holy Shock) - retail does
/// not self-cast those. Not 63, which is a destination rather than a unit and
/// carries Fire Bomb and Rain of Darkness as well as Circle of Healing.
inline bool requiresFriendlyTarget(uint32_t implicitTargetA) {
    return implicitTargetA == kImplicitTargetAlly ||
           implicitTargetA == kImplicitTargetParty ||
           implicitTargetA == kImplicitTargetChainHeal ||
           implicitTargetA == kImplicitTargetRaid;
}

/// Whether a spell has to be aimed at a hostile unit, so that casting it with
/// nothing selected is an error rather than something to guess a target for.
///
/// There is no safe guess. An empty SpellCastTargets is TARGET_FLAG_SELF on the
/// wire, and the server reads that as "the caster is the target" - so a shot
/// sent with no target is a shot the server aims at the hunter who fired it.
inline bool requiresHostileTarget(uint32_t implicitTargetA) {
    return implicitTargetA == kImplicitTargetEnemy;
}

/// Legacy client IDs for the three repeating ranged weapon attacks.
///
/// Their Spell.dbc rows carry a dummy one-point rage cost even for classes that do
/// not use rage. The server drives their real weapon/ammunition readiness, so UI
/// resource checks must not treat that placeholder as a cast cost.
inline bool isRangedWeaponAutoAttack(uint32_t spellId) {
    return spellId == 75 ||    // Auto Shot
           spellId == 5019 ||  // Shoot (wand)
           spellId == 2764;    // Throw
}

/// The learnable Fishing casts across Classic, TBC, and WotLK.
///
/// Fishing is unusual: it has no explicit unit target. The server chooses a point in
/// a narrow cone in front of the caster and validates water depth there. Keeping this
/// classification explicit prevents a selected creature from leaking into the cast
/// packet and lets the client select the pole-specific animation sequence.
inline bool isFishingCast(uint32_t spellId) {
    return spellId == 7620 ||   // Fishing (Apprentice)
           spellId == 7731 ||   // Fishing (Journeyman)
           spellId == 7732 ||   // Fishing (Expert)
           spellId == 18248 ||  // Fishing (Artisan)
           spellId == 33095 ||  // Fishing (Master)
           spellId == 51294;    // Fishing (Grand Master)
}

/// Restoration food/water and instant potions use the dedicated consumption
/// animation. Match canonical Food/Drink names and actual potion-use suffixes
/// without classifying recipe spells that create potions.
enum class RestChannelKind {
    NONE,
    FOOD,
    DRINK,
    POTION,
    ALCOHOL,
    /// The undead racial. Grouped here because it is the same kind of thing -
    /// a channel whose whole visible content is the eating animation - but it
    /// is neither seated nor a single swig: the character stays standing over
    /// the corpse and eats for the length of the channel.
    CANNIBALIZE
};

inline RestChannelKind classifyRestChannel(const std::string& spellName) {
    if (spellName == "Food") return RestChannelKind::FOOD;
    if (spellName == "Drink") return RestChannelKind::DRINK;
    if (spellName == "Cannibalize") return RestChannelKind::CANNIBALIZE;
    constexpr const char* potionSuffix = "Potion";
    if (spellName.size() >= 6 &&
        spellName.compare(spellName.size() - 6, 6, potionSuffix) == 0 &&
        spellName.rfind("Create ", 0) != 0) {
        return RestChannelKind::POTION;
    }
    return RestChannelKind::NONE;
}

inline bool hasInebriateEffect(const uint32_t* effectIds, size_t count) {
    constexpr uint32_t kSpellEffectInebriate = 100;
    if (!effectIds) return false;
    for (size_t i = 0; i < count; ++i) {
        if (effectIds[i] == kSpellEffectInebriate) return true;
    }
    return false;
}

/// Whether a spell takes apart the item it is cast on rather than changing it.
///
/// Disenchant, prospecting and milling give materials back and consume what
/// they were aimed at. Nothing is enchanted and nothing is bound, so none of
/// the enchant warnings belongs in front of one - "Enchanting this item will
/// bind it to you" in front of a disenchant asks about binding an item that is
/// about to be dust.
///
/// Effect ids read off the client's own Spell.dbc rather than recalled: 13262
/// Disenchant is 99 in 1.12 and in 3.3.5, 31252 Prospecting is 127 and 51005
/// Milling is 158.
inline bool destroysTargetItem(const uint32_t* effectIds, size_t count) {
    constexpr uint32_t kSpellEffectDisenchant  = 99;
    constexpr uint32_t kSpellEffectProspecting = 127;
    constexpr uint32_t kSpellEffectMilling     = 158;
    if (!effectIds) return false;
    for (size_t i = 0; i < count; ++i) {
        if (effectIds[i] == kSpellEffectDisenchant ||
            effectIds[i] == kSpellEffectProspecting ||
            effectIds[i] == kSpellEffectMilling) {
            return true;
        }
    }
    return false;
}

/// Disenchant in particular, which is the one this client asks about first.
inline bool hasDisenchantEffect(const uint32_t* effectIds, size_t count) {
    constexpr uint32_t kSpellEffectDisenchant = 99;
    if (!effectIds) return false;
    for (size_t i = 0; i < count; ++i) {
        if (effectIds[i] == kSpellEffectDisenchant) return true;
    }
    return false;
}

/// Spell.dbc stores the rank as a display string ("Rank 3"). Rankless spells sort as 0.
inline int rankValue(const std::string& rank) {
    int value = 0;
    bool sawDigit = false;
    for (char c : rank) {
        if (c >= '0' && c <= '9') {
            sawDigit = true;
            value = value * 10 + (c - '0');
        } else if (sawDigit) {
            break;  // stop at the first non-digit after the number
        }
    }
    return value;
}

/// Name and rank as they appear in Spell.dbc.
struct SpellRankInfo {
    std::string name;
    std::string rank;
};

/// Maps a superseded spell rank onto the highest rank the player actually knows.
///
/// Action bars restored from the server can still hold a rank that a higher rank has
/// since superseded. The server drops casts of superseded ranks without sending any
/// error at all, so the client has to correct them itself.
///
/// Returns spellId unchanged when it is already known, or when no known spell shares
/// its name (item, gather and vehicle spells are absent from the known list too, and
/// must be left alone).
///
/// `lookup` resolves a spell id to its Spell.dbc name/rank, or null if unknown.
inline uint32_t resolveHighestKnownRank(
        uint32_t spellId,
        const std::unordered_set<uint32_t>& knownSpells,
        const std::function<const SpellRankInfo*(uint32_t)>& lookup) {
    if (spellId == 0 || knownSpells.count(spellId) > 0) return spellId;

    const SpellRankInfo* info = lookup(spellId);
    if (!info || info->name.empty()) return spellId;

    // Keep the requested name by value. Some DBC adapters reuse one scratch object
    // for every lookup; retaining its pointer here lets the first candidate overwrite
    // the requested spell and makes unrelated known spells look like matching ranks.
    const std::string requestedName = info->name;

    uint32_t best = 0;
    int bestRank = -1;
    for (uint32_t known : knownSpells) {
        const SpellRankInfo* candidate = lookup(known);
        if (!candidate || candidate->name != requestedName) continue;
        const int rank = rankValue(candidate->rank);
        if (rank > bestRank) {
            bestRank = rank;
            best = known;
        }
    }
    return best != 0 ? best : spellId;
}

} // namespace spellclass
} // namespace game
} // namespace wowee
