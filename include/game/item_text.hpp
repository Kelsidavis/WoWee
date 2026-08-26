#pragma once

/**
 * item_text.hpp - the words an item's and a spell's numbers are displayed with.
 *
 * Kept apart from world_packets.hpp so a tooltip can ask for a stat's name
 * without pulling in every packet structure in the game. Four things render item
 * tooltips - this client's bag, the chat item link, the one built for FrameXML,
 * and a shim written in Lua - and each of them had its own copy of these tables
 * until the copies were found to disagree.
 */

#include <cstdint>
#include <string>

#include "game/protocol_constants.hpp"

namespace wowee {
namespace game {

/// The word an item's spell line starts with - "Use", "Equip", "Chance on Hit".
///
/// Three tooltips render this and their tables disagreed. The chat item link
/// called trigger 5 "Teaches" where the other two call it "Use", and WoW writes
/// a recipe as "Use: Teaches you how to make..." - the word is Use and the
/// teaching is in the spell's own description. The chat link also had no entry
/// for triggers 4 and 6 and skipped the line entirely, so an item whose spell
/// fires on use with no delay showed no spell at all there.
///
/// Null for a trigger with nothing to say. The values are the same in Classic,
/// TBC and WotLK.
inline const char* itemSpellTriggerText(uint32_t spellTrigger) {
    switch (spellTrigger) {
        case 0: return "Use";            // on use
        case 1: return "Equip";          // on equip
        case 2: return "Chance on Hit";  // proc on melee hit
        case 4: return "Use";            // soulstone, still a use
        case 5: return "Use";            // on use, no delay
        case 6: return "Use";            // learn spell - recipe or pattern
        default: return nullptr;
    }
}

/// What a spell's powerType is called, or null for one with no name.
///
/// Four places wrote this out: the combat log twice, the spellbook, and the
/// tooltip built for FrameXML. The spellbook's copy had Focus at 4, which is
/// Happiness - so a spell costing Focus fell through to "Mana" and one costing
/// Happiness said "Focus".
///
/// 5 is the death knight's runes, which are drawn as runes rather than written
/// as a number, so it has no word here. 5 and 6 exist only from WotLK; naming
/// them costs the earlier expansions nothing, because their servers never send
/// them.
inline const char* powerTypeName(uint32_t powerType) {
    switch (powerType) {
        case 0: return "Mana";
        case 1: return "Rage";
        case 2: return "Focus";
        case 3: return "Energy";
        case 4: return "Happiness";
        case 6: return "Runic Power";
        default: return nullptr;
    }
}

/// What an item's bindType says, or null for one that binds to nobody.
///
/// Four places render this line - this client's own bag tooltip, the chat item
/// link, the tooltip built for FrameXML, and a fourth copy written in Lua inside
/// the tooltip shim. The three in C++ share this now. They did not all agree:
/// two knew that 4 is a quest item and one did not, so a quest item's tooltip
/// said nothing about being one, depending on which tooltip you were looking at.
///
/// The colour stays with each renderer. They genuinely differ - one draws this
/// line gold and one draws it dimmed - and that is a decision about a tooltip,
/// not about what bindType 4 means.
inline const char* itemBindText(uint32_t bindType) {
    switch (bindType) {
        case 1: return "Binds when picked up";
        case 2: return "Binds when equipped";
        case 3: return "Binds when used";
        case 4: return "Quest Item";
        default: return nullptr;
    }
}

/// The name of an ItemQueryResponseData::ExtraStat type, or null for one there
/// is nothing to say about.
///
/// Shared because two tooltips read it - this client's own bag tooltip and the
/// one FrameXML asks for through GameTooltip:SetBagItem - and a second copy of
/// this table would drift the first time a rating was added to one of them.
/// Several ids map to the same words on purpose: 16, 17, 18 and 31 are melee,
/// ranged, spell and generic hit, and WoW writes all four as "Hit Rating".
inline const char* itemStatName(uint32_t statType) {
    switch (statType) {
        case 0:  return "Mana";
        case 1:  return "Health";
        case 12: return "Defense Rating";
        case 13: return "Dodge Rating";
        case 14: return "Parry Rating";
        case 15: return "Block Rating";
        case 16: case 17: case 18: case 31: return "Hit Rating";
        case 19: case 20: case 21: case 32: return "Crit Rating";
        case 28: case 29: case 30: case 36: return "Haste Rating";
        case 35: return "Resilience";
        case 37: return "Expertise Rating";
        case 38: return "Attack Power";
        case 39: return "Ranged Attack Power";
        case 41: return "Healing Power";
        case 42: return "Spell Damage";
        case 43: return "Mana per 5 sec";
        case 44: return "Armor Penetration";
        case 45: return "Spell Power";
        case 46: return "Health per 5 sec";
        case 47: return "Spell Penetration";
        case 48: return "Block Value";
        default: return nullptr;
    }
}

/// The name of any ITEM_MOD stat type, the five primaries included.
///
/// itemStatName answers for the extra stats alone, and leaves Strength through
/// Spirit out on purpose: a query response carries those in fields of their own
/// and the tooltips print them from there, so naming them again would list each
/// one twice.
///
/// A random suffix has no such fields. It rolls raw ITEM_MOD types, and the
/// five it leaves out are exactly what the animal suffixes are made of - "of
/// the Boar" is Strength and Spirit, "of the Tiger" is Agility and Strength -
/// so every one of them came out with a name and no stats at all, while "of
/// Spell Power" and the rating suffixes listed theirs.
inline const char* itemModStatName(uint32_t statType) {
    switch (statType) {
        case 3: return "Agility";
        case 4: return "Strength";
        case 5: return "Intellect";
        case 6: return "Spirit";
        case 7: return "Stamina";
        default: return itemStatName(statType);
    }
}


/// What resistance school `index` is, 0..5, as an item line reads it.
///
/// Holy first and Arcane last, which is the order the six resistance values
/// arrive in and not the order the spell schools are numbered. Written out in
/// the bag tooltip and the chat tooltip both.
inline const char* resistanceSchoolName(uint32_t index) {
    static constexpr const char* kBySchool[] = {
        "Holy Resistance", "Fire Resistance", "Nature Resistance",
        "Frost Resistance", "Shadow Resistance", "Arcane Resistance",
    };
    return index < 6 ? kBySchool[index] : "Resistance";
}

/// An item's quality colour as an eight-digit hex string, alpha first.
///
/// The same eight colours the interface uses, and the form a link's |c escape
/// wants. Two tables carried these - one with the alpha prefix and one without
/// - and they had already disagreed once: heirloom was 00ccff in the second,
/// which is a later expansion's token colour and not a quality 3.3.5 has, so
/// an heirloom link came out cyan.
inline const char* itemQualityColorHex(uint32_t quality) {
    static constexpr const char* kByQuality[] = {
        "ff9d9d9d",  // poor
        "ffffffff",  // common
        "ff1eff00",  // uncommon
        "ff0070dd",  // rare
        "ffa335ee",  // epic
        "ffff8000",  // legendary
        "ffe6cc80",  // artifact
        "ffe6cc80",  // heirloom - the same gold as an artifact
    };
    return quality < 8 ? kByQuality[quality] : "ffffffff";
}

/// The same colour without its alpha, for the callers that write "|cff".
///
/// Half the places that build a link write the alpha themselves and half take
/// it from the string, so the two forms lived as two tables. Deriving one from
/// the other is what keeps them the same colour: the pair that were written
/// out separately disagreed about heirlooms for as long as both existed.
inline const char* itemQualityColorHexRGB(uint32_t quality) {
    return itemQualityColorHex(quality) + 2;
}

/// A chat hyperlink for an item, as 3.3.5a writes one.
///
/// Nine fields after "item:": the id, then enchant, four gems, suffix, unique
/// id and level. Six places built this by hand and they did not agree - the
/// three on the Lua side wrote eight, one short, so a link handed to an addon
/// by GetContainerItemLink had a different shape from one produced by
/// shift-clicking a quest reward.
///
/// Nothing visibly breaks today: this client's own two parsers read the id and
/// stop at the first colon, and FrameXML hands the whole link back to
/// SetHyperlink rather than splitting it. It is a difference waiting for the
/// first thing that does split it.
/// `enchantId` and `randomPropertyId` are the second and seventh fields. They
/// default to none, which is what every caller but the guild bank wants: a
/// link written with an enchant is what a bank slot's contents actually are,
/// and dropping it would show an enchanted weapon as a plain one.
inline std::string itemChatLink(uint32_t itemId, uint32_t quality,
                                const std::string& name,
                                uint32_t enchantId = 0,
                                int32_t randomPropertyId = 0) {
    return std::string("|c") + itemQualityColorHex(quality) + "|Hitem:" +
           std::to_string(itemId) + ":" + std::to_string(enchantId) +
           ":0:0:0:0:" + std::to_string(randomPropertyId) + ":0:0|h[" + name +
           "]|h|r";
}


/// A chat hyperlink for a spell.
///
/// The colour is not a choice: retail writes a spell link in ff71d5ff, and a
/// player who links a spell expects the same blue every other client shows.
/// Four places built one here and produced three different colours - the
/// spellbook's context menu gold, the chat-link catalog white, and only the
/// Lua API the blue. Linking the same spell two ways gave two colours.
inline std::string spellChatLink(uint32_t spellId, const std::string& name) {
    return "|cff71d5ff|Hspell:" + std::to_string(spellId) + "|h[" + name + "]|h|r";
}


/// An amount of copper split into the three coins.
///
/// Twenty-one places did this division themselves, and the constants for it
/// have been in protocol_constants.hpp all along - most wrote 10000 and 100 as
/// literals instead.
struct CoinAmount {
    uint32_t gold = 0;
    uint32_t silver = 0;
    uint32_t copper = 0;
};

inline CoinAmount splitCopper(uint64_t amount) {
    CoinAmount coins;
    coins.gold = static_cast<uint32_t>(amount / COPPER_PER_GOLD);
    coins.silver = static_cast<uint32_t>((amount / COPPER_PER_SILVER) % 100);
    coins.copper = static_cast<uint32_t>(amount % COPPER_PER_SILVER);
    return coins;
}

/// An amount of money as a line of text: "5g 20s 3c", with the parts that are
/// zero left out - except when everything is, which reads "0c" rather than
/// nothing at all.
///
/// Written out twice as a file-scope function, in the game handler and the
/// quest handler, and reached from a third file by forward-declaring it across
/// translation units - which linked only because neither copy was in an
/// anonymous namespace, and would have become two definitions the moment one
/// was.
inline std::string formatCopperAmount(uint32_t amount) {
    const CoinAmount coins = splitCopper(amount);
    std::string out;
    if (coins.gold > 0) out += std::to_string(coins.gold) + "g";
    if (coins.silver > 0) {
        if (!out.empty()) out += " ";
        out += std::to_string(coins.silver) + "s";
    }
    if (coins.copper > 0 || out.empty()) {
        if (!out.empty()) out += " ";
        out += std::to_string(coins.copper) + "c";
    }
    return out;
}

/// The same amount the way a price is written, which is not the same rule.
///
/// A price runs from its highest coin down and keeps the zeros under it: five
/// gold and three copper is "5g 0s 3c", because a player reading a cost scans
/// the coins by position. A looted amount drops them, which is what
/// formatCopperAmount does.
///
/// Four places wrote this as the same three-branch snprintf, and several more
/// printed all three coins unconditionally - so a forty-five copper vendor
/// price appeared as "0g 0s 45c".
inline std::string formatCoinPrice(const CoinAmount& coins) {
    if (coins.gold > 0) {
        return std::to_string(coins.gold) + "g " + std::to_string(coins.silver) +
               "s " + std::to_string(coins.copper) + "c";
    }
    if (coins.silver > 0) {
        return std::to_string(coins.silver) + "s " + std::to_string(coins.copper) + "c";
    }
    return std::to_string(coins.copper) + "c";
}

/// The same, for a caller holding the amount rather than the split.
inline std::string formatCopperPrice(uint64_t amount) {
    return formatCoinPrice(splitCopper(amount));
}

}  // namespace game
}  // namespace wowee
