#include "game/world_packets.hpp"
#include "game/protocol_constants.hpp"
#include "game/packet_parsers.hpp"
#include "game/spline_packet.hpp"
#include "game/opcodes.hpp"
#include "game/character.hpp"
#include "auth/crypto.hpp"
#include "core/logger.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <zlib.h>

namespace wowee {
namespace game {

network::Packet AttackSwingPacket::build(uint64_t targetGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ATTACKSWING));
    packet.writeUInt64(targetGuid);
    LOG_DEBUG("Built CMSG_ATTACKSWING for target: 0x", std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet AttackStopPacket::build() {
    network::Packet packet(wireOpcode(Opcode::CMSG_ATTACKSTOP));
    LOG_DEBUG("Built CMSG_ATTACKSTOP");
    return packet;
}

network::Packet CancelCastPacket::build(uint32_t spellId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CANCEL_CAST));
    packet.writeUInt32(0); // cast count/sequence
    packet.writeUInt32(spellId);
    LOG_DEBUG("Built CMSG_CANCEL_CAST for spell: ", spellId);
    return packet;
}

// ============================================================
// Random Roll
// ============================================================

network::Packet RandomRollPacket::build(uint32_t minRoll, uint32_t maxRoll) {
    network::Packet packet(wireOpcode(Opcode::MSG_RANDOM_ROLL));
    packet.writeUInt32(minRoll);
    packet.writeUInt32(maxRoll);
    LOG_DEBUG("Built MSG_RANDOM_ROLL: ", minRoll, "-", maxRoll);
    return packet;
}

bool RandomRollParser::parse(network::Packet& packet, RandomRollData& data) {
    // WotLK 3.3.5a format: min(4) + max(4) + result(4) + rollerGuid(8) = 20 bytes.
    // Previously read guid first (treating min|max as a uint64 GUID), producing
    // garbled roller identity and random numbers in /roll chat messages.
    if (!packet.hasRemaining(20)) {
        LOG_WARNING("SMSG_RANDOM_ROLL: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.minRoll = packet.readUInt32();
    data.maxRoll = packet.readUInt32();
    data.result = packet.readUInt32();
    data.rollerGuid = packet.readUInt64();
    data.targetGuid = 0;  // not present in protocol; kept for struct compatibility
    LOG_DEBUG("Parsed SMSG_RANDOM_ROLL: roller=0x", std::hex, data.rollerGuid, std::dec,
              " result=", data.result, " (", data.minRoll, "-", data.maxRoll, ")");
    return true;
}

network::Packet NameQueryPacket::build(uint64_t playerGuid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_NAME_QUERY));
    packet.writeUInt64(playerGuid);
    LOG_DEBUG("Built CMSG_NAME_QUERY: guid=0x", std::hex, playerGuid, std::dec);
    return packet;
}

bool NameQueryResponseParser::parse(network::Packet& packet, NameQueryResponseData& data) {
    // 3.3.5a: packedGuid, uint8 found
    // If found==0: CString name, CString realmName, uint8 race, uint8 gender, uint8 classId
    // Validation: packed GUID (1-8 bytes) + found flag (1 byte minimum)
    if (!packet.hasRemaining(2)) return false; // At least 1 for packed GUID + 1 for found

    size_t startPos = packet.getReadPos();
    data.guid = packet.readPackedGuid();

    // Validate found flag read
    if (!packet.hasRemaining(1)) {
        packet.setReadPos(startPos);
        return false;
    }
    data.found = packet.readUInt8();

    if (data.found != 0) {
        LOG_DEBUG("Name query: player not found for GUID 0x", std::hex, data.guid, std::dec);
        return true; // Valid response, just not found
    }

    // Validate strings: need at least 2 null terminators for empty strings
    if (!packet.hasRemaining(2)) {
        data.name.clear();
        data.realmName.clear();
        return !data.name.empty(); // Fail if name was required
    }

    data.name = packet.readString();
    data.realmName = packet.readString();

    // Validate final 3 uint8 fields (race, gender, classId)
    if (!packet.hasRemaining(3)) {
        LOG_WARNING("Name query: truncated fields after realmName, expected 3 uint8s");
        data.race = 0;
        data.gender = 0;
        data.classId = 0;
        return !data.name.empty();
    }

    data.race = packet.readUInt8();
    data.gender = packet.readUInt8();
    data.classId = packet.readUInt8();

    LOG_DEBUG("Name query response: ", data.name, " (race=", static_cast<int>(data.race),
             " class=", static_cast<int>(data.classId), ")");
    return true;
}

network::Packet CreatureQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CREATURE_QUERY));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_CREATURE_QUERY: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

bool CreatureQueryResponseParser::parse(network::Packet& packet, CreatureQueryResponseData& data) {
    // Validate minimum packet size: entry(4)
    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_CREATURE_QUERY_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.entry = packet.readUInt32();

    // High bit set means creature not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        LOG_DEBUG("Creature query: entry ", data.entry, " not found");
        data.name = "";
        return true;
    }

    // 4 name strings (only first is usually populated)
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4
    data.subName = packet.readString();
    data.iconName = packet.readString();

    // WotLK: 4 fixed fields after iconName (typeFlags, creatureType, family, rank)
    // Validate minimum size for these fields: 4×4 = 16 bytes
    if (!packet.hasRemaining(16)) {
        LOG_WARNING("SMSG_CREATURE_QUERY_RESPONSE: truncated before typeFlags (entry=", data.entry, ")");
        data.typeFlags = 0;
        data.creatureType = 0;
        data.family = 0;
        data.rank = 0;
        return true;  // Have name/sub/icon; base fields are important but optional
    }

    data.typeFlags = packet.readUInt32();
    data.creatureType = packet.readUInt32();
    data.family = packet.readUInt32();
    data.rank = packet.readUInt32();

    // killCredit[2] + displayId[4] = 6 × 4 = 24 bytes
    if (!packet.hasRemaining(24)) {
        LOG_WARNING("SMSG_CREATURE_QUERY_RESPONSE: truncated before displayIds (entry=", data.entry, ")");
        LOG_DEBUG("Creature query response: ", data.name, " (type=", data.creatureType,
                 " rank=", data.rank, ")");
        return true;
    }

    packet.readUInt32();  // killCredit[0]
    packet.readUInt32();  // killCredit[1]
    data.displayId[0] = packet.readUInt32();
    data.displayId[1] = packet.readUInt32();
    data.displayId[2] = packet.readUInt32();
    data.displayId[3] = packet.readUInt32();

    // Skip remaining fields (healthMultiplier, powerMultiplier, racialLeader, questItems, movementId)

    LOG_DEBUG("Creature query response: ", data.name, " (type=", data.creatureType,
             " rank=", data.rank, " displayIds=[", data.displayId[0], ",",
             data.displayId[1], ",", data.displayId[2], ",", data.displayId[3], "])");
    return true;
}

// ---- GameObject Query ----

network::Packet GameObjectQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_GAMEOBJECT_QUERY));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_GAMEOBJECT_QUERY: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

bool parseGameObjectQueryBody(network::Packet& packet,
                              GameObjectQueryResponseData& data,
                              int extraStrings) {
    // Validate minimum packet size: entry(4)
    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_GAMEOBJECT_QUERY_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.entry = packet.readUInt32();

    // High bit set means gameobject not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        LOG_DEBUG("GameObject query: entry ", data.entry, " not found");
        data.name = "";
        return true;
    }

    // Validate minimum size for fixed fields: type(4) + displayId(4)
    if (!packet.hasRemaining(8)) {
        LOG_ERROR("SMSG_GAMEOBJECT_QUERY_RESPONSE: truncated before names (entry=", data.entry, ")");
        return false;
    }

    data.type = packet.readUInt32();       // GameObjectType
    data.displayId = packet.readUInt32();
    // 4 name strings (only first is usually populated)
    data.name = packet.readString();
    // name2, name3, name4
    packet.readString();
    packet.readString();
    packet.readString();

    // The 2.0.3 block: iconName, castBarCaption, unk1. Vanilla has none of it.
    //
    // This client reads two of the three on TBC and three on WotLK, and that
    // difference is not settled. AzerothCore's QueryHandler writes all three
    // and marks all three "2.0.3", which would make TBC's count one short -
    // and one short does not raise, because the strings are usually empty, so
    // it costs a single NUL and every one of the twenty-four data fields after
    // it shifts by a byte. A chest then reports a lock it does not have.
    //
    // Left as it was rather than changed on one source: the only thing that
    // settles it is a 2.4.3 server's bytes, and guessing wrong breaks TBC
    // either way. test_gameobject_query_layout pins all three counts, so
    // whichever way it is settled, the change is one number and a failing
    // test rather than an archaeology exercise. The alignment check after the
    // data fields below is what asks the server, so one TBC session answers it.
    for (int i = 0; i < extraStrings; ++i) packet.readString();

    // Read 24 type-specific data fields
    size_t remaining = packet.getRemainingSize();
    if (remaining >= 24 * 4) {
        for (uint32_t& field : data.data) {
            field = packet.readUInt32();
        }
        data.hasData = true;
        // Whether the string count above was right, asked without knowing what
        // the tail is on this expansion.
        //
        // Everything past the data fields is four bytes wide - the size float,
        // and the quest item ids where they exist - so a correct count leaves a
        // whole number of them. A count one short leaves the unread string's
        // NUL in front of the data fields instead, every one of the
        // twenty-four is read a byte early, and what is left over stops
        // dividing by four. That is the whole tell, and it does not need the
        // tail's length: only that the tail is made of four-byte fields.
        //
        // This is what settles the TBC count named above. Two strings there
        // against AzerothCore's three is a difference no source on hand can
        // decide, and one short costs nothing visible - the strings are almost
        // always empty, so it is one byte, and a chest simply reports a lock it
        // does not have. Connecting to a 2.4.3 server once now says which it
        // is, in one line.
        // Only when a whole tail was there to measure. A packet cut short
        // inside the tail leaves one, two or three bytes for that reason and
        // not because a string was missed, and the test cannot tell those
        // apart from the leftover alone - test_gameobject_query_layout trims
        // the response to every length and drew this warning at three of them.
        const size_t leftOver = packet.getRemainingSize();
        if (remaining >= 24 * 4 + 4 && leftOver % 4 != 0) {
            LOG_WARNING("SMSG_GAMEOBJECT_QUERY_RESPONSE: ", leftOver,
                        " bytes left after the data fields, which is not a whole"
                        " number of them - the ", extraStrings,
                        " strings read before them is one too few (entry=",
                        data.entry, ")");
        }
    } else if (remaining > 0) {
        // Partial data field; read what we can
        uint32_t fieldsToRead = remaining / 4;
        for (uint32_t i = 0; i < fieldsToRead && i < 24; i++) {
            data.data[i] = packet.readUInt32();
        }
        if (fieldsToRead < 24) {
            LOG_WARNING("SMSG_GAMEOBJECT_QUERY_RESPONSE: truncated in data fields (", fieldsToRead,
                        " of 24 read, entry=", data.entry, ")");
        }
    }

    LOG_DEBUG("GameObject query response: ", data.name, " (type=", data.type, " entry=", data.entry, ")");
    return true;
}

bool GameObjectQueryResponseParser::parse(network::Packet& packet,
                                          GameObjectQueryResponseData& data) {
    return parseGameObjectQueryBody(packet, data, /*extraStrings=*/3);
}

network::Packet PageTextQueryPacket::build(uint32_t pageId, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_PAGE_TEXT_QUERY));
    packet.writeUInt32(pageId);
    packet.writeUInt64(guid);
    return packet;
}

bool PageTextQueryResponseParser::parse(network::Packet& packet, PageTextQueryResponseData& data) {
    if (!packet.hasRemaining(4)) return false;
    data.pageId = packet.readUInt32();
    data.text = normalizeWowTextTokens(packet.readString());
    if (packet.hasRemaining(4)) {
        data.nextPageId = packet.readUInt32();
    } else {
        data.nextPageId = 0;
    }
    return data.isValid();
}

// ---- Item Query ----

network::Packet ItemQueryPacket::build(uint32_t entry, uint64_t guid) {
    network::Packet packet(wireOpcode(Opcode::CMSG_ITEM_QUERY_SINGLE));
    packet.writeUInt32(entry);
    packet.writeUInt64(guid);
    LOG_DEBUG("Built CMSG_ITEM_QUERY_SINGLE: entry=", entry, " guid=0x", std::hex, guid, std::dec);
    return packet;
}

const char* getItemSubclassName(uint32_t itemClass, uint32_t subClass) {
    if (itemClass == 2) { // Weapon
        switch (subClass) {
            case 0: return "Axe"; case 1: return "Axe";
            case 2: return "Bow"; case 3: return "Gun";
            case 4: return "Mace"; case 5: return "Mace";
            case 6: return "Polearm"; case 7: return "Sword";
            case 8: return "Sword"; case 9: return "Obsolete";
            case 10: return "Staff"; case 13: return "Fist Weapon";
            case 15: return "Dagger"; case 16: return "Thrown";
            case 18: return "Crossbow"; case 19: return "Wand";
            case 20: return "Fishing Pole";
            default: return "Weapon";
        }
    }
    if (itemClass == 4) { // Armor
        switch (subClass) {
            case 0: return "Miscellaneous"; case 1: return "Cloth";
            case 2: return "Leather"; case 3: return "Mail";
            case 4: return "Plate"; case 6: return "Shield";
            default: return "Armor";
        }
    }
    if (itemClass == 1) { // Container (plain bags stay unnamed to avoid "Bag  Bag" tooltips)
        switch (subClass) {
            case 1: return "Soul Bag"; case 2: return "Herb Bag";
            case 3: return "Enchanting Bag"; case 4: return "Engineering Bag";
            case 5: return "Gem Bag"; case 6: return "Mining Bag";
            case 7: return "Leatherworking Bag"; case 8: return "Inscription Bag";
            default: return "";
        }
    }
    if (itemClass == 11) { // Quiver
        switch (subClass) {
            case 3: return "Ammo Pouch";
            default: return "Quiver";
        }
    }
    return "";
}

network::Packet SocketGemsPacket::build(uint64_t itemGuid,
                                        const std::array<uint64_t, 3>& gemGuids) {
    network::Packet packet(wireOpcode(Opcode::CMSG_SOCKET_GEMS));
    packet.writeUInt64(itemGuid);
    for (uint64_t guid : gemGuids) packet.writeUInt64(guid);
    return packet;
}

bool isBandageItem(const ItemQueryResponseData* info) {
    return info && info->valid &&
           info->itemClass == ITEM_CLASS_CONSUMABLE &&
           info->subClass == ITEM_SUBCLASS_BANDAGE;
}

bool ItemQueryResponseParser::parse(network::Packet& packet, ItemQueryResponseData& data) {
    // Validate minimum packet size: entry(4) + item not found check
    if (packet.getSize() < 4) {
        LOG_ERROR("SMSG_ITEM_QUERY_SINGLE_RESPONSE: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.entry = packet.readUInt32();

    // High bit set means item not found
    if (data.entry & 0x80000000) {
        data.entry &= ~0x80000000;
        LOG_DEBUG("Item query: entry ", data.entry, " not found");
        return true;
    }

    // Validate minimum size for fixed fields before reading: itemClass(4) + subClass(4) + soundOverride(4)
    // + 4 name strings + displayInfoId(4) + quality(4) = at least 24 bytes more
    if (!packet.hasRemaining(24)) {
        LOG_ERROR("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before displayInfoId (entry=", data.entry, ")");
        return false;
    }

    uint32_t itemClass = packet.readUInt32();
    uint32_t subClass = packet.readUInt32();
    data.itemClass = itemClass;
    data.subClass = subClass;
    packet.readUInt32(); // SoundOverrideSubclass

    data.subclassName = getItemSubclassName(itemClass, subClass);

    // 4 name strings
    data.name = packet.readString();
    packet.readString(); // name2
    packet.readString(); // name3
    packet.readString(); // name4

    data.displayInfoId = packet.readUInt32();
    data.quality = packet.readUInt32();

    // WotLK 3.3.5a (TrinityCore/AzerothCore): Flags, Flags2, BuyCount, BuyPrice,
    // SellPrice. Some server variants omit BuyCount, which shifts everything
    // after it by one field.
    //
    // Telling them apart from InventoryType alone does not work. On a server
    // without BuyCount, reading five fields lands on AllowableClass - and a
    // class-restricted item's mask is a small number, so it passes any range
    // check InventoryType would. Priest-only is 16, which is a plausible enough
    // "Back" that Friar's Robes of the Light claimed to be a cloak while the
    // server equipped it to the chest.
    //
    // Score both readings across several fields instead. A wrong guess shifts
    // all of them at once, so the right one is the one where every field is
    // credible, not just the first.
    const size_t postQualityPos = packet.getReadPos();
    if (!packet.hasRemaining(24)) {
        LOG_ERROR("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before flags (entry=", data.entry, ")");
        return false;
    }

    struct PriceLayout {
        uint32_t flags = 0;
        uint32_t sellPrice = 0;
        uint32_t inventoryType = 0;
        int32_t score = -1;
    };

    auto scoreLayout = [&](bool withBuyCount) -> PriceLayout {
        PriceLayout out;
        packet.setReadPos(postQualityPos);
        const size_t need = withBuyCount ? 36u : 32u;
        if (!packet.hasRemaining(need)) return out;

        out.flags = packet.readUInt32();          // Flags
        packet.readUInt32();                      // Flags2
        const uint32_t buyCount = withBuyCount ? packet.readUInt32() : 1u;
        const uint32_t buyPrice = packet.readUInt32();
        out.sellPrice = packet.readUInt32();      // SellPrice
        out.inventoryType = packet.readUInt32();

        const uint32_t allowableClass = packet.readUInt32();
        const uint32_t allowableRace  = packet.readUInt32();
        const uint32_t itemLevel      = packet.readUInt32();
        const uint32_t requiredLevel  = packet.readUInt32();

        // Every field is bounded by the game's own rules, so an off-by-one-field
        // read shows up as at least one of them being nonsense.
        constexpr uint32_t kMaxInventoryType = 28;   // INVTYPE_RELIC
        constexpr uint32_t kAllClasses = 0xFFFFFFFFu;
        constexpr uint32_t kClassMask  = 0x7FFu;     // 11 classes
        constexpr uint32_t kRaceMask   = 0x7FFu;     // and 11 playable races
        constexpr uint32_t kMaxItemLevel = 1000;
        constexpr uint32_t kMaxRequiredLevel = 255;

        out.score = 0;
        if (out.inventoryType <= kMaxInventoryType) out.score++;
        if (allowableClass == kAllClasses || (allowableClass & ~kClassMask) == 0) out.score++;
        if (allowableRace == kAllClasses || (allowableRace & ~kRaceMask) == 0) out.score++;
        if (itemLevel <= kMaxItemLevel) out.score++;
        if (requiredLevel <= kMaxRequiredLevel) out.score++;
        // A vendor never sells an item for more than it buys it back for.
        if (out.sellPrice <= buyPrice) out.score++;
        // The decisive one when everything else looks plausible either way:
        // BuyCount is how many the vendor sells at once - 1 for almost
        // everything, a stack at most. Reading a layout that has no BuyCount as
        // though it did lands a price there, and prices are not small.
        constexpr uint32_t kMaxBuyCount = 100;
        if (withBuyCount && buyCount > kMaxBuyCount) out.score -= 2;
        return out;
    };

    const PriceLayout withCount = scoreLayout(true);
    const PriceLayout noCount   = scoreLayout(false);
    const bool useBuyCount = withCount.score >= noCount.score;
    const PriceLayout& chosen = useBuyCount ? withCount : noCount;

    if (chosen.score < 0) {
        LOG_ERROR("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before flags (entry=", data.entry, ")");
        return false;
    }

    // Re-read the chosen layout so the stream is positioned for the fields below.
    packet.setReadPos(postQualityPos);
    data.itemFlags = packet.readUInt32(); // Flags
    packet.readUInt32();                  // Flags2
    if (useBuyCount) packet.readUInt32(); // BuyCount
    packet.readUInt32();                  // BuyPrice
    data.sellPrice = packet.readUInt32(); // SellPrice
    data.inventoryType = packet.readUInt32();

    // Validate minimum size for remaining fixed fields before inventoryType through containerSlots: 13×4 = 52 bytes
    if (!packet.hasRemaining(52)) {
        LOG_ERROR("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before statsCount (entry=", data.entry, ")");
        return false;
    }
    // The run every expansion sends identically, read in one place.
    if (!data.readCommonRequirements(packet)) return false;

    // Read statsCount with bounds validation
    if (!packet.hasRemaining(4)) {
        LOG_WARNING("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated at statsCount (entry=", data.entry, ")");
        return true;  // Have enough for core fields; stats are optional
    }
    uint32_t statsCount = packet.readUInt32();

    // Cap statsCount to prevent excessive iteration
    constexpr uint32_t kMaxItemStats = 10;
    if (statsCount > kMaxItemStats) {
        LOG_WARNING("SMSG_ITEM_QUERY_SINGLE_RESPONSE: statsCount=", statsCount, " exceeds max ",
                    kMaxItemStats, " (entry=", data.entry, "), capping");
        statsCount = kMaxItemStats;
    }

    // Server sends exactly statsCount stat pairs (not always 10).
    uint32_t statsToRead = std::min(statsCount, 10u);
    for (uint32_t i = 0; i < statsToRead; i++) {
        // Each stat is 2 uint32s (type + value) = 8 bytes
        if (!packet.hasRemaining(8)) {
            LOG_WARNING("SMSG_ITEM_QUERY_SINGLE_RESPONSE: stat ", i, " truncated (entry=", data.entry, ")");
            break;
        }
        uint32_t statType = packet.readUInt32();
        int32_t statValue = static_cast<int32_t>(packet.readUInt32());
        data.applyStat(statType, statValue);
    }

    // ScalingStatDistribution and ScalingStatValue
    if (!packet.hasRemaining(8)) {
        LOG_WARNING("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before scaling stats (entry=", data.entry, ")");
        return true;  // Have core fields; scaling is optional
    }
    packet.readUInt32(); // ScalingStatDistribution
    packet.readUInt32(); // ScalingStatValue

    // WotLK 3.3.5a: 2 damage entries (12 bytes each) + armor + 6 resists + delay + ammoType + rangedModRange
    // = 24 + 36 + 4 = 64 bytes minimum. Guard here because the section above
    // returns early on truncation, and every other section has its own guard.
    if (!packet.hasRemaining(64)) {
        LOG_WARNING("SMSG_ITEM_QUERY_SINGLE_RESPONSE: truncated before damage/armor (entry=", data.entry, ")");
        return true;
    }

    // WotLK 3.3.5a: MAX_ITEM_PROTO_DAMAGES = 2
    bool haveWeaponDamage = false;
    for (int i = 0; i < 2; i++) {
        float dmgMin = packet.readFloat();
        float dmgMax = packet.readFloat();
        uint32_t damageType = packet.readUInt32();
        if (!haveWeaponDamage && dmgMax > 0.0f) {
            if (damageType == 0 || data.damageMax <= 0.0f) {
                data.damageMin = dmgMin;
                data.damageMax = dmgMax;
                haveWeaponDamage = (damageType == 0);
            }
        }
    }

    data.armor = static_cast<int32_t>(packet.readUInt32());
    data.holyRes   = static_cast<int32_t>(packet.readUInt32()); // HolyRes
    data.fireRes   = static_cast<int32_t>(packet.readUInt32()); // FireRes
    data.natureRes = static_cast<int32_t>(packet.readUInt32()); // NatureRes
    data.frostRes  = static_cast<int32_t>(packet.readUInt32()); // FrostRes
    data.shadowRes = static_cast<int32_t>(packet.readUInt32()); // ShadowRes
    data.arcaneRes = static_cast<int32_t>(packet.readUInt32()); // ArcaneRes
    data.delayMs = packet.readUInt32();
    packet.readUInt32(); // AmmoType
    packet.readFloat();  // RangedModRange

    // 5 item spells: SpellId, SpellTrigger, SpellCharges, SpellCooldown, SpellCategory, SpellCategoryCooldown
    for (int i = 0; i < 5; i++) {
        if (!packet.hasRemaining(24)) break;
        data.spells[i].spellId = packet.readUInt32();
        data.spells[i].spellTrigger = packet.readUInt32();
        packet.readUInt32(); // SpellCharges
        packet.readUInt32(); // SpellCooldown
        packet.readUInt32(); // SpellCategory
        packet.readUInt32(); // SpellCategoryCooldown
    }

    // Bonding type (0=none, 1=BoP, 2=BoE, 3=BoU, 4=BoQ)
    if (packet.hasRemaining(4))
        data.bindType = packet.readUInt32();

    // Flavor/lore text (Description cstring)
    if (packet.hasData())
        data.description = packet.readString();

    // Post-description fields: PageText, LanguageID, PageMaterial, StartQuest
    if (packet.hasRemaining(16)) {
        data.pageTextId = packet.readUInt32(); // PageText
        packet.readUInt32(); // LanguageID
        data.pageMaterial = packet.readUInt32(); // PageMaterial
        data.startQuestId = packet.readUInt32(); // StartQuest
    }

    // WotLK 3.3.5a: additional fields after StartQuest (read up to socket data)
    // LockID(4), Material(4), Sheath(4), RandomProperty(4), RandomSuffix(4),
    // Block(4), ItemSet(4), MaxDurability(4), Area(4), Map(4), BagFamily(4),
    // TotemCategory(4) = 48 bytes before sockets
    constexpr size_t kPreSocketSkip = 48;
    if (packet.getReadPos() + kPreSocketSkip + 28 <= packet.getSize()) {
        // LockID(0), Material(1), Sheath(2), RandomProperty(3), RandomSuffix(4), Block(5)
        for (size_t i = 0; i < 6; ++i) packet.readUInt32();
        data.itemSetId = packet.readUInt32(); // ItemSet(6)
        // MaxDurability(7), Area(8), Map(9), BagFamily(10), TotemCategory(11)
        for (size_t i = 0; i < 5; ++i) packet.readUInt32();
        // The three sockets, colour and content together, one socket at a
        // time - `for (s) { << Socket[s].Color; << Socket[s].Content; }`.
        // Read as three colours followed by three contents this took the
        // first socket's *content* as the second socket's colour, and a
        // template's sockets are empty, so that is zero: an item with three
        // sockets showed one, and a two-socket item showed its second colour
        // in the third position. The bonus that landed after them was right
        // only because six reads is six reads either way.
        for (size_t i = 0; i < 3; ++i) {
            data.socketColor[i]   = packet.readUInt32();
            data.socketContent[i] = packet.readUInt32();
        }
        // socketBonus (enchantmentId)
        data.socketBonus = packet.readUInt32();
        // GemProperties.dbc id - non-zero when this item *is* a gem, and how
        // its colour is known. An item carries no socket colour of its own.
        if (packet.hasRemaining(4)) {
            data.gemProperties = packet.readUInt32();
        }
    }

    data.valid = !data.name.empty();
    return true;
}

// ============================================================
// Creature Movement
// ============================================================

bool MonsterMoveParser::parse(network::Packet& packet, MonsterMoveData& data) {
    // PackedGuid
    data.guid = packet.readPackedGuid();
    if (data.guid == 0) return false;

    // uint8 unk (toggle for MOVEMENTFLAG2_UNK7)
    if (!packet.hasData()) return false;
    packet.readUInt8();

    // Current position (server coords: float x, y, z)
    if (!packet.hasRemaining(12)) return false;
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();

    // uint32 splineId
    if (!packet.hasRemaining(4)) return false;
    packet.readUInt32();

    // uint8 moveType
    {
        bool stopped = false;
        if (!parseMonsterMoveFacing(packet, data, stopped)) return false;
        if (stopped) return true;
    }

    // uint32 splineFlags
    if (!packet.hasRemaining(4)) return false;
    data.splineFlags = packet.readUInt32();

    // Consolidated spline body parser
    {
        SplineBlockData spline;
        if (!parseMonsterMoveSplineBody(packet, spline, data.splineFlags,
                                        glm::vec3(data.x, data.y, data.z),
                                        /*useTbcUncompressedMask=*/false,
                                        SplineFlagSet::Wotlk)) {
            return false;
        }
        data.duration = spline.duration;
        if (spline.hasDest) {
            data.destX = spline.destination.x;
            data.destY = spline.destination.y;
            data.destZ = spline.destination.z;
            data.hasDest = true;
        }
        for (const auto& wp : spline.waypoints) {
            data.waypoints.push_back({wp.x, wp.y, wp.z});
        }
    }

    LOG_DEBUG("MonsterMove: guid=0x", std::hex, data.guid, std::dec,
              " type=", static_cast<int>(data.moveType), " dur=", data.duration, "ms",
              " dest=(", data.destX, ",", data.destY, ",", data.destZ, ")");

    return true;
}

bool MonsterMoveParser::parseVanilla(network::Packet& packet, MonsterMoveData& data) {
    data.guid = packet.readPackedGuid();
    if (data.guid == 0) return false;

    if (!packet.hasRemaining(12)) return false;
    data.x = packet.readFloat();
    data.y = packet.readFloat();
    data.z = packet.readFloat();

    // Turtle WoW movement payload uses a spline-style layout after XYZ:
    //   uint32 splineIdOrTick
    //   uint8  moveType
    //   [if moveType 2/3/4] facing payload
    //   uint32 splineFlags
    //   [if Animation] uint8 + uint32
    //   uint32 duration
    //   [if Parabolic] float + uint32
    //   uint32 pointCount
    //   float[3] dest
    //   uint32 packedPoints[pointCount-1]
    if (!packet.hasRemaining(4)) return false;
    /*uint32_t splineIdOrTick =*/ packet.readUInt32();

    if (!packet.hasData()) return false;
    {
        bool stopped = false;
        if (!parseMonsterMoveFacing(packet, data, stopped)) return false;
        if (stopped) return true;
    }

    if (!packet.hasRemaining(4)) return false;
    data.splineFlags = packet.readUInt32();

    // Consolidated Vanilla spline body parser (always compressed)
    {
        SplineBlockData spline;
        if (!parseMonsterMoveSplineBodyVanilla(packet, spline, data.splineFlags,
                                               glm::vec3(data.x, data.y, data.z))) {
            return false;
        }
        data.duration = spline.duration;
        if (spline.hasDest) {
            data.destX = spline.destination.x;
            data.destY = spline.destination.y;
            data.destZ = spline.destination.z;
            data.hasDest = true;
        }
        for (const auto& wp : spline.waypoints) {
            data.waypoints.push_back({wp.x, wp.y, wp.z});
        }
    }

    LOG_DEBUG("MonsterMove(turtle): guid=0x", std::hex, data.guid, std::dec,
              " type=", static_cast<int>(data.moveType), " dur=", data.duration, "ms",
              " dest=(", data.destX, ",", data.destY, ",", data.destZ, ")");

    return true;
}


// ============================================================
// Combat Core
// ============================================================

bool AttackStartParser::parse(network::Packet& packet, AttackStartData& data) {
    if (packet.getSize() < 16) return false;
    data.attackerGuid = packet.readUInt64();
    data.victimGuid = packet.readUInt64();
    LOG_DEBUG("Attack started: 0x", std::hex, data.attackerGuid,
             " -> 0x", data.victimGuid, std::dec);
    return true;
}

bool AttackStopParser::parse(network::Packet& packet, AttackStopData& data) {
    data.attackerGuid = packet.readPackedGuid();
    data.victimGuid = packet.readPackedGuid();
    if (packet.hasData()) {
        data.unknown = packet.readUInt32();
    }
    LOG_DEBUG("Attack stopped: 0x", std::hex, data.attackerGuid, std::dec);
    return true;
}

bool AttackerStateUpdateParser::parse(network::Packet& packet, AttackerStateUpdateData& data) {
    // Upfront validation: hitInfo(4) + packed GUIDs(1-8 each) + totalDamage(4) + overkill(4) + subDamageCount(1) = 17 bytes minimum
    if (!packet.hasRemaining(17)) return false;

    size_t startPos = packet.getReadPos();
    data.hitInfo = packet.readUInt32();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.attackerGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.targetGuid = packet.readPackedGuid();

    // Validate totalDamage + overkill + subDamageCount can be read (9 bytes)
    // WotLK (AzerothCore) sends: damage(4) + overkill(4) + subDamageCount(1)
    if (!packet.hasRemaining(9)) {
        packet.setReadPos(startPos);
        return false;
    }

    data.totalDamage = static_cast<int32_t>(packet.readUInt32());
    data.overkill = static_cast<int32_t>(packet.readUInt32());
    data.subDamageCount = packet.readUInt8();

    // Cap subDamageCount: each entry is 20 bytes. If the claimed count
    // exceeds what the remaining bytes can hold, a GUID was mis-parsed
    // (off by one byte), causing the school-mask byte to be read as count.
    // In that case clamp to the number of full entries that fit.
    {
        size_t remaining = packet.getRemainingSize();
        size_t maxFit    = remaining / 20;
        if (data.subDamageCount > maxFit) {
            data.subDamageCount = static_cast<uint8_t>(std::min<size_t>(maxFit, 64));
        } else if (data.subDamageCount > 64) {
            data.subDamageCount = 64;
        }
    }
    if (data.subDamageCount == 0) return false;

    data.subDamages.reserve(data.subDamageCount);
    for (uint8_t i = 0; i < data.subDamageCount; ++i) {
        // Each sub-damage entry needs 20 bytes: schoolMask(4) + damage(4) + intDamage(4) + absorbed(4) + resisted(4)
        if (!packet.hasRemaining(20)) {
            data.subDamageCount = i;
            break;
        }
        SubDamage sub;
        sub.schoolMask = packet.readUInt32();
        sub.damage = packet.readFloat();
        sub.intDamage = packet.readUInt32();
        sub.absorbed = packet.readUInt32();
        sub.resisted = packet.readUInt32();
        data.subDamages.push_back(sub);
    }

    // Validate victimState + overkill fields (8 bytes)
    if (!packet.hasRemaining(8)) {
        data.victimState = 0;
        return !data.subDamages.empty();
    }

    data.victimState = packet.readUInt32();
    // WotLK: attackerState(4) + meleeSpellId(4) follow victimState
    auto rem = [&]() { return packet.getRemainingSize(); };
    if (rem() >= 4) packet.readUInt32(); // attackerState (always 0)
    if (rem() >= 4) packet.readUInt32(); // meleeSpellId (0 for auto-attack)

    // hitInfo-conditional fields: HITINFO_BLOCK(0x2000), RAGE_GAIN(0x20000), FAKE_DAMAGE(0x40)
    if ((data.hitInfo & 0x2000) && rem() >= 4)  data.blocked  = packet.readUInt32();
    else data.blocked = 0;
    // RAGE_GAIN and FAKE_DAMAGE both add a uint32 we can skip
    if ((data.hitInfo & 0x20000) && rem() >= 4) packet.readUInt32(); // rage gain
    if ((data.hitInfo & 0x40)    && rem() >= 4) packet.readUInt32(); // fake damage total

    LOG_DEBUG("Melee hit: ", data.totalDamage, " damage",
              data.isCrit() ? " (CRIT)" : "",
              data.isMiss() ? " (MISS)" : "");
    return true;
}

bool SpellDamageLogParser::parse(network::Packet& packet, SpellDamageLogData& data) {
    // Upfront validation:
    // packed GUIDs(1-8 each) + spellId(4) + damage(4) + overkill(4) + schoolMask(1)
    // + absorbed(4) + resisted(4) + periodicLog(1) + unused(1) + blocked(4) + flags(4)
    // = 33 bytes minimum.
    if (!packet.hasRemaining(33)) return false;

    size_t startPos = packet.getReadPos();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.targetGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.attackerGuid = packet.readPackedGuid();

    // Validate core fields (spellId + damage + overkill + schoolMask + absorbed + resisted = 21 bytes)
    if (!packet.hasRemaining(21)) {
        packet.setReadPos(startPos);
        return false;
    }

    data.spellId = packet.readUInt32();
    data.damage = packet.readUInt32();
    data.overkill = packet.readUInt32();
    data.schoolMask = packet.readUInt8();
    data.absorbed = packet.readUInt32();
    data.resisted = packet.readUInt32();

    // Remaining fields are required for a complete event.
    // Reject truncated packets so we do not emit partial/incorrect combat entries.
    if (!packet.hasRemaining(10)) {
        packet.setReadPos(startPos);
        return false;
    }

    (void)packet.readUInt8(); // periodicLog (not displayed)
    packet.readUInt8(); // unused
    packet.readUInt32(); // blocked
    uint32_t flags = packet.readUInt32();  // flags IS used - bit 0x02 = crit
    data.isCrit = (flags & 0x02) != 0;

    LOG_DEBUG("Spell damage: spellId=", data.spellId, " dmg=", data.damage,
              data.isCrit ? " CRIT" : "");
    return true;
}

bool SpellHealLogParser::parse(network::Packet& packet, SpellHealLogData& data) {
    // Upfront validation: packed GUIDs(1-8 each) + spellId(4) + heal(4) + overheal(4) + absorbed(4) + critFlag(1) = 21 bytes minimum
    if (!packet.hasRemaining(21)) return false;

    size_t startPos = packet.getReadPos();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.targetGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.casterGuid = packet.readPackedGuid();

    // Validate remaining fields (spellId + heal + overheal + absorbed + critFlag = 17 bytes)
    if (!packet.hasRemaining(17)) {
        packet.setReadPos(startPos);
        return false;
    }

    data.spellId = packet.readUInt32();
    data.heal = packet.readUInt32();
    data.overheal = packet.readUInt32();
    data.absorbed = packet.readUInt32();
    uint8_t critFlag = packet.readUInt8();
    data.isCrit = (critFlag != 0);

    LOG_DEBUG("Spell heal: spellId=", data.spellId, " heal=", data.heal,
              data.isCrit ? " CRIT" : "");
    return true;
}

// ============================================================
// XP Gain
// ============================================================

bool XpGainParser::parse(network::Packet& packet, XpGainData& data) {
    // Validate minimum packet size: victimGuid(8) + totalXp(4) + type(1)
    if (!packet.hasRemaining(13)) {
        LOG_WARNING("SMSG_LOG_XPGAIN: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.victimGuid = packet.readUInt64();
    data.totalXp = packet.readUInt32();
    data.type = packet.readUInt8();
    if (data.type == 0) {
        // Kill XP: float groupRate (1.0 = solo) + uint8 RAF flag
        // Validate before reading conditional fields
        if (packet.hasRemaining(5)) {
            float groupRate = packet.readFloat();
            packet.readUInt8(); // RAF bonus flag
            // Group bonus = total - (total / rate); only if grouped (rate > 1)
            if (groupRate > 1.0f) {
                data.groupBonus = data.totalXp - static_cast<uint32_t>(data.totalXp / groupRate);
            }
        }
    }
    LOG_DEBUG("XP gain: ", data.totalXp, " xp (type=", static_cast<int>(data.type), ")");
    return data.totalXp > 0;
}

// ============================================================
// Spells, Action Bar, Auras
// ============================================================

bool InitialSpellsParser::parse(network::Packet& packet, InitialSpellsData& data,
                                bool vanillaFormat) {
    // Validate minimum packet size for header: talentSpec(1) + spellCount(2)
    if (!packet.hasRemaining(3)) {
        LOG_ERROR("SMSG_INITIAL_SPELLS: packet too small (", packet.getSize(), " bytes)");
        return false;
    }

    data.talentSpec = packet.readUInt8();
    uint16_t spellCount = packet.readUInt16();

    // Cap spell count to prevent excessive iteration.
    // WotLK characters with all ranks, mounts, professions, and racials can
    // know 400-600 spells; 1024 covers all practical cases with headroom.
    constexpr uint16_t kMaxSpells = 1024;
    if (spellCount > kMaxSpells) {
        LOG_WARNING("SMSG_INITIAL_SPELLS: spellCount=", spellCount, " exceeds max ", kMaxSpells,
                    ", capping");
        spellCount = kMaxSpells;
    }

    LOG_DEBUG("SMSG_INITIAL_SPELLS: spellCount=", spellCount,
              vanillaFormat ? " (uint16 spell format)" : " (uint32 spell format)");

    data.spellIds.reserve(spellCount);
    for (uint16_t i = 0; i < spellCount; ++i) {
        // Classic/TBC spell: spellId(2) + slot/unknown(2) = 4 bytes
        // WotLK spell: spellId(4) + unknown(2) = 6 bytes
        size_t spellEntrySize = vanillaFormat ? 4 : 6;
        if (!packet.hasRemaining(spellEntrySize)) {
            LOG_WARNING("SMSG_INITIAL_SPELLS: spell ", i, " truncated (", spellCount, " expected)");
            break;
        }

        uint32_t spellId;
        if (vanillaFormat) {
            spellId = packet.readUInt16();
            packet.readUInt16(); // slot
        } else {
            spellId = packet.readUInt32();
            packet.readUInt16(); // unknown (always 0)
        }
        if (spellId != 0) {
            data.spellIds.push_back(spellId);
        }
    }

    // Validate minimum packet size for cooldownCount (2 bytes)
    if (!packet.hasRemaining(2)) {
        LOG_WARNING("SMSG_INITIAL_SPELLS: truncated before cooldownCount (parsed ", data.spellIds.size(),
                    " spells)");
        return true;  // Have spells; cooldowns are optional
    }

    uint16_t cooldownCount = packet.readUInt16();

    // Cap cooldown count to prevent excessive iteration.
    // Some servers include entries for all spells (even with zero remaining time)
    // to communicate category cooldown data, so the count can be high.
    constexpr uint16_t kMaxCooldowns = 1024;
    if (cooldownCount > kMaxCooldowns) {
        LOG_WARNING("SMSG_INITIAL_SPELLS: cooldownCount=", cooldownCount, " exceeds max ", kMaxCooldowns,
                    ", capping");
        cooldownCount = kMaxCooldowns;
    }

    data.cooldowns.reserve(cooldownCount);
    for (uint16_t i = 0; i < cooldownCount; ++i) {
        // Classic/TBC cooldown: spellId(2) + itemId(2) + categoryId(2) + cooldownMs(4) + categoryCooldownMs(4) = 14 bytes
        // WotLK cooldown: spellId(4) + itemId(2) + categoryId(2) + cooldownMs(4) + categoryCooldownMs(4) = 16 bytes
        size_t cooldownEntrySize = vanillaFormat ? 14 : 16;
        if (!packet.hasRemaining(cooldownEntrySize)) {
            LOG_WARNING("SMSG_INITIAL_SPELLS: cooldown ", i, " truncated (", cooldownCount, " expected)");
            break;
        }

        SpellCooldownEntry entry;
        if (vanillaFormat) {
            entry.spellId = packet.readUInt16();
        } else {
            entry.spellId = packet.readUInt32();
        }
        entry.itemId = packet.readUInt16();
        entry.categoryId = packet.readUInt16();
        entry.cooldownMs = packet.readUInt32();
        entry.categoryCooldownMs = packet.readUInt32();
        data.cooldowns.push_back(entry);
    }

    LOG_INFO("Initial spells parsed: ", data.spellIds.size(), " spells, ",
             data.cooldowns.size(), " cooldowns");

    if (!data.spellIds.empty()) {
        std::string first10;
        for (size_t i = 0; i < std::min(size_t(10), data.spellIds.size()); ++i) {
            if (!first10.empty()) first10 += ", ";
            first10 += std::to_string(data.spellIds[i]);
        }
        LOG_DEBUG("Initial spell IDs (first 10): ", first10);
    }

    return true;
}

network::Packet CastSpellPacket::build(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CAST_SPELL));
    packet.writeUInt8(castCount);
    packet.writeUInt32(spellId);
    packet.writeUInt8(0x00); // castFlags = 0 for normal cast

    // SpellCastTargets
    if (targetGuid != 0) {
        packet.writeUInt32(0x02); // TARGET_FLAG_UNIT

        // Write packed GUID
        packet.writePackedGuid(targetGuid);
    } else {
        packet.writeUInt32(0x00); // TARGET_FLAG_SELF
    }

    LOG_DEBUG("Built CMSG_CAST_SPELL: spell=", spellId, " target=0x",
              std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet CastSpellPacket::buildGameObjectTarget(uint32_t spellId, uint64_t targetGuid, uint8_t castCount) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CAST_SPELL));
    packet.writeUInt8(castCount);
    packet.writeUInt32(spellId);
    packet.writeUInt8(0x00); // castFlags = 0 for normal cast
    packet.writeUInt32(0x0800); // TARGET_FLAG_GAMEOBJECT
    packet.writePackedGuid(targetGuid);

    LOG_DEBUG("Built CMSG_CAST_SPELL: spell=", spellId, " gameObject=0x",
              std::hex, targetGuid, std::dec);
    return packet;
}

network::Packet CastSpellPacket::buildItemTarget(uint32_t spellId, uint64_t itemGuid,
                                                 uint8_t castCount) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CAST_SPELL));
    packet.writeUInt8(castCount);
    packet.writeUInt32(spellId);
    packet.writeUInt8(0x00); // castFlags = 0 for normal cast
    // Disenchant, Prospecting, Milling and the enchant formulas are cast at an
    // item rather than a unit; the server reads the item out of SpellCastTargets
    // and answers "can't be disenchanted" when there is nothing there.
    packet.writeUInt32(0x10); // TARGET_FLAG_ITEM
    packet.writePackedGuid(itemGuid);

    LOG_DEBUG("Built CMSG_CAST_SPELL: spell=", spellId, " item=0x",
              std::hex, itemGuid, std::dec);
    return packet;
}

network::Packet CancelAuraPacket::build(uint32_t spellId) {
    network::Packet packet(wireOpcode(Opcode::CMSG_CANCEL_AURA));
    packet.writeUInt32(spellId);
    return packet;
}

network::Packet DismissCritterPacket::build(uint64_t critterGuid) {
    // CMSG_DISMISS_CRITTER: critterGuid(8)
    network::Packet packet(wireOpcode(Opcode::CMSG_DISMISS_CRITTER));
    packet.writeUInt64(critterGuid);
    return packet;
}

network::Packet PetActionPacket::build(uint64_t petGuid, uint32_t action, uint64_t targetGuid) {
    // CMSG_PET_ACTION: petGuid(8) + action(4) + targetGuid(8)
    network::Packet packet(wireOpcode(Opcode::CMSG_PET_ACTION));
    packet.writeUInt64(petGuid);
    packet.writeUInt32(action);
    packet.writeUInt64(targetGuid);
    return packet;
}

bool CastFailedParser::parse(network::Packet& packet, CastFailedData& data) {
    // WotLK format: castCount(1) + spellId(4) + result(1) = 6 bytes minimum
    if (!packet.hasRemaining(6)) return false;

    data.castCount = packet.readUInt8();
    data.spellId = packet.readUInt32();
    data.result = packet.readUInt8();
    // Spell-focus and totem failures append ids naming the missing
    // station/tool so the client can surface them.
    readCastResultArgs(packet, data.result, data.miscArg, data.miscArg2);
    LOG_INFO("Cast failed: spell=", data.spellId, " result=", static_cast<int>(data.result));
    return true;
}

bool SpellStartParser::parse(network::Packet& packet, SpellStartData& data) {
    data = SpellStartData{};

    // Packed GUIDs are variable-length; only require minimal packet shape up front:
    // two GUID masks + castCount(1) + spellId(4) + castFlags(4) + castTime(4).
    if (!packet.hasRemaining(15)) return false;

    size_t startPos = packet.getReadPos();
    if (!packet.hasFullPackedGuid()) {
        return false;
    }
    data.casterGuid = packet.readPackedGuid();
    if (!packet.hasFullPackedGuid()) {
        packet.setReadPos(startPos);
        return false;
    }
    data.casterUnit = packet.readPackedGuid();

    // Validate remaining fixed fields (castCount + spellId + castFlags + castTime = 13 bytes)
    if (!packet.hasRemaining(13)) {
        packet.setReadPos(startPos);
        return false;
    }

    data.castCount = packet.readUInt8();
    data.spellId = packet.readUInt32();
    data.castFlags = packet.readUInt32();
    data.castTime = packet.readUInt32();

    // SpellCastTargets starts with target flags and is mandatory.
    if (!packet.hasRemaining(4)) {
        LOG_WARNING("Spell start: missing targetFlags");
        packet.setReadPos(startPos);
        return false;
    }

    // WotLK 3.3.5a SpellCastTargets - consume ALL target payload bytes so that
    // subsequent fields (e.g. school mask, cast flags 0x20 extra data) are not
    // misaligned for ground-targeted or AoE spells.
    uint32_t targetFlags = packet.readUInt32();

    auto readPackedTarget = [&](uint64_t* out) -> bool {
        if (!packet.hasFullPackedGuid()) return false;
        uint64_t g = packet.readPackedGuid();
        if (out) *out = g;
        return true;
    };
    auto skipPackedAndFloats3 = [&]() -> bool {
        if (!packet.hasFullPackedGuid()) return false;
        packet.readPackedGuid(); // transport GUID (may be zero)
        if (!packet.hasRemaining(12)) return false;
        packet.readFloat(); packet.readFloat(); packet.readFloat();
        return true;
    };

    // UNIT/UNIT_MINIPET/CORPSE_ALLY/GAMEOBJECT share a single object target GUID
    if (targetFlags & (0x0002u | 0x0004u | 0x0400u | 0x0800u)) {
        readPackedTarget(&data.targetGuid); // best-effort; ignore failure
    }
    // ITEM/TRADE_ITEM share a single item target GUID
    if (targetFlags & (0x0010u | 0x0100u)) {
        readPackedTarget(nullptr);
    }
    // SOURCE_LOCATION: PackedGuid (transport) + float x,y,z
    if (targetFlags & 0x0020u) {
        skipPackedAndFloats3();
    }
    // DEST_LOCATION: PackedGuid (transport) + float x,y,z
    if (targetFlags & 0x0040u) {
        skipPackedAndFloats3();
    }
    // STRING: null-terminated
    if (targetFlags & 0x0200u) {
        while (packet.hasData() && packet.readUInt8() != 0) {}
    }

    LOG_DEBUG("Spell start: spell=", data.spellId, " castTime=", data.castTime, "ms");
    return true;
}

} // namespace game
} // namespace wowee
