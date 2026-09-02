#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace game {

/**
 * Party/Group member data
 */
struct GroupMember {
    std::string name;
    uint64_t guid = 0;
    uint8_t isOnline = 0;      // 0 = offline, 1 = online
    uint8_t subGroup = 0;      // Raid subgroup (0 for party)
    uint8_t flags = 0;         // Assistant, main tank, etc.
    uint8_t roles = 0;         // LFG roles (3.3.5a)

    // Party member stats (from SMSG_PARTY_MEMBER_STATS)
    uint32_t curHealth = 0;
    uint32_t maxHealth = 0;
    uint8_t powerType = 0;
    uint16_t curPower = 0;
    uint16_t maxPower = 0;
    uint16_t level = 0;
    uint16_t zoneId = 0;
    int16_t posX = 0;
    int16_t posY = 0;
    uint16_t onlineStatus = 0;   // GROUP_UPDATE_FLAG_STATUS bitmask
    bool hasPartyStats = false;  // true once we've received stats
};

/**
 * Full group/party data from SMSG_GROUP_LIST
 */
/// What a loot method is called, as it is written to chat.
///
/// Five, not the three the field's comment used to name. The party window and
/// the /loot command each wrote the five out; a parallel table of lowercase
/// tokens lives in lua_api_helpers.hpp for parsing the command's argument, and
/// the two are in the same order on purpose - the token's index is the wire
/// value this names.
inline const char* lootMethodName(uint8_t method) {
    static constexpr const char* kByMethod[] = {
        "Free for All", "Round Robin", "Master Looter",
        "Group Loot", "Need Before Greed",
    };
    return method < 5 ? kByMethod[method] : "Unknown";
}

/// What an instance difficulty is called, or null for a value that is not one.
///
/// Three places wrote the four words out, and they disagreed about the answer
/// for a value outside the four: two said "Normal" and one said "Unknown".
/// Null here so each keeps its own - the Lua bindings hand this to FrameXML,
/// which compares it against names, while the minimap is only labelling a
/// corner and can say it does not know.
inline const char* instanceDifficultyName(uint32_t difficulty) {
    static constexpr const char* kByDifficulty[] = {
        "Normal", "Heroic", "25 Normal", "25 Heroic",
    };
    return difficulty < 4 ? kByDifficulty[difficulty] : nullptr;
}

/// Whether a group's type says raid.
///
/// The field changed meaning between expansions and the client tested it as a
/// number. 1.12 writes 0 for a party and 1 for a raid; 3.3.5 writes a set of
/// flags - 0x01 battleground, 0x02 raid, 0x04 and 0x08 for the dungeon finder -
/// so `== 1` read a WotLK raid as no raid at all, and every test for a party
/// written as `== 0` read a finder group (0x0C) as no party either. A dungeon
/// group had neither kind of frame.
///
/// The low two bits cover both conventions: 1.12's raid is 1, WotLK's is 2, and
/// a battleground group is a raid as well. The finder's bits mean neither.
inline bool groupIsRaid(uint8_t groupType) {
    return (groupType & 0x03) != 0;
}

struct GroupListData {
    uint8_t groupType = 0;       // 0 = party, 1 = raid
    uint8_t subGroup = 0;
    uint8_t flags = 0;
    uint8_t roles = 0;
    uint8_t lootMethod = 0;      ///< 0..4; see lootMethodName
    uint64_t looterGuid = 0;
    uint8_t lootThreshold = 0;
    uint8_t difficultyId = 0;
    uint8_t raidDifficultyId = 0;
    uint32_t memberCount = 0;
    std::vector<GroupMember> members;
    uint64_t leaderGuid = 0;

    [[nodiscard]] bool isValid() const { return true; }
    [[nodiscard]] bool isEmpty() const { return memberCount == 0; }
};

/**
 * Party command types
 */
enum class PartyCommand : uint32_t {
    INVITE = 0,
    UNINVITE = 1,
    LEAVE = 2,
    // 3 is not an operation. The server's PARTY_OP_SWAP is 4, and SWAP was
    // declared 3, so the operation field of SMSG_PARTY_COMMAND_RESULT could
    // never be recognised as a swap.
    SWAP = 4
};

/**
 * Party command result codes
 */
enum class PartyResult : uint32_t {
    OK = 0,
    BAD_PLAYER_NAME = 1,
    TARGET_NOT_IN_GROUP = 2,
    TARGET_NOT_IN_INSTANCE = 3,
    GROUP_FULL = 4,
    ALREADY_IN_GROUP = 5,
    NOT_IN_GROUP = 6,
    NOT_LEADER = 7,
    PLAYER_WRONG_FACTION = 8,
    IGNORING_YOU = 9,
    LFG_PENDING = 12,
    INVITE_RESTRICTED = 13
};

} // namespace game
} // namespace wowee
