// SMSG_GROUP_LIST, built the way the server writes it and read back.
//
// The packet has two shapes: a plain group, and a dungeon finder group, which
// puts the LFG state and the dungeon entry - five bytes - between the roles
// byte and the group guid. The finder shape was being read six bytes long, on
// a guess that a seventh field "may or may not" be there, tested as "is there
// more than thirteen bytes left" - which there always is. So every member of
// every dungeon group was read one byte late.
//
// Nothing failed loudly. The member count came out of the last three bytes of
// the real count plus the first letter of the first member's name, was clamped
// to forty, and forty members were then read out of the packet's own bytes.
// The party frames drew forty strangers, or nobody.
//
// These build the bytes rather than capture them, so the layout is written
// down in the test as well as in the parser, and the two have to agree. The
// oracle is AzerothCore's Group::SendUpdate.
#include <catch_amalgamated.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "core/application.hpp"
#include "game/group_defines.hpp"
#include "game/world_packets.hpp"
#include "network/packet.hpp"

namespace wowee {
namespace core {
// The packet layer reaches for the running application to ask which expansion
// is active. There is none here, and the parse under test is told which
// expansion it is reading for.
Application* Application::instance = nullptr;
}
}

using wowee::game::GroupListData;
using wowee::game::GroupListParser;

namespace {

struct Writer {
    std::vector<uint8_t> bytes;
    void u8(uint8_t v) { bytes.push_back(v); }
    void u32(uint32_t v) {
        for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<uint8_t>(v >> (i * 8)));
    }
    void u64(uint64_t v) {
        for (int i = 0; i < 8; ++i) bytes.push_back(static_cast<uint8_t>(v >> (i * 8)));
    }
    void str(const std::string& s) {
        for (char c : s) bytes.push_back(static_cast<uint8_t>(c));
        bytes.push_back(0);
    }
};

struct Member {
    std::string name;
    uint64_t guid;
    uint8_t roles;
};

/// One SMSG_GROUP_LIST as a 3.3.5 server writes it.
std::vector<uint8_t> groupList(uint8_t groupType, const std::vector<Member>& members,
                               uint64_t leaderGuid, bool lfg) {
    Writer w;
    w.u8(groupType);
    w.u8(0);            // subgroup
    w.u8(0);            // flags
    w.u8(0);            // roles
    if (lfg) {
        w.u8(0);        // lfg state
        w.u32(285);     // lfg dungeon entry
    }
    w.u64(0x1234);      // group guid
    w.u32(7);           // update counter
    w.u32(static_cast<uint32_t>(members.size()));
    for (const auto& m : members) {
        w.str(m.name);
        w.u64(m.guid);
        w.u8(1);        // online
        w.u8(0);        // subgroup
        w.u8(0);        // flags
        w.u8(m.roles);
    }
    w.u64(leaderGuid);
    if (!members.empty()) {
        w.u8(2);        // loot method
        w.u64(leaderGuid);
        w.u8(2);        // loot threshold
        w.u8(1);        // dungeon difficulty
        w.u8(0);        // raid difficulty
        w.u8(0);        // dynamic difficulty
    }
    return w.bytes;
}

const std::vector<Member>& fourOthers() {
    // A name beginning with T, because that letter is what the misread count
    // was made of: 0x54000000 is three zero bytes of the real count and a 'T'.
    static const std::vector<Member> members = {
        {"Tholan", 0x1001, 2}, {"Cetrix", 0x1002, 4},
        {"Veleya", 0x1003, 8}, {"Jackaroe", 0x1004, 8},
    };
    return members;
}

GroupListData parsed(const std::vector<uint8_t>& bytes, bool& ok) {
    wowee::network::Packet packet(0, bytes);
    GroupListData data;
    ok = GroupListParser::parse(packet, data, /*hasRoles=*/true,
                                /*hasBattleGroupFlag=*/false);
    return data;
}

}  // namespace

TEST_CASE("a plain party reads back as it was written", "[group-list]") {
    bool ok = false;
    const GroupListData data = parsed(groupList(0x00, fourOthers(), 0x1001, false), ok);
    REQUIRE(ok);
    REQUIRE(data.memberCount == 4);
    REQUIRE(data.members.size() == 4);
    CHECK(data.members[0].name == "Tholan");
    CHECK(data.members[0].guid == 0x1001);
    CHECK(data.members[3].name == "Jackaroe");
    CHECK(data.leaderGuid == 0x1001);
    CHECK(data.lootMethod == 2);
}

TEST_CASE("a dungeon finder group reads back as it was written", "[group-list]") {
    // The whole bug: the same four members, five bytes further along. 0x08 is
    // the finder bit; servers set 0x04 beside it, and the parser takes either.
    for (const uint8_t groupType : {uint8_t{0x08}, uint8_t{0x0C}}) {
        INFO("groupType 0x" << std::hex << static_cast<int>(groupType));
        bool ok = false;
        const GroupListData data = parsed(groupList(groupType, fourOthers(), 0x1002, true), ok);
        REQUIRE(ok);
        REQUIRE(data.memberCount == 4);
        REQUIRE(data.members.size() == 4);
        CHECK(data.members[0].name == "Tholan");
        CHECK(data.members[1].name == "Cetrix");
        CHECK(data.members[1].guid == 0x1002);
        CHECK(data.members[3].roles == 8);
        CHECK(data.leaderGuid == 0x1002);
    }
}

TEST_CASE("a count no group can have is refused", "[group-list]") {
    // What a shifted read produces. Clamping it to forty and carrying on is
    // what put forty members made of packet bytes in front of the player.
    std::vector<uint8_t> bytes = groupList(0x00, fourOthers(), 0x1001, false);
    // The member count sits after type, subgroup, flags, roles, guid, counter.
    const size_t countAt = 4 + 8 + 4;
    bytes[countAt + 3] = 0x54;
    bool ok = true;
    const GroupListData data = parsed(bytes, ok);
    CHECK_FALSE(ok);
    CHECK(data.members.empty());
}

TEST_CASE("a solo group carries no loot block", "[group-list]") {
    // Members-1 is nought when the player is alone, and the server writes the
    // leader guid and stops. Reading a loot method there would take it from
    // past the end.
    bool ok = false;
    const GroupListData data = parsed(groupList(0x00, {}, 0x1001, false), ok);
    REQUIRE(ok);
    CHECK(data.memberCount == 0);
    CHECK(data.leaderGuid == 0x1001);
}
