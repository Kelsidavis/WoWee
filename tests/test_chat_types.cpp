#include <catch_amalgamated.hpp>

#include "game/world_packets.hpp"

using wowee::game::ChatType;

// A chat type is a byte off the wire, and the name built from it decides which
// event the interface hears: "CHAT_MSG_" with the type's name on the end. A
// value that is absent from this enum falls to the default, is named UNKNOWN,
// and the message is dropped by anything listening for the real name — no
// error, no log, just a line of chat that never appears.
//
// That is what happened to the whole run between LOOT and the battleground
// block: money, opening, tradeskills, pet info, combat misc, experience,
// honour and reputation were missing, so the loot line, the money line and
// "you gain 150 experience" went nowhere. These pin the numbers against
// AzerothCore's SharedDefines, which is where they were read from.

TEST_CASE("The narrated messages run unbroken from LOOT to the BG block",
          "[chat][wire]") {
    // Contiguous by construction on the server, so a gap here means one was
    // dropped again — which is exactly how they went missing the first time.
    REQUIRE(static_cast<int>(ChatType::LOOT) == 0x1B);
    REQUIRE(static_cast<int>(ChatType::MONEY) == 0x1C);
    REQUIRE(static_cast<int>(ChatType::OPENING) == 0x1D);
    REQUIRE(static_cast<int>(ChatType::TRADESKILLS) == 0x1E);
    REQUIRE(static_cast<int>(ChatType::PET_INFO) == 0x1F);
    REQUIRE(static_cast<int>(ChatType::COMBAT_MISC_INFO) == 0x20);
    REQUIRE(static_cast<int>(ChatType::COMBAT_XP_GAIN) == 0x21);
    REQUIRE(static_cast<int>(ChatType::COMBAT_HONOR_GAIN) == 0x22);
    REQUIRE(static_cast<int>(ChatType::COMBAT_FACTION_CHANGE) == 0x23);
    REQUIRE(static_cast<int>(ChatType::BG_SYSTEM_NEUTRAL) == 0x24);
}

TEST_CASE("The two that sit between the raid and battleground blocks",
          "[chat][wire]") {
    // FILTERED falls between RAID_BOSS_WHISPER and BATTLEGROUND, and RESTRICTED
    // just past BATTLEGROUND_LEADER. Both are easy to leave out precisely
    // because they are not next to anything they belong with.
    REQUIRE(static_cast<int>(ChatType::RAID_BOSS_WHISPER) == 0x2A);
    REQUIRE(static_cast<int>(ChatType::FILTERED) == 0x2B);
    REQUIRE(static_cast<int>(ChatType::BATTLEGROUND) == 0x2C);
    REQUIRE(static_cast<int>(ChatType::BATTLEGROUND_LEADER) == 0x2D);
    REQUIRE(static_cast<int>(ChatType::RESTRICTED) == 0x2E);
}

TEST_CASE("The types the player sends keep the values they always had",
          "[chat][wire]") {
    // Not new, and pinned anyway: these are what CMSG_MESSAGECHAT is built
    // with, so a shift here sends a party message to the guild.
    REQUIRE(static_cast<int>(ChatType::SYSTEM) == 0x00);
    REQUIRE(static_cast<int>(ChatType::SAY) == 0x01);
    REQUIRE(static_cast<int>(ChatType::PARTY) == 0x02);
    REQUIRE(static_cast<int>(ChatType::RAID) == 0x03);
    REQUIRE(static_cast<int>(ChatType::GUILD) == 0x04);
    REQUIRE(static_cast<int>(ChatType::OFFICER) == 0x05);
    REQUIRE(static_cast<int>(ChatType::YELL) == 0x06);
    REQUIRE(static_cast<int>(ChatType::WHISPER) == 0x07);
    REQUIRE(static_cast<int>(ChatType::RAID_WARNING) == 0x28);
}

TEST_CASE("Every value in the enum is distinct", "[chat][wire]") {
    // Two names on one byte would make the second unreachable, and the
    // compiler says nothing about it.
    const int all[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        static_cast<int>(ChatType::LOOT),
        static_cast<int>(ChatType::MONEY),
        static_cast<int>(ChatType::OPENING),
        static_cast<int>(ChatType::TRADESKILLS),
        static_cast<int>(ChatType::PET_INFO),
        static_cast<int>(ChatType::COMBAT_MISC_INFO),
        static_cast<int>(ChatType::COMBAT_XP_GAIN),
        static_cast<int>(ChatType::COMBAT_HONOR_GAIN),
        static_cast<int>(ChatType::COMBAT_FACTION_CHANGE),
        static_cast<int>(ChatType::BG_SYSTEM_NEUTRAL),
        static_cast<int>(ChatType::RAID_LEADER),
        static_cast<int>(ChatType::RAID_WARNING),
        static_cast<int>(ChatType::RAID_BOSS_EMOTE),
        static_cast<int>(ChatType::RAID_BOSS_WHISPER),
        static_cast<int>(ChatType::FILTERED),
        static_cast<int>(ChatType::BATTLEGROUND),
        static_cast<int>(ChatType::BATTLEGROUND_LEADER),
        static_cast<int>(ChatType::RESTRICTED),
        static_cast<int>(ChatType::ACHIEVEMENT),
        static_cast<int>(ChatType::GUILD_ACHIEVEMENT),
        static_cast<int>(ChatType::PARTY_LEADER),
    };
    const int count = static_cast<int>(sizeof(all) / sizeof(all[0]));
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            INFO("index " << i << " and " << j << " share value " << all[i]);
            REQUIRE(all[i] != all[j]);
        }
    }
}
