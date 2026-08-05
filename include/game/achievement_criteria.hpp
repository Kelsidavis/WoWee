#pragma once

#include <cstdint>

#include "network/packet.hpp"

namespace wowee::game {

/// One criteria's progress, as the server sends it.
///
/// Two messages carry this record and they carry it identically:
/// SMSG_CRITERIA_UPDATE for a single criteria that just moved, and the second
/// half of SMSG_ALL_ACHIEVEMENT_DATA for every criteria at login. AzerothCore
/// writes both from the same six lines — AchievementMgr::SendCriteriaUpdate and
/// BuildAllDataPacket — so they are read from one place here.
struct CriteriaProgressRecord {
    uint32_t criteriaId = 0;
    /// How far along. This is the number the achievement panel shows.
    uint64_t counter = 0;
    /// Whose progress it is. Always the player on these two messages; it
    /// differs on the inspect reply, which shares the same builder.
    uint64_t playerGuid = 0;
    /// 1 marks a timed criteria whose window ran out, which is the server
    /// telling the client to show the counter at zero rather than at `counter`.
    uint32_t flags = 0;
    /// Packed time the counter last moved.
    uint32_t date = 0;
    /// Seconds since then. For a timed criteria this is how far into the
    /// window the player is; the window's length is in Achievement_Criteria.dbc
    /// rather than on the wire.
    uint32_t timeElapsed = 0;
};

/// Reads everything after the criteria id, which the caller has already taken.
///
/// The id is left to the caller because SMSG_ALL_ACHIEVEMENT_DATA ends each of
/// its two lists with an id of -1 and the caller has to test for that sentinel
/// before it can know there is a record here at all.
///
/// **The counter is a packed guid, not a uint64.** It is written with
/// appendPackGUID — a mask byte and then only the non-zero bytes — so a counter
/// of five is two bytes on the wire and not eight. Reading it as a fixed uint64
/// swallows the player guid behind it and puts every following field out of
/// place, and in the login message, where records repeat until a sentinel, the
/// first misread takes the rest of the list with it: the sentinel is then never
/// where the reader is looking, so the loop reads whatever the packet happens to
/// hold as ids and counters until it runs out. That was live until 2026-08-05,
/// and it is why criteria progress at login was a list of numbers that matched
/// nothing.
///
/// Returns false if the packet ends mid-record, leaving `out` untouched from
/// that point — a truncated record must not become an entry, since a criteria
/// id read out of noise is indistinguishable from a real one.
inline bool readCriteriaProgressTail(network::Packet& packet,
                                     CriteriaProgressRecord& out) {
    // The two packed fields are one mask byte plus up to eight, so the smallest
    // possible tail is two mask bytes and the four fixed words behind them.
    if (!packet.hasRemaining(2 + 16)) return false;
    out.counter = packet.readPackedGuid();
    out.playerGuid = packet.readPackedGuid();
    if (!packet.hasRemaining(16)) return false;
    out.flags = packet.readUInt32();
    out.date = packet.readUInt32();
    out.timeElapsed = packet.readUInt32();
    // Written twice: seconds since the date, and then the same again except on
    // an "average" criteria, where the second is seconds since the character
    // was created. Nothing here distinguishes the two, so the second is read
    // and dropped rather than guessed at.
    packet.readUInt32();
    return true;
}

}  // namespace wowee::game
