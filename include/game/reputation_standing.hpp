#pragma once

#include <cstdint>

// The eight standings a reputation can be at, and where each begins and ends.
//
// These thresholds were written out twice - once for the panel this client
// draws and once for the original interface's GetFactionInfo - and the two
// disagreeing would put the same faction at different standings depending on
// which window was open. Numbers like these do not fail when they drift; they
// render something plausible and wrong.
//
// Exalted is a thousand wide like the rest rather than open-ended: the bar
// reads out of a thousand, and giving it the width of revered drew a faction at
// exalted as an almost empty bar.

namespace wowee::game {

struct ReputationStanding {
    int         id;        ///< 1..8, as the interface numbers them
    const char* name;
    int32_t     floor;     ///< lowest value still at this standing
    int32_t     ceiling;   ///< highest value still at this standing
};

inline constexpr ReputationStanding kReputationStandings[8] = {
    {.id = 1, .name = "Hated",      .floor = -42000, .ceiling = -6001},
    {.id = 2, .name = "Hostile",     .floor = -6000, .ceiling = -3001},
    {.id = 3, .name = "Unfriendly",  .floor = -3000,    .ceiling = -1},
    {.id = 4, .name = "Neutral",         .floor = 0,  .ceiling = 2999},
    {.id = 5, .name = "Friendly",     .floor = 3000,  .ceiling = 8999},
    {.id = 6, .name = "Honored",      .floor = 9000, .ceiling = 20999},
    {.id = 7, .name = "Revered",     .floor = 21000, .ceiling = 41999},
    {.id = 8, .name = "Exalted",     .floor = 42000, .ceiling = 42999},
};

/// The standing's name by the index an item's requiredReputationRank uses.
///
/// Two id spaces name the same eight words and they are one apart: the
/// interface numbers standings 1..8, which is the `id` above, while an item's
/// required rank is 0..7 - the array index. Four places wrote the eight words
/// out again rather than pick one, and any of them could have indexed with the
/// wrong one of the two. Off by one here is an item that says it needs Revered
/// when it needs Honored, which is exactly as plausible.
inline constexpr const char* reputationRankName(uint32_t rank) {
    return rank < 8 ? kReputationStandings[rank].name : "Unknown";
}

/// Which standing a raw reputation value falls in. Below hated is still hated;
/// the server does not send lower.
inline constexpr const ReputationStanding& reputationStandingFor(int32_t value) {
    for (int i = 7; i > 0; --i) {
        if (value >= kReputationStandings[i].floor) return kReputationStandings[i];
    }
    return kReputationStandings[0];
}

} // namespace wowee::game
