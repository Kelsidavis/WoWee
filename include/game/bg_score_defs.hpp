#pragma once

#include <cstdint>
#include <cstddef>

namespace wowee {
namespace game {

/**
 * Battleground score presentation, keyed by map.
 *
 * The server sends world states as bare key/value pairs - SMSG_INIT_WORLD_STATES
 * carries no labels - so which key means "Alliance flags" and what it counts up
 * to is client knowledge. This table is that knowledge.
 *
 * It lives here rather than beside a renderer because two surfaces present the
 * same scores: this client's own heads-up display, and FrameXML's
 * WorldStateAlwaysUpFrame through GetWorldStateUIInfo. Keeping one table means
 * a new battleground is described once.
 */
struct BgScoreDef {
    uint32_t    mapId;
    const char* name;
    uint32_t    allianceKey;   // world state key for the Alliance value
    uint32_t    hordeKey;      // world state key for the Horde value
    uint32_t    maxKey;        // world state key for the maximum (0 = use hardcodedMax)
    uint32_t    hardcodedMax;  // used when maxKey is 0
    const char* unit;          // suffix label ("flags", "resources"); empty = no maximum shown
};

inline constexpr BgScoreDef kBgScoreDefs[] = {
    // Warsong Gulch: 3 flag captures wins
    { .mapId = 489, .name = "Warsong Gulch",          .allianceKey = 1581, .hordeKey = 1582, .maxKey = 0,    .hardcodedMax = 3, .unit = "flags" },
    // Arathi Basin: 1600 resources wins
    { .mapId = 529, .name = "Arathi Basin",           .allianceKey = 1218, .hordeKey = 1219, .maxKey = 0, .hardcodedMax = 1600, .unit = "resources" },
    // Alterac Valley: reinforcements count down from 600 / 800 etc.
    {  .mapId = 30, .name = "Alterac Valley",         .allianceKey = 1322, .hordeKey = 1323, .maxKey = 0,  .hardcodedMax = 600, .unit = "reinforcements" },
    // Eye of the Storm: 1600 resources wins
    { .mapId = 566, .name = "Eye of the Storm",       .allianceKey = 2757, .hordeKey = 2758, .maxKey = 0, .hardcodedMax = 1600, .unit = "resources" },
    // Strand of the Ancients (WotLK)
    { .mapId = 607, .name = "Strand of the Ancients", .allianceKey = 3476, .hordeKey = 3477, .maxKey = 0,    .hardcodedMax = 4, .unit = "" },
    // Isle of Conquest (WotLK): reinforcements (300 default)
    { .mapId = 628, .name = "Isle of Conquest",       .allianceKey = 4221, .hordeKey = 4222, .maxKey = 0,  .hardcodedMax = 300, .unit = "reinforcements" },
};

/// What each battleground's per-player objective columns are called.
///
/// BuildObjectivesBlock sends a count and that many bare numbers - no labels,
/// the same way the world states carry none. Which column is which is client
/// knowledge, and this is that knowledge for the scoreboard the way the table
/// above is for the heads-up score.
///
/// Read off each battleground's own BuildObjectivesBlock, in the order it
/// writes them. The count is part of the contract: Alterac Valley sends five
/// and Eye of the Storm one, so a reader that assumes two is wrong twice.
struct BgObjectiveColumns {
    uint32_t    mapId;
    const char* labels[5];   // nullptr-terminated within the row
};

inline constexpr BgObjectiveColumns kBgObjectiveColumns[] = {
    { .mapId = 489, .labels = {"Flags Captured", "Flags Returned", nullptr, nullptr, nullptr} },
    { .mapId = 529, .labels = {"Bases Assaulted", "Bases Defended", nullptr, nullptr, nullptr} },
    {  .mapId = 30, .labels = {"Graveyards Assaulted", "Graveyards Defended",
            "Towers Assaulted", "Towers Defended", "Mines Captured"} },
    { .mapId = 566, .labels = {"Flags Captured", nullptr, nullptr, nullptr, nullptr} },
    { .mapId = 607, .labels = {"Demolishers Destroyed", "Gates Destroyed", nullptr, nullptr, nullptr} },
    { .mapId = 628, .labels = {"Bases Assaulted", "Bases Defended", nullptr, nullptr, nullptr} },
};

/// The label for a column, or nullptr when the map is unknown or the index is
/// past what that battleground sends.
inline const char* bgObjectiveLabel(uint32_t mapId, size_t index) {
    if (index >= 5) return nullptr;
    for (const auto& c : kBgObjectiveColumns) {
        if (c.mapId == mapId) return c.labels[index];
    }
    return nullptr;
}

/// The definition for a map, or nullptr when the map is not a scored battleground.
inline const BgScoreDef* findBgScoreDef(uint32_t mapId) {
    for (const auto& d : kBgScoreDefs) {
        if (d.mapId == mapId) return &d;
    }
    return nullptr;
}

}  // namespace game
}  // namespace wowee
