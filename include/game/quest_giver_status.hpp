#pragma once

/// What the server says an NPC has for you, and the mark that goes over its
/// head.
///
/// The values are DIALOG_STATUS_* from the server's QuestDef.h and there are
/// eleven of them. This client named six, and two of those it named wrongly:
/// 7 was called "available, low level" when the server means available-with-
/// reputation, and low-level-available is 2 - a value nothing here had a name
/// for at all.
///
/// A status with no name matched no branch in any of the four places that draw
/// the mark, so nothing was drawn. Statuses 2, 3 and 4 are what a questgiver
/// reports once you out-level its quests, and 9 is a turn-in that keeps its
/// mark off the minimap - so the mark simply stopped appearing over exactly
/// the NPCs a player has most of by mid-level, and appeared over the rest.
///
/// Four places mapped the status to a symbol and a colour, and they are one
/// mapping: the nameplate, the target frame, the focus frame and the minimap.

#include <cstdint>

namespace wowee::game {

/// DIALOG_STATUS_*, as the server defines it.
enum class QuestGiverStatus : uint8_t {
    NONE                    = 0,
    UNAVAILABLE             = 1,
    LOW_LEVEL_AVAILABLE     = 2,   ///< ! you have out-levelled
    LOW_LEVEL_REWARD_REP    = 3,
    LOW_LEVEL_AVAILABLE_REP = 4,
    INCOMPLETE              = 5,   ///< ? started, not finished
    REWARD_REP              = 6,
    AVAILABLE_REP           = 7,
    AVAILABLE               = 8,   ///< ! the ordinary one
    REWARD2                 = 9,   ///< ? turn-in, with no minimap dot
    REWARD                  = 10,  ///< ? turn-in
};

/// The mark to draw over a questgiver, or an empty symbol for none.
struct QuestGiverMarker {
    const char* symbol = nullptr;  ///< "!", "?", or null to draw nothing
    bool dim = false;              ///< grey rather than gold
    const char* tooltip = nullptr;
    /// Whether this also earns a dot on the minimap. Status 9 is the one that
    /// does not, which is the only thing separating it from 10.
    bool onMinimap = false;
};

/// One mapping, so the nameplate and the minimap cannot disagree about what an
/// NPC is offering.
inline QuestGiverMarker questGiverMarker(QuestGiverStatus status) {
    switch (status) {
        case QuestGiverStatus::AVAILABLE:
        case QuestGiverStatus::AVAILABLE_REP:
            return {.symbol = "!", .dim = false, .tooltip = "Has a quest available", .onMinimap = true};
        case QuestGiverStatus::LOW_LEVEL_AVAILABLE:
        case QuestGiverStatus::LOW_LEVEL_AVAILABLE_REP:
            return {.symbol = "!", .dim = true, .tooltip = "Has a low-level quest available", .onMinimap = true};
        case QuestGiverStatus::REWARD:
        case QuestGiverStatus::REWARD_REP:
            return {.symbol = "?", .dim = false, .tooltip = "Quest ready to turn in", .onMinimap = true};
        // The same mark, and deliberately not on the minimap.
        case QuestGiverStatus::REWARD2:
            return {.symbol = "?", .dim = false, .tooltip = "Quest ready to turn in", .onMinimap = false};
        case QuestGiverStatus::LOW_LEVEL_REWARD_REP:
            return {.symbol = "?", .dim = true, .tooltip = "Quest ready to turn in", .onMinimap = true};
        case QuestGiverStatus::INCOMPLETE:
            return {.symbol = "?", .dim = true, .tooltip = "Quest in progress", .onMinimap = true};
        case QuestGiverStatus::NONE:
        case QuestGiverStatus::UNAVAILABLE:
            break;
    }
    return {};
}

}  // namespace wowee::game
