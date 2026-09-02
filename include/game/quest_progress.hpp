#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace wowee::game {

// PLAYER_QUEST_LOG counter layouts differ by expansion:
// Classic packs four 6-bit counters beside the state byte, TBC stores four
// byte counters, and WotLK stores four uint16 counters across two fields.
inline std::array<uint32_t, 4> decodeQuestObjectiveCounts(
    uint8_t questLogStride, uint32_t firstWord, uint32_t secondWord = 0) {
    if (questLogStride >= 5) {
        return {
            firstWord & 0xFFFFu,
            (firstWord >> 16) & 0xFFFFu,
            secondWord & 0xFFFFu,
            (secondWord >> 16) & 0xFFFFu,
        };
    }
    if (questLogStride == 4) {
        return {
            firstWord & 0xFFu,
            (firstWord >> 8) & 0xFFu,
            (firstWord >> 16) & 0xFFu,
            (firstWord >> 24) & 0xFFu,
        };
    }
    return {
        firstWord & 0x3Fu,
        (firstWord >> 6) & 0x3Fu,
        (firstWord >> 12) & 0x3Fu,
        (firstWord >> 18) & 0x3Fu,
    };
}

inline uint8_t questObjectiveCountFieldOffset(uint8_t questLogStride) {
    // Classic combines state and counts in field 1. TBC/WotLK have a separate
    // state field and begin counters at field 2.
    return questLogStride <= 3 ? 1 : 2;
}

inline uint32_t normalizeQuestObjectiveEntry(uint32_t wireEntry) {
    // SMSG_QUESTUPDATE_ADD_KILL marks game-object entries with the high bit.
    return wireEntry & 0x7FFFFFFFu;
}

/// The quest slot's state byte, wherever the expansion keeps it.
inline uint32_t questSlotState(uint8_t questLogStride, uint32_t stateField) {
    return questLogStride <= 3 ? (stateField >> 24) & 0xFFu : stateField;
}

inline bool isQuestSlotComplete(uint8_t questLogStride, uint32_t stateField) {
    return (questSlotState(questLogStride, stateField) & 0x1u) != 0;
}

/// A timed quest whose timer ran out, or one the server failed for any other
/// reason. The bit sits beside the complete one in the same field - the
/// server's own header names them QUEST_STATE_COMPLETE = 0x0001 and
/// QUEST_STATE_FAIL = 0x0002 - so this was being read and dropped for as long
/// as completion was being read.
inline bool isQuestSlotFailed(uint8_t questLogStride, uint32_t stateField) {
    return (questSlotState(questLogStride, stateField) & 0x2u) != 0;
}

/// One objective line, the way the game writes it.
///
/// QUEST_MONSTERS_KILLED is "%s slain: %d/%d" and QUEST_OBJECTS_FOUND is
/// "%s: %d/%d" - a chest is found rather than slain. The name is whatever the
/// creature or game object query answered; while that is still out there is
/// nothing to name it with, and the generic word stands in for the moment.
///
/// The quest log and the tracker both read this, and both said "Creature
/// slain: 12/15" for every kill objective at once - which names none of the
/// fifteen things and is the one thing an objective line is for.
inline std::string questObjectiveLine(const std::string& name, bool isObject,
                                      uint32_t current, uint32_t required) {
    const std::string counts =
        std::to_string(current) + "/" + std::to_string(required);
    if (isObject) return (name.empty() ? "Object" : name) + ": " + counts;
    return (name.empty() ? "Creature" : name) + " slain: " + counts;
}

} // namespace wowee::game
