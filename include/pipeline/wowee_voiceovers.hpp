#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Voiceover Audio catalog (.wvox) - novel
// replacement for the implicit per-NPC voice dialog
// system vanilla WoW encoded across CreatureTextSounds
// (server-side aggro / death barks), npc_text (gossip
// audio cross-references), and per-quest dialog blobs.
// Each entry binds one NPC to one voice clip for one
// triggering event (greeting on talk, aggro on combat
// start, special-mechanic shout during boss fight,
// death scream, quest completion line, etc.).
//
// Cross-references with previously-added formats:
//   WCRT: npcId references the WCRT creature catalog.
//   WSND: each WVOX entry can be cross-referenced to a
//         WSND entry by audioPath (the underlying sound
//         resource); WVOX adds the per-NPC, per-event
//         binding context that WSND alone doesn't carry.
//
// Binary layout (little-endian):
//   magic[4]            = "WVOX"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     voiceId (uint32)
//     nameLen + name
//     descLen + description
//     npcId (uint32)
//     eventKind (uint8)          - Greeting / Aggro /
//                                   Death / QuestStart /
//                                   QuestProgress /
//                                   QuestComplete /
//                                   Goodbye / Special /
//                                   Phase
//     genderHint (uint8)         - Male / Female / Both
//                                   for randomized casts
//     variantIndex (uint8)       - 0..N for multiple
//                                   lines per event
//                                   (random pick at
//                                   trigger time)
//     pad0 (uint8)
//     pathLen + audioPath        - sound resource path
//     transcriptLen + transcript - printable line text
//                                   for accessibility +
//                                   chat-bubble display
//     durationMs (uint32)
//     volumeDb (int8)            - relative volume
//                                   (-20 to +6 typical)
//     pad1 (uint8) / pad2 (uint8) / pad3 (uint8)
//     iconColorRGBA (uint32)
struct WoweeVoiceovers {
    enum EventKind : uint8_t {
        Greeting       = 0,    // initial NPC chat
        Aggro          = 1,    // combat start
        Death          = 2,    // NPC dies
        QuestStart     = 3,    // accept quest
        QuestProgress  = 4,    // mid-quest checkpoint
        QuestComplete  = 5,    // turn-in
        Goodbye        = 6,    // chat close
        Special        = 7,    // boss-fight mechanic call
        Phase          = 8,    // boss phase transition
    };

    enum GenderHint : uint8_t {
        Male   = 0,
        Female = 1,
        Both   = 2,
    };

    struct Entry {
        uint32_t voiceId = 0;
        std::string name;
        std::string description;
        uint32_t npcId = 0;
        uint8_t eventKind = Greeting;
        uint8_t genderHint = Both;
        uint8_t variantIndex = 0;
        uint8_t pad0 = 0;
        std::string audioPath;
        std::string transcript;
        uint32_t durationMs = 0;
        int8_t volumeDb = 0;
        uint8_t pad1 = 0;
        uint8_t pad2 = 0;
        uint8_t pad3 = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t voiceId) const;

    // Returns all voice entries for a given (npc, event)
    // pair. The trigger handler picks one randomly when
    // multiple variantIndex values are available - the
    // boss-aggro handler might have 3 lines and pick one
    // per encounter for variety.
    [[nodiscard]] std::vector<const Entry*> findForTrigger(uint32_t npcId,
                                                uint8_t eventKind) const;
};

class WoweeVoiceoversLoader {
public:
    static bool save(const WoweeVoiceovers& cat,
                     const std::string& basePath);
    static WoweeVoiceovers load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-vox* variants.
    //
    //   makeQuestgiver  - 5 voice clips for one quest
    //                      NPC (Greeting / QuestStart /
    //                      QuestProgress / QuestComplete
    //                      / Goodbye).
    //   makeBoss        - 6 boss voice clips with phase
    //                      milestones (Aggro / 75% /
    //                      50% / 25% / Special Mechanic
    //                      / Death).
    //   makeVendor      - 4 vendor voice clips (Greeting
    //                      / Buy = Goodbye / Sell /
    //                      Goodbye-final).
    static WoweeVoiceovers makeQuestgiver(const std::string& catalogName);
    static WoweeVoiceovers makeBoss(const std::string& catalogName);
    static WoweeVoiceovers makeVendor(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
