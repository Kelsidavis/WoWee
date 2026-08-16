#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Emote Definition catalog (.wemo) - novel
// replacement for the hardcoded EmotesText.dbc /
// EmotesTextSound.dbc / EmotesTextData.dbc trio that maps
// /slash-emote commands (e.g. /dance, /wave, /laugh) to
// their visible text, animation ID, and per-race voice
// audio. Each entry is one emote definition.
//
// Cross-references with previously-added formats:
//   WANI: animationId references the WANI animation
//         catalog. Emote anims map to AnimationData.dbc
//         IDs in the original system (e.g. ANIM_DANCE=29,
//         ANIM_WAVE=70).
//   WSND: soundId references the WSND sound catalog (the
//         per-race voice clip; falls back to no sound if
//         soundId=0).
//   WCHC: sex bit constraints use WCHC's gender enum
//         (0=both, 1=male, 2=female). Race-restricted
//         emotes (e.g. orcgrunt) embed the requiredRace
//         field.
//
// Binary layout (little-endian):
//   magic[4]            = "WEMO"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     emoteId (uint32)
//     nameLen + name
//     descLen + description
//     cmdLen + slashCommand     - e.g. "dance" (no slash)
//     animationId (uint32)
//     soundId (uint32)
//     targetMsgLen + targetMessage      - "%s waves at %s"
//     noTargetMsgLen + noTargetMessage  - "%s waves"
//     emoteKind (uint8)         - Social / Combat /
//                                  RolePlay / System
//     sex (uint8)               - 0=both, 1=male, 2=female
//     requiredRace (uint8)      - 0=any, else WCHC race
//                                  bit
//     ttsHint (uint8)           - Whisper / Yell / Talk /
//                                  Silent for accessibility
//                                  TTS engines
//     iconColorRGBA (uint32)
struct WoweeEmotes {
    enum EmoteKind : uint8_t {
        Social    = 0,    // wave, bow, laugh, etc.
        Combat    = 1,    // roar, threaten, charge
        RolePlay  = 2,    // bonk, ponder, soothe
        System    = 3,    // ready, group, AFK / DND
                           // status emotes
    };

    enum SexFilter : uint8_t {
        SexBoth   = 0,
        MaleOnly  = 1,
        FemaleOnly = 2,
    };

    enum TtsHint : uint8_t {
        TtsTalk    = 0,    // normal speaking volume
        TtsWhisper = 1,    // soft, intimate
        TtsYell    = 2,    // loud, aggressive
        TtsSilent  = 3,    // pure animation, no audio TTS
    };

    struct Entry {
        uint32_t emoteId = 0;
        std::string name;
        std::string description;
        std::string slashCommand;      // e.g. "dance"
        uint32_t animationId = 0;
        uint32_t soundId = 0;
        std::string targetMessage;     // "%s waves at %s"
        std::string noTargetMessage;   // "%s waves"
        uint8_t emoteKind = Social;
        uint8_t sex = SexBoth;
        uint8_t requiredRace = 0;
        uint8_t ttsHint = TtsTalk;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t emoteId) const;

    // Looks up an emote by its slash-command string (the
    // bit after the slash - "dance", "wave"). Used by the
    // chat input parser to dispatch /<cmd> to the right
    // emote without scanning the full table.
    [[nodiscard]] const Entry* findByCommand(const std::string& cmd) const;

    // Returns all emotes of one kind - used by the social
    // wheel UI to populate per-tab listings (Social /
    // Combat / RolePlay / System).
    [[nodiscard]] std::vector<const Entry*> findByKind(uint8_t emoteKind) const;
};

class WoweeEmotesLoader {
public:
    static bool save(const WoweeEmotes& cat,
                     const std::string& basePath);
    static WoweeEmotes load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-emo* variants.
    //
    //   makeBasic    - 8 universal social emotes (wave,
    //                   bow, laugh, cheer, cry, sleep,
    //                   kneel, applaud).
    //   makeCombat   - 5 combat-themed emotes (roar,
    //                   threaten, charge, victory,
    //                   surrender).
    //   makeRolePlay - 6 RP-focused emotes (bonk, ponder,
    //                   soothe, plead, shoo, scoff).
    static WoweeEmotes makeBasic(const std::string& catalogName);
    static WoweeEmotes makeCombat(const std::string& catalogName);
    static WoweeEmotes makeRolePlay(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
