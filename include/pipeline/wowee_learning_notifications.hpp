#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Learning Notification catalog (.wldn) -
// novel replacement for the hardcoded server-side
// milestone messages that fire when a player crosses a
// progression threshold ("You can now learn Apprentice
// Riding" at level 20, "Dual specialization is now
// available" at level 40, "You have unlocked the
// auction house"). Each entry binds one trigger
// (LevelReach / FactionStanding / ItemAcquired / etc.)
// to a delivery channel (RaidWarning banner /
// SystemMsg / Subtitle / Tutorial popup) and an optional
// fanfare sound.
//
// Cross-references with previously-added formats:
//   WSND: soundId references the WSND sound catalog
//         (the per-notification fanfare).
//   WSPL: when triggerKind=SpellLearned, triggerValue
//         is a WSPL spellId.
//   WIT:  when triggerKind=ItemAcquired, triggerValue
//         is a WIT itemId.
//   WQTM: when triggerKind=QuestComplete, triggerValue
//         is a WQTM questId.
//   WMS:  when triggerKind=ZoneEntered, triggerValue is
//         a WMS areaId.
//
// The triggerValue field is polymorphic - its semantics
// depend on triggerKind. The validator can't fully
// cross-check without all referenced catalogs in the
// directory; it does range-check what it can per kind.
//
// Binary layout (little-endian):
//   magic[4]            = "WLDN"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     notificationId (uint32)
//     nameLen + name
//     descLen + description
//     msgLen + messageText
//     triggerKind (uint8)        - LevelReach /
//                                   FactionStanding /
//                                   ItemAcquired /
//                                   QuestComplete /
//                                   SpellLearned /
//                                   ZoneEntered
//     channelKind (uint8)        - RaidWarning /
//                                   SystemMsg /
//                                   Subtitle / Tutorial /
//                                   MOTD
//     factionFilter (uint8)      - 1=A / 2=H / 3=Both
//     pad0 (uint8)
//     triggerValue (int32)       - level / standing /
//                                   itemId / etc.
//     soundId (uint32)           - 0 if silent
//     minTotalTimePlayed (uint32)- seconds; 0 = always
//                                   fire (else only
//                                   first-time players
//                                   below threshold)
//     iconColorRGBA (uint32)
struct WoweeLearningNotifications {
    enum TriggerKind : uint8_t {
        LevelReach     = 0,    // triggerValue = level
        FactionStanding = 1,   // triggerValue = standing
                               // value (Hated=-42000,
                               // Exalted=42000)
        ItemAcquired   = 2,    // triggerValue = itemId
        QuestComplete  = 3,    // triggerValue = questId
        SpellLearned   = 4,    // triggerValue = spellId
        ZoneEntered    = 5,    // triggerValue = areaId
    };

    enum ChannelKind : uint8_t {
        RaidWarning  = 0,    // SMSG_RAID_WARNING red
                              // banner across center screen
        SystemMsg    = 1,    // standard system channel
                              // chat line
        Subtitle     = 2,    // bottom-of-screen tutorial
                              // subtitle (fade after 5s)
        Tutorial     = 3,    // tutorial popup with image
                              // (player must dismiss)
        MOTDAppend   = 4,    // appended to next session
                              // login MOTD chain
    };

    enum FactionFilter : uint8_t {
        AllianceOnly = 1,
        HordeOnly    = 2,
        Both         = 3,
    };

    struct Entry {
        uint32_t notificationId = 0;
        std::string name;
        std::string description;
        std::string messageText;
        uint8_t triggerKind = LevelReach;
        uint8_t channelKind = SystemMsg;
        uint8_t factionFilter = Both;
        uint8_t pad0 = 0;
        int32_t triggerValue = 0;
        uint32_t soundId = 0;
        uint32_t minTotalTimePlayed = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t notificationId) const;

    // Returns all notifications of one trigger kind. Used
    // by the per-trigger dispatcher (level-up logic
    // queries kind=LevelReach; quest complete logic
    // queries kind=QuestComplete; etc.) to scope the
    // search.
    [[nodiscard]] std::vector<const Entry*> findByTrigger(uint8_t triggerKind) const;
};

class WoweeLearningNotificationsLoader {
public:
    static bool save(const WoweeLearningNotifications& cat,
                     const std::string& basePath);
    static WoweeLearningNotifications load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-ldn* variants.
    //
    //   makeLevelMilestones - 5 LevelReach notifications
    //                          covering the canonical
    //                          unlock thresholds (mounts
    //                          at 20, talent reset gold
    //                          at 30, epic mount at 40,
    //                          dual spec at 40, flying
    //                          mount at 60).
    //   makeAccountUnlocks  - 4 ItemAcquired / SpellLearned
    //                          notifications for major
    //                          UI unlocks (mailbox usage,
    //                          auction house, dual spec
    //                          activation, transmog vendor).
    //   makeReputation      - 3 FactionStanding milestones
    //                          (Honored / Revered / Exalted
    //                          with a major faction).
    static WoweeLearningNotifications makeLevelMilestones(const std::string& catalogName);
    static WoweeLearningNotifications makeAccountUnlocks(const std::string& catalogName);
    static WoweeLearningNotifications makeReputation(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
