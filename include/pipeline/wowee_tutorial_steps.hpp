#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Tutorial Steps catalog (.wtur) -
// novel format covering what vanilla WoW had as
// a hard-coded LUA tipbox sequence (TutorialFrame.
// xml + Tutorial.lua client-side). Each WTUR entry
// binds one tutorial step to a trigger event
// (Login / ZoneEnter / LevelUp / ItemPickup /
// SkillTrain), an ordered stepIndex within that
// trigger group, a title + body for the popup,
// optional UI-element name to highlight, and a
// hide-after timer.
//
// Cross-references with previously-added formats:
//   WMS:  triggerValue for ZoneEnter is a WMS
//         mapId.
//   WIT:  triggerValue for ItemPickup is a WIT
//         itemId.
//   WSKL: triggerValue for SkillTrain is a WSKL
//         skillId.
//
// Binary layout (little-endian):
//   magic[4]            = "WTUR"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     tutId (uint32)
//     nameLen + name (admin label)
//     stepIndex (uint8)            - sequential
//                                     order within
//                                     triggerEvent
//     triggerEvent (uint8)         - 0=Login /
//                                     1=ZoneEnter /
//                                     2=LevelUp /
//                                     3=ItemPickup /
//                                     4=SkillTrain
//     pad0 (uint16)
//     triggerValue (uint32)        - interpretation
//                                     depends on
//                                     event (mapId
//                                     / itemId /
//                                     skillId / 0
//                                     for Login)
//     iconIndex (uint32)
//     hideAfterSec (uint32)        - auto-dismiss
//                                     timer (0 = no
//                                     auto-dismiss)
//     titleLen + title
//     bodyLen + body
//     targetLen + targetUIElementName  - name of UI
//                                         widget to
//                                         highlight
//                                         (empty =
//                                         no
//                                         highlight)
struct WoweeTutorialSteps {
    enum TriggerEvent : uint8_t {
        Login        = 0,
        ZoneEnter    = 1,
        LevelUp      = 2,
        ItemPickup   = 3,
        SkillTrain   = 4,
    };

    struct Entry {
        uint32_t tutId = 0;
        std::string name;
        uint8_t stepIndex = 0;
        uint8_t triggerEvent = Login;
        uint16_t pad0 = 0;
        uint32_t triggerValue = 0;
        uint32_t iconIndex = 0;
        uint32_t hideAfterSec = 0;
        std::string title;
        std::string body;
        std::string targetUIElementName;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t tutId) const;

    // Returns all steps that fire on a given event,
    // sorted by stepIndex. Used by the tutorial
    // dispatcher to play steps in order.
    [[nodiscard]] std::vector<const Entry*> findByEvent(uint8_t triggerEvent) const;
};

class WoweeTutorialStepsLoader {
public:
    static bool save(const WoweeTutorialSteps& cat,
                     const std::string& basePath);
    static WoweeTutorialSteps load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-tut* variants.
    //
    //   makeNewbieFlow   - 5 Login-trigger steps for
    //                       first-time players
    //                       (Welcome / Movement /
    //                       Interact / OpenQuestLog
    //                       / OpenInventory).
    //   makeLevelUpFlow  - 3 LevelUp-trigger steps
    //                       (level 1 unlock spells
    //                       hint / level 5 train
    //                       skill hint / level 10
    //                       talent unlock).
    //   makeBgFlow       - 3 ZoneEnter-trigger steps
    //                       gated to BG mapIds (BG
    //                       queue / capture flag /
    //                       turn in marks).
    static WoweeTutorialSteps makeNewbieFlow(const std::string& catalogName);
    static WoweeTutorialSteps makeLevelUpFlow(const std::string& catalogName);
    static WoweeTutorialSteps makeBgFlow(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
