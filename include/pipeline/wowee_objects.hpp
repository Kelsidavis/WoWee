#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Game Object Template (.wgot) - novel replacement
// for AzerothCore-style gameobject_template SQL tables PLUS
// the Blizzard GameObjectDisplayInfo.dbc / GameObject types
// metadata. The 16th open format added to the editor.
//
// Game objects are the non-creature interactable scenery:
// chests (with loot), doors, buttons, mailboxes, herb / ore
// gathering nodes, fishing pools, signposts, mounts. Each
// has a displayId for the model, a typeId driving its
// interaction logic, and optional cross-references to a lock
// (future WLCK) and loot table (existing WLOT).
//
// Cross-references with previously-added formats:
//   WSPN.entry.entryId (kind=GameObject) → WGOT.entry.objectId
//   WGOT.entry.lootTableId               → WLOT.entry.creatureId
//                                           (loot tables are
//                                            universal - game
//                                            objects + creatures
//                                            both key by ID)
//
// Binary layout (little-endian):
//   magic[4]            = "WGOT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     objectId (uint32)
//     displayId (uint32)
//     nameLen + name
//     typeId (uint8) + pad[3]
//     size (float)
//     castBarLen + castBarCaption       -- e.g. "Mining"
//     requiredSkill (uint32)            -- 0 = none, else SkillLine ID
//     requiredSkillValue (uint32)
//     lockId (uint32)                   -- 0 = no lock
//     lootTableId (uint32)              -- 0 = no loot
//     minOpenTimeMs (uint32)
//     maxOpenTimeMs (uint32)
//     flags (uint32)
struct WoweeGameObject {
    enum TypeId : uint8_t {
        Door         = 0,
        Button       = 1,
        Chest        = 2,
        Container    = 3,
        QuestGiver   = 4,
        Text         = 5,
        Trap         = 6,
        Goober       = 7,    // generic activatable script object
        Transport    = 8,
        Mailbox      = 9,
        MineralNode  = 10,
        HerbNode     = 11,
        FishingNode  = 12,
        Mount        = 13,
        Sign         = 14,
        Bonfire      = 15,
    };

    enum Flags : uint32_t {
        Disabled            = 0x01,
        ScriptOnly          = 0x02,    // not interactable except by scripts
        UsableFromMount     = 0x04,
        Despawn             = 0x08,    // disappears after first use
        Frozen              = 0x10,    // never animates
        QuestGated          = 0x20,    // visible only with matching quest
    };

    struct Entry {
        uint32_t objectId = 0;
        uint32_t displayId = 0;
        std::string name;
        uint8_t typeId = Goober;
        float size = 1.0f;
        std::string castBarCaption;
        uint32_t requiredSkill = 0;
        uint32_t requiredSkillValue = 0;
        uint32_t lockId = 0;
        uint32_t lootTableId = 0;
        uint32_t minOpenTimeMs = 0;
        uint32_t maxOpenTimeMs = 0;
        uint32_t flags = 0;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by objectId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t objectId) const;

    static const char* typeName(uint8_t t);
};

class WoweeGameObjectLoader {
public:
    static bool save(const WoweeGameObject& cat,
                     const std::string& basePath);
    static WoweeGameObject load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-objects* variants.
    //
    //   makeStarter - 1 chest + 1 mailbox + 1 sign.
    //   makeDungeon - door + button + 2 chests (regular + boss
    //                  loot) + trap. The bandit chest in slot 2000
    //                  matches WLOT.makeBandit.
    //   makeGather  - gathering nodes: 1 herb (Peacebloom),
    //                  1 ore (Tin Vein), 1 fishing pool.
    static WoweeGameObject makeStarter(const std::string& catalogName);
    static WoweeGameObject makeDungeon(const std::string& catalogName);
    static WoweeGameObject makeGather(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
