#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Creature Template (.wcrt) - novel replacement
// for AzerothCore-style creature_template SQL tables PLUS
// the CreatureTemplate / CreatureFamily / CreatureType.dbc
// trio. The 14th open format added to the editor.
//
// This is the gameplay data side of creatures: HP, level
// range, faction, AI flags, NPC role bits (vendor /
// trainer / quest-giver / innkeeper), base damage, equipped
// gear references. The WSPN file says where creatures
// spawn; the WLOT file says what they drop on death; the
// WCRT file says what they ARE - the canonical metadata
// shared across every spawn instance.
//
// Cross-references:
//   WSPN.entry.entryId  → WCRT.entry.creatureId
//   WLOT.entry.creatureId → WCRT.entry.creatureId
//   WCRT.entry.equipped* → WIT.entry.itemId
//
// Binary layout (little-endian):
//   magic[4]            = "WCRT"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     creatureId (uint32)
//     displayId (uint32)
//     nameLen + name
//     subnameLen + subname
//     minLevel (uint16) / maxLevel (uint16)
//     baseHealth (uint32)        -- at minLevel
//     healthPerLevel (uint16)
//     baseMana (uint32)
//     manaPerLevel (uint16)
//     factionId (uint32)
//     npcFlags (uint32)
//     typeId (uint8) / familyId (uint8) / pad[2]
//     damageMin (uint32) / damageMax (uint32) / attackSpeedMs (uint32)
//     baseArmor (uint32)
//     walkSpeed (float) / runSpeed (float)
//     gossipId (uint32)            -- 0 if none
//     equippedMain (uint32)        -- WIT itemId, 0 if none
//     equippedOffhand (uint32)
//     equippedRanged (uint32)
//     aiFlags (uint32)
struct WoweeCreature {
    enum NpcFlags : uint32_t {
        Vendor       = 0x01,
        QuestGiver   = 0x02,
        Trainer      = 0x04,
        Banker       = 0x08,
        Innkeeper    = 0x10,
        FlightMaster = 0x20,
        Auctioneer   = 0x40,
        Repair       = 0x80,
        Stable       = 0x100,
    };

    enum TypeId : uint8_t {
        Beast      = 1,
        Dragon     = 2,
        Demon      = 3,
        Elemental  = 4,
        Giant      = 5,
        Undead     = 6,
        Humanoid   = 7,
        Critter    = 8,
        Mechanical = 9,
    };

    enum FamilyId : uint8_t {
        FamNone   = 0,
        FamWolf   = 1,
        FamCat    = 2,
        FamBear   = 3,
        FamBoar   = 4,
        FamRaptor = 5,
        FamHyena  = 6,
        FamSpider = 7,
        FamGorilla = 8,
        FamCrab   = 9,
    };

    enum AiFlags : uint32_t {
        AiPassive    = 0x01,    // never attacks unless attacked
        AiAggressive = 0x02,    // attacks players within aggro radius
        AiFleeLowHp  = 0x04,    // runs at low health
        AiCallHelp   = 0x08,    // calls allies into combat
        AiNoLeash    = 0x10,    // does not return to spawn point
    };

    struct Entry {
        uint32_t creatureId = 0;
        uint32_t displayId = 0;
        std::string name;
        std::string subname;       // e.g. "Innkeeper", "Stable Master"
        uint16_t minLevel = 1;
        uint16_t maxLevel = 1;
        uint32_t baseHealth = 50;
        uint16_t healthPerLevel = 10;
        uint32_t baseMana = 0;
        uint16_t manaPerLevel = 0;
        uint32_t factionId = 35;     // friendly default
        uint32_t npcFlags = 0;
        uint8_t typeId = Humanoid;
        uint8_t familyId = FamNone;
        uint32_t damageMin = 1;
        uint32_t damageMax = 3;
        uint32_t attackSpeedMs = 2000;
        uint32_t baseArmor = 0;
        float walkSpeed = 1.0f;
        float runSpeed = 1.14f;     // canonical WoW base
        uint32_t gossipId = 0;
        uint32_t equippedMain = 0;
        uint32_t equippedOffhand = 0;
        uint32_t equippedRanged = 0;
        uint32_t aiFlags = AiPassive;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by creatureId - nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t creatureId) const;

    static const char* typeName(uint8_t t);
    static const char* familyName(uint8_t f);
};

class WoweeCreatureLoader {
public:
    static bool save(const WoweeCreature& cat,
                     const std::string& basePath);
    static WoweeCreature load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-creatures* variants.
    //
    //   makeStarter  - 1 friendly humanoid (innkeeper), level
    //                   30, vendor + innkeeper flags.
    //   makeBandit   - 1 hostile humanoid (creatureId=1000,
    //                   matches WSPN camp + WLOT bandit table).
    //                   level 5..7, aggressive AI, equips a
    //                   sword (WIT itemId=1001).
    //   makeMerchants - 3 NPCs covering the WSPN village
    //                    creatures (innkeeper / smith / alchemist),
    //                    creatureIds 4001/4002/4003.
    static WoweeCreature makeStarter(const std::string& catalogName);
    static WoweeCreature makeBandit(const std::string& catalogName);
    static WoweeCreature makeMerchants(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
