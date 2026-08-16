#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spawn Catalog (.wspn) - novel replacement for
// the scattered creature_template / gameobject SQL tables
// AzerothCore-style servers use, plus the static doodad
// placements that ADT MDDF / MODF chunks encode. One file
// holds all spawn points for a zone, regardless of kind:
// creatures (NPCs, monsters), game objects (chests, doors,
// signs), and pure-render doodads (trees, rocks, grass).
//
// Server runtimes read it to know what to spawn; the editor
// reads it to draw spawn markers; the renderer reads the
// doodad subset to know what static props to draw without
// going through the server roundtrip.
//
// Binary layout (little-endian):
//   magic[4]            = "WSPN"
//   version (uint32)    = current 1
//   nameLen (uint32) + name bytes              -- catalog label
//   entryCount (uint32)
//   entries (each):
//     kind (uint8)
//     pad[3]
//     entryId (uint32)              -- creature_template/gameobject/displayId
//     position[3] (3*float)
//     rotation[3] (3*float)         -- euler XYZ in radians
//     scale (float)
//     flags (uint32)
//     respawnSec (uint32)           -- 0 = static (doodad)
//     factionId (uint32)            -- 0 if N/A (game objects, doodads)
//     questIdRequired (uint32)      -- 0 = no quest gating
//     wanderRadius (float)          -- creatures only; 0 for objects
//     labelLen (uint32) + label bytes
struct WoweeSpawns {
    enum Kind : uint8_t {
        Creature   = 0,
        GameObject = 1,
        Doodad     = 2,
    };

    enum Flags : uint32_t {
        Disabled       = 0x01,   // present in catalog but not spawned
        EventOnly      = 0x02,   // spawned only during a world event
        QuestPhased    = 0x04,   // visible only to players with quest state
    };

    struct Entry {
        uint8_t kind = Creature;
        uint32_t entryId = 0;
        glm::vec3 position{0};
        glm::vec3 rotation{0};
        float scale = 1.0f;
        uint32_t flags = 0;
        uint32_t respawnSec = 300;       // 5 minutes default
        uint32_t factionId = 0;
        uint32_t questIdRequired = 0;
        float wanderRadius = 5.0f;
        std::string label;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Per-kind counts - useful for the editor info display.
    [[nodiscard]] uint32_t countByKind(uint8_t k) const;

    static const char* kindName(uint8_t k);
};

class WoweeSpawnsLoader {
public:
    static bool save(const WoweeSpawns& cat,
                     const std::string& basePath);
    static WoweeSpawns load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spawns* variants.
    //
    //   makeStarter - one entry per kind: 1 creature + 1 game
    //                  object + 1 doodad, all near (0,0,0).
    //                  Useful as a template for hand-edit.
    //   makeCamp    - bandit-camp spawn cluster: 4 creatures
    //                  in a wander ring + 1 chest (game object)
    //                  + 2 tent doodads.
    //   makeVillage - small village: 6 NPC creatures spread
    //                  out + 2 game-object signs + 4 tree
    //                  doodads.
    static WoweeSpawns makeStarter(const std::string& catalogName);
    static WoweeSpawns makeCamp(const std::string& catalogName);
    static WoweeSpawns makeVillage(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
