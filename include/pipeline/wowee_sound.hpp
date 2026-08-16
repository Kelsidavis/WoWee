#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Sound Catalog (.wsnd) - novel replacement for
// Blizzard's SoundEntries.dbc + SoundEntriesAdvanced.dbc.
// One file holds all sound metadata for a zone or feature: UI
// clicks, ambient loops, spell SFX, voice clips, combat hits.
// The runtime keys lookups by soundId; the catalog name is
// just a label for debugging / tooling.
//
// 3D positional sounds are described by minDistance (the
// radius within which the sound plays at full volume) and
// maxDistance (beyond which it is fully attenuated). Loop
// flag, stream flag, and 3D flag are packed into the flags
// field so a future format extension can reuse other bits.
//
// Binary layout (little-endian):
//   magic[4]            = "WSND"
//   version (uint32)    = current 1
//   nameLen (uint32) + name bytes              -- catalog label
//   entryCount (uint32)
//   entries (each):
//     soundId (uint32)
//     kind (uint8)
//     pad[3]
//     flags (uint32)
//     volume (float)
//     minDistance (float)
//     maxDistance (float)
//     filePathLen (uint32) + filePath bytes
//     labelLen (uint32) + label bytes
struct WoweeSound {
    enum Kind : uint8_t {
        Sfx     = 0,
        Music   = 1,
        Ambient = 2,
        Ui      = 3,
        Voice   = 4,
        Spell   = 5,
        Combat  = 6,
    };

    enum Flags : uint32_t {
        Loop   = 0x01,
        Is3D   = 0x02,
        Stream = 0x04,
    };

    struct Entry {
        uint32_t soundId = 0;
        uint8_t kind = Sfx;
        uint32_t flags = 0;
        float volume = 1.0f;
        float minDistance = 5.0f;
        float maxDistance = 30.0f;
        std::string filePath;
        std::string label;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    // Lookup by soundId - returns nullptr if not present.
    [[nodiscard]] const Entry* findById(uint32_t soundId) const;

    static const char* kindName(uint8_t k);
};

class WoweeSoundLoader {
public:
    static bool save(const WoweeSound& cat,
                     const std::string& basePath);
    static WoweeSound load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-sound-catalog* variants.
    //
    //   makeStarter - one entry per kind covering the common
    //                  sound categories (sfx / music / ambient
    //                  / ui / voice / spell / combat). Useful
    //                  as a template for hand-edit.
    //   makeAmbient - wilderness-style: bird-loop ambient +
    //                  wind ambient + footstep variants.
    //   makeTavern  - interior: crackling-fire ambient + crowd
    //                  murmur + drink-clink + door-creak.
    static WoweeSound makeStarter(const std::string& catalogName);
    static WoweeSound makeAmbient(const std::string& catalogName);
    static WoweeSound makeTavern(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
