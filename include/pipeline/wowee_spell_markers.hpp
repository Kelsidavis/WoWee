#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Spell Persistent Marker catalog (.wspm) -
// novel replacement for the SpellAreaTrigger.dbc +
// AreaTriggerCreateProperties combination that vanilla
// WoW used for AoE ground decals (Blizzard, Hurricane,
// Consecration, Death and Decay, Rain of Fire, Flame
// Strike), boss-arena hazard zones (Putricide poison
// pools, Sindragosa frost tombs), and environmental
// effects (Wintergrasp lightning storm strike radius,
// Silithus sandstorm cones).
//
// Each entry binds one spellId to a ground decal: a
// texture path, radius (in world units), duration,
// damage tick interval, edge-fade rendering mode, and
// stack/destroy semantics. The visual effects pipeline
// reads this catalog at spell cast time to spawn the
// right ground-tracked decal.
//
// Cross-references with previously-added formats:
//   WSPL: spellId references the WSPL spell catalog
//         (the spell whose cast creates this marker).
//   WSND: tickSoundId references the WSND sound catalog
//         (the per-tick audio cue, e.g. crackling fire).
//
// Binary layout (little-endian):
//   magic[4]            = "WSPM"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     markerId (uint32)
//     nameLen + name
//     descLen + description
//     spellId (uint32)
//     pathLen + groundTexturePath  - BLP/WOT path
//     radius (float)               - world units
//     duration (float)             - seconds (0 = until
//                                     caster cancels /
//                                     mana drains)
//     tickIntervalMs (uint32)      - ms between damage
//                                     ticks
//     decalColor (uint32)          - RGBA tint applied to
//                                     the texture
//     edgeFadeMode (uint8)         - Hard / SoftEdge /
//                                     Pulse
//     stackable (uint8)            - 0/1 bool
//     destroyOnCancel (uint8)      - 0/1: vanish when the
//                                     caster cancels the
//                                     channel
//     pad0 (uint8)
//     tickSoundId (uint32)         - 0 if silent
//     iconColorRGBA (uint32)
struct WoweeSpellMarkers {
    enum EdgeFadeMode : uint8_t {
        Hard      = 0,    // sharp circle edge - no fade
        SoftEdge  = 1,    // alpha-fade outer 20% of radius
        Pulse     = 2,    // sinusoidal alpha pulse, full
                           // radius
    };

    struct Entry {
        uint32_t markerId = 0;
        std::string name;
        std::string description;
        uint32_t spellId = 0;
        std::string groundTexturePath;
        float radius = 8.0f;
        float duration = 8.0f;
        uint32_t tickIntervalMs = 1000;
        uint32_t decalColor = 0xFFFFFFFFu;
        uint8_t edgeFadeMode = SoftEdge;
        uint8_t stackable = 0;
        uint8_t destroyOnCancel = 1;
        uint8_t pad0 = 0;
        uint32_t tickSoundId = 0;
        uint32_t iconColorRGBA = 0xFFFFFFFFu;
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t markerId) const;

    // Returns the marker (if any) bound to the given
    // spellId. Used by the spell-cast pipeline to look
    // up "which decal does this spell spawn?"
    [[nodiscard]] const Entry* findBySpell(uint32_t spellId) const;
};

class WoweeSpellMarkersLoader {
public:
    static bool save(const WoweeSpellMarkers& cat,
                     const std::string& basePath);
    static WoweeSpellMarkers load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-spm* variants.
    //
    //   makeMageAoE      - 4 mage AoE ground spells
    //                       (Blizzard / Flamestrike /
    //                       BlastWave / Frost Nova
    //                       visual ring).
    //   makeRaidHazards  - 5 boss-arena hazard zones
    //                       (Putricide poison pool /
    //                       Sindragosa frost tomb /
    //                       Saurfang blood-frenzy bonus /
    //                       DBS shadow puddle / Marrowgar
    //                       Bone Storm radius).
    //   makeEnvironment  - 3 environmental effects
    //                       (Wintergrasp lightning strike
    //                       radius / Silithus sandstorm
    //                       cone / open-world blizzard
    //                       zone).
    static WoweeSpellMarkers makeMageAoE(const std::string& catalogName);
    static WoweeSpellMarkers makeRaidHazards(const std::string& catalogName);
    static WoweeSpellMarkers makeEnvironment(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
