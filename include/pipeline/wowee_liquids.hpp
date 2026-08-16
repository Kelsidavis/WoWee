#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wowee {
namespace pipeline {

// Wowee Open Liquid Type catalog (.wliq) - novel replacement
// for Blizzard's LiquidType.dbc plus the AzerothCore-style
// terrain liquid descriptor data. The 45th open format added
// to the editor.
//
// Defines liquid materials used by terrain (MCNK liquid
// layers), WMO interior pools, and procedurally-generated
// fluid bodies in custom zones. Each liquid pairs a render
// material (shader + texture array + flow vectors) with
// gameplay data (entry damage spell, swim immunity, ambient
// audio). Used by both the renderer (to draw the surface)
// and the physics tick (to apply DoTs / breath timer).
//
// Cross-references with previously-added formats:
//   WLIQ.entry.ambientSoundId → WSND.entry.soundId
//                                 (loop while submerged)
//   WLIQ.entry.splashSoundId  → WSND.entry.soundId
//                                 (entry / exit one-shot)
//   WLIQ.entry.damageSpellId  → WSPL.spellId
//                                 (DoT applied while in liquid)
//
// Binary layout (little-endian):
//   magic[4]            = "WLIQ"
//   version (uint32)    = current 1
//   nameLen + name (catalog label)
//   entryCount (uint32)
//   entries (each):
//     liquidId (uint32)
//     nameLen + name
//     descLen + description
//     shaderLen + shaderPath
//     matLen + materialPath
//     liquidKind (uint8) / fogColorR (uint8) /
//       fogColorG (uint8) / fogColorB (uint8)
//     fogDensity (float)
//     ambientSoundId (uint32)
//     splashSoundId (uint32)
//     damageSpellId (uint32)
//     damagePerSecond (uint32)
//     minimapColor (uint32)        - RGBA packed
//     flowDirection (float)        - radians
//     flowSpeed (float)
//     viscosity (float)            - 0=water, 1=thick slime
struct WoweeLiquid {
    enum LiquidKind : uint8_t {
        Water         = 0,    // standard fresh / sea water
        Magma         = 1,    // lava - DoT applied
        Slime         = 2,    // green slime (Naxx, Sludge Fields)
        OceanSalt     = 3,    // salt water - separate audio
        FelFire       = 4,    // green fel-burn - magical DoT
        HolyLight     = 5,    // shimmering light - heal-over-time
        TarOil        = 6,    // dark tar - slow movement
        AcidBog       = 7,    // greenish acid - armor damage
        FrozenWater   = 8,    // walkable surface (Wintergrasp ice)
        UnderworldGoo = 9,    // shadowfang / void liquid
    };

    struct Entry {
        uint32_t liquidId = 0;
        std::string name;
        std::string description;
        std::string shaderPath;
        std::string materialPath;
        uint8_t liquidKind = Water;
        uint8_t fogColorR = 0, fogColorG = 0, fogColorB = 0;
        float fogDensity = 0.0f;          // 0=clear, 1=opaque
        uint32_t ambientSoundId = 0;      // WSND cross-ref
        uint32_t splashSoundId = 0;       // WSND cross-ref
        uint32_t damageSpellId = 0;       // WSPL cross-ref
        uint32_t damagePerSecond = 0;     // raw HP if no spell
        uint32_t minimapColor = 0;        // RGBA packed
        float flowDirection = 0.0f;       // radians
        float flowSpeed = 0.0f;           // units / sec
        float viscosity = 0.0f;           // 0=water, 1=slime
    };

    std::string name;
    std::vector<Entry> entries;

    [[nodiscard]] bool isValid() const { return !entries.empty(); }

    [[nodiscard]] const Entry* findById(uint32_t liquidId) const;

    static const char* liquidKindName(uint8_t k);
};

class WoweeLiquidLoader {
public:
    static bool save(const WoweeLiquid& cat,
                     const std::string& basePath);
    static WoweeLiquid load(const std::string& basePath);
    static bool exists(const std::string& basePath);

    // Preset emitters used by --gen-liquids* variants.
    //
    //   makeStarter   - 3 stock liquids (Water / Magma /
    //                    Slime) covering the canonical fluid
    //                    triad in classic terrain.
    //   makeMagical   - 4 magical liquids (Fel Fire / Holy
    //                    Light / Underworld Goo / Cosmic
    //                    Plasma) for set-piece zones.
    //   makeHazardous - 3 high-damage liquids (Naxx Slime /
    //                    Acid Bog / Fel Lava) with damage
    //                    spells cross-ref WSPL.
    static WoweeLiquid makeStarter(const std::string& catalogName);
    static WoweeLiquid makeMagical(const std::string& catalogName);
    static WoweeLiquid makeHazardous(const std::string& catalogName);
};

} // namespace pipeline
} // namespace wowee
