#include "pipeline/wowee_player_movement_anim.hpp"
#include "pipeline/wowee_binary_io.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace wowee {
namespace pipeline {

namespace {

constexpr char kMagic[4] = {'W', 'P', 'H', 'M'};
constexpr uint32_t kVersion = 1;
constexpr char kExtension[] = ".wphm";

} // namespace

const WoweePlayerMovementAnim::Entry*
WoweePlayerMovementAnim::findById(uint32_t mapId) const {
    for (const auto& e : entries)
        if (e.mapId == mapId) return &e;
    return nullptr;
}

const WoweePlayerMovementAnim::Entry*
WoweePlayerMovementAnim::find(uint8_t raceId,
                                uint8_t genderId,
                                uint8_t state) const {
    for (const auto& e : entries) {
        if (e.raceId == raceId &&
            e.genderId == genderId &&
            e.movementState == state) return &e;
    }
    return nullptr;
}

std::vector<const WoweePlayerMovementAnim::Entry*>
WoweePlayerMovementAnim::findByRaceGender(uint8_t raceId,
                                            uint8_t genderId) const {
    std::vector<const Entry*> out;
    for (const auto& e : entries) {
        if (e.raceId == raceId && e.genderId == genderId)
            out.push_back(&e);
    }
    return out;
}

bool WoweePlayerMovementAnimLoader::save(
    const WoweePlayerMovementAnim& cat,
    const std::string& basePath) {
    return saveCatalog(cat, basePath, kMagic, kVersion, kExtension,
                       [](std::ofstream& os, const auto& e) {
        writePOD(os, e.mapId);
        writePOD(os, e.raceId);
        writePOD(os, e.genderId);
        writePOD(os, e.movementState);
        writePOD(os, e.pad0);
        writePOD(os, e.baseAnimId);
        writePOD(os, e.variantAnimId);
        writePOD(os, e.transitionMs);
        writePOD(os, e.pad1);
    });
}

WoweePlayerMovementAnim WoweePlayerMovementAnimLoader::load(
    const std::string& basePath) {
    return loadCatalog<WoweePlayerMovementAnim>(basePath, kMagic, kVersion, kExtension,
                              [](std::ifstream& is, WoweePlayerMovementAnim::Entry& e) {
        if (!readPOD(is, e.mapId) ||
            !readPOD(is, e.raceId) ||
            !readPOD(is, e.genderId) ||
            !readPOD(is, e.movementState) ||
            !readPOD(is, e.pad0) ||
            !readPOD(is, e.baseAnimId) ||
            !readPOD(is, e.variantAnimId) ||
            !readPOD(is, e.transitionMs) ||
            !readPOD(is, e.pad1)) { return false; }
                                  return true;
                              });
}

bool WoweePlayerMovementAnimLoader::exists(
    const std::string& basePath) {
    return catalogExists(basePath, kExtension);
}

namespace {

// Helper to build an 8-state machine for one
// (raceId, genderId) pair. Canonical M2 anim ids:
//   Stand=0, Death=1, Walk=4, Run=5, Swim=12,
//   Fly=68 (vanilla had no flying except taxi),
//   Sit=20, Mount=91.
struct StateRow {
    uint8_t state;
    uint32_t baseAnim;
    uint32_t variantAnim;
    uint16_t transitionMs;
};

WoweePlayerMovementAnim::Entry makeEntry(uint32_t mapId,
                                            uint8_t race,
                                            uint8_t gender,
                                            const StateRow& r) {
    WoweePlayerMovementAnim::Entry e;
    e.mapId = mapId;
    e.raceId = race;
    e.genderId = gender;
    e.movementState = r.state;
    e.baseAnimId = r.baseAnim;
    e.variantAnimId = r.variantAnim;
    e.transitionMs = r.transitionMs;
    return e;
}

void appendRaceGender(WoweePlayerMovementAnim& c,
                       uint32_t baseId, uint8_t race,
                       uint8_t gender,
                       const std::vector<StateRow>& rows) {
    using P = WoweePlayerMovementAnim;
    for (size_t i = 0; i < rows.size(); ++i) {
        c.entries.push_back(makeEntry(
            baseId + static_cast<uint32_t>(i),
            race, gender, rows[i]));
    }
    (void)P::StateIdle;
}

} // namespace

WoweePlayerMovementAnim
WoweePlayerMovementAnimLoader::makeHumanMovement(
    const std::string& catalogName) {
    using P = WoweePlayerMovementAnim;
    WoweePlayerMovementAnim c;
    c.name = catalogName;
    // Human Male: variantAnim on Walk = drunk-walk
    // sequence (anim id 39 in the canonical M2 table).
    // Other states have no variant.
    appendRaceGender(c, 1000, 1 /* Human */, 0 /* M */, {
        {.state = P::StateIdle,  .baseAnim = 0,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateWalk,  .baseAnim = 4, .variantAnim = 39, .transitionMs = 250},
        {.state = P::StateRun,   .baseAnim = 5,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateSwim, .baseAnim = 12,  .variantAnim = 0, .transitionMs = 350},
        {.state = P::StateFly,  .baseAnim = 68,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateSit,  .baseAnim = 20,  .variantAnim = 0, .transitionMs = 300},
        {.state = P::StateMount,.baseAnim = 91,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateDeath, .baseAnim = 1,  .variantAnim = 0, .transitionMs = 100},
    });
    // Human Female: identical state shape but anim
    // base ids differ (M2 sex models have separate
    // anim tables) - using same numeric ids here as
    // placeholder; in production these would be the
    // female-model-specific anim indices.
    appendRaceGender(c, 1100, 1, 1, {
        {.state = P::StateIdle,  .baseAnim = 0,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateWalk,  .baseAnim = 4, .variantAnim = 39, .transitionMs = 250},
        {.state = P::StateRun,   .baseAnim = 5,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateSwim, .baseAnim = 12,  .variantAnim = 0, .transitionMs = 350},
        {.state = P::StateFly,  .baseAnim = 68,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateSit,  .baseAnim = 20,  .variantAnim = 0, .transitionMs = 300},
        {.state = P::StateMount,.baseAnim = 91,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateDeath, .baseAnim = 1,  .variantAnim = 0, .transitionMs = 100},
    });
    return c;
}

WoweePlayerMovementAnim
WoweePlayerMovementAnimLoader::makeOrcMovement(
    const std::string& catalogName) {
    using P = WoweePlayerMovementAnim;
    WoweePlayerMovementAnim c;
    c.name = catalogName;
    // Orc Run uses a more aggressive variant (anim 17
    // = AttackRun) for war-stance flavor.
    appendRaceGender(c, 2000, 2 /* Orc */, 0, {
        {.state = P::StateIdle,  .baseAnim = 0,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateWalk,  .baseAnim = 4,  .variantAnim = 0, .transitionMs = 250},
        {.state = P::StateRun,   .baseAnim = 5, .variantAnim = 17, .transitionMs = 200},
        {.state = P::StateSwim, .baseAnim = 12,  .variantAnim = 0, .transitionMs = 350},
        {.state = P::StateFly,  .baseAnim = 68,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateSit,  .baseAnim = 20,  .variantAnim = 0, .transitionMs = 300},
        {.state = P::StateMount,.baseAnim = 91,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateDeath, .baseAnim = 1,  .variantAnim = 0, .transitionMs = 100},
    });
    appendRaceGender(c, 2100, 2, 1, {
        {.state = P::StateIdle,  .baseAnim = 0,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateWalk,  .baseAnim = 4,  .variantAnim = 0, .transitionMs = 250},
        {.state = P::StateRun,   .baseAnim = 5, .variantAnim = 17, .transitionMs = 200},
        {.state = P::StateSwim, .baseAnim = 12,  .variantAnim = 0, .transitionMs = 350},
        {.state = P::StateFly,  .baseAnim = 68,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateSit,  .baseAnim = 20,  .variantAnim = 0, .transitionMs = 300},
        {.state = P::StateMount,.baseAnim = 91,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateDeath, .baseAnim = 1,  .variantAnim = 0, .transitionMs = 100},
    });
    return c;
}

WoweePlayerMovementAnim
WoweePlayerMovementAnimLoader::makeUndeadMovement(
    const std::string& catalogName) {
    using P = WoweePlayerMovementAnim;
    WoweePlayerMovementAnim c;
    c.name = catalogName;
    // Undead Run uses a "shambling" variant anim (38)
    // as the wounded-low-health renderer override.
    // Walk uses a stiffer cadence variant (40).
    appendRaceGender(c, 5000, 5 /* Undead */, 0, {
        {.state = P::StateIdle,  .baseAnim = 0,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateWalk,  .baseAnim = 4, .variantAnim = 40, .transitionMs = 250},
        {.state = P::StateRun,   .baseAnim = 5, .variantAnim = 38, .transitionMs = 200},
        {.state = P::StateSwim, .baseAnim = 12,  .variantAnim = 0, .transitionMs = 400},  // slower blend
                                        //  - undead aren't
                                        //  graceful in
                                        //  water
        {.state = P::StateFly,  .baseAnim = 68,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateSit,  .baseAnim = 20,  .variantAnim = 0, .transitionMs = 300},
        {.state = P::StateMount,.baseAnim = 91,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateDeath, .baseAnim = 1,  .variantAnim = 0, .transitionMs = 100},
    });
    appendRaceGender(c, 5100, 5, 1, {
        {.state = P::StateIdle,  .baseAnim = 0,  .variantAnim = 0, .transitionMs = 200},
        {.state = P::StateWalk,  .baseAnim = 4, .variantAnim = 40, .transitionMs = 250},
        {.state = P::StateRun,   .baseAnim = 5, .variantAnim = 38, .transitionMs = 200},
        {.state = P::StateSwim, .baseAnim = 12,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateFly,  .baseAnim = 68,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateSit,  .baseAnim = 20,  .variantAnim = 0, .transitionMs = 300},
        {.state = P::StateMount,.baseAnim = 91,  .variantAnim = 0, .transitionMs = 400},
        {.state = P::StateDeath, .baseAnim = 1,  .variantAnim = 0, .transitionMs = 100},
    });
    return c;
}

} // namespace pipeline
} // namespace wowee
