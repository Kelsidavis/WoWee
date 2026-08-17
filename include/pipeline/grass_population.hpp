#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "pipeline/grass_profile.hpp"
#include "pipeline/grass_terrain.hpp"

// Turning "grass grows here" into actual blades.
//
// Every property of a blade comes from a hash of the world-space lattice cell
// it sits in, and from nothing else. Not the frame, not the camera, not the
// order tiles happened to load in. That is what lets the population be thrown
// away and rebuilt whenever the player moves without a single blade appearing
// to jump: rebuilding produces the same blades again (spec §36).

namespace wowee {
namespace pipeline {

/// One generated blade, in the same world space the terrain is indexed by.
struct GrassBladeSample {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float height = 0.0f;
    float facing = 0.0f;
    float width = 0.0f;
    float phase = 0.0f;
    /// Index into the profile table the shaders read colour and stiffness from.
    uint32_t profileIndex = 0;
    /// The tones of the terrain under the root, which the blade is coloured
    /// from; w of the upload carries whether they are real, so a population
    /// without them renders on the profile alone.
    glm::vec3 groundShadow{0.0f};
    glm::vec3 groundHighlight{0.0f};
    bool hasGroundColor = false;
    /// Under water: drab, and never in bloom.
    bool submerged = false;
};

/// What a ground effect's profile means for generating a blade: the scales
/// that change geometry and count, and the index the shaders look the rest up
/// by. Resolved by the caller, which owns the table.
struct GrassProfileRef {
    uint32_t index = 0;
    float heightScale = 1.0f;
    float widthScale = 1.0f;
    float densityScale = 1.0f;
};

/// Profile for a ground effect. An empty function means one profile for
/// everything, which is what the phase before this had.
using ProfileLookupFn = std::function<GrassProfileRef(uint32_t effectId)>;

struct GrassPopulationParams {
    /// Lattice pitch in yards. One candidate blade per cell.
    ///
    /// Pitch and width together decide how much of the ground the field
    /// actually covers, and coverage is what makes grass read as grass. At
    /// 0.32 pitch and 0.046 wide the field covered 14% of the ground: looking
    /// along it you see through many blades and it reads as a field, looking
    /// down at it you see 86% bare earth. That is the whole of "the grass
    /// disappears depending on which way I look".
    float spacing = 0.20f;
    /// Scales how many candidates survive, on top of terrain suitability.
    /// TerrainManager::getGroundClutterDensityScale() feeds this.
    float densityScale = 1.0f;
    /// Knee-high rather than shin-high: travelling wind waves are read from
    /// blade tips changing angle, and there is not enough tip on short grass
    /// for a wave to be visible crossing it.
    float baseHeight = 0.42f;
    float heightVariation = 0.45f;
    /// Wide enough to hold pixels at gameplay camera distances, and wide
    /// enough that the field covers ground. 0.024 was a realistic blade and an
    /// invisible one: one to two pixels at twenty yards, which sampling and
    /// upscaling simply ate. A blade here stands for a tuft rather than a
    /// single leaf - with pitch above, the field now covers about a third of
    /// the ground it stands on.
    float baseWidth = 0.09f;
    /// Mixed into every hash. Changing it reshuffles the whole world's grass.
    uint32_t seed = 0x9e3779b9u;
};

/// What the terrain says at a world position.
using SuitabilitySampler = std::function<GrassSuitability(float worldX, float worldY)>;

/// Fill `out` with blades within `radius` of (centerX, centerY).
///
/// The lattice is anchored to the world, not to the centre, so moving the
/// centre slides a window over a fixed population rather than regenerating a
/// different one. Stops at `maxBlades` and reports whether it had to.
///
/// `out` is cleared first.
bool populateArea(float centerX, float centerY, float radius,
                  const GrassPopulationParams& params,
                  const SuitabilitySampler& sample,
                  std::vector<GrassBladeSample>& out,
                  size_t maxBlades,
                  const ProfileLookupFn& profileFor = {});

} // namespace pipeline
} // namespace wowee
