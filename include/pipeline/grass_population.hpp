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
    /// How far from the viewer this blade may draw before it has sunk away,
    /// in yards; 0 means only the global range cap applies. Persistent per
    /// blade - it comes from the blade's own lattice level - so a ride
    /// toward it raises it smoothly out of the ground, and no rebuild can
    /// pop it.
    float fadeDistance = 0.0f;
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

/// Profile for a ground effect on a given area's ground. The area id is what
/// keys the per-zone biome overrides; a caller with no regional table just
/// ignores it. An empty function means one profile for everything, which is
/// what the phase before this had.
using ProfileLookupFn = std::function<GrassProfileRef(uint32_t effectId, uint32_t areaId)>;

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
    /// Radius of the innermost, full-density octave. Past it the lattice
    /// coarsens by powers of two - each distance doubling keeps one cell in
    /// four, chosen by a nested descent so every coarser ring's blades are a
    /// subset of the finer ring's. Density still follows (r0/d)^2, but a
    /// blade's survival is a property of its cell rather than a per-rebuild
    /// roll: it fades by its own level's distance, never pops. Zero disables
    /// the octaves; the whole window runs at full density.
    float fullDensityRadius = 0.0f;
    /// Extra reach past each octave's boundary, matched to how far the
    /// window centre can go stale (the rebuild step). Without it a blade
    /// approaches the player faster than rebuilds can admit it, and fades in
    /// late instead of gently.
    float ringSlack = 0.0f;
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

/// populateArea, spread over frames.
///
/// A 45 yard window walks a couple of hundred thousand lattice cells and fits
/// in a frame; the windows the grass distance slider allows walk tens of
/// millions and do not. begin() records the window and step() advances
/// through it a bounded number of cells at a time, so a rebuild costs the
/// same per frame however large the window is - it just takes more frames,
/// and the field grows in rather than hitching.
///
/// Blade identity is the world-anchored cell hash, so a population built in
/// slices is identical to one built in a single call. populateArea itself is
/// begin() plus one unbounded step(), which is what keeps the two one code
/// path.
class GrassPopulationBuilder {
public:
    /// Start a build. Replaces any build in progress.
    void begin(float centerX, float centerY, float radius,
               const GrassPopulationParams& params, size_t maxBlades);

    /// Walk up to `cellBudget` lattice cells. Returns true when the window is
    /// finished (including a window that hit maxBlades and stopped early).
    bool step(const SuitabilitySampler& sample, const ProfileLookupFn& profileFor,
              size_t cellBudget);

    /// A build has begun and step() has not yet finished it.
    [[nodiscard]] bool active() const { return active_; }
    /// The finished window covered every cell; false when maxBlades cut it off.
    [[nodiscard]] bool complete() const { return complete_; }

    [[nodiscard]] float centerX() const { return centerX_; }
    [[nodiscard]] float centerY() const { return centerY_; }

    /// The blades built so far; the full population once step() returns true.
    [[nodiscard]] std::vector<GrassBladeSample>& blades() { return out_; }

private:
    /// Set up the block bounds for the octave ring the cursor is entering.
    void enterRing();

    GrassPopulationParams params_{};
    std::vector<GrassBladeSample> out_;
    size_t maxBlades_ = 0;
    float centerX_ = 0.0f;
    float centerY_ = 0.0f;
    float radius_ = 0.0f;
    float capFactor_ = 1.0f;
    // The octave rings. Ring k walks blocks of 2^k lattice cells and keeps
    // one survivor per block; ring_ and the block cursor make a build
    // resumable mid-ring.
    int ring_ = 0;
    int maxRing_ = 0;
    float ringInner_ = 0.0f;   // annulus bounds for the current ring, yards
    float ringOuter_ = 0.0f;   // including slack (or the window edge on top)
    float ringEdge_ = 0.0f;    // the ring's own boundary, before slack
    int32_t minBlockX_ = 0;
    int32_t maxBlockX_ = 0;
    int32_t maxBlockY_ = 0;
    int32_t cursorX_ = 0;
    int32_t cursorY_ = 0;
    bool active_ = false;
    bool complete_ = false;
};

} // namespace pipeline
} // namespace wowee
