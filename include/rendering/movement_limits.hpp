#pragma once

#include <cmath>
#include <glm/glm.hpp>

namespace wowee::rendering::movement {

// The retail client rejects ground steeper than 50 degrees and steps higher
// than roughly 0.6 yards. Keep these limits shared by terrain, WMO, and M2
// collision paths so release builds cannot silently diverge by surface type.
inline constexpr float kMaxWalkableSlopeDegrees = 50.0f;
inline constexpr float kMinWalkableNormalZ = 0.642787635f; // cos(50 degrees)
inline constexpr float kMaxStepUp = 0.60f;

inline bool isWalkableNormal(float normalZ) {
    return normalZ >= kMinWalkableNormalZ;
}

/// The upward component of the heightfield's normal at a point, from four
/// samples around it.
///
/// The terrain query answers a height and nothing else, so the slope limit that
/// governs WMO and M2 floors was never applied to the ground itself - the
/// constant for it existed and was only ever used as the fallback limit for
/// those other two. A mountain of any steepness was therefore walkable, held
/// back by nothing but the per-step height budget, which a smooth heightfield
/// never trips. This recovers the missing normal by finite difference.
///
/// sample() returns the height at a point, or nothing where there is no ground
/// - at a hole, or off the loaded tiles. A missing neighbour makes the slope
/// unknowable rather than steep, so the answer is 1 and the caller lets it
/// pass: refusing on absent data would stop the player at every tile edge.
template <typename SampleFn>
glm::vec3 heightfieldNormal(SampleFn&& sample, float x, float y, float spacing) {
    const auto west  = sample(x - spacing, y);
    const auto east  = sample(x + spacing, y);
    const auto south = sample(x, y - spacing);
    const auto north = sample(x, y + spacing);
    if (!west || !east || !south || !north) return {0.0f, 0.0f, 1.0f};

    const float dzdx = (*east - *west) / (2.0f * spacing);
    const float dzdy = (*north - *south) / (2.0f * spacing);
    return glm::vec3(-dzdx, -dzdy, 1.0f) / std::sqrt(dzdx * dzdx + dzdy * dzdy + 1.0f);
}

template <typename SampleFn>
float heightfieldNormalZ(SampleFn&& sample, float x, float y, float spacing) {
    return heightfieldNormal(sample, x, y, spacing).z;
}

/// Where a step taken in flight ends, when it would end inside the ground.
///
/// Flight has no use for the step-up budget that keeps a walker on the ground:
/// a flying mount at speed rises more than 0.6 yards a frame up any steep
/// slope, so the floor pick refused the hillside ahead as out of reach, and
/// nothing else holds the heightfield up in flight. The mount flew into the
/// hill and on through it.
///
/// So the step goes as far as the surface, and what is left of it slides along
/// the surface with the part driving into the slope taken out: a gentle rise
/// is skimmed up, a steep one climbed slowly, and a sheer face stops the step.
/// Flying straight down ends on the ground, which is how a landing starts.
///
/// Only a step that crosses the surface is touched. One that starts under it is
/// in a cave or a tunnel, or somewhere else this cannot vouch for, and ground
/// that answers nothing - a hole, a tile not loaded - is no ground.
template <typename SampleFn>
glm::vec3 flightStepAgainstGround(SampleFn&& sample, const glm::vec3& from,
                                  const glm::vec3& to, float normalSpacing) {
    const auto endGround = sample(to.x, to.y);
    if (!endGround || to.z >= *endGround) return to;
    // Feet on the ground read a hair either side of it.
    constexpr float kContactSlack = 0.25f;
    const auto startGround = sample(from.x, from.y);
    if (!startGround || from.z < *startGround - kContactSlack) return to;

    // Where along the step the feet meet the surface: from is above it (or
    // touching), to is below.
    const glm::vec3 step = to - from;
    float above = 0.0f;
    float below = 1.0f;
    for (int i = 0; i < 10; ++i) {
        const float mid = 0.5f * (above + below);
        const glm::vec3 p = from + step * mid;
        const auto ground = sample(p.x, p.y);
        if (!ground || p.z >= *ground) above = mid;
        else below = mid;
    }
    const glm::vec3 contact = from + step * above;

    glm::vec3 rest = step * (1.0f - above);
    const glm::vec3 n = heightfieldNormal(sample, contact.x, contact.y, normalSpacing);
    const float into = glm::dot(rest, n);
    if (into < 0.0f) rest -= into * n;

    glm::vec3 end = contact + rest;
    if (const auto ground = sample(end.x, end.y); ground && end.z < *ground) end.z = *ground;
    return end;
}

inline bool isReachableStep(float deltaZ) {
    return deltaZ >= -0.25f && deltaZ <= kMaxStepUp;
}

/// Whether the outdoor heightfield at this spot is a roof rather than a floor.
///
/// Inside an interior WMO group - Undercity's halls, a building's rooms - the
/// heightfield overhead is meaningless, and letting it stand as a floor
/// candidate kicks the player up to the surface whenever the WMO floor query
/// finds nothing underfoot.
///
/// But being "inside" is decided by bounding-box containment, and an
/// underground WMO's interior box reaches up through the ground above it. So
/// standing on the hillside over a cave counts as inside, and discarding the
/// terrain there leaves the WMO as the only candidate - whose nearest surface
/// below is the cave's ceiling. The player drops through the hill they are
/// walking on and stands on the roof of the room underneath.
///
/// Hence the height test as well as the containment one: ground higher than the
/// player could step onto cannot be the ground they are standing on, which is
/// the whole of what the veto ever meant. Undercity's surface sits ~113m above
/// its halls and is still refused; a hillside at the feet is kept.
/// Whether this ground is too far above the feet to be the ground underfoot.
///
/// The one question behind both rules below: terrain the player could not step
/// onto is not terrain they are standing on.
inline bool terrainOutOfReach(float terrainZ, float feetZ, float stepUpBudget) {
    return terrainZ > feetZ + stepUpBudget + 0.5f;
}

inline bool terrainIsOverheadRoof(bool insideInteriorWmo, float terrainZ,
                                  float feetZ, float stepUpBudget) {
    return insideInteriorWmo && terrainOutOfReach(terrainZ, feetZ, stepUpBudget);
}

/// The height a floor pick should measure "nearest" from.
///
/// Nearest to the feet is right while the player is standing: it is what lets a
/// step onto a ledge win once the feet are level with it, and what stops a
/// stray frame on the storey above latching the pick up there.
///
/// It is wrong the moment the feet are under the floor they are on. At the
/// Undercity elevator ramp the feet sat 2.81 below the landing - still a
/// candidate - and 2.39 above the hall floor five yards further down, so
/// measured from the feet the lower one was nearer by 0.42 and the player
/// dropped through the ramp. Below the last ground by more than a step is
/// falling, whatever the grounded flag says, so anchor where the airborne case
/// already anchors and the floor underfoot wins by its true margin.
///
/// This cannot haul anyone up onto a floor they really walked off: once they
/// are past its edge it is not a candidate over their feet at all.
inline bool standingOnLastGround(bool grounded, float feetZ, float lastGroundZ) {
    return grounded && feetZ > lastGroundZ - kMaxStepUp;
}

inline float floorArbitrationAnchor(bool grounded, float feetZ, float lastGroundZ) {
    return standingOnLastGround(grounded, feetZ, lastGroundZ) ? feetZ : lastGroundZ;
}

/// At a tunnel seam, whether the WMO floor should be taken over the terrain.
///
/// Take it only when the terrain is not ground the player could be standing
/// on. That is what a tunnel mouth looks like from inside it: the heightfield
/// overhead is the hillside over the tunnel, and letting it win would lift the
/// player back out rather than let them walk in.
///
/// The seam used to prefer the WMO floor whatever it was, and that also fires
/// where a ramp merely passes under the heightfield beside it. The Orgrimmar
/// valley entrance runs its ramp about 1.3 yards below the ground it meets, so
/// a player standing on that ground - terrain right at their feet - was pulled
/// down onto the ramp and ended up inside the hillside, walking through the
/// terrain with the ramp overhead.
///
/// Measured against the feet rather than against the gap between the two
/// surfaces: 1.3 yards is more than a step, so a gap test calls the Orgrimmar
/// ramp a tunnel. Where the player is standing answers it and the gap does not.
inline bool wmoFloorIsWayIn(float terrainZ, float feetZ, float stepUpBudget) {
    return terrainOutOfReach(terrainZ, feetZ, stepUpBudget);
}

} // namespace wowee::rendering::movement
