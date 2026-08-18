#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// What the grass on a patch of ground looks like, derived from what the map
// says grows there.
//
// A ground effect names up to four detail doodads and a weight for each. Those
// names carry the classification already: the asset set is named
// <zone><type><n>, and the type is one of about a dozen three-letter codes -
// gra, bus, roc, flo, bon and so on. An effect that places mostly `gra` is
// meadow; one that places mostly `roc` and `bon` is scree with a little dry
// growth between the stones.
//
// So the profile is the weighted mix of those categories, not a table of
// zones. Nothing here knows what Elwynn or Durotar is: what grows is the
// effect data's answer. How it *looks* per region is a separate layer -
// grass_biomes.hpp holds an authored table of per-zone overrides that the
// renderer applies on top of the profile this file derives.

namespace wowee {
namespace pipeline {

/// How grass grows and looks where one ground effect applies.
struct GrassProfile {
    float heightScale = 1.0f;
    float widthScale = 1.0f;
    /// Multiplies the terrain's own suitability. Scree grows less than meadow
    /// even where both say something grows.
    float densityScale = 1.0f;
    glm::vec3 rootColor{0.09f, 0.17f, 0.05f};
    glm::vec3 tipColor{0.34f, 0.50f, 0.16f};
    /// How much blade-to-blade colour varies, 0..1.
    float colorVariation = 0.15f;
    /// Resistance to wind and to being walked through. Higher is stiffer.
    float stiffness = 1.0f;
    /// Chance a blade carries a coloured bloom at its tip. Highest where the
    /// map plants flower doodads; the bloom colour itself is per blade.
    float bloomChance = 0.05f;
    /// Chance a blade has bolted to seed: taller, wispier, a pale seed head.
    /// Highest for dry growth - going to seed is what drying grass does.
    float seedChance = 0.20f;
    /// The bloom palette's two anchors; each blade's flower is a per-seed
    /// blend between them, so two anchors give a family of related colours
    /// rather than two flowers. Defaults are a warm meadow yellow and pink.
    glm::vec3 bloomColorA{0.95f, 0.80f, 0.25f};
    glm::vec3 bloomColorB{0.90f, 0.45f, 0.65f};
    /// The seed head's range, straw through plum by default. Wheat country
    /// narrows this to golds.
    glm::vec3 headColorA{0.72f, 0.64f, 0.42f};
    glm::vec3 headColorB{0.32f, 0.22f, 0.24f};
};

/// The categories the three-letter type codes fall into.
enum class GrassCategory {
    Meadow,   ///< gra, igr, wea, cre, vin, pla
    Scrub,    ///< bus, ibu, sap, tho, bra, leaf
    Flowers,  ///< flo, ifl, clover
    Dry,      ///< bon, con, ras
    Barren,   ///< roc, cor, shl, she, mus - not vegetation at all
};

/// Classify one detail-doodad model path.
///
/// Matches on the type code anywhere in the basename rather than at a fixed
/// offset: a handful of names are longer than six characters
/// (stranglethornfern, maraudondetail) and slicing would read the wrong three
/// letters out of them.
[[nodiscard]] GrassCategory categoriseDoodad(const std::string& modelPath);

/// The profile for a single category.
[[nodiscard]] GrassProfile profileFor(GrassCategory category);

/// Whether a terrain texture is a made surface - road, path, cobble, brick.
///
/// These carry perfectly real ground effects with real densities, so nothing
/// in the effect data says "do not grow here"; the shipped ground-clutter
/// placer keeps detail doodads off roads with exactly this name test and
/// nothing else. Grass needs the same test for the same reason.
[[nodiscard]] bool isRoadLikeTexture(const std::string& texturePath);

/// Blend the profiles of an effect's doodads, weighted as the effect weights
/// them. Equal weights when every weight is zero, which some effects have.
///
/// An empty list gives the meadow profile, so an effect whose doodad models
/// are missing still grows ordinary grass rather than nothing.
[[nodiscard]] GrassProfile deriveProfile(const std::vector<std::string>& modelPaths,
                                         const std::vector<uint32_t>& weights);

} // namespace pipeline
} // namespace wowee
