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
// zones. Nothing here knows what Elwynn or Durotar is, and adding a zone would
// be the wrong fix for any of it looking wrong.

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

/// Blend the profiles of an effect's doodads, weighted as the effect weights
/// them. Equal weights when every weight is zero, which some effects have.
///
/// An empty list gives the meadow profile, so an effect whose doodad models
/// are missing still grows ordinary grass rather than nothing.
[[nodiscard]] GrassProfile deriveProfile(const std::vector<std::string>& modelPaths,
                                         const std::vector<uint32_t>& weights);

} // namespace pipeline
} // namespace wowee
