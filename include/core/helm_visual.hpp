#pragma once

// Where a helmet's model and texture live, given its ItemDisplayInfo id.
//
// Head gear is race and gender specific - Helm_Plate_B_01_HuM.m2 is the human
// male cut of the same helm - with a base model as the fallback for the pieces
// that do not vary. Both the local player and every other player need the same
// answer, and they resolve appearance through different classes, so the rule
// lives here rather than in either of them. The cape texture is the standing
// example of what happens otherwise: it was applied in one path and not the
// other, and showed up on the paperdoll while the character in the world wore
// nothing on their back.

#include <cstdint>
#include <string>

namespace wowee {
namespace pipeline { class AssetManager; }

namespace core {

/// The M2 attachment point a helmet hangs from. Attachment 0 is the shield
/// mount; 11 is the head.
///
/// Written out separately in appearance_composer, entity_spawner and
/// entity_spawner_player, which is how one of the three came to keep its copy
/// after it stopped attaching anything.
constexpr uint32_t kAttachHelm = 11;

/// The suffix a per-race, per-gender model carries - "_HuM" and the like.
///
/// Helm_Plate_B_01_HuM.m2 is the human male cut of a helm, and the shoulders
/// use the same suffix. Both the helmet resolver and the shoulder attachment
/// had their own ten-entry map of it, and a race corrected in one would leave
/// the other asking for a file that is not there - which reads as "this piece
/// has no per-race cut" rather than as an error, because that is exactly how a
/// missing file is treated.
///
/// Empty for a race with no per-race art, which is the answer for goblin and
/// worgen: they arrive in Cataclysm and a 3.3.5 install has nothing to point
/// a suffix at.
inline std::string raceGenderSuffix(uint8_t raceId, uint8_t genderId) {
    // Read off the file names in the archive rather than from a race enum,
    // which is why the undead are "Sc" - the art uses Scourge.
    switch (raceId) {
        case 1:  return genderId == 0 ? "_HuM" : "_HuF";
        case 2:  return genderId == 0 ? "_OrM" : "_OrF";
        case 3:  return genderId == 0 ? "_DwM" : "_DwF";
        case 4:  return genderId == 0 ? "_NiM" : "_NiF";
        case 5:  return genderId == 0 ? "_ScM" : "_ScF";
        case 6:  return genderId == 0 ? "_TaM" : "_TaF";
        case 7:  return genderId == 0 ? "_GnM" : "_GnF";
        case 8:  return genderId == 0 ? "_TrM" : "_TrF";
        case 10: return genderId == 0 ? "_BeM" : "_BeF";
        case 11: return genderId == 0 ? "_DrM" : "_DrF";
        default: return {};
    }
}

struct HelmVisual {
    /// Candidate model paths, most specific first. The caller loads them in
    /// order because only it knows how to load an M2, and stops at the first
    /// that parses.
    std::string racialModelPath;   ///< Empty when the race has no known suffix.
    std::string baseModelPath;
    std::string texturePath;       ///< Already resolved against what exists.

    [[nodiscard]] bool valid() const { return !baseModelPath.empty(); }
};

/// Resolve the head model and texture for an ItemDisplayInfo id. Returns an
/// empty result when the id is unknown or carries no model.
HelmVisual resolveHelmVisual(pipeline::AssetManager& assets,
                             uint32_t itemDisplayInfoId,
                             uint8_t raceId,
                             uint8_t genderId);

/// Whether this head item covers the hair.
///
/// Not every head slot item does: a circlet, tiara or crown sits over the hair
/// and leaves it showing, and the data says which is which. ItemDisplayInfo
/// points at a HelmetGeosetVisData row per gender, and that row carries the
/// masks of what to hide - the row circlets and crowns use is all zeroes, while
/// a plate helm's is not. An id of 0 likewise hides nothing.
bool helmHidesHair(pipeline::AssetManager& assets,
                   uint32_t itemDisplayInfoId,
                   uint8_t genderId);

} // namespace core
} // namespace wowee
