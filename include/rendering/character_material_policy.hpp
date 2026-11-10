#pragma once

#include <cstdint>

namespace wowee::rendering {

// The character renderer also draws non-character models. An attachment's
// transform comes from its parent, but its geosets and materials belong to the
// attached item, not the character wearing it.
enum class CharacterMaterialDomain { Character, Creature, Scene, Attachment };

inline CharacterMaterialDomain characterMaterialDomain(bool scene, bool creature,
                                                        bool attached, bool effect) {
    if (scene) return CharacterMaterialDomain::Scene;
    // Preserve the existing enchant-effect path separately from item models.
    if (attached && !effect) return CharacterMaterialDomain::Attachment;
    if (creature) return CharacterMaterialDomain::Creature;
    return CharacterMaterialDomain::Character;
}

// Character geoset numbers do not have the same meaning on creatures,
// vehicles or equipment. Only an actual hair texture slot identifies hair on
// creatures; attachments follow their own material rather than hair rules.
inline bool characterBatchIsHair(CharacterMaterialDomain domain, bool hasHairTexture,
                                  uint16_t submeshId, uint16_t blendMode,
                                  uint16_t textureCount) {
    if (domain == CharacterMaterialDomain::Scene ||
        domain == CharacterMaterialDomain::Attachment) return false;
    if (hasHairTexture) return true;
    if (domain == CharacterMaterialDomain::Creature) return false;
    const uint16_t group = submeshId / 100;
    const bool hairGeoset = (group >= 1 && group <= 3) ||
                            (group == 0 && submeshId > 0 && submeshId <= 99);
    return hairGeoset && (blendMode != 0 || textureCount > 1);
}

// Opaque M2 materials on creatures, vehicles and attachments ignore texture
// alpha: the same atlas often contains an alpha mask for a different batch.
// Using it for siege vehicle bodywork or Bloodsurge's sword geometry cuts holes
// through otherwise painted surfaces.
//
// Preserve the existing player-character heuristics until they can be checked
// independently. A declared type-6 hair slot remains a hair cutout for a
// creature, but an arbitrary creature geoset number does not make it hair.
inline bool characterBatchNeedsCutout(CharacterMaterialDomain domain,
                                      uint16_t blendMode, bool textureHasAlpha,
                                      bool hairMaterial) {
    if (domain == CharacterMaterialDomain::Scene ||
        domain == CharacterMaterialDomain::Attachment) return blendMode == 1;
    if (domain == CharacterMaterialDomain::Creature) return blendMode == 1 || hairMaterial;
    return blendMode == 1 || (blendMode == 0 && textureHasAlpha) ||
           (blendMode >= 2 && !textureHasAlpha) || hairMaterial;
}

} // namespace wowee::rendering
