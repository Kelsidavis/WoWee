#pragma once

/// What an M2 material's blend mode means for the pipeline it draws on.
///
/// A batch whose texture carries no alpha channel used to be forced onto the
/// cutout pipeline whenever its blend mode was 2 or higher. For blend mode 2 -
/// alpha blend - that is reasonable: blending by an alpha that is 1 everywhere
/// is the same as drawing opaque, so cutting out at least gives the artist's
/// silhouette a chance.
///
/// For the additive modes it is wrong, and destroys the effect. Additive does
/// not use alpha for transparency at all - black is what disappears, because
/// adding zero changes nothing. A glow card is authored exactly that way: an
/// opaque texture, bright in the middle, black at the edges.
///
/// Orgrimmar's bonfire is the case that showed it. Its glow is
/// GENERICGLOW_ALPHA_128.BLP - named for an alpha it does not have, alphaDepth
/// 0 in the file - on a material with blend mode 4. Forced to cutout, every
/// texel passed the test and the flame rendered as a flat opaque grey disc
/// sitting behind the logs.

#include <cstdint>

namespace wowee::rendering {

/// M2 material blend modes, as stored.
enum M2BlendMode : uint8_t {
    M2_BLEND_OPAQUE = 0,
    M2_BLEND_ALPHA_KEY = 1,   ///< cutout: alpha tested against a threshold
    M2_BLEND_ALPHA = 2,       ///< blended by the texture's alpha
    M2_BLEND_ADD = 3,         ///< added to what is behind; black is invisible
    M2_BLEND_ADD_ALPHA = 4,   ///< additive, scaled by alpha where there is one
    M2_BLEND_MODULATE = 5,
    M2_BLEND_MODULATE2X = 6,
};

/// Back-face culling for solid/cutout foliage with an authored one-sided
/// material. Do not guess when a material is missing, or when a draw group
/// contains mirrored/degenerate transforms whose winding can differ. The sky,
/// transparent effects and other models keep their existing pipelines.
inline bool m2UseFoliageCull(bool enabled, bool foliage, bool sky,
                             bool hasAuthoredMaterial, bool windingPreserved,
                             uint16_t materialFlags, uint16_t blendMode) {
    constexpr uint16_t kTwoSided = 0x04;
    return enabled && foliage && !sky && hasAuthoredMaterial && windingPreserved &&
           !(materialFlags & kTwoSided) && blendMode <= M2_BLEND_ALPHA_KEY;
}

/// True when the mode adds to the framebuffer rather than covering it.
///
/// These never need an alpha channel: black is the transparent colour.
inline bool m2BlendIsAdditive(uint8_t blendMode) {
    return blendMode == M2_BLEND_ADD || blendMode == M2_BLEND_ADD_ALPHA;
}

/// Should this batch be alpha tested?
///
/// Alpha key is, that being what it means - but only where there is alpha to
/// key on. A blended batch whose texture has no alpha is tested as a fallback,
/// because blending by a missing alpha draws it solid. An additive batch never
/// is: it has nothing to test and does not need one.
///
/// An alpha key over a texture with no transparent texel is the case worth
/// naming. Every texel passes, so the test decides nothing - but it still puts
/// the batch on the cutout pipeline, where alpha-to-coverage decides how much
/// of each sample the batch covers. Tirisfal's canopy trees are two batches,
/// and their trunk is exactly this: TirrisFallCanopyTree01_Trunk is DXT1 with
/// no punch-through block anywhere in it, drawn alpha-keyed.
///
/// Canopytree07, the one of the seven whose trunk is plain opaque, was read
/// here as the one that rendered right. It was not. Its trunk samples
/// SilverPineTree01TrunkSkin, which does carry alpha, and the foliage rule in
/// m2_renderer_render.cpp forced it onto the cutout pipeline whatever this
/// function said - so it had the same hole through its middle that every
/// Silverpine tree had. Opaque was not reaching the pipeline at all until that
/// rule learned to ask BLPImage::alphaIsSilhouette.
///
/// Safe to lean on, because hasAlpha is measured rather than guessed:
/// BLPImage::hasTransparency walks level 0 and, for DXT1, counts a block only
/// when it is in punch-through mode and a texel actually selects index 3.
inline bool m2BatchNeedsAlphaTest(uint8_t blendMode, bool hasAlpha) {
    if (blendMode == M2_BLEND_ALPHA_KEY) return hasAlpha;
    if (m2BlendIsAdditive(blendMode)) return false;
    return blendMode >= M2_BLEND_ALPHA && !hasAlpha;
}

/// Should this batch have its black keyed out?
///
/// The key discards every texel darker than a threshold, and exists so a card
/// authored with a black backing can be drawn opaquely without the backing
/// showing. An additive batch has no such problem - black adds nothing - and
/// the key actively destroys it: a glow card is a radial gradient from black
/// to white, so discarding everything below the threshold turns a soft falloff
/// into a hard-edged disc. Orgrimmar's bonfires used a threshold of 0.7 on
/// exactly such a card.
inline bool m2BatchWantsColorKey(uint8_t blendMode, bool textureIsKeyed) {
    return textureIsKeyed && !m2BlendIsAdditive(blendMode);
}

}  // namespace wowee::rendering
