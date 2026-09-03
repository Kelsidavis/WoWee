#pragma once

#include <glm/glm.hpp>
#include <string>
#include <string_view>
#include <cstddef>

namespace wowee {
namespace rendering {

/// The file name of an asset path, lower case, without its extension.
///
/// Token checks run on this rather than on the whole path. Model and texture
/// names arrive as full asset paths, and a directory name poisons every token
/// under it: every model in a PassiveDoodads/Lights directory matched the
/// "light" lantern token and turned wall torches into floating glow sprites,
/// and every character leg texture in Item/TextureComponents/LegLowerTexture
/// matched "glow", because the directory name spells it. The file name
/// carries the real semantics.
///
/// A .m2, .mdx or .blp extension is stripped, so a rule that looks at how a
/// name ends is not reading the extension as part of it.
std::string assetTokenName(const std::string& path);

/// True when the asset's file name contains `token`.
///
/// The one place that decides what "the name contains" means, so a caller
/// cannot accidentally ask the whole path.
bool assetNameHasToken(const std::string& path, std::string_view token);

/// True when the asset's file name contains `token` as a word rather than as
/// the tail of a longer one.
///
/// `fire` is in `hellfire`, and Outland's sky is called HellfireSkyNebula01.
/// Matched as a substring it made every layer of that sky a flame texture, so
/// the black colour key was applied to it - and the colour key discards each
/// fragment darker than a threshold, which most of a nebula is. Turning the
/// camera moves each fragment's sampled luminance across that threshold, so
/// pixels dropped in and out and the sky flickered while the view moved and
/// stood still when it did not.
///
/// The rule is that the token may not be preceded by a letter. `campfire` and
/// `bonfire` still match themselves, being tokens in their own right, and
/// `firebeam` still matches `fire`.
bool assetNameHasWordToken(const std::string& path, std::string_view token);

/// True when the asset's file name names something that burns, and whose dark
/// pixels are therefore background rather than picture.
///
/// One list, because there were two: eleven tokens in M2Renderer::loadTexture
/// and four in CharacterRenderer::loadTexture, both spelled as a search of the
/// whole path. Which textures got a colour key depended on which renderer had
/// asked for them.
bool assetNameLooksLikeFlame(const std::string& path);

/// Ambient sound emitter type for doodad models (fire, water, etc.).
enum class AmbientEmitterType : uint8_t {
    None           = 0,
    FireplaceSmall = 1, ///< Small fire / campfire
    FireplaceLarge = 2, ///< Large brazier / bonfire
    Torch          = 3, ///< Wall torch / standing torch
    Fountain       = 4, ///< Fountain water loop
    Waterfall      = 5, ///< Waterfall ambient
    Forge          = 6, ///< Forge / anvil fire
};

/**
 * Output of classifyM2Model(): all name/geometry-based flags for an M2 model.
 * Pure data - no Vulkan, GPU, or asset-manager dependencies.
 */
struct M2ClassificationResult {
    // --- Collision shape selectors ---
    bool collisionNoBlock            = false; ///< Foliage/soft-trees/rugs: no blocking
    bool collisionBridge             = false; ///< Walk-on-top bridge/plank/walkway
    bool collisionPlanter            = false; ///< Low stepped planter/curb
    bool collisionSteppedFountain    = false; ///< Stepped fountain base
    bool collisionSteppedLowPlatform = false; ///< Low stepped platform (curb/planter/bridge)
    bool collisionStatue             = false; ///< Statue/monument/sculpture
    bool collisionSmallSolidProp     = false; ///< Blockable solid prop (crate/chest/barrel)
    bool collisionNarrowVerticalProp = false; ///< Narrow tall prop (lamp/post/pole)
    bool collisionTreeTrunk          = false; ///< Tree trunk cylinder

    // --- Rendering / effect classification ---
    bool isFoliageLike      = false; ///< Foliage or tree (wind sway, disabled animation)
    bool isSmallFoliage     = false; ///< Small bush/grass/plant (skip during taxi/flight)
    bool isSpellEffect      = false; ///< Spell effect / particle-dominated visual
    bool isLavaModel        = false; ///< Lava surface (UV scroll animation)
    bool isInstancePortal   = false; ///< Instance portal (additive, spin, no collision)
    bool isWaterVegetation  = false; ///< Aquatic vegetation (cattails, kelp, reeds, etc.)
    bool isFireflyEffect    = false; ///< Ambient creature (exempt from particle dampeners)
    bool isElvenLike        = false; ///< Night elf / Blood elf themed model
    bool isLanternLike      = false; ///< Lantern/lamp/light model
    bool isKoboldFlame      = false; ///< Kobold candle/torch model
    bool isGroundDetail     = false; ///< Ground-clutter detail doodad (always non-blocking)
    bool isInvisibleTrap    = false; ///< Event-object invisible trap (no render, no collision)
    bool isSmoke            = false; ///< Smoke model (UV scroll animation)
    bool isWaterfall        = false; ///< Waterfall model (ambient sound + splash particles)
    bool isBrazierOrFire    = false; ///< Brazier / campfire / bonfire model
    bool isGroundFire       = false; ///< Ground fire whose halo follows its lowest flame emitter
    bool isTorch            = false; ///< Wall-mounted or standing torch
    bool isForge            = false; ///< Smithy forge - a contained fire that lights its surroundings
    bool isSkyBird          = false; ///< Flying bird/bat doodad (hide until animation range)
    bool isLightBeam        = false; ///< Distant rotating lighthouse/light-ray beam
    bool isTransportDoodad  = false; ///< Ship sail/paddle child whose motion must remain visible

    // --- Ambient emitter type (for sound system) ---
    AmbientEmitterType ambientEmitterType = AmbientEmitterType::None;

    // --- Animation flags ---
    bool disableAnimation   = false; ///< Keep visually stable (foliage, chest lids, etc.)
    bool shadowWindFoliage  = false; ///< Apply wind sway in shadow pass for foliage/trees
    /// A banner, a flag, a tapestry: cloth hung from its top edge.
    ///
    /// Held at the bar and free at the hem, which is the opposite of a plant -
    /// so it gets a sway of its own rather than the foliage one, and a small
    /// one: a banner indoors moves, it does not flap.
    bool isHangingCloth     = false;
};

/**
 * Classify an M2 model by name and geometry.
 *
 * Pure function - no Vulkan, VkContext, or AssetManager dependencies.
 * All results are derived solely from the model name string and tight vertex bounds.
 *
 * @param name         Full model path/name from the M2 header (any case)
 * @param boundsMin    Per-vertex tight bounding-box minimum
 * @param boundsMax    Per-vertex tight bounding-box maximum
 * @param vertexCount  Number of mesh vertices
 * @param emitterCount Number of particle emitters
 */
M2ClassificationResult classifyM2Model(
    const std::string& name,
    const glm::vec3&   boundsMin,
    const glm::vec3&   boundsMax,
    std::size_t        vertexCount,
    std::size_t        emitterCount);

// ---------------------------------------------------------------------------
// Batch texture classification
// ---------------------------------------------------------------------------

/**
 * Per-batch texture key classification - glow / tint token flags.
 * Input must be a lowercased, backslash-normalised texture path (as stored in
 * M2Renderer's textureKeysLower vector).  Pure data - no Vulkan dependencies.
 */
struct M2BatchTexClassification {
    bool exactLanternGlowTex = false; ///< One of the known exact lantern-glow texture paths
    bool hasGlowToken        = false; ///< glow / flare / halo / light
    bool hasFlameToken       = false; ///< flame / fire / flamelick / ember
    bool hasGlowCardToken    = false; ///< glow / flamelick / lensflare / t_vfx / lightbeam / glowball / genericglow
    bool likelyFlame         = false; ///< fire / flame / torch
    bool lanternFamily       = false; ///< lantern / lamp / elf / silvermoon / quel / thalas
    bool softGlowSurface     = false; ///< Lit glass surface that keeps its mesh beneath a soft halo
    int  glowTint            = 0;     ///< 0 = neutral, 1 = cool (blue/arcane), 2 = warm (red/scarlet)
    bool starPointLayer      = false; ///< A sky model's star-point layer, as opposed to its clouds or planets
};

/**
 * Classify a batch texture by its lowercased path for glow/tint hinting.
 *
 * Pure function - no Vulkan, VkContext, or AssetManager dependencies.
 *
 * @param lowerTexKey Lowercased, backslash-normalised texture path (may be empty)
 */
M2BatchTexClassification classifyBatchTexture(const std::string& lowerTexKey);

// ---------------------------------------------------------------------------
// Lightweight ambient emitter classification (name-only, no geometry needed)
// ---------------------------------------------------------------------------

/// Whether a creature model is one of the deliberately invisible helpers that
/// scripts hang their effects on - triggers, bunnies, and the plain measuring
/// boxes under World\\Scale.
///
/// Only invisiblestalker was recognised, and the list was written out twice in
/// the spawner, so invisibleman and the scale boxes rendered as solid objects
/// standing in the world. UNIT_FLAG_NOT_SELECTABLE covers the same creatures
/// from the other side, for their nameplates and for clicking; this is what
/// keeps the model from being drawn in the first place.
///
/// @param lowerPath Lowercased, backslash-normalised model path
bool isHelperCreatureModel(const std::string& lowerPath);

/**
 * Classify an M2 model path for ambient sound emitter type.
 * Faster than the full classifyM2Model() when only the emitter type is needed.
 *
 * @param lowerName Lowercased model path/name
 * @return AmbientEmitterType::None if the model is not an ambient emitter source
 */
AmbientEmitterType classifyAmbientEmitter(const std::string& lowerName);

} // namespace rendering
} // namespace wowee
