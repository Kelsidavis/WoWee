#include "rendering/m2_model_classifier.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <string_view>

namespace wowee {
namespace rendering {

namespace {

// Returns true if `lower` contains `token` as a substring.
// Caller must provide an already-lowercased string.
inline bool has(const std::string& lower, std::string_view token) noexcept {
    return lower.find(token) != std::string::npos;
}

/// `has`, but the token may not be the tail of a longer word.
///
/// Written for `fire`, which is in `hellfire`. Outland's sky model is
/// HellfireSkyBox, so it classified as a brazier - and the renderer gives an
/// additive batch of a brazier a lamp flicker keyed on the instance position.
/// A sky dome's position is the camera's, rewritten every frame, so the flicker
/// re-rolled its phase whenever the camera crossed a one-unit cell and the sky
/// strobed. lampFlicker's own comment says a drifting seed does exactly that.
///
/// The same shape as the `forge` rule below, which was added when Ironforge
/// made all 64 doodads of the city into forges.
inline bool hasWord(const std::string& lower, std::string_view token) noexcept {
    for (std::size_t at = lower.find(token); at != std::string::npos;
         at = lower.find(token, at + 1)) {
        if (at == 0 || !std::isalpha(static_cast<unsigned char>(lower[at - 1])))
            return true;
    }
    return false;
}

// Where in the name a token matched, so competing tokens can be ranked.
// Model names are head-final compounds - StranglethornRuins is a ruin,
// DustwallowTree is a tree - so the match ending furthest right is the one that
// says what the model is. A longer token wins a tie on the same end position,
// which is how "corner" beats the "corn" inside it.
struct TokenMatch {
    std::size_t end = 0;
    std::size_t len = 0;
    bool found = false;

    // Ranks after `other`: ends further right, or ends level but is longer.
    [[nodiscard]] bool outranks(const TokenMatch& other) const noexcept {
        if (!found) return false;
        if (!other.found) return true;
        if (end != other.end) return end > other.end;
        return len > other.len;
    }
};

inline TokenMatch lastMatch(const std::string& lower, std::string_view token) noexcept {
    const std::size_t i = lower.rfind(token);
    if (i == std::string::npos) return {};
    return {.end = i + token.size(), .len = token.size(), .found = true};
}

template <std::size_t N>
TokenMatch lastMatchAny(const std::string& lower,
                        const std::array<std::string_view, N>& tokens) noexcept {
    TokenMatch best;
    for (auto tok : tokens) {
        const TokenMatch m = lastMatch(lower, tok);
        if (m.outranks(best)) best = m;
    }
    return best;
}

// Returns true if any token in the compile-time array is a substring of `lower`.
template <std::size_t N>
bool hasAny(const std::string& lower,
            const std::array<std::string_view, N>& tokens) noexcept {
    for (auto tok : tokens)
        if (lower.find(tok) != std::string::npos) return true;
    return false;
}

/// hasAny, refusing a token that is only the tail of a longer word. Used for
/// the flame families, where "fire" is in "hellfire".
template <std::size_t N>
bool hasAnyWord(const std::string& lower,
                const std::array<std::string_view, N>& tokens) noexcept {
    for (auto tok : tokens)
        if (hasWord(lower, tok)) return true;
    return false;
}

} // namespace

std::string assetTokenName(const std::string& path) {
    std::string name = path;
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const size_t lastSep = name.find_last_of("\\/");
    if (lastSep != std::string::npos) name = name.substr(lastSep + 1);
    for (std::string_view ext : {".m2", ".mdx", ".blp"}) {
        if (name.size() > ext.size() &&
            name.compare(name.size() - ext.size(), ext.size(), ext) == 0) {
            name.resize(name.size() - ext.size());
            break;
        }
    }
    return name;
}

bool assetNameHasToken(const std::string& path, std::string_view token) {
    return assetTokenName(path).find(token) != std::string::npos;
}

bool assetNameHasWordToken(const std::string& path, std::string_view token) {
    const std::string name = assetTokenName(path);
    for (std::size_t at = name.find(token); at != std::string::npos;
         at = name.find(token, at + 1)) {
        // Preceded by a letter means this is the tail of a longer word:
        // `fire` inside `hellfire`. A digit or a separator is a boundary,
        // so `fire2` and `stone_fire` still match.
        if (at == 0 || !std::isalpha(static_cast<unsigned char>(name[at - 1]))) {
            return true;
        }
    }
    return false;
}

bool assetNameLooksLikeFlame(const std::string& path) {
    // The tokens both renderers were matching, merged. Character textures used
    // to be excluded by hand because Item/TextureComponents/LegLowerTexture
    // spells "glow" - that exclusion is gone because the directory is gone
    // from the question: assetTokenName reads the file name alone.
    static constexpr std::string_view kFlameTokens[] = {
        "candle", "flame", "fire", "torch", "lamp", "lantern",
        "glow", "flare", "brazier", "campfire", "bonfire",
    };
    for (std::string_view token : kFlameTokens) {
        if (assetNameHasWordToken(path, token)) return true;
    }
    return false;
}

M2ClassificationResult classifyM2Model(
    const std::string& name,
    const glm::vec3&   boundsMin,
    const glm::vec3&   boundsMax,
    std::size_t        vertexCount,
    std::size_t        emitterCount)
{
    // Lowercased full name for explicitly path-based checks (ground detail).
    std::string fullPath = name;
    std::transform(fullPath.begin(), fullPath.end(), fullPath.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    // Token checks run on the file name only. See assetTokenName().
    const std::string n = assetTokenName(name);

    M2ClassificationResult r;

    // ---------------------------------------------------------------
    // Geometry metrics
    // ---------------------------------------------------------------
    const glm::vec3 dims = boundsMax - boundsMin;
    const float horiz    = std::max(dims.x, dims.y);
    const float vert     = std::max(0.0f, dims.z);
    const bool lowWide   = (horiz > 1.4f && vert > 0.2f && vert < horiz * 0.70f);
    const bool lowPlat   = (horiz > 1.8f && vert > 0.2f && vert < 1.8f);

    // ---------------------------------------------------------------
    // Simple single-token flags
    // ---------------------------------------------------------------
    r.isInvisibleTrap = has(n, "invisibletrap");
    r.isGroundDetail  = has(fullPath, "\\nodxt\\detail\\") || has(fullPath, "\\detail\\");
    // A model under PARTICLEEMITTERS\ is pure VFX - the 61 models there are
    // auras, wisps and splashes, and not one of them is a physical object.
    // kEffectTokens already carries "particleemitter", but tokens are matched
    // on the BASENAME, and for these the word is only ever in the directory:
    // the basename is "AuraPurple". So the token never fired and the aura fell
    // through to the generic-solid shape rule instead.
    //
    // That is the Rut'theran portal's second invisible wall. AuraPurple.m2 is
    // the glow inside the arch, placed at scale 10.69, and its unscaled 1.96 x
    // 1.96 x 3.08 bounds land squarely in genericSolid's window - so it became
    // a small solid prop, and at that scale a ~21 x 21 x 33 unit block of solid
    // nothing centred on the portal. The player was stopped exactly where the
    // glow begins.
    const bool particleEmitterVfx = has(fullPath, "\\particleemitters\\")
                                 || has(fullPath, "/particleemitters/");
    r.isSmoke         = has(n, "smoke");
    r.isLavaModel     = has(n, "forgelava") || has(n, "lavapot") || has(n, "lavaflow")
                    || has(n, "lavapool");

    r.isInstancePortal  = has(n, "instanceportal") || has(n, "instancenewportal")
                        || has(n, "portalfx")       || has(n, "spellportal");

    r.isWaterVegetation = has(n, "cattail") || has(n, "reed")     || has(n, "bulrush")
                        || has(n, "seaweed") || has(n, "kelp")    || has(n, "lilypad")
                        || has(n, "waterlily");

    r.isWaterfall       = has(n, "waterfall");

    r.isElvenLike   = has(n, "elf")     || has(n, "elven") || has(n, "quel");
    // Directional volumetric light effects (lighthouse beam, light rays/shafts) match
    // the broad "light" token but are NOT point lanterns. They are animated additive
    // meshes - e.g. the Stormwind lighthouse beam rotates via a looped single-bone
    // animation. Classifying them as lanterns routed their batch through the glow-card
    // path, which skips the mesh and draws a static billboard instead, freezing the
    // sweep. Exclude beams/rays/shafts so their animated mesh renders normally.
    const bool volumetricLightBeam =
        has(n, "beam")     || has(n, "lighthouse") || has(n, "lightray") ||
        has(n, "lightshaft") || has(n, "godray")   || has(n, "sunray");
    r.isLanternLike = (has(n, "lantern") || has(n, "lamp")  || has(n, "light") ||
                       has(n, "sconce")  || has(n, "candle") ||
                       has(n, "candelabra") || has(n, "chandelier"))
                      && !volumetricLightBeam;
    r.isKoboldFlame = has(n, "kobold")
                    && (has(n, "candle") || has(n, "torch") || has(n, "mine"));

    // Fire / brazier / torch model detection (for ambient emitter + rendering)
    const bool fireName    = hasWord(n, "fire") || hasWord(n, "campfire") || hasWord(n, "bonfire");
    const bool brazierName = has(n, "brazier") || has(n, "cauldronfire");
    // A forge is a forge only when "forge" is what the name ends on. Matched as
    // a bare substring it also caught Ironforge, so all 64 doodads of the city
    // - benches, statues, cliffs, elevators, lanterns - were treated as forge
    // fire and drawn additive, which is to say translucent. Anything after the
    // token names something else: IronforgeBench is a bench, ForgeArms are
    // arms, CrystalForgeController is a control panel.
    const bool forgeName = [&] {
        if (has(n, "forgelava")) return false;
        const std::size_t i = n.rfind("forge");
        if (i == std::string::npos) return false;
        for (std::size_t k = i + 5; k < n.size(); ++k) {
            const char c = n[k];
            const bool qualifier = (c >= '0' && c <= '9') || c == '_' || c == '-'
                                || c == '.' || c == ' ';
            if (!qualifier) return false;
        }
        return true;
    }();
    const bool torchName   = has(n, "torch") && !r.isKoboldFlame;
    r.isBrazierOrFire = fireName || brazierName;
    // TaurenLampPost is the small ground-level path fire used around Camp
    // Narache, despite its misleading model name. Its decorative halo must
    // follow the lowest flame emitter rather than the mesh/card center.
    r.isGroundFire    = (fireName && !brazierName) || has(n, "taurenlamppost");
    r.isTorch         = torchName;
    r.isForge         = forgeName;

    // ---------------------------------------------------------------
    // Collision: shape categories (mirrors original logic ordering)
    // ---------------------------------------------------------------
    const bool isPlanter      = has(n, "planter");
    const bool likelyCurb     = isPlanter || has(n, "curb")  || has(n, "base")
                                           || has(n, "ring")  || has(n, "well");
    const bool knownSwPlanter = has(n, "stormwindplanter")
                              || has(n, "stormwindwindowplanter");
    const bool bridgeName     = has(n, "bridge") || has(n, "plank") || has(n, "walkway");
    const bool statueName     = has(n, "statue") || has(n, "monument") || has(n, "sculpture");
    const bool sittable       = has(n, "chair")  || has(n, "bench") || has(n, "stool")
                                                 || has(n, "seat")  || has(n, "throne");
    const bool smallSolid     = (statueName && !sittable)
                              || has(n, "crate") || has(n, "box")
                              || has(n, "chest") || has(n, "barrel")
                              || has(n, "anvil") || has(n, "mailbox")
                              || has(n, "cauldron") || has(n, "cannon")
                              || has(n, "wagon") || has(n, "cart")
                              || has(n, "table") || has(n, "desk");
    const bool chestName      = has(n, "chest");

    r.collisionSteppedFountain    = has(n, "fountain");
    r.collisionSteppedLowPlatform = !r.collisionSteppedFountain
                                  && (knownSwPlanter || bridgeName
                                      || (likelyCurb && (lowPlat || lowWide)));
    r.collisionBridge             = bridgeName;
    r.collisionPlanter            = isPlanter;
    r.collisionStatue             = statueName;

    const bool narrowVertName  = has(n, "lamp")    || has(n, "lantern")
                               || has(n, "post")    || has(n, "pole");
    const bool narrowVertShape = (horiz > 0.12f && horiz < 2.0f
                               && vert  > 2.2f  && vert  > horiz * 1.8f);
    r.collisionNarrowVerticalProp = !r.collisionSteppedFountain
                                  && !r.collisionSteppedLowPlatform
                                  && (narrowVertName || narrowVertShape);

    // ---------------------------------------------------------------
    // Foliage token table (sorted alphabetically)
    // ---------------------------------------------------------------
    static constexpr auto kFoliageTokens = std::to_array<std::string_view>({
        "algae",      "bamboo",     "banana",     "barley",     "bean",
        "bracken",    "bramble",    "branch",     "briar",      "brush",
        "bush",
        "cactus",     "canopy",     "carrot",     "cattail",    "clover",
        "clump",      "coconut",    "coral",      "corn",       "crop",
        "dead-grass", "dead_grass", "deadgrass",
        "dry-grass",  "dry_grass",  "drygrass",
        "fern",       "fernleaf",   "fireflies",  "firefly",    "fireflys",
        "flower",     "frond",      "fungus",     "gourd",      "grapes",
        "grass",
        "hay",        "hedge",      "herb",       "hops",       "ivy",
        "kelp",       "leaf",       "leaves",     "lettuce",    "lichen",
        "lily",
        "melon",      "moss",       "mushroom",   "nettle",
        "okra",       "onion",
        "palm",       "pepper",     "pinecone",   "potato",     "pumpkin",
        "reed",       "root",
        "sapling",    "seaweed",    "seedling",   "shrub",
        // Blizzard's own misspelling, and it is the only spelling those models
        // have: WETLANDSSHURB09.M2 and its siblings. Reported as grass with
        // cobwebs that could not be walked through, and no amount of reading
        // the token list would have found it - the name had to come off the
        // model. The client names what blocks it now for exactly this reason.
        "shurb",
        "sprout",
        "squash",     "stalk",      "thorn",      "thistle",    "toadstool",
        "tomato",     "turnip",
        "underbrush", "vine",       "watermelon", "weed",       "wheat",
    });

    // Words that name a structure. Foliage tokens have to be matched as
    // substrings, because model names run words together with no separator
    // (StranglethornFern01), which rules out matching on word boundaries. The
    // price is that a short token lands inside an unrelated word: "thorn" in
    // Stranglethorn, "corn" in Corner, "hops" in ShopSign, "crop" in Outcrop,
    // "tree" in StreetSign. Every one of those made a rigid prop sway in the
    // wind. Ranking the matches by where they end resolves it without a list of
    // exceptions: StranglethornRuins is a ruin, DustwallowTree is still a tree.
    static constexpr auto kStructureTokens = std::to_array<std::string_view>({
        "arch",      "bridge",    "brick",     "cage",      "chest",
        "cliff",     "column",    "corner",    "door",      "fence",
        "floor",     "frame",     "gate",      "herbalism", "herbalist",
        // A street lamp is ironwork on a post. "street" contains "tree" - the
        // same trap this list already catches for StreetSign - so without a
        // token that ends later in the name a StreetLamp read as a tree, and
        // Stormwind's lamps swayed in the wind like saplings.
        "lamp",      "lantern",
        "outcrop",   "pillar",    "pylon",     "roof",      "rock",
        "ruin",      "shield",    "sign",      "stair",     "statue",
        "stone",     "tomb",      "tower",     "wall",
    });
    const TokenMatch structureHit = lastMatchAny(n, kStructureTokens);

    // "plant" is foliage unless "planter" is also present (planters are solid curbs).
    TokenMatch foliageHit = lastMatchAny(n, kFoliageTokens);
    if (!isPlanter) {
        const TokenMatch plantHit = lastMatch(n, "plant");
        if (plantHit.outranks(foliageHit)) foliageHit = plantHit;
    }
    const TokenMatch treeHit = lastMatch(n, "tree");

    const bool foliageName  = foliageHit.found && !structureHit.outranks(foliageHit);
    const bool treeLike     = treeHit.found && !structureHit.outranks(treeHit);
    const bool hardTreePart = has(n, "trunk") || has(n, "stump") || has(n, "log");

    // A teleport structure is a doorway, not an object: you walk into or onto
    // it. TeleportTree.m2 - the Rut'theran Village portal to Darnassus - is the
    // case that proved it. The file carries no collision geometry at all
    // (nBoundingTriangles = 0, zeroed bounding box), so the real client lets you
    // walk straight through the arch. But the name ends in "tree", so treeLike
    // fired and the trunk-cylinder rule planted a solid block dead centre in the
    // archway - an invisible wall over the portal, only passable with /unstuck.
    //
    // The token-ranking mechanism cannot express this: outranks() needs the
    // structure token to appear LATER in the name, and here "teleport" comes
    // first. So this is checked ahead of the tree rules and wins outright.
    // It covers the whole family - arches, pads and PvP/goblin teleporters -
    // none of which is ever an obstacle.
    const bool teleportStructure = has(n, "teleport");

    // A standing trunk is the solid part of a tree by definition. A stump or a
    // fallen log is a low prop you step over or onto, which the small-solid
    // rules below already describe, so those two keep their exemption.
    const bool standingTrunk = has(n, "trunk");
    const bool lowTreePart   = has(n, "stump") || has(n, "log");

    // Trees big enough to have a visible trunk → solid cylinder collision.
    //
    // Height decides this, not the width of the canopy. Requiring six yards of
    // spread meant a conifer - twenty yards tall and four across - failed the
    // test, fell through to softTree and had its collision turned off outright:
    // a full-sized pine forest that could be walked through. Anything past
    // about eight yards has a trunk whatever its spread, and the width rule is
    // kept for the shorter, broader trees it was written for.
    const bool bigEnoughForTrunk = vert > 4.0f && (horiz > 6.0f || vert > 8.0f);
    const bool treeWithTrunk = (treeLike || standingTrunk) && !lowTreePart && !foliageName
                             && !teleportStructure
                             && bigEnoughForTrunk;
    const bool softTree      = treeLike && !hardTreePart && !treeWithTrunk;

    r.collisionTreeTrunk = treeWithTrunk;

    const bool genericSolid     = (horiz > 0.6f && horiz < 6.0f
                                && vert  > 0.30f && vert  < 4.0f
                                && vert  > horiz * 0.16f) || statueName;
    const bool curbLikeName     = has(n, "curb")   || has(n, "planter")
                                || has(n, "ring")   || has(n, "well")  || has(n, "base");
    const bool lowPlatLikeShape = lowWide || lowPlat;

    r.collisionSmallSolidProp = !r.collisionSteppedFountain
                              && !r.collisionSteppedLowPlatform
                              && !r.collisionNarrowVerticalProp
                              && !r.collisionTreeTrunk
                              && !curbLikeName
                              && !lowPlatLikeShape
                              && (smallSolid
                                  || (genericSolid && !foliageName && !softTree));

    const bool carpetOrRug    = has(n, "carpet") || has(n, "rug");
    const bool forceSolidCurb = r.collisionSteppedLowPlatform || knownSwPlanter
                              || likelyCurb || r.collisionPlanter;
    r.collisionNoBlock        = (foliageName || softTree || carpetOrRug) && !forceSolidCurb;
    // Walk into the arch, walk onto the pad - never around either. Set after the
    // curb override so nothing can put the block back.
    if (teleportStructure) r.collisionNoBlock = true;
    // isSpellEffect already excludes VFX from every collision path; say it
    // outright too, so a future path that reads only this flag agrees.
    if (particleEmitterVfx) r.collisionNoBlock = true;
    // Ground-clutter detail cards are always non-blocking.
    if (r.isGroundDetail) r.collisionNoBlock = true;
    // Small doodads that aren't explicitly solid should not block movement.
    // In WoW, only named solid objects (crates, barrels, anvils, etc.) and
    // large structural doodads have collision - small decorative models are
    // always walkthrough regardless of their name.
    if (!r.collisionNoBlock && !smallSolid && !forceSolidCurb
        && !r.collisionSteppedFountain && !r.collisionTreeTrunk
        && !r.collisionNarrowVerticalProp && !r.collisionStatue
        && horiz < 2.0f && vert < 2.0f) {
        r.collisionNoBlock = true;
    }

    // ---------------------------------------------------------------
    // Ambient creatures: fireflies, dragonflies, moths, butterflies
    // ---------------------------------------------------------------
    static constexpr auto kAmbientTokens = std::to_array<std::string_view>({
        "butterfly", "dragonflies", "dragonfly",
        "fireflies", "firefly",     "fireflys", "moth",
    });
    const bool ambientCreature = hasAny(n, kAmbientTokens);

    // ---------------------------------------------------------------
    // Sky birds / bats: animated flying doodads that look frozen beyond bone range
    // ---------------------------------------------------------------
    static constexpr auto kSkyBirdTokens = std::to_array<std::string_view>({
        "albatross", "carrionbird", "crane", "crow",
        "eagle",     "gull",        "hawk",  "osprey",
        "owl",       "parrot",      "pelican",
        "raven",     "seagull",     "vulture",
    });
    r.isSkyBird = hasAny(n, kSkyBirdTokens) || has(n, "\\bird")
                || has(n, "\\bat\\") || has(n, "\\bat.");
    // These meshes are visible from far beyond the normal skeletal LOD radius,
    // and their sweep is entirely bone-driven. Freezing distant bone matrices
    // therefore freezes the whole lighthouse beam in its bind pose.
    r.isLightBeam = has(n, "lighthousebeam") || has(n, "lightbeam")
                 || has(n, "lightray");
    r.isTransportDoodad = has(n, "transportship_sails")
                       || has(n, "icebreaker_paddlewheel");

    // ---------------------------------------------------------------
    // Animation / foliage rendering flags
    // ---------------------------------------------------------------
    const bool foliageOrTree = foliageName || treeLike;
    r.isFoliageLike    = foliageOrTree && !ambientCreature;
    // Deliberately not ground detail, which was tried and reverted. Every
    // detail doodad ships exactly one bone and one sequence, and that sequence
    // is not always a sway - a number of them carry a small insect or butterfly
    // that flits around the plant, and silencing the lot of them to save the
    // bone took the ambient life of a field with it. The wind in the shader
    // stands down for clutter instead; see the isFoliage == 2 path in
    // m2.vert.glsl, which brushes it aside for the player and leaves the
    // authored motion alone.
    r.disableAnimation = r.isFoliageLike || chestName;
    // Ground clutter is foliage whether or not its name says so: the detail
    // doodads the ground-effect scatterer places are grass, weeds and flowers,
    // and most of them are named for their tileset rather than for a plant
    // (ElwGra01, 8DE_Detail02). Wind and the player's passage apply to them.
    r.shadowWindFoliage = r.isFoliageLike || r.isGroundDetail;
    r.isFireflyEffect   = ambientCreature;

    // Small foliage: foliage-like models with a small bounding box.
    // Used to skip rendering during taxi/flight for performance.
    r.isSmallFoliage = r.isFoliageLike && !treeLike
                     && horiz < 3.0f && vert < 2.0f;

    // ---------------------------------------------------------------
    // Spell effects (named tokens + particle-dominated geometry heuristic)
    // ---------------------------------------------------------------
    static constexpr auto kEffectTokens = std::to_array<std::string_view>({
        "bubbles",        "dustcloud",        "hazardlight",
        "instancenewportal", "instanceportal",
        "lavabubble",     "lavasplash",        "lavasteam",         "levelup",
        "lightshaft",     "mageportal",        "particleemitter",
        "smokepuff",      "sparkle",           "spotlight",
        "volumetriclight", "wisps",            "worldtreeportal",
    });
    // A bare "steam" substring collides with solid models - the Steam Tonk /
    // Steam Tank vehicles, Steamwheedle doodads - turning them additive/unlit
    // (glowing translucent). The low-poly gate below alone is not enough: the
    // TBC/Turtle SteamTonk overlay models are tiny proxies that slip under the
    // vertex threshold, so also exclude vehicle names outright - a "tonk"/"tank"
    // is never a steam VFX (real ones are steam/steamgeyser/lavasteam/etc.).
    const bool steamVehicle = has(n, "tonk") || has(n, "tank");
    const bool steamVfx = has(n, "steam") && !steamVehicle
                        && emitterCount >= 1 && vertexCount <= 200;
    r.isSpellEffect = hasAny(n, kEffectTokens) || steamVfx || particleEmitterVfx
                    || (emitterCount >= 3 && vertexCount <= 200);
    // Instance portals are spell effects too.
    if (r.isInstancePortal) r.isSpellEffect = true;

    // ---------------------------------------------------------------
    // Ambient emitter type (for sound system integration)
    // ---------------------------------------------------------------
    if (r.isBrazierOrFire) {
        const bool isSmallFire = has(n, "small") || has(n, "campfire");
        r.ambientEmitterType = isSmallFire ? AmbientEmitterType::FireplaceSmall
                                           : AmbientEmitterType::FireplaceLarge;
    } else if (r.isTorch) {
        r.ambientEmitterType = AmbientEmitterType::Torch;
    } else if (forgeName) {
        r.ambientEmitterType = AmbientEmitterType::Forge;
    } else if (r.collisionSteppedFountain) {
        r.ambientEmitterType = AmbientEmitterType::Fountain;
    } else if (r.isWaterfall) {
        r.ambientEmitterType = AmbientEmitterType::Waterfall;
    }

    return r;
}

// ---------------------------------------------------------------------------
// classifyBatchTexture
// ---------------------------------------------------------------------------

bool isHelperCreatureModel(const std::string& lowerPath) {
    static constexpr auto kHelperModels = std::to_array<std::string_view>({
        "invisibleman",
        "invisiblestalker",
        "invisible_stalker",
    });
    for (auto token : kHelperModels) {
        if (lowerPath.find(token) != std::string::npos) return true;
    }
    // The measuring boxes: world\scale\1000x1000, 200yardradiusdisc and the
    // rest. Matched by directory, because their names are only numbers and a
    // token like "100x100" would be a poor thing to look for anywhere else.
    return lowerPath.find("world\\scale\\") != std::string::npos ||
           lowerPath.find("world/scale/") != std::string::npos;
}

M2BatchTexClassification classifyBatchTexture(const std::string& lowerTexKey)
{
    M2BatchTexClassification r;

    // The sky models keep their star points in one texture apiece, alongside
    // the cloud, nebula and planet layers of the same dome. Only the star
    // layers are named for stars - except STARSANDCLOUDS, which is both and
    // must stay, and STARBRIGHTENER, which is a brightening card rather than
    // the field itself. The directory is checked as well as the name: a floor
    // doodad called UL_SKY_FLOOR_STARS is not a sky.
    if (lowerTexKey.find("environment\\stars\\") != std::string::npos) {
        // The directory is called stars too, so the name has to be read on its
        // own: matching the whole path made every cloud and planet in the
        // folder a star layer, which would have deleted the sky.
        const std::size_t slash = lowerTexKey.find_last_of("\\/");
        const std::string base = (slash == std::string::npos)
            ? lowerTexKey : lowerTexKey.substr(slash + 1);
        if (base.find("stars") != std::string::npos &&
            base.find("starsandclouds") == std::string::npos &&
            base.find("starbrightener") == std::string::npos) {
            r.starPointLayer = true;
        }
    }

    // Exact paths for well-known lantern / lamp glow-card textures.
    static constexpr auto kExactGlowTextures = std::to_array<std::string_view>({
        "world\\azeroth\\karazahn\\passivedoodads\\bonfire\\flamelicksmallblue.blp",
        "world\\expansion06\\doodads\\nightelf\\7ne_druid_streetlamp01_light.blp",
        "world\\generic\\human\\passive doodads\\stormwind\\t_vfx_glow01_64.blp",
        "world\\generic\\nightelf\\passive doodads\\lamps\\glowblue32.blp",
        "world\\generic\\nightelf\\passive doodads\\magicalimplements\\glow.blp",
    });
    for (auto s : kExactGlowTextures)
        if (lowerTexKey == s) { r.exactLanternGlowTex = true; break; }

    static constexpr auto kGlowTokens = std::to_array<std::string_view>({
        "flare", "glow", "halo", "light",
    });
    static constexpr auto kFlameTokens = std::to_array<std::string_view>({
        "ember", "fire", "flame", "flamelick",
    });
    static constexpr auto kGlowCardTokens = std::to_array<std::string_view>({
        "flamelick", "genericglow", "glow", "glowball",
        "lensflare", "lightbeam",   "t_vfx",
    });
    static constexpr auto kLikelyFlameTokens = std::to_array<std::string_view>({
        "fire", "flame", "torch",
    });
    static constexpr auto kLanternFamilyTokens = std::to_array<std::string_view>({
        "elf", "lamp", "lantern", "quel", "silvermoon", "thalas",
    });
    static constexpr auto kCoolTintTokens = std::to_array<std::string_view>({
        "arcane", "blue", "nightelf",
    });
    static constexpr auto kRedTintTokens = std::to_array<std::string_view>({
        "red", "ruby", "scarlet",
    });

    r.hasGlowToken     = hasAny(lowerTexKey, kGlowTokens);
    r.hasFlameToken    = hasAnyWord(lowerTexKey, kFlameTokens);
    r.hasGlowCardToken = hasAny(lowerTexKey, kGlowCardTokens);
    if (r.exactLanternGlowTex) r.hasGlowCardToken = true;
    r.likelyFlame      = hasAnyWord(lowerTexKey, kLikelyFlameTokens);
    r.lanternFamily    = hasAny(lowerTexKey, kLanternFamilyTokens);
    // Stormwind street lamps use an opaque unlit glass texture rather than a
    // named glow card or particle emitter. Preserve that glass mesh and layer
    // a soft halo over it in the renderer.
    r.softGlowSurface  = lowerTexKey ==
        "dungeons\\textures\\doodads\\stormwindlampglass.blp";
    r.glowTint         = hasAny(lowerTexKey, kCoolTintTokens) ? 1
                       : hasAny(lowerTexKey, kRedTintTokens)  ? 2
                       : 0;

    return r;
}

// ---------------------------------------------------------------------------
// classifyAmbientEmitter - lightweight name-only emitter type detection
// ---------------------------------------------------------------------------

AmbientEmitterType classifyAmbientEmitter(const std::string& lowerName)
{
    const bool fireName    = hasWord(lowerName, "fire") || hasWord(lowerName, "campfire")
                           || hasWord(lowerName, "bonfire");
    const bool brazierName = has(lowerName, "brazier") || has(lowerName, "cauldronfire");
    const bool forgeName   = has(lowerName, "forge") && !has(lowerName, "forgelava");

    if (fireName || brazierName) {
        const bool isSmall = has(lowerName, "small") || has(lowerName, "campfire");
        return isSmall ? AmbientEmitterType::FireplaceSmall
                       : AmbientEmitterType::FireplaceLarge;
    }
    if (has(lowerName, "torch"))     return AmbientEmitterType::Torch;
    if (forgeName)                   return AmbientEmitterType::Forge;
    if (has(lowerName, "fountain"))  return AmbientEmitterType::Fountain;
    if (has(lowerName, "waterfall")) return AmbientEmitterType::Waterfall;
    return AmbientEmitterType::None;
}

} // namespace rendering
} // namespace wowee
